#include "mtf/SSM/Homography.h"
#include "mtf/SSM/HomographyEstimator.h"
#include "mtf/Utilities/warpUtils.h"
#include "mtf/Utilities/miscUtils.h"

_MTF_BEGIN_NAMESPACE

HomographyParams::HomographyParams(const SSMParams *ssm_params, 
bool _normalized_init,
bool _corner_based_sampling, bool _debug_mode):
SSMParams(ssm_params),
normalized_init(_normalized_init),
corner_based_sampling(_corner_based_sampling),
debug_mode(_debug_mode){}

HomographyParams::HomographyParams(const HomographyParams *params) :
SSMParams(params),
normalized_init(HOM_NORMALIZED_BASIS),
corner_based_sampling(HOM_DIRECT_SAMPLES),
debug_mode(HOM_DEBUG_MODE){
	if(params){
		corner_based_sampling = params->corner_based_sampling;
		normalized_init = params->normalized_init;
		debug_mode = params->debug_mode;
	}
}

Homography::Homography(
	const ParamType *_params) : 
	ProjectiveBase(_params), params(_params){

	printf("\n");
	printf("Using Homography SSM with:\n");
	printf("resx: %d\n", resx);
	printf("resy: %d\n", resy);
	printf("corner_based_sampling: %d\n", params.corner_based_sampling);
	printf("normalized_init: %d\n", params.normalized_init);
	printf("debug_mode: %d\n", params.debug_mode);

	name = "homography";
	state_size = 8;
	curr_state.resize(state_size);
	warp_mat = Matrix3d::Identity();
}

void Homography::setCorners(const CornersT& corners){
	curr_corners = corners;
	utils::homogenize(curr_corners, curr_corners_hm);

	getPtsFromCorners(curr_warp, curr_pts, curr_pts_hm, curr_corners);

	if(params.normalized_init){
		init_corners = getNormCorners();
		init_corners_hm = getHomNormCorners();
		init_pts = getNormPts();
		init_pts_hm = getHomNormPts();

		getStateFromWarp(curr_state, curr_warp);
	} else{
		init_corners = curr_corners;
		init_corners_hm = curr_corners_hm;
		init_pts = curr_pts;
		init_pts_hm = curr_pts_hm;
		curr_warp = Matrix3d::Identity();
		curr_state.fill(0);
	}
}

void Homography::compositionalUpdate(const VectorXd& state_update){
	validate_ssm_state(state_update);
	//utils::printMatrix(curr_warp, "curr_warp before");

	getWarpFromState(warp_update_mat, state_update);
	curr_warp = curr_warp * warp_update_mat;
	curr_warp /= curr_warp(2, 2);

	//utils::printMatrix(state_update, "state_update");
	//utils::printMatrix(warp_update_mat, "warp_update_mat");
	//utils::printMatrix(curr_warp, "curr_warp after");

	getStateFromWarp(curr_state, curr_warp);

	curr_pts_hm.noalias() = curr_warp * init_pts_hm;
	curr_corners_hm.noalias() = curr_warp * init_corners_hm;

	utils::dehomogenize(curr_pts_hm, curr_pts);
	utils::dehomogenize(curr_corners_hm, curr_corners);
}

void Homography::getWarpFromState(Matrix3d &warp_mat,
	const VectorXd& ssm_state){
	validate_ssm_state(ssm_state);

	warp_mat(0, 0) = 1 + ssm_state(0);
	warp_mat(0, 1) = ssm_state(1);
	warp_mat(0, 2) = ssm_state(2);
	warp_mat(1, 0) = ssm_state(3);
	warp_mat(1, 1) = 1 + ssm_state(4);
	warp_mat(1, 2) = ssm_state(5);
	warp_mat(2, 0) = ssm_state(6);
	warp_mat(2, 1) = ssm_state(7);
	warp_mat(2, 2) = 1;
}

void Homography::invertState(VectorXd& inv_state, const VectorXd& state){
	getWarpFromState(warp_mat, state);
	inv_warp_mat = warp_mat.inverse();
	inv_warp_mat /= inv_warp_mat(2, 2);
	getStateFromWarp(inv_state, inv_warp_mat);
}

void Homography::getStateFromWarp(VectorXd &state_vec,
	const Matrix3d& warp_mat){
	validate_ssm_state(state_vec);
	// since homography matrix is defined only up to a scale factor, this function assumes that 
	// the provided warp matrix has its bottom left entry as 1
	//utils::printMatrix(warp_mat, "warp_mat");
	assert(fabs(warp_mat(2, 2) - 1.0)<DBL_EPSILON);

	state_vec(0) = warp_mat(0, 0) - 1;
	state_vec(1) = warp_mat(0, 1);
	state_vec(2) = warp_mat(0, 2);
	state_vec(3) = warp_mat(1, 0);
	state_vec(4) = warp_mat(1, 1) - 1;
	state_vec(5) = warp_mat(1, 2);
	state_vec(6) = warp_mat(2, 0);
	state_vec(7) = warp_mat(2, 1);
}

void Homography::getInitPixGrad(Matrix2Xd &ssm_grad, int pix_id) {
	double x = init_pts(0, pix_id);
	double y = init_pts(1, pix_id);

	ssm_grad <<
		x, y, 1, 0, 0, 0, -x*x, -y*x,
		0, 0, 0, x, y, 1, -x*y, -y*y;
}

void Homography::getCurrPixGrad(Matrix2Xd &dw_dp, int pt_id) {
	double x = init_pts(0, pt_id);
	double y = init_pts(1, pt_id);

	double curr_x = curr_pts(0, pt_id);
	double curr_y = curr_pts(1, pt_id);
	double inv_d = 1.0 / curr_pts_hm(2, pt_id);

	dw_dp <<
		x, y, 1, 0, 0, 0, -x*curr_x, -y*curr_x,
		0, 0, 0, x, y, 1, -x*curr_y, -y*curr_y;
	dw_dp *= inv_d;
}

void Homography::cmptInitPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dw){
	validate_ssm_jacobian(dI_dp, dI_dw);

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; ++pt_id){

		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);
		
		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		for(int ch_id = 0; ch_id < n_channels; ++ch_id){

			double Ix = dI_dw(ch_pt_id, 0);
			double Iy = dI_dw(ch_pt_id, 1);

			double Ixx = Ix * x;
			double Iyy = Iy * y;
			double Ixy = Ix * y;
			double Iyx = Iy * x;

			dI_dp(ch_pt_id, 0) = Ixx;
			dI_dp(ch_pt_id, 1) = Ixy;
			dI_dp(ch_pt_id, 2) = Ix;
			dI_dp(ch_pt_id, 3) = Iyx;
			dI_dp(ch_pt_id, 4) = Iyy;
			dI_dp(ch_pt_id, 5) = Iy;
			dI_dp(ch_pt_id, 6) = -x*Ixx - y*Iyx;
			dI_dp(ch_pt_id, 7) = -x*Ixy - y*Iyy;

			++ch_pt_id;
		}
	}
}

void Homography::cmptPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dw){
	validate_ssm_jacobian(dI_dp, dI_dw);
	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		double curr_x = curr_pts(0, pt_id);
		double curr_y = curr_pts(1, pt_id);
		double inv_d = 1.0 / curr_pts_hm(2, pt_id);		

		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = dI_dw(ch_pt_id, 0) * inv_d;
			double Iy = dI_dw(ch_pt_id, 1) * inv_d;

			double Ixx = Ix * x;
			double Iyy = Iy * y;
			double Ixy = Ix * y;
			double Iyx = Iy * x;

			dI_dp(ch_pt_id, 0) = Ixx;
			dI_dp(ch_pt_id, 1) = Ixy;
			dI_dp(ch_pt_id, 2) = Ix;
			dI_dp(ch_pt_id, 3) = Iyx;
			dI_dp(ch_pt_id, 4) = Iyy;
			dI_dp(ch_pt_id, 5) = Iy;
			dI_dp(ch_pt_id, 6) = (-curr_x*Ixx - curr_y*Iyx);
			dI_dp(ch_pt_id, 7) = (-curr_x*Ixy - curr_y*Iyy);

			++ch_pt_id;
		}
	}
	//dI_dw.array().colwise() /= curr_pts_hm.array().row(2).transpose();
}

void Homography::cmptWarpedPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dw) {
	validate_ssm_jacobian(dI_dp, dI_dw);

	double a00 = curr_warp(0, 0);
	double a01 = curr_warp(0, 1);
	double a10 = curr_warp(1, 0);
	double a11 = curr_warp(1, 1);
	double a20 = curr_warp(2, 0);
	double a21 = curr_warp(2, 1);

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; ++pt_id) {
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double w_x = curr_pts(0, pt_id);
		double w_y = curr_pts(1, pt_id);

		double D = curr_pts_hm(2, pt_id);
		double inv_det = 1.0 / D;

		double dwx_dx = (a00 - a20*w_x);
		double dwx_dy = (a01 - a21*w_x);
		double dwy_dx = (a10 - a20*w_y);
		double dwy_dy = (a11 - a21*w_y);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		//double Ix = pix_grad(pt_id, 0);
		//double Iy = pix_grad(pt_id, 1);
		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = (dwx_dx*dI_dw(ch_pt_id, 0) + dwy_dx*dI_dw(ch_pt_id, 1))*inv_det;
			double Iy = (dwx_dy*dI_dw(ch_pt_id, 0) + dwy_dy*dI_dw(ch_pt_id, 1))*inv_det;

			double Ixx = Ix * x;
			double Ixy = Ix * y;
			double Iyy = Iy * y;
			double Iyx = Iy * x;

			//dI_dp(ch_pt_id, 0) = (Ixx*a + Iyx*c) * inv_det2;
			//dI_dp(ch_pt_id, 1) = (Ixy*a + Iyy*c) * inv_det2;
			//dI_dp(ch_pt_id, 2) = (Ix*a + Iy*c) * inv_det2;
			//dI_dp(ch_pt_id, 3) = (Ixx*b + Iyx*d) * inv_det2;
			//dI_dp(ch_pt_id, 4) = (Ixy*b + Iyy*d) * inv_det2;
			//dI_dp(ch_pt_id, 5) = (Ix*b + Iy*d) * inv_det2;
			//dI_dp(ch_pt_id, 6) = -(Ixx*(a*x + b*y) +
			//	Iyx*(c*x + d*y)) * inv_det2;
			//dI_dp(ch_pt_id, 7) = -(Ixy*(a*x + b*y) +
			//	Iyy*(c*x + d*y)) * inv_det2;

			dI_dp(ch_pt_id, 0) = Ixx;
			dI_dp(ch_pt_id, 1) = Ixy;
			dI_dp(ch_pt_id, 2) = Ix;
			dI_dp(ch_pt_id, 3) = Iyx;
			dI_dp(ch_pt_id, 4) = Iyy;
			dI_dp(ch_pt_id, 5) = Iy;
			dI_dp(ch_pt_id, 6) = -x*Ixx - y*Iyx;
			dI_dp(ch_pt_id, 7) = -x*Ixy - y*Iyy;

			++ch_pt_id;
		}
	}
}

void Homography::cmptApproxPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dw) {
	validate_ssm_jacobian(dI_dp, dI_dw);

	double h00_plus_1 = curr_warp(0, 0);
	double h01 = curr_warp(0, 1);
	double h10 = curr_warp(1, 0);
	double h11_plus_1 = curr_warp(1, 1);
	double h20 = curr_warp(2, 0);
	double h21 = curr_warp(2, 1);

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double curr_x = curr_pts(0, pt_id);
		double curr_y = curr_pts(1, pt_id);

		double a = (h00_plus_1 - h20*curr_x);
		double b = (h01 - h21*curr_x);
		double c = (h10 - h20*curr_y);
		double d = (h11_plus_1 - h21*curr_y);

		double inv_factor = 1.0 / (a*d - b*c);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		for(int ch_id = 0; ch_id < n_channels; ++ch_id){

			double Ix = (d*dI_dw(ch_pt_id, 0) - c*dI_dw(ch_pt_id, 1))*inv_factor;
			double Iy = (a*dI_dw(ch_pt_id, 1) - b*dI_dw(ch_pt_id, 0))*inv_factor;

			double Ixx = Ix * x;
			double Ixy = Ix * y;
			double Iyy = Iy * y;
			double Iyx = Iy * x;

			dI_dp(ch_pt_id, 0) = Ixx;
			dI_dp(ch_pt_id, 1) = Ixy;
			dI_dp(ch_pt_id, 2) = Ix;
			dI_dp(ch_pt_id, 3) = Iyx;
			dI_dp(ch_pt_id, 4) = Iyy;
			dI_dp(ch_pt_id, 5) = Iy;
			dI_dp(ch_pt_id, 6) = (-curr_x*Ixx - curr_y*Iyx);
			dI_dp(ch_pt_id, 7) = (-curr_x*Ixy - curr_y*Iyy);

			//dI_dp(ch_pt_id, 0) = (Ixx*d - Iyx*c) * inv_det;
			//dI_dp(ch_pt_id, 1) = (Ixy*d - Iyy*c) * inv_det;
			//dI_dp(ch_pt_id, 2) = (Ix*d - Iy*c) * inv_det;
			//dI_dp(ch_pt_id, 3) = (Iyx*a - Ixx*b) * inv_det;
			//dI_dp(ch_pt_id, 4) = (Iyy*a - Ixy*b) * inv_det;
			//dI_dp(ch_pt_id, 5) = (Iy*a - Ix*b) * inv_det;
			//dI_dp(ch_pt_id, 6) = (Ix*(b*curr_y*x - d*curr_x*x) + 
			//	Iy*(c*curr_x*x - a*curr_y*x)) * inv_det;
			//dI_dp(ch_pt_id, 7) = (Ix*(b*curr_y*y - d*curr_x*y) +
			//	Iy*(c*curr_x*y - a*curr_y*y)) * inv_det;

			++ch_pt_id;

		}
	}
}

void Homography::cmptInitPixHessian(MatrixXd &_d2I_dp2, const PixHessT &d2I_dw2,
	const PixGradT &dI_dw){
	validate_ssm_hessian(_d2I_dp2, d2I_dw2, dI_dw);
	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		Matrix28d dw_dp;
		dw_dp <<
			x, y, 1, 0, 0, 0, -x*x, -x*y,
			0, 0, 0, x, y, 1, -y*x, -y*y;

		for(int ch_id = 0; ch_id < n_channels; ++ch_id){

			double Ix = dI_dw(ch_pt_id, 0);
			double Iy = dI_dw(ch_pt_id, 1);

			double Ixx = Ix * x;
			double Ixy = Ix * y;
			double Iyy = Iy * y;
			double Iyx = Iy * x;

			double Ixxx = Ixx * x;
			double Ixxy = Ixx * y;
			double Ixyy = Ixy * y;

			double Iyyy = Iyy * y;
			double Iyyx = Iyy * x;
			double Iyxx = Iyx * x;

			Map<Matrix8d> d2I_dp2(_d2I_dp2.col(ch_pt_id).data());

			d2I_dp2 = dw_dp.transpose()*
				Map<const Matrix2d>(d2I_dw2.col(ch_pt_id).data())*dw_dp;

			d2I_dp2(0, 6) -= Ixxx; 
			d2I_dp2(0, 7) -= Ixxy;
			d2I_dp2(1, 6) -= Ixxy;
			d2I_dp2(1, 7) -= Ixyy; 
			d2I_dp2(2, 6) -= Ixx; 
			d2I_dp2(2, 7) -= Ixy; 

			d2I_dp2(3, 6) -= Iyxx;
			d2I_dp2(3, 7) -= Iyyx; 
			d2I_dp2(4, 6) -= Iyyx; 
			d2I_dp2(4, 7) -= Iyyy; 
			d2I_dp2(5, 6) -= Iyx; 
			d2I_dp2(5, 7) -= Iyy;

			d2I_dp2(6, 6) += 2 * (Ixxx*x + Iyxx*y);
			d2I_dp2(6, 7) += 2 * (Ixxy*x + Iyyx*y);
			d2I_dp2(7, 6) += 2 * (Ixxy*x + Iyyx*y);
			d2I_dp2(7, 7) += 2 * (Ixyy*x + Iyyy*y);

			d2I_dp2.bottomLeftCorner<2, 5>() = d2I_dp2.topRightCorner<5, 2>().transpose();


			++ch_pt_id;
		}
	}
}

void Homography::cmptPixHessian(MatrixXd &_d2I_dp2, const PixHessT &d2I_dw2,
	const PixGradT &dI_dw){
	validate_ssm_hessian(_d2I_dp2, d2I_dw2, dI_dw);

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		double curr_x = curr_pts(0, pt_id);
		double curr_y = curr_pts(1, pt_id);

		Matrix28d dw_dp;
		dw_dp << 
			x, y, 1, 0, 0, 0, -curr_x*x, -curr_x*y,
			0, 0, 0, x, y, 1, -curr_y*x, -curr_y*y;
		dw_dp /= curr_pts_hm(2, pt_id);

		double inv_d_squared = 1.0 / (curr_pts_hm(2, pt_id)*curr_pts_hm(2, pt_id));

		for(int ch_id = 0; ch_id < n_channels; ++ch_id){

			Map<Matrix8d> d2I_dp2(_d2I_dp2.col(ch_pt_id).data());
			d2I_dp2 = dw_dp.transpose()*Map<const Matrix2d>(d2I_dw2.col(ch_pt_id).data())*dw_dp;

			double Ix = dI_dw(ch_pt_id, 0);
			double Iy = dI_dw(ch_pt_id, 1);

			double Ixx = Ix * x;
			double Ixy = Ix * y;
			double Iyy = Iy * y;
			double Iyx = Iy * x;

			double Ixxx = Ixx * x;
			double Ixxy = Ixx * y;
			double Ixyy = Ixy * y;

			double Iyyy = Iyy * y;
			double Iyyx = Iyy * x;
			double Iyxx = Iyx * x;

			//d2I_dp2(0, 6) += Ixxx; d2I_dp2(6, 0) += Ixxx;
			//d2I_dp2(1, 6) += Ixxy; d2I_dp2(6, 1) += Ixxy;
			//d2I_dp2(2, 6) += Ixx; d2I_dp2(6, 2) += Ixx;
			//d2I_dp2(3, 6) += Iyxx; d2I_dp2(6, 3) += Iyxx;
			//d2I_dp2(4, 6) += Iyyx; d2I_dp2(6, 4) += Iyyx;
			//d2I_dp2(5, 6) += Iyx; d2I_dp2(6, 5) += Iyx;
			//d2I_dp2(6, 6) -= Ixxx*curr_x + Iyxx*curr_y;
			//d2I_dp2(7, 6) -= Ixxy*curr_x + Iyyx*curr_y;

			//d2I_dp2(0, 7) += Ixxy; d2I_dp2(7, 0) += Ixxy;
			//d2I_dp2(1, 7) += Ixyy; d2I_dp2(7, 1) += Ixyy;
			//d2I_dp2(2, 7) += Ixy; d2I_dp2(7, 2) += Ixy;
			//d2I_dp2(3, 7) += Iyyx; d2I_dp2(7, 3) += Iyyx;
			//d2I_dp2(4, 7) += Iyyy; d2I_dp2(7, 4) += Iyyy;
			//d2I_dp2(5, 7) += Iyy; d2I_dp2(7, 5) += Iyy;
			//d2I_dp2(6, 7) -= Ixxy*curr_x + Iyyx*curr_y;
			//d2I_dp2(7, 7) -= Ixyy*curr_x + Iyyy*curr_y;

			d2I_dp2(0, 6) -= Ixxx*inv_d_squared;
			d2I_dp2(1, 6) -= Ixxy*inv_d_squared;
			d2I_dp2(2, 6) -= Ixx*inv_d_squared;
			d2I_dp2(3, 6) -= Iyxx*inv_d_squared;
			d2I_dp2(4, 6) -= Iyyx*inv_d_squared;
			d2I_dp2(5, 6) -= Iyx*inv_d_squared;
			d2I_dp2(6, 6) += 2 * (Ixxx*curr_x + Iyxx*curr_y)*inv_d_squared;
			d2I_dp2(7, 6) += 2 * (Ixxy*curr_x + Iyyx*curr_y)*inv_d_squared;

			d2I_dp2(0, 7) -= Ixxy*inv_d_squared;
			d2I_dp2(1, 7) -= Ixyy*inv_d_squared;
			d2I_dp2(2, 7) -= Ixy*inv_d_squared;
			d2I_dp2(3, 7) -= Iyyx*inv_d_squared;
			d2I_dp2(4, 7) -= Iyyy*inv_d_squared;
			d2I_dp2(5, 7) -= Iyy*inv_d_squared;
			d2I_dp2(6, 7) += 2 * (Ixxy*curr_x + Iyyx*curr_y)*inv_d_squared;
			d2I_dp2(7, 7) += 2 * (Ixyy*curr_x + Iyyy*curr_y)*inv_d_squared;

			d2I_dp2.bottomLeftCorner<2, 5>() = d2I_dp2.topRightCorner<5, 2>().transpose();

			++ch_pt_id;

		}
	}
}

/**
second order derivative of I(w(u(X, q), p)) w.r.t. q evaluated at q=0 with X=(x, y), w=(w_x, w_y)
d2I_dq2 =
*/
void Homography::cmptWarpedPixHessian(MatrixXd &_d2I_dp2, const PixHessT &d2I_dw2,
	const PixGradT &dI_dw) {
	validate_ssm_hessian(_d2I_dp2, d2I_dw2, dI_dw);

	double a00 = curr_warp(0, 0);
	double a01 = curr_warp(0, 1);
	double a10 = curr_warp(1, 0);
	double a11 = curr_warp(1, 1);
	double a20 = curr_warp(2, 0);
	double a21 = curr_warp(2, 1);

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; ++pt_id) {
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		Matrix2d dw_dX, d2wx_dX2, d2wy_dX2;

		double w_x = curr_pts(0, pt_id);
		double w_y = curr_pts(1, pt_id);
		double D = curr_pts_hm(2, pt_id);
		double D_inv = 1.0 / D;

		double dwx_dx = (a00 - a20*w_x) * D_inv;
		double dwx_dy = (a01 - a21*w_x) * D_inv;
		double dwy_dx = (a10 - a20*w_y) * D_inv;
		double dwy_dy = (a11 - a21*w_y) * D_inv;
		dw_dX <<
			dwx_dx, dwx_dy,
			dwy_dx, dwy_dy;

		double d2wx_dx2 = -2 * a20*dwx_dx*D_inv;
		double d2wx_dxdy = -(a21*dwx_dx + a20*dwx_dy)*D_inv;
		double d2wx_dydx = d2wx_dxdy;
		double d2wx_dy2 = -2 * a21*dwx_dy*D_inv;
		d2wx_dX2 <<
			d2wx_dx2, d2wx_dxdy,
			d2wx_dydx, d2wx_dy2;

		double d2wy_dx2 = -2 * a20*dwy_dx*D_inv;
		double d2wy_dxdy = -(a21*dwy_dx + a20*dwy_dy)*D_inv;
		double d2wy_dydx = d2wy_dxdy;
		double d2wy_dy2 = -2 * a21*dwy_dy*D_inv;
		d2wy_dX2 << d2wy_dx2, d2wy_dxdy, d2wy_dydx, d2wy_dy2;

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		Matrix28d dw_dp;
		dw_dp <<
			x, y, 1, 0, 0, 0, -x*x, -y*x,
			0, 0, 0, x, y, 1, -x*y, -y*y;

		for(int ch_id = 0; ch_id < n_channels; ++ch_id){

			Map<Matrix8d> d2I_dp2(_d2I_dp2.col(ch_pt_id).data());

			d2I_dp2 = dw_dp.transpose()*(
				dw_dX.transpose() * Map<const Matrix2d>(d2I_dw2.col(ch_pt_id).data()) * dw_dX
				+
				dI_dw(ch_pt_id, 0)*d2wx_dX2 + dI_dw(ch_pt_id, 1)*d2wy_dX2
				)*dw_dp;

			double Ix = dwx_dx*dI_dw(ch_pt_id, 0) + dwy_dx*dI_dw(ch_pt_id, 1);
			double Iy = dwx_dy*dI_dw(ch_pt_id, 0) + dwy_dy*dI_dw(ch_pt_id, 1);

			double Ix_x = Ix * x;
			double Ix_y = Ix * y;
			double Iy_y = Iy * y;
			double Iy_x = Iy * x;

			double Ix_x2 = Ix_x * x;
			double Ix_xy = Ix_x * y;
			double Ix_y2 = Ix_y * y;

			double Iy_y2 = Iy_y * y;
			double Iy_yx = Iy_y * x;
			double Iy_x2 = Iy_x * x;

			d2I_dp2(0, 6) -= Ix_x2; 
			d2I_dp2(1, 6) -= Ix_xy; 
			d2I_dp2(2, 6) -= Ix_x; 
			d2I_dp2(3, 6) -= Iy_x2; 
			d2I_dp2(4, 6) -= Iy_yx; 
			d2I_dp2(5, 6) -= Iy_x; 

			d2I_dp2(0, 7) -= Ix_xy; 
			d2I_dp2(1, 7) -= Ix_y2; 
			d2I_dp2(2, 7) -= Ix_y; 
			d2I_dp2(3, 7) -= Iy_yx; 
			d2I_dp2(4, 7) -= Iy_y2; 
			d2I_dp2(5, 7) -= Iy_y; 

			d2I_dp2(6, 6) += 2 * (Ix_x2*x + Iy_x2*y);
			d2I_dp2(6, 7) += 2 * (Ix_xy*x + Iy_yx*y);
			d2I_dp2(7, 6) += 2 * (Ix_xy*x + Iy_yx*y);
			d2I_dp2(7, 7) += 2 * (Ix_y2*x + Iy_y2*y);

			d2I_dp2.bottomLeftCorner<2, 5>() = d2I_dp2.topRightCorner<5, 2>().transpose();

			++ch_pt_id;
		}
	}
}

void Homography::cmptWarpedPixHessian2(MatrixXd &d2I_dp2, const PixHessT &d2I_dw2,
	const PixGradT &dI_dw) {
	validate_ssm_hessian(d2I_dp2, d2I_dw2, dI_dw);

	double h00_plus_1 = curr_warp(0, 0);
	double h01 = curr_warp(0, 1);
	double h10 = curr_warp(1, 0);
	double h11_plus_1 = curr_warp(1, 1);
	double h20 = curr_warp(2, 0);
	double h21 = curr_warp(2, 1);

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; ++pt_id) {
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double curr_x = curr_pts(0, pt_id);
		double curr_y = curr_pts(1, pt_id);
		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		double D = curr_pts_hm(2, pt_id);
		double inv_det = 1.0 / D;

		double a = (h00_plus_1 - h20*curr_x) * inv_det;
		double b = (h01 - h21*curr_x) * inv_det;
		double c = (h10 - h20*curr_y) * inv_det;
		double d = (h11_plus_1 - h21*curr_y) * inv_det;

		Matrix2d dw_dx;
		dw_dx <<
			a, b,
			c, d;

		Matrix28d dw_dp;
		dw_dp <<
			x, y, 1, 0, 0, 0, -x*x, -y*x,
			0, 0, 0, x, y, 1, -x*y, -y*y;

		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = dI_dw(ch_pt_id, 0);
			double Iy = dI_dw(ch_pt_id, 1);

			double Ixx = Ix * x;
			double Ixy = Ix * y;
			double Iyy = Iy * y;
			double Iyx = Iy * x;

			double Ixxx = Ixx * x;
			double Ixxy = Ixx * y;
			double Ixyy = Ixy * y;

			double Iyyy = Iyy * y;
			double Iyyx = Iyy * x;
			double Iyxx = Iyx * x;

			Matrix8d term1, term2;
			term1 = dw_dp.transpose()*Map<const Matrix2d>(d2I_dw2.col(ch_pt_id).data()).transpose()*dw_dx*dw_dp;

			double k = -Ixx - Iyy;
			double kx = k*x, ky = k*y;
			double kxx = kx*x, kxy = kx*y, kyy = ky*y;
			term2 <<
				Ixx, Ixy, Ix, 0, 0, 0, -Ixxx, -Ixxy,
				0, 0, 0, Ixx, Ixy, Ix, -Ixxy, -Ixyy,
				0, 0, 0, 0, 0, 0, 0, 0,
				Iyx, Iyy, Iy, 0, 0, 0, -Iyxx, -Iyyx,
				0, 0, 0, Iyx, Iyy, Iy, -Iyyx, -Iyyy,
				0, 0, 0, 0, 0, 0, 0, 0,
				kx, ky, k, 0, 0, 0, -kxx, -kxy,
				0, 0, 0, kx, ky, k, -kxy, -kyy;
			Map<Matrix8d>(d2I_dp2.col(ch_pt_id).data()) = term1 + term2;

			++ch_pt_id;
		}
	}
}

void Homography::cmptApproxPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
	const PixGradT &pix_grad) {
	validate_ssm_hessian(pix_hess_ssm, pix_hess_coord, pix_grad);

	double h00_plus_1 = curr_warp(0, 0);
	double h01 = curr_warp(0, 1);
	double h10 = curr_warp(1, 0);
	double h11_plus_1 = curr_warp(1, 1);
	double h20 = curr_warp(2, 0);
	double h21 = curr_warp(2, 1);
	
	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; ++pt_id) {
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double curr_x = curr_pts(0, pt_id);
		double curr_y = curr_pts(1, pt_id);
		double D = curr_pts_hm(2, pt_id);
		double inv_det2 = 1.0 / (D*D);
		double inv_det = 1.0 / D;


		double a = (h00_plus_1 - h20*curr_x) * inv_det;
		double b = (h01 - h21*curr_x) * inv_det;
		double c = (h10 - h20*curr_y) * inv_det;
		double d = (h11_plus_1 - h21*curr_y) * inv_det;
		double inv_factor = 1.0 / (a*d - b*c);

		Matrix2d dw_dx_inv;
		dw_dx_inv << d, -b, -c, a;
		dw_dx_inv *= inv_factor;

		Matrix2d d2w_dx2_x, d2w_dx2_y;

		double ax = -h20*(h00_plus_1 + a*D - h20*curr_x)*inv_det2;
		double bx = -(h20*h01 + h21*(a*D - h20*curr_x))*inv_det2;
		double cx = -h20*(h10 + c*D - h20*curr_y)*inv_det2;
		double dx = -(h20*h11_plus_1 + h21*(c*D - h20*curr_y))*inv_det2;
		d2w_dx2_x << ax, bx, cx, dx;

		double ay = -(h21*h00_plus_1 + h20*(b*D - h21*curr_x))*inv_det2;
		double by = -h21*(h01 + b*D - h21*curr_x)*inv_det2;
		double cy = -(h21*h10 + h20*(d*D - h21*curr_y))*inv_det2;
		double dy = -h21*(h11_plus_1 + d*D - h21*curr_y)*inv_det2;
		d2w_dx2_y << ay, by, cy, dy;

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		Matrix28d dw_dp;
		dw_dp <<
			x, y, 1, 0, 0, 0, -x*x, -y*x,
			0, 0, 0, x, y, 1, -x*y, -y*y;

		for(int ch_id = 0; ch_id < n_channels; ++ch_id){

			Map<Matrix8d> d2I_dp2(pix_hess_ssm.col(ch_pt_id).data());

			double Ix = (d*pix_grad(ch_pt_id, 0) - c*pix_grad(ch_pt_id, 1))*inv_factor;
			double Iy = (a*pix_grad(ch_pt_id, 1) - b*pix_grad(ch_pt_id, 0))*inv_factor;

			d2I_dp2 = dw_dp.transpose()*(
				dw_dx_inv.transpose() * (
				Map<const Matrix2d>(pix_hess_coord.col(ch_pt_id).data()) -
				(Ix*d2w_dx2_x + Iy*d2w_dx2_y)
				) * dw_dx_inv
				)*dw_dp;

			double Ixx = Ix * x;
			double Ixy = Ix * y;
			double Iyy = Iy * y;
			double Iyx = Iy * x;

			double Ixxx = Ixx * x;
			double Ixxy = Ixx * y;
			double Ixyy = Ixy * y;

			double Iyyy = Iyy * y;
			double Iyyx = Iyy * x;
			double Iyxx = Iyx * x;

			d2I_dp2(0, 6) += Ixxx; 
			d2I_dp2(0, 7) += Ixxy; 
			d2I_dp2(1, 6) += Ixxy;
			d2I_dp2(1, 7) += Ixyy;
			d2I_dp2(2, 6) += Ixx;
			d2I_dp2(2, 7) += Ixy;

			d2I_dp2(3, 6) += Iyxx;
			d2I_dp2(3, 7) += Iyyx;
			d2I_dp2(4, 6) += Iyyx;
			d2I_dp2(4, 7) += Iyyy;
			d2I_dp2(5, 6) += Iyx;
			d2I_dp2(5, 7) += Iyy;

			d2I_dp2(6, 6) -= Ixxx*x + Iyxx*y;
			d2I_dp2(6, 7) -= Ixxy*x + Iyyx*y;
			d2I_dp2(7, 6) -= Ixxy*x + Iyyx*y;
			d2I_dp2(7, 7) -= Ixyy*x + Iyyy*y;

			d2I_dp2.bottomLeftCorner<2, 5>() = d2I_dp2.topRightCorner<5, 2>().transpose();

			++ch_pt_id;
		}
	}
}

void Homography::updateGradPts(double grad_eps){
	Vector3d diff_vec_x_warped = curr_warp.col(0) * grad_eps;
	Vector3d diff_vec_y_warped = curr_warp.col(1) * grad_eps;

	Vector3d pt_inc_warped, pt_dec_warped;
	for(int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check(spi_mask, pt_id);

		pt_inc_warped = curr_pts_hm.col(pt_id) + diff_vec_x_warped;
		grad_pts(0, pt_id) = pt_inc_warped(0) / pt_inc_warped(2);
		grad_pts(1, pt_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pt_id) - diff_vec_x_warped;
		grad_pts(2, pt_id) = pt_dec_warped(0) / pt_dec_warped(2);
		grad_pts(3, pt_id) = pt_dec_warped(1) / pt_dec_warped(2);

		pt_inc_warped = curr_pts_hm.col(pt_id) + diff_vec_y_warped;
		grad_pts(4, pt_id) = pt_inc_warped(0) / pt_inc_warped(2);
		grad_pts(5, pt_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pt_id) - diff_vec_y_warped;
		grad_pts(6, pt_id) = pt_dec_warped(0) / pt_dec_warped(2);
		grad_pts(7, pt_id) = pt_dec_warped(1) / pt_dec_warped(2);
	}
}

void Homography::updateHessPts(double hess_eps){
	double hess_eps2 = 2 * hess_eps;

	Vector3d diff_vec_xx_warped = curr_warp.col(0) * hess_eps2;
	Vector3d diff_vec_yy_warped = curr_warp.col(1) * hess_eps2;
	Vector3d diff_vec_xy_warped = (curr_warp.col(0) + curr_warp.col(1)) * hess_eps;
	Vector3d diff_vec_yx_warped = (curr_warp.col(0) - curr_warp.col(1)) * hess_eps;

	Vector3d pt_inc_warped, pt_dec_warped;

	for(int pt_id = 0; pt_id < n_pts; ++pt_id){

		spi_pt_check(spi_mask, pt_id);

		pt_inc_warped = curr_pts_hm.col(pt_id) + diff_vec_xx_warped;
		hess_pts(0, pt_id) = pt_inc_warped(0) / pt_inc_warped(2);
		hess_pts(1, pt_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pt_id) - diff_vec_xx_warped;
		hess_pts(2, pt_id) = pt_dec_warped(0) / pt_dec_warped(2);
		hess_pts(3, pt_id) = pt_dec_warped(1) / pt_dec_warped(2);

		pt_inc_warped = curr_pts_hm.col(pt_id) + diff_vec_yy_warped;
		hess_pts(4, pt_id) = pt_inc_warped(0) / pt_inc_warped(2);
		hess_pts(5, pt_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pt_id) - diff_vec_yy_warped;
		hess_pts(6, pt_id) = pt_dec_warped(0) / pt_dec_warped(2);
		hess_pts(7, pt_id) = pt_dec_warped(1) / pt_dec_warped(2);

		pt_inc_warped = curr_pts_hm.col(pt_id) + diff_vec_xy_warped;
		hess_pts(8, pt_id) = pt_inc_warped(0) / pt_inc_warped(2);
		hess_pts(9, pt_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pt_id) - diff_vec_xy_warped;
		hess_pts(10, pt_id) = pt_dec_warped(0) / pt_dec_warped(2);
		hess_pts(11, pt_id) = pt_dec_warped(1) / pt_dec_warped(2);

		pt_inc_warped = curr_pts_hm.col(pt_id) + diff_vec_yx_warped;
		hess_pts(12, pt_id) = pt_inc_warped(0) / pt_inc_warped(2);
		hess_pts(13, pt_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pt_id) - diff_vec_yx_warped;
		hess_pts(14, pt_id) = pt_dec_warped(0) / pt_dec_warped(2);
		hess_pts(15, pt_id) = pt_dec_warped(1) / pt_dec_warped(2);
	}
}

void Homography::estimateWarpFromCorners(VectorXd &state_update, const Matrix24d &in_corners,
	const Matrix24d &out_corners){
	validate_ssm_state(state_update);
	warp_update_mat = utils::computeHomographyDLT(in_corners, out_corners);
	warp_update_mat /= warp_update_mat(2, 2);
	getStateFromWarp(state_update, warp_update_mat);
}

void Homography::estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
	const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
	const SSMEstimatorParams &est_params){
	cv::Mat warp_mat_cv = estimateHomography(in_pts, out_pts, mask, est_params);
	state_update(0) = warp_mat_cv.at<double>(0, 0) - 1;
	state_update(1) = warp_mat_cv.at<double>(0, 1);
	state_update(2) = warp_mat_cv.at<double>(0, 2);
	state_update(3) = warp_mat_cv.at<double>(1, 0);
	state_update(4) = warp_mat_cv.at<double>(1, 1) - 1;
	state_update(5) = warp_mat_cv.at<double>(1, 2);
	state_update(6) = warp_mat_cv.at<double>(2, 0);
	state_update(7) = warp_mat_cv.at<double>(2, 1);
}

void Homography::generatePerturbation(VectorXd &state_update){
	if(params.corner_based_sampling){
		rand_t(0) = rand_dist[0](rand_gen[0]);
		rand_t(1) = rand_dist[0](rand_gen[0]);
		for(int corner_id = 0; corner_id < 4; corner_id++){
			rand_d(0, corner_id) = rand_dist[1](rand_gen[1]);
			rand_d(1, corner_id) = rand_dist[1](rand_gen[1]);
		}
		disturbed_corners = init_corners + rand_d;
		disturbed_corners = disturbed_corners.colwise() + rand_t;
		estimateWarpFromCorners(state_update, init_corners, disturbed_corners);
	} else{
		for(int state_id = 0; state_id < 8; state_id++){
			state_update(state_id) = rand_dist[state_id](rand_gen[state_id]);
		}
	}
}

void Homography::compositionalRandomWalk(VectorXd &perturbed_state,
	const VectorXd &base_state){
	generatePerturbation(state_perturbation);
	ProjWarpT base_warp, warp_perturbation;
	getWarpFromState(base_warp, base_state);
	getWarpFromState(warp_perturbation, state_perturbation);
	ProjWarpT perturbed_warp = base_warp * warp_perturbation;
	perturbed_warp /= perturbed_warp(2, 2);
	getStateFromWarp(perturbed_state, perturbed_warp);
}

void Homography::compositionalAutoRegression1(VectorXd &perturbed_state, VectorXd &perturbed_ar,
	const VectorXd &base_state, const VectorXd &base_ar, double a){
	generatePerturbation(state_perturbation);
	ProjWarpT base_warp, warp_perturbation, base_ar_warp;
	getWarpFromState(base_warp, base_state);
	getWarpFromState(warp_perturbation, state_perturbation);
	getWarpFromState(base_ar_warp, base_ar);
	ProjWarpT perturbed_warp = base_warp * base_ar_warp * warp_perturbation;
	perturbed_warp /= perturbed_warp(2, 2);
	ProjWarpT perturbed_ar_warp = base_warp.inverse() * perturbed_warp;
	perturbed_ar_warp /= perturbed_ar_warp(2, 2);
	getStateFromWarp(perturbed_state, perturbed_warp);
	getStateFromWarp(perturbed_ar, perturbed_ar_warp);
	perturbed_ar *= a;
}



_MTF_END_NAMESPACE

