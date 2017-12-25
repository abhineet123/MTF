#include "mtf/SSM/Affine.h"
#include "mtf/SSM/AffineEstimator.h"
#include "mtf/Utilities/warpUtils.h"
#include "mtf/Utilities/miscUtils.h"

#include <Eigen/SVD>

#define VALIDATE_AFFINE_WARP(warp)\
	assert(warp(2, 0) == 0.0 && warp(2, 1) == 0.0);\
	assert(warp(2, 2) == 1.0)

#define AFF_NORMALIZED_INIT 0
#define AFF_PT_BASED_SAMPLING 0
#define AFF_DEBUG_MODE 0

_MTF_BEGIN_NAMESPACE

AffineParams::AffineParams(const SSMParams *ssm_params,
bool _normalized_init, int _pt_based_sampling,
bool _debug_mode) :
SSMParams(ssm_params),
normalized_init(_normalized_init),
pt_based_sampling(_pt_based_sampling),
debug_mode(_debug_mode){}

AffineParams::AffineParams(const AffineParams *params) :
SSMParams(params),
normalized_init(AFF_NORMALIZED_INIT),
pt_based_sampling(AFF_PT_BASED_SAMPLING),
debug_mode(AFF_DEBUG_MODE){
	if(params){
		normalized_init = params->normalized_init;
		pt_based_sampling = params->pt_based_sampling;
		debug_mode = params->debug_mode;
	}
}
Affine::Affine(
	const ParamType *_params) : ProjectiveBase(_params),
	params(_params){

	printf("\n");
	printf("Using Affine SSM with:\n");
	printf("resx: %d\n", resx);
	printf("resy: %d\n", resy);
	printf("normalized_init: %d\n", params.normalized_init);
	printf("pt_based_sampling: %d\n", params.pt_based_sampling);
	printf("debug_mode: %d\n", params.debug_mode);

	name = "affine";
	state_size = 6;
	curr_state.resize(state_size);

	utils::getNormUnitSquarePts(norm_pts, norm_corners, resx, resy,
		1 - resx / 2.0, 1 - resy / 2.0, resx / 2.0, resy / 2.0);
	utils::homogenize(norm_pts, norm_pts_hm);
	utils::homogenize(norm_corners, norm_corners_hm);

	init_corners = getNormCorners();
	init_corners_hm = getHomNormCorners();
	init_pts = getNormPts();
	init_pts_hm = getHomNormPts();
}

void Affine::setCorners(const CornersT& corners){
	if(params.normalized_init){
		curr_warp = utils::computeAffineNDLT(init_corners, corners);
		getStateFromWarp(curr_state, curr_warp);

		curr_pts.noalias() = curr_warp.topRows<2>() * init_pts_hm;
		curr_corners.noalias() = curr_warp.topRows<2>() * init_corners_hm;

		utils::homogenize(curr_pts, curr_pts_hm);
		utils::homogenize(curr_corners, curr_corners_hm);
	} else {
		curr_corners = corners;
		utils::homogenize(curr_corners, curr_corners_hm);

		getPtsFromCorners(curr_warp, curr_pts, curr_pts_hm, curr_corners);

		init_corners = curr_corners;
		init_pts = curr_pts;
		utils::homogenize(init_corners, init_corners_hm);
		utils::homogenize(init_pts, init_pts_hm);

		curr_warp = Matrix3d::Identity();
		curr_state.fill(0);
	}
}

void Affine::compositionalUpdate(const VectorXd& state_update){
	validate_ssm_state(state_update);

	getWarpFromState(warp_update_mat, state_update);
	curr_warp = curr_warp * warp_update_mat;
	getStateFromWarp(curr_state, curr_warp);

	//curr_pts_hm.noalias() = curr_warp * init_pts_hm;
	//curr_corners_hm.noalias() = curr_warp * init_corners_hm;
	//utils::dehomogenize(curr_pts_hm, curr_pts);
	//utils::dehomogenize(curr_corners_hm, curr_corners);

	curr_pts.noalias() = curr_warp.topRows<2>() * init_pts_hm;
	curr_corners.noalias() = curr_warp.topRows<2>() * init_corners_hm;

	//utils::printMatrix(curr_warp, "curr_warp", "%15.9f");
	//utils::printMatrix(affine_warp_mat, "affine_warp_mat", "%15.9f");
}

void Affine::setState(const VectorXd &ssm_state){
	validate_ssm_state(ssm_state);
	curr_state = ssm_state;
	getWarpFromState(curr_warp, curr_state);
	curr_pts.noalias() = curr_warp.topRows<2>() * init_pts_hm;
	curr_corners.noalias() = curr_warp.topRows<2>() * init_corners_hm;
}

void Affine::getWarpFromState(Matrix3d &warp_mat,
	const VectorXd& ssm_state){
	validate_ssm_state(ssm_state);

	warp_mat(0, 0) = 1 + ssm_state(2);
	warp_mat(0, 1) = ssm_state(3);
	warp_mat(0, 2) = ssm_state(0);
	warp_mat(1, 0) = ssm_state(4);
	warp_mat(1, 1) = 1 + ssm_state(5);
	warp_mat(1, 2) = ssm_state(1);
	warp_mat(2, 0) = 0;
	warp_mat(2, 1) = 0;
	warp_mat(2, 2) = 1;
}

void Affine::getStateFromWarp(VectorXd &state_vec,
	const Matrix3d& warp_mat){
	validate_ssm_state(state_vec);
	VALIDATE_AFFINE_WARP(warp_mat);

	state_vec(0) = warp_mat(0, 2);
	state_vec(1) = warp_mat(1, 2);
	state_vec(2) = warp_mat(0, 0) - 1;
	state_vec(3) = warp_mat(0, 1);
	state_vec(4) = warp_mat(1, 0);
	state_vec(5) = warp_mat(1, 1) - 1;
}

void Affine::invertState(VectorXd& inv_state, const VectorXd& state){
	getWarpFromState(warp_mat, state);
	inv_warp_mat = warp_mat.inverse();
	inv_warp_mat /= inv_warp_mat(2, 2);
	getStateFromWarp(inv_state, inv_warp_mat);
}

void Affine::getInitPixGrad(Matrix2Xd &ssm_grad, int pix_id) {
	double x = init_pts(0, pix_id);
	double y = init_pts(1, pix_id);
	ssm_grad <<
		1, 0, x, y, 0, 0,
		0, 1, 0, 0, x, y;
}

void Affine::cmptInitPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dx){
	validate_ssm_jacobian(dI_dp, dI_dx);
	unsigned int ch_pt_id = 0;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		for(unsigned int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = dI_dx(ch_pt_id, 0);
			double Iy = dI_dx(ch_pt_id, 1);
			dI_dp(ch_pt_id, 0) = Ix;
			dI_dp(ch_pt_id, 1) = Iy;
			dI_dp(ch_pt_id, 2) = Ix * x;
			dI_dp(ch_pt_id, 3) = Ix * y;
			dI_dp(ch_pt_id, 4) = Iy * x;
			dI_dp(ch_pt_id, 5) = Iy * y;
			++ch_pt_id;
		}
	}
}

void Affine::cmptApproxPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dx) {
	validate_ssm_jacobian(dI_dp, dI_dx);
	double a = curr_state(2) + 1, b = curr_state(3);
	double c = curr_state(4), d = curr_state(5) + 1;
	double inv_det = 1.0 / (a*d - b*c);
	unsigned int ch_pt_id = 0;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		for(unsigned int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = dI_dx(ch_pt_id, 0);
			double Iy = dI_dx(ch_pt_id, 1);
			double Ixx = Ix * x;
			double Ixy = Ix * y;
			double Iyy = Iy * y;
			double Iyx = Iy * x;
			dI_dp(ch_pt_id, 0) = (Ix*d - Iy*c) * inv_det;
			dI_dp(ch_pt_id, 1) = (Iy*a - Ix*b) * inv_det;
			dI_dp(ch_pt_id, 2) = (Ixx*d - Iyx*c) * inv_det;
			dI_dp(ch_pt_id, 3) = (Ixy*d - Iyy*c) * inv_det;
			dI_dp(ch_pt_id, 4) = (Iyx*a - Ixx*b) * inv_det;
			dI_dp(ch_pt_id, 5) = (Iyy*a - Ixy*b) * inv_det;
			++ch_pt_id;
		}
	}
}

void Affine::cmptWarpedPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dx) {
	validate_ssm_jacobian(dI_dp, dI_dx);
	double a = curr_state(2) + 1, b = curr_state(3);
	double c = curr_state(4), d = curr_state(5) + 1;

	unsigned int ch_pt_id = 0;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		for(unsigned int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = dI_dx(ch_pt_id, 0);
			double Iy = dI_dx(ch_pt_id, 1);
			double Ixx = Ix * x;
			double Ixy = Ix * y;
			double Iyy = Iy * y;
			double Iyx = Iy * x;

			dI_dp(ch_pt_id, 0) = Ix*a + Iy*c;
			dI_dp(ch_pt_id, 1) = Ix*b + Iy*d;
			dI_dp(ch_pt_id, 2) = Ixx*a + Iyx*c;
			dI_dp(ch_pt_id, 3) = Ixy*a + Iyy*c;
			dI_dp(ch_pt_id, 4) = Ixx*b + Iyx*d;
			dI_dp(ch_pt_id, 5) = Ixy*b + Iyy*d;
			++ch_pt_id;
		}
	}
}
void Affine::cmptInitPixHessian(MatrixXd &d2I_dp2, const PixHessT &d2I_dw2,
	const PixGradT &dI_dw){
	validate_ssm_hessian(d2I_dp2, d2I_dw2, dI_dw);

	unsigned int ch_pt_id = 0;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		Matrix26d dw_dp;
		dw_dp <<
			1, 0, x, y, 0, 0,
			0, 1, 0, 0, x, y;
		for(unsigned int ch_id = 0; ch_id < n_channels; ++ch_id){
			Map<Matrix6d>(d2I_dp2.col(ch_pt_id).data()) = dw_dp.transpose()*
				Map<const Matrix2d>(d2I_dw2.col(ch_pt_id).data())*dw_dp;
			++ch_pt_id;
		}
	}
}
void Affine::cmptWarpedPixHessian(MatrixXd &d2I_dp2, const PixHessT &d2I_dw2,
	const PixGradT &dI_dw) {
	validate_ssm_hessian(d2I_dp2, d2I_dw2, dI_dw);
	double a2 = curr_state(2) + 1, a3 = curr_state(3);
	double a4 = curr_state(4), a5 = curr_state(5) + 1;
	Matrix2d dw_dx;
	dw_dx <<
		a2, a3,
		a4, a5;

	unsigned int ch_pt_id = 0;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id) {
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		Matrix26d dw_dp;
		dw_dp <<
			1, 0, x, y, 0, 0,
			0, 1, 0, 0, x, y;

		for(unsigned int ch_id = 0; ch_id < n_channels; ++ch_id){
			Map<Matrix6d>(d2I_dp2.col(ch_pt_id).data()) = dw_dp.transpose()*
				dw_dx.transpose()*Map<const Matrix2d>(d2I_dw2.col(ch_pt_id).data())*dw_dx*dw_dp;
			++ch_pt_id;
		}
	}
}
void Affine::updateGradPts(double grad_eps){
	Vector2d diff_vec_x_warped = curr_warp.topRows<2>().col(0) * grad_eps;
	Vector2d diff_vec_y_warped = curr_warp.topRows<2>().col(1) * grad_eps;

	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check(spi_mask, pt_id);

		grad_pts(0, pt_id) = curr_pts(0, pt_id) + diff_vec_x_warped(0);
		grad_pts(1, pt_id) = curr_pts(1, pt_id) + diff_vec_x_warped(1);

		grad_pts(2, pt_id) = curr_pts(0, pt_id) - diff_vec_x_warped(0);
		grad_pts(3, pt_id) = curr_pts(1, pt_id) - diff_vec_x_warped(1);

		grad_pts(4, pt_id) = curr_pts(0, pt_id) + diff_vec_y_warped(0);
		grad_pts(5, pt_id) = curr_pts(1, pt_id) + diff_vec_y_warped(1);

		grad_pts(6, pt_id) = curr_pts(0, pt_id) - diff_vec_y_warped(0);
		grad_pts(7, pt_id) = curr_pts(1, pt_id) - diff_vec_y_warped(1);
	}
}


void Affine::updateHessPts(double hess_eps){
	double hess_eps2 = 2 * hess_eps;

	Vector2d diff_vec_xx_warped = curr_warp.topRows<2>().col(0) * hess_eps2;
	Vector2d diff_vec_yy_warped = curr_warp.topRows<2>().col(1) * hess_eps2;
	Vector2d diff_vec_xy_warped = (curr_warp.topRows<2>().col(0) + curr_warp.topRows<2>().col(1)) * hess_eps;
	Vector2d diff_vec_yx_warped = (curr_warp.topRows<2>().col(0) - curr_warp.topRows<2>().col(1)) * hess_eps;

	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check(spi_mask, pt_id);

		hess_pts(0, pt_id) = curr_pts(0, pt_id) + diff_vec_xx_warped(0);
		hess_pts(1, pt_id) = curr_pts(1, pt_id) + diff_vec_xx_warped(1);

		hess_pts(2, pt_id) = curr_pts(0, pt_id) - diff_vec_xx_warped(0);
		hess_pts(3, pt_id) = curr_pts(1, pt_id) - diff_vec_xx_warped(1);

		hess_pts(4, pt_id) = curr_pts(0, pt_id) + diff_vec_yy_warped(0);
		hess_pts(5, pt_id) = curr_pts(1, pt_id) + diff_vec_yy_warped(1);

		hess_pts(6, pt_id) = curr_pts(0, pt_id) - diff_vec_yy_warped(0);
		hess_pts(7, pt_id) = curr_pts(1, pt_id) - diff_vec_yy_warped(1);

		hess_pts(8, pt_id) = curr_pts(0, pt_id) + diff_vec_xy_warped(0);
		hess_pts(9, pt_id) = curr_pts(1, pt_id) + diff_vec_xy_warped(1);

		hess_pts(10, pt_id) = curr_pts(0, pt_id) - diff_vec_xy_warped(0);
		hess_pts(11, pt_id) = curr_pts(1, pt_id) - diff_vec_xy_warped(1);

		hess_pts(12, pt_id) = curr_pts(0, pt_id) + diff_vec_yx_warped(0);
		hess_pts(13, pt_id) = curr_pts(1, pt_id) + diff_vec_yx_warped(1);

		hess_pts(14, pt_id) = curr_pts(0, pt_id) - diff_vec_yx_warped(0);
		hess_pts(15, pt_id) = curr_pts(1, pt_id) - diff_vec_yx_warped(1);
	}
}

void Affine::estimateWarpFromCorners(VectorXd &state_update, const Matrix24d &in_corners,
	const Matrix24d &out_corners){
	validate_ssm_state(state_update);
	Matrix3d warp_update_mat = utils::computeAffineDLT(in_corners, out_corners);
	getStateFromWarp(state_update, warp_update_mat);
}

void Affine::estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
	const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
	const EstimatorParams &est_params){
	cv::Mat warp_mat_cv = estimateAffine(in_pts, out_pts, mask, est_params);
	state_update(0) = warp_mat_cv.at<double>(0, 2);
	state_update(1) = warp_mat_cv.at<double>(1, 2);
	state_update(2) = warp_mat_cv.at<double>(0, 0) - 1;
	state_update(3) = warp_mat_cv.at<double>(0, 1);
	state_update(4) = warp_mat_cv.at<double>(1, 0);
	state_update(5) = warp_mat_cv.at<double>(1, 1) - 1;
}

void Affine::applyWarpToCorners(Matrix24d &warped_corners, const Matrix24d &orig_corners,
	const VectorXd &ssm_state){
	getWarpFromState(warp_mat, ssm_state);
	for(unsigned int corner_id = 0; corner_id < 4; corner_id++){
		warped_corners(0, corner_id) = warp_mat(0, 0)*orig_corners(0, corner_id) + warp_mat(0, 1)*orig_corners(1, corner_id) +
			warp_mat(0, 2);
		warped_corners(1, corner_id) = warp_mat(1, 0)*orig_corners(0, corner_id) + warp_mat(1, 1)*orig_corners(1, corner_id) +
			warp_mat(1, 2);
	}
}

void Affine::applyWarpToPts(Matrix2Xd &warped_pts, const Matrix2Xd &orig_pts,
	const VectorXd &ssm_state){
	getWarpFromState(warp_mat, ssm_state);
	unsigned int n_pts = orig_pts.cols();
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
		warped_pts(0, pt_id) = warp_mat(0, 0)*orig_pts(0, pt_id) + warp_mat(0, 1)*orig_pts(1, pt_id) +
			warp_mat(0, 2);
		warped_pts(1, pt_id) = warp_mat(1, 0)*orig_pts(0, pt_id) + warp_mat(1, 1)*orig_pts(1, pt_id) +
			warp_mat(1, 2);
	}
}
Vector6d Affine::geomToState(const Vector6d &geom){
	double s = geom[2], r = geom[4];
	double theta = geom[3], phi = geom[5];
	double cos_theta = cos(theta), sin_theta = sin(theta);
	double cos_phi = cos(phi), sin_phi = sin(phi);
	double ccc = cos_theta*cos_phi*cos_phi;
	double ccs = cos_theta*cos_phi*sin_phi;
	double css = cos_theta*sin_phi*sin_phi;
	double scc = sin_theta*cos_phi*cos_phi;
	double scs = sin_theta*cos_phi*sin_phi;
	double sss = sin_theta*sin_phi*sin_phi;
	Vector6d state;
	state[0] = geom[0];
	state[1] = geom[1];
	state[2] = s*(ccc + scs + r*(css - scs)) - 1;
	state[3] = s*(r*(ccs - scc) - ccs - sss);
	state[4] = s*(scc - ccs + r*(ccs + sss));
	state[5] = s*(r*(ccc + scs) - scs + css) - 1;
	return state;
}
Vector6d Affine::stateToGeom(const Vector6d &est){
	Matrix2d A;
	A << est[2] + 1, est[3], est[4], est[5] + 1;
	JacobiSVD<Matrix2d> svd(A, ComputeFullU | ComputeFullV);
	Vector2d singular_vals = svd.singularValues();
	Matrix2d S = singular_vals.asDiagonal();
	Matrix2d V = svd.matrixV().transpose();
	Matrix2d U = svd.matrixU();
	if(U.determinant() < 0){
		Matrix2d U_temp;
		U_temp << U(0, 1), U(0, 0), U(1, 1), U(1, 0);
		U = U_temp;
		Matrix2d V_temp;
		V_temp << V(0, 1), V(0, 0), V(1, 1), V(1, 0);
		V = V_temp;
		Matrix2d S_temp;
		S_temp << S(1, 1), S(1, 0), S(0, 1), S(0, 0);
		S = S_temp;
	}
	Vector6d q;
	q[0] = est[0];
	q[1] = est[1];
	q[3] = atan2(U(1, 0) * V(0, 0) + U(1, 1) * V(0, 1),
		U(0, 0) * V(0, 0) + U(0, 1) * V(0, 1));

	double phi = atan2(V(0, 1), V(0, 0));
	const double pi = 3.14159265358979323846;
	if(phi <= -pi / 2){
		double cos_phi = cos(-pi / 2);
		double sin_phi = sin(-pi / 2);
		Matrix2d R;
		R << cos_phi, -sin_phi, sin_phi, cos_phi;
		V = V * R;
		S = R.transpose()*S*R;
	}

	if(phi >= pi / 2){
		double cos_phi = cos(pi / 2);
		double sin_phi = sin(pi / 2);
		Matrix2d R;
		R << cos_phi, -sin_phi, sin_phi, cos_phi;
		V = V * R;
		S = R.transpose()*S*R;
	}
	q[2] = S(0, 0);
	q[4] = S(1, 1) / S(0, 0);
	q[5] = atan2(V(0, 1), V(0, 0));
	return q;
}


void Affine::generatePerturbation(VectorXd &perturbation){
	assert(perturbation.size() == state_size);
	if(params.pt_based_sampling){
		//! perturb three canonical points and estimate affine transformation using DLT
		Matrix23d orig_pts, perturbed_pts;
		//! use the bottom left, bottom right and top center points
		//! as canaonical points to add the random perturbations to;
		orig_pts.col(0) = init_corners.col(2);
		orig_pts.col(1) = init_corners.col(3);
		orig_pts.col(2) = (init_corners.col(0) + init_corners.col(1)) / 2.0;

		if(params.pt_based_sampling == 1){
			perturbed_pts = orig_pts;
			perturbed_pts(0, 0) += rand_dist[0](rand_gen[0]);
			perturbed_pts(1, 0) += rand_dist[1](rand_gen[1]);
			perturbed_pts(0, 1) += rand_dist[2](rand_gen[2]);
			perturbed_pts(1, 1) += rand_dist[3](rand_gen[3]);
			perturbed_pts(0, 2) += rand_dist[4](rand_gen[4]);
			perturbed_pts(1, 2) += rand_dist[5](rand_gen[5]);
		} else {
			//! different perturbation for x,y coordinates of each point
			//! followed by consistent translational perturbation to all corners
			Matrix23d rand_d;
			for(unsigned int pt_id = 0; pt_id < 3; ++pt_id){
				rand_d(0, pt_id) = rand_dist[1](rand_gen[1]);
				rand_d(1, pt_id) = rand_dist[1](rand_gen[1]);
			}
			perturbed_pts = (orig_pts + rand_d).colwise() + Vector2d(rand_dist[0](rand_gen[0]), rand_dist[0](rand_gen[0]));
		}
		Matrix3d aff_warp = utils::computeAffineDLT(orig_pts, perturbed_pts);
		getStateFromWarp(perturbation, aff_warp);
	} else{
		//! perform geometric perturbation
		Vector6d geom_perturbation;
		for(unsigned int state_id = 0; state_id < 6; state_id++){
			geom_perturbation(state_id) = rand_dist[state_id](rand_gen[state_id]);
		}
		perturbation = geomToState(geom_perturbation);
	}

}

// use Random Walk model to generate perturbed sample
void Affine::additiveRandomWalk(VectorXd &perturbed_state,
	const VectorXd &base_state){
	if(params.pt_based_sampling){
		throw mtf::utils::FunctonNotImplemented("Affine::additiveRandomWalk :: point based sampling is not implemented yet");
	} else{
		Vector6d geom_perturbation;
		for(unsigned int state_id = 0; state_id < 6; ++state_id){
			geom_perturbation(state_id) = rand_dist[state_id](rand_gen[state_id]);
		}
		Vector6d base_geom = stateToGeom(base_state);
		Vector6d perturbed_geom = base_geom + geom_perturbation;
		perturbed_state = geomToState(perturbed_geom);
	}
}

// use first order Auto Regressive model to generate perturbed sample
void Affine::additiveAutoRegression1(VectorXd &perturbed_state, VectorXd &perturbed_ar,
	const VectorXd &base_state, const VectorXd &base_ar, double a){
	if(params.pt_based_sampling){
		throw mtf::utils::FunctonNotImplemented("Affine::additiveAutoRegression1 :: point based sampling is not implemented yet");
	} else{
		Vector6d geom_perturbation;
		for(unsigned int state_id = 0; state_id < 6; ++state_id){
			geom_perturbation(state_id) = rand_dist[state_id](rand_gen[state_id]);
		}
		Vector6d base_geom = stateToGeom(base_state);
		Vector6d base_ar_geom = stateToGeom(base_ar);
		Vector6d perturbed_geom = base_geom + base_ar_geom + geom_perturbation;
		Vector6d perturbed_ar_geom = a*(perturbed_geom - base_geom);
		perturbed_state = geomToState(perturbed_geom);
		perturbed_ar = geomToState(perturbed_ar_geom);
	}
}
void Affine::compositionalRandomWalk(VectorXd &perturbed_state,
	const VectorXd &base_state){
	if(params.pt_based_sampling){
		generatePerturbation(state_perturbation);
		ProjWarpT base_warp, warp_perturbation;
		getWarpFromState(base_warp, base_state);
		getWarpFromState(warp_perturbation, state_perturbation);
		ProjWarpT perturbed_warp = base_warp * warp_perturbation;
		getStateFromWarp(perturbed_state, perturbed_warp);
	} else{
		throw mtf::utils::FunctonNotImplemented("Affine::compositionalRandomWalk :: geometric sampling is not implemented yet");

	}
}
_MTF_END_NAMESPACE

