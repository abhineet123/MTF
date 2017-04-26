#include "mtf/SSM/LieHomography.h"
#include "mtf/SSM/HomographyEstimator.h"
#include "mtf/Utilities/warpUtils.h"
#include "mtf/Utilities/miscUtils.h"
#include <unsupported/Eigen/MatrixFunctions>

#define VALIDATE_LIE_HOM_WARP(warp) \
	assert(warp.determinant() == 1.0);

#define LHOM_NORMALIZED_INIT 0
#define LHOM_GRAD_EPS 1e-8
#define LHOM_DEBUG_MODE 0

#define MAX_VALID_VAL 1e10
#define is_unbounded(eig_mat) (eig_mat.array().cwiseAbs().eval() > MAX_VALID_VAL).any()

_MTF_BEGIN_NAMESPACE

//! value constructor
LieHomographyParams::LieHomographyParams(
const SSMParams *ssm_params,
bool _normalized_init, double _grad_eps,
bool _debug_mode) :
SSMParams(ssm_params),
normalized_init(_normalized_init),
grad_eps(_grad_eps),
debug_mode(_debug_mode){}

//! copy/default constructor
LieHomographyParams::LieHomographyParams(const LieHomographyParams *params) :
SSMParams(params),
normalized_init(LHOM_NORMALIZED_INIT),
grad_eps(LHOM_GRAD_EPS),
debug_mode(LHOM_DEBUG_MODE){
	if(params){
		normalized_init = params->normalized_init;
		grad_eps = params->grad_eps;
		debug_mode = params->debug_mode;
	}
}

LieHomography::LieHomography(
	const ParamType *lhom_params) :
	ProjectiveBase(lhom_params), params(lhom_params){

	printf("\n");
	printf("Using Lie Homography SSM with:\n");
	printf("resx: %d\n", resx);
	printf("resy: %d\n", resy);
	printf("normalized_init: %d\n", params.normalized_init);
	printf("grad_eps: %f\n", params.grad_eps);
	printf("debug_mode: %d\n", params.debug_mode);


	name = "lie_homography";
	state_size = 8;
	curr_state.resize(state_size);

	zero_vec = RowVector3d::Zero();
	lie_alg_mat = Matrix3d::Zero();
	warp_mat = Matrix3d::Identity();

	lieAlgBasis[0] <<
		1, 0, 0,
		0, -1, 0,
		0, 0, 0;
	lieAlgBasis[1] <<
		0, 1, 0,
		0, 0, 0,
		0, 0, 0;
	lieAlgBasis[2] <<
		0, 0, 1,
		0, 0, 0,
		0, 0, 0;
	lieAlgBasis[3] <<
		0, 0, 0,
		1, 0, 0,
		0, 0, 0;
	lieAlgBasis[4] <<
		0, 0, 0,
		0, 1, 0,
		0, 0, -1;
	lieAlgBasis[5] <<
		0, 0, 0,
		0, 0, 1,
		0, 0, 0;
	lieAlgBasis[6] <<
		0, 0, 0,
		0, 0, 0,
		1, 0, 0;
	lieAlgBasis[7] <<
		0, 0, 0,
		0, 0, 0,
		0, 1, 0;
}

void LieHomography::setCorners(const CornersT& corners){
	if(!corners.allFinite()){
		utils::printMatrix(corners, "corners");
		throw mtf::utils::InvalidTrackerState("LieHomography::setCorners :: Invalid corners provided");
	}

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

void LieHomography::compositionalUpdate(const VectorXd& state_update){
	validate_ssm_state(state_update);

	if(!state_update.allFinite() || is_unbounded(state_update)){
		utils::printMatrix(state_update, "state_update");
		throw mtf::utils::InvalidTrackerState("LieHomography::compositionalUpdate :: Invalid state update provided");
	}

	getWarpFromState(warp_update_mat, state_update);
	//utils::printMatrix(state_update, "state_update");
	//utils::printMatrix(warp_update_mat, "warp_update_mat");

	curr_warp = curr_warp * warp_update_mat;

	curr_pts_hm.noalias() = curr_warp * init_pts_hm;
	curr_corners_hm.noalias() = curr_warp * init_corners_hm;

	utils::dehomogenize(curr_pts_hm, curr_pts);
	utils::dehomogenize(curr_corners_hm, curr_corners);

	getStateFromWarp(curr_state, curr_warp);

}

void LieHomography::getWarpFromState(Matrix3d &warp_mat,
	const VectorXd& ssm_state){
	validate_ssm_state(ssm_state);

	assert(ssm_state.size() == state_size);
	//for(int i=0;i<state_size;i++){
	//	lie_alg_mat += ssm_state(i) * lieAlgBasis[i];
	//}

	if(!ssm_state.allFinite()){
		utils::printMatrix(ssm_state.transpose(), "ssm_state");
		throw mtf::utils::InvalidTrackerState("LieHomography::getWarpFromState :: Invalid state provided");
	}

	lie_alg_mat(0, 0) = ssm_state(0);
	lie_alg_mat(0, 1) = ssm_state(1);
	lie_alg_mat(0, 2) = ssm_state(2);
	lie_alg_mat(1, 0) = ssm_state(3);
	lie_alg_mat(1, 1) = ssm_state(4) - ssm_state(0);
	lie_alg_mat(1, 2) = ssm_state(5);
	lie_alg_mat(2, 0) = ssm_state(6);
	lie_alg_mat(2, 1) = ssm_state(7);
	lie_alg_mat(2, 2) = -ssm_state(4);
	warp_mat = lie_alg_mat.exp();
}

void LieHomography::getStateFromWarp(VectorXd &state_vec,
	const Matrix3d& warp_mat){
	validate_ssm_state(state_vec);
	if(!warp_mat.allFinite() || is_unbounded(warp_mat)){
		utils::printMatrix(warp_mat, "warp_mat");
		throw mtf::utils::InvalidTrackerState("LieHomography::getStateFromWarp :: Invalid warp matrix provided");
	}
	double warp_det = warp_mat.determinant();
	if(warp_det == 0 || std::isnan(warp_det) || std::isinf(warp_det)){
		utils::printScalar(warp_det, "warp_det");
		throw mtf::utils::InvalidTrackerState("LieHomography::getStateFromWarp :: Warp matrix has zero/invalid determinant");
	}
	Matrix3d norm_warp_mat = warp_mat / cbrt(warp_det);
	if(!norm_warp_mat.allFinite() || is_unbounded(norm_warp_mat)){
		utils::printMatrix(warp_mat, "warp_mat");
		utils::printScalar(warp_det, "warp_det");
		utils::printMatrix(norm_warp_mat, "norm_warp_mat");
		throw mtf::utils::InvalidTrackerState("LieHomography::getStateFromWarp :: Invalid normalized warp matrix obtained");
	}
	//double norm_warp_det = norm_warp_mat.determinant();
	//printf("warp_det: %f\n", warp_det);
	//printf("norm_warp_det: %f\n", norm_warp_det);
	//utils::printMatrix(warp_mat, "warp_mat");
	//utils::printScalar(warp_det, "warp_det");
	//utils::printMatrix(norm_warp_mat, "norm_warp_mat");

	Matrix3d lie_alg_mat = norm_warp_mat.log();
	state_vec(0) = lie_alg_mat(0, 0);
	state_vec(1) = lie_alg_mat(0, 1);
	state_vec(2) = lie_alg_mat(0, 2);
	state_vec(3) = lie_alg_mat(1, 0);
	state_vec(4) = -lie_alg_mat(2, 2);
	state_vec(5) = lie_alg_mat(1, 2);
	state_vec(6) = lie_alg_mat(2, 0);
	state_vec(7) = lie_alg_mat(2, 1);

	//Matrix3d norm_warp_mat_rec;
	//getWarpFromState(norm_warp_mat_rec, state_vec);
	//Matrix24d init_corners_rec;
	//Matrix34d init_corners_rec_hm;
	//init_corners_rec_hm.noalias() = norm_warp_mat_rec * init_corners_hm;
	//utils::dehomogenize(init_corners_rec_hm, init_corners_rec);
	//assert(norm_warp_det == 1.0);
	//assert(lie_alg_mat(1,1) == state_vec(4) - state_vec(0));

	//utils::printMatrix(warp_mat, "warp_mat");
	//utils::printMatrix(norm_warp_mat, "norm_warp_mat");
	//utils::printMatrix(norm_warp_mat_rec, "norm_warp_mat_rec");
	//utils::printMatrix(lie_alg_mat, "lie_alg_mat");
	//utils::printMatrix(state_vec, "state_vec");
	//utils::printMatrix(init_corners, "init_corners");
	//utils::printMatrix(init_corners_rec, "init_corners_rec");
}

void LieHomography::getInitPixGrad(Matrix2Xd &ssm_grad, int pt_id) {
	double x = init_pts(0, pt_id);
	double y = init_pts(1, pt_id);

	ssm_grad <<
		x, y, 1, 0, x, 0, -x*x, -x*y,
		-y, 0, 0, x, 2 * y, 1, -y*x, -y*y;
}


void LieHomography::getCurrPixGrad(Matrix2Xd &ssm_grad, int pt_id) {
	double x = init_pts(0, pt_id);
	double y = init_pts(1, pt_id);

	double curr_x = curr_pts(0, pt_id);
	double curr_y = curr_pts(1, pt_id);

	double xx = x*curr_x, xy = x*curr_y, yy = y*curr_y, yx = y*curr_x;

	Matrix3d curr_warp;
	getWarpFromState(curr_warp, curr_state);
	double a1 = curr_warp(0, 0), a2 = curr_warp(0, 1), a3 = curr_warp(0, 2);
	double a4 = curr_warp(1, 0), a5 = curr_warp(1, 1), a6 = curr_warp(1, 2);
	double a7 = curr_warp(2, 0), a8 = curr_warp(2, 1), a9 = curr_warp(2, 2);

	double p11 = a1*x - a2*y - a7*xx + a8*yx;
	double p12 = a1*y - a7*yx;
	double p13 = a1 - a7*curr_x;
	double p14 = a2*x - a8*xx;
	double p15 = a2*y - a3 - a8*yx + a9*curr_x;
	double p16 = a2 - a8*curr_x;
	double p17 = a3*x - a9*xx;
	double p18 = a3*y - a9*yx;

	double p21 = a4*x - a5*y - a7*xy + a8*yy;
	double p22 = a4*y - a7*yy;
	double p23 = a4 - a7*curr_y;
	double p24 = a5*x - a8*xy;
	double p25 = a5*y - a6 - a8*yy + a9*curr_y;
	double p26 = a5 - a8*curr_y;
	double p27 = a6*x - a9*xy;
	double p28 = a6*y - a9*yy;

	ssm_grad <<
		p11, p12, p13, p14, p15, p16, p17, p18,
		p21, p22, p23, p24, p25, p26, p27, p28;

	double inv_d = 1.0 / curr_pts_hm(2, pt_id);
	ssm_grad *= inv_d;
}

void LieHomography::cmptInitPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dw){
	validate_ssm_jacobian(dI_dp, dI_dw);

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; pt_id++){
		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = dI_dw(ch_pt_id, 0);
			double Iy = dI_dw(ch_pt_id, 1);


			double Ixx = Ix * x;
			double Iyy = Iy * y;
			double Ixy = Ix * y;
			double Iyx = Iy * x;

			dI_dp(ch_pt_id, 0) = Ixx - Iyy;
			dI_dp(ch_pt_id, 1) = Ixy;
			dI_dp(ch_pt_id, 2) = Ix;
			dI_dp(ch_pt_id, 3) = Iyx;
			dI_dp(ch_pt_id, 4) = Ixx + 2 * Iyy;
			dI_dp(ch_pt_id, 5) = Iy;
			dI_dp(ch_pt_id, 6) = -Ixx*x - Iyy*x;
			dI_dp(ch_pt_id, 7) = -Ixx*y - Iyy*y;
			++ch_pt_id;
		}
	}
}

void LieHomography::cmptWarpedPixJacobian(MatrixXd &dI_dp,
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

		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = (dwx_dx*dI_dw(ch_pt_id, 0) + dwy_dx*dI_dw(ch_pt_id, 1))*inv_det;
			double Iy = (dwx_dy*dI_dw(ch_pt_id, 0) + dwy_dy*dI_dw(ch_pt_id, 1))*inv_det;

			double Ixx = Ix * x;
			double Ixy = Ix * y;
			double Iyy = Iy * y;
			double Iyx = Iy * x;

			dI_dp(ch_pt_id, 0) = Ixx - Iyy;
			dI_dp(ch_pt_id, 1) = Ixy;
			dI_dp(ch_pt_id, 2) = Ix;
			dI_dp(ch_pt_id, 3) = Iyx;
			dI_dp(ch_pt_id, 4) = Ixx + 2 * Iyy;
			dI_dp(ch_pt_id, 5) = Iy;
			dI_dp(ch_pt_id, 6) = -Ixx*x - Iyy*x;
			dI_dp(ch_pt_id, 7) = -Ixx*y - Iyy*y;

			++ch_pt_id;
		}
	}
}

void LieHomography::cmptPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dw){
	validate_ssm_jacobian(dI_dp, dI_dw);

	MatrixXd dw_dp_t(16, n_pts);
	computeJacobian(dw_dp_t, init_pts_hm);

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; pt_id++){

		//Matrix2Xd dw_dp(2, 8);
		//getCurrPixGrad(dw_dp, pt_id);

		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			//dI_dp.row(ch_pt_id) = dI_dw.row(ch_pt_id)*dw_dp;

			double Ix = dI_dw(ch_pt_id, 0);
			double Iy = dI_dw(ch_pt_id, 1);

			dI_dp(ch_pt_id, 0) = Ix * dw_dp_t(0, pt_id) + Iy * dw_dp_t(1, pt_id);
			dI_dp(ch_pt_id, 1) = Ix * dw_dp_t(2, pt_id) + Iy * dw_dp_t(3, pt_id);
			dI_dp(ch_pt_id, 2) = Ix * dw_dp_t(4, pt_id) + Iy * dw_dp_t(5, pt_id);
			dI_dp(ch_pt_id, 3) = Ix * dw_dp_t(6, pt_id) + Iy * dw_dp_t(7, pt_id);
			dI_dp(ch_pt_id, 4) = Ix * dw_dp_t(8, pt_id) + Iy * dw_dp_t(9, pt_id);
			dI_dp(ch_pt_id, 5) = Ix * dw_dp_t(10, pt_id) + Iy * dw_dp_t(11, pt_id);
			dI_dp(ch_pt_id, 6) = Ix * dw_dp_t(12, pt_id) + Iy * dw_dp_t(13, pt_id);
			dI_dp(ch_pt_id, 7) = Ix * dw_dp_t(14, pt_id) + Iy * dw_dp_t(15, pt_id);

			++ch_pt_id;
		}
	}
	//jacobian_prod.array().colwise() /= curr_pts_hm.array().row(2).transpose();
}

// numerically compute the jacobian of the grid points w.r.t. x, y coordinates of the grid corners
void LieHomography::computeJacobian(MatrixXd &dw_dp, Matrix3Xd &pts_hm){
	VectorXd inc_state(8), dec_state(8);
	Matrix3d inc_warp, dec_warp;
	Matrix2Xd inc_pts(2, n_pts), dec_pts(2, n_pts);
	inc_state = dec_state = curr_state;
	for(int state_id = 0; state_id < 8; ++state_id){
		inc_state(state_id) += params.grad_eps;
		getWarpFromState(inc_warp, inc_state);
		utils::dehomogenize(inc_warp * pts_hm, inc_pts);
		inc_state(state_id) = curr_state(state_id);

		dec_state(state_id) -= params.grad_eps;
		getWarpFromState(dec_warp, dec_state);
		utils::dehomogenize(dec_warp * pts_hm, dec_pts);
		dec_state(state_id) = curr_state(state_id);

		dw_dp.middleRows(state_id * 2, 2) = (inc_pts - dec_pts) / (2 * params.grad_eps);
	}
}


void LieHomography::cmptApproxPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dw) {
	validate_ssm_jacobian(dI_dp, dI_dw);

	MatrixXd dw_dp_t(16, n_pts);
	computeJacobian(dw_dp_t, init_pts_hm);

	curr_warp /= curr_warp(2, 2);

	double h00_plus_1 = curr_warp(0, 0);
	double h01 = curr_warp(0, 1);
	double h10 = curr_warp(1, 0);
	double h11_plus_1 = curr_warp(1, 1);
	double h20 = curr_warp(2, 0);
	double h21 = curr_warp(2, 1);

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; pt_id++){

		double Nx = curr_pts_hm(0, pt_id);
		double Ny = curr_pts_hm(1, pt_id);
		double D = curr_pts_hm(2, pt_id);
		double D_sqr_inv = 1.0 / (D*D);

		double a = (h00_plus_1*D - h21*Nx) * D_sqr_inv;
		double b = (h01*D - h21*Nx) * D_sqr_inv;
		double c = (h10*D - h20*Ny) * D_sqr_inv;
		double d = (h11_plus_1*D - h21*Ny) * D_sqr_inv;
		double inv_det = 1.0 / ((a*d - b*c)*D);

		//double x = init_pts(0, pt_id);
		//double y = init_pts(1, pt_id);

		//double curr_x = curr_pts(0, pt_id);
		//double curr_y = curr_pts(1, pt_id);

		for(int ch_id = 0; ch_id < n_channels; ++ch_id){

			double Ix = dI_dw(ch_pt_id, 0);
			double Iy = dI_dw(ch_pt_id, 1);


			dI_dp(ch_pt_id, 0) = (Ix * (d*dw_dp_t(0, pt_id) - b*dw_dp_t(1, pt_id)) + 
				Iy * (a*dw_dp_t(1, pt_id) - c*dw_dp_t(0, pt_id)))*inv_det;
			dI_dp(ch_pt_id, 1) = (Ix * (d*dw_dp_t(2, pt_id) - b*dw_dp_t(3, pt_id)) + 
				Iy * (a*dw_dp_t(3, pt_id) - c*dw_dp_t(2, pt_id)))*inv_det;
			dI_dp(ch_pt_id, 2) = (Ix * (d*dw_dp_t(4, pt_id) - b*dw_dp_t(5, pt_id)) + 
				Iy * (a*dw_dp_t(5, pt_id) - c*dw_dp_t(4, pt_id)))*inv_det;
			dI_dp(ch_pt_id, 3) = (Ix * (d*dw_dp_t(6, pt_id) - b*dw_dp_t(7, pt_id)) + 
				Iy * (a*dw_dp_t(7, pt_id) - c*dw_dp_t(6, pt_id)))*inv_det;
			dI_dp(ch_pt_id, 4) = (Ix * (d*dw_dp_t(8, pt_id) - b*dw_dp_t(9, pt_id)) +
				Iy * (a*dw_dp_t(9, pt_id) - c*dw_dp_t(8, pt_id)))*inv_det;
			dI_dp(ch_pt_id, 5) = (Ix * (d*dw_dp_t(10, pt_id) - b*dw_dp_t(11, pt_id)) + 
				Iy * (a*dw_dp_t(11, pt_id) - c*dw_dp_t(10, pt_id)))*inv_det;
			dI_dp(ch_pt_id, 6) = (Ix * (d*dw_dp_t(12, pt_id) - b*dw_dp_t(13, pt_id)) + 
				Iy * (a*dw_dp_t(13, pt_id) - c*dw_dp_t(12, pt_id)))*inv_det;
			dI_dp(ch_pt_id, 7) = (Ix * (d*dw_dp_t(14, pt_id) - b*dw_dp_t(15, pt_id)) + 
				Iy * (a*dw_dp_t(15, pt_id) - c*dw_dp_t(14, pt_id)))*inv_det;


			//double Ixx = Ix * x;
			//double Ixy = Ix * y;
			//double Iyy = Iy * y;
			//double Iyx = Iy * x;

			//double factor1 = b*curr_y - d*curr_x;
			//double factor2 = c*curr_x - a*curr_y;

			//dI_dp(ch_pt_id, 0) = (Ixx*d + Ixy*b - Iyx*c - Iyy*a) * inv_det;
			//dI_dp(ch_pt_id, 1) = (Ixy*d - Iyy*c) * inv_det;
			//dI_dp(ch_pt_id, 2) = (Ix*d - Iy*c) * inv_det;
			//dI_dp(ch_pt_id, 3) = (Iyx*a - Ixx*b) * inv_det;
			//dI_dp(ch_pt_id, 4) = (Iyy*a - Ix*factor1 - Ixy*b - Iy*factor2) * inv_det;
			//dI_dp(ch_pt_id, 5) = (Iy*a - Ix*b) * inv_det;
			//dI_dp(ch_pt_id, 6) = (Ixx*factor1 + Iyx*factor2) * inv_det;
			//dI_dp(ch_pt_id, 7) = (Ixy*factor1 + Iyy*factor2) * inv_det;

			//jacobian_prod(i, 0) = (Ix*(d*x + b*y) - Iy*(c*x + a*y)) * inv_det;
			//jacobian_prod(i, 1) = (Ix*d*y - Iy*c*y) * inv_det;
			//jacobian_prod(i, 2) = (Ix*d - Iy*c) * inv_det;
			//jacobian_prod(i, 3) = (Iy*a*x - Ix*b*x) * inv_det;
			//jacobian_prod(i, 4) = (Ix*(d*curr_x - b*(y + curr_y)) + Iy*(-c*curr_x + a*(y + curr_y))) * inv_det;
			//jacobian_prod(i, 5) = (Iy*a - Ix*b) * inv_det;
			//jacobian_prod(i, 6) = (Ix*(-d*x*curr_x + b*x*curr_y) + Iy*(c*x*curr_x - a*x*curr_y)) * inv_det;
			//jacobian_prod(i, 7) = (Ix*(-d*y*curr_x + b*y*curr_y) + Iy*(c*y*curr_x - a*y*curr_y)) * inv_det;
			++ch_pt_id;
		}
	}
}

void LieHomography::estimateWarpFromCorners(VectorXd &state_update, const Matrix24d &in_corners,
	const Matrix24d &out_corners){
	validate_ssm_state(state_update);
	Matrix3d warp_update_mat = utils::computeHomographyDLT(in_corners, out_corners);
	getStateFromWarp(state_update, warp_update_mat);
}

void LieHomography::estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
	const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
	const EstimatorParams &est_params){
	cv::Mat warp_mat_cv = estimateHomography(in_pts, out_pts, mask, est_params);
	utils::copyCVToEigen<double, Matrix3d>(warp_mat, warp_mat_cv);
	getStateFromWarp(state_update, warp_mat);
}
void LieHomography::cmptInitPixHessian(MatrixXd &pix_hess_ssm,
	const PixHessT &pix_hess_coord, const PixGradT &pix_grad){
	pix_hess_ssm.fill(0);
}

_MTF_END_NAMESPACE

