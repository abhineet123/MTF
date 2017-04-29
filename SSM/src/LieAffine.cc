#include "mtf/SSM/LieAffine.h"
#include "mtf/SSM/AffineEstimator.h"
#include "mtf/Utilities/warpUtils.h"
#include "mtf/Utilities/miscUtils.h"
#include <unsupported/Eigen/MatrixFunctions>
#include "opencv2/calib3d/calib3d.hpp"

#define VALIDATE_LIE_AFF_WARP(warp) \
	assert(warp.determinant() == 1.0);

#define LAFF_NORMALIZED_INIT 0
#define LAFF_GRAD_EPS 0
#define LAFF_DEBUG_MODE 0

_MTF_BEGIN_NAMESPACE

//! value constructor
LieAffineParams::LieAffineParams(const SSMParams *ssm_params,
bool _normalized_init, double _grad_eps, bool _debug_mode) :
SSMParams(ssm_params),
normalized_init(_normalized_init),
grad_eps(_grad_eps),
debug_mode(_debug_mode){}

//! copy/default constructor
LieAffineParams::LieAffineParams(const LieAffineParams *params) :
normalized_init(LAFF_NORMALIZED_INIT),
grad_eps(LAFF_GRAD_EPS),
debug_mode(LAFF_DEBUG_MODE){
	if(params){
		normalized_init = params->normalized_init;
		grad_eps = params->grad_eps;
		debug_mode = params->debug_mode;
	}
}

LieAffine::LieAffine(const ParamType *_params) :
ProjectiveBase(_params), params(_params){
	printf("\n");
	printf("Using Lie Affine SSM with:\n");
	printf("resx: %d\n", resx);
	printf("resy: %d\n", resy);
	printf("normalized_init: %d\n", params.normalized_init);
	printf("grad_eps: %f\n", params.grad_eps);
	printf("debug_mode: %d\n", params.debug_mode);

	name = "lie_affine";
	state_size = 6;
	curr_state.resize(state_size);

	zero_vec = RowVector3d::Zero();
	lie_alg_mat = Matrix3d::Zero();
	warp_mat = Matrix3d::Identity();

	lieAlgBasis[0] <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 0;
	lieAlgBasis[1] <<
		1, 0, 0,
		0, -1, 0,
		0, 0, 0;
	lieAlgBasis[2] <<
		0, -1, 0,
		1, 0, 0,
		0, 0, 0;
	lieAlgBasis[3] <<
		0, 1, 0,
		1, 0, 0,
		0, 0, 0;
	lieAlgBasis[4] <<
		0, 0, 1,
		0, 0, 0,
		0, 0, 0;
	lieAlgBasis[5] <<
		0, 0, 0,
		0, 0, 1,
		0, 0, 0;
}

void LieAffine::setCorners(const CornersT& corners){
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

void LieAffine::compositionalUpdate(const VectorXd& state_update){
	validate_ssm_state(state_update);

	getWarpFromState(warp_update_mat, state_update);
	curr_warp = curr_warp * warp_update_mat;

	curr_pts_hm.noalias() = curr_warp * init_pts_hm;
	curr_corners_hm.noalias() = curr_warp * init_corners_hm;

	utils::dehomogenize(curr_pts_hm, curr_pts);
	utils::dehomogenize(curr_corners_hm, curr_corners);

	getStateFromWarp(curr_state, curr_warp);

}

void LieAffine::getWarpFromState(Matrix3d &warp_mat,
	const VectorXd& ssm_state){
	validate_ssm_state(ssm_state);

	assert(ssm_state.size() == state_size);
	//for(int i=0;i<state_size;i++){
	//	lie_alg_mat += ssm_state(i) * lieAlgBasis[i];
	//}
	lie_alg_mat(0, 0) = ssm_state(0) + ssm_state(1);
	lie_alg_mat(0, 1) = ssm_state(3) - ssm_state(2);
	lie_alg_mat(0, 2) = ssm_state(4);
	lie_alg_mat(1, 0) = ssm_state(2) + ssm_state(3);
	lie_alg_mat(1, 1) = ssm_state(0) - ssm_state(1);
	lie_alg_mat(1, 2) = ssm_state(5);
	lie_alg_mat(2, 0) = 0;
	lie_alg_mat(2, 1) = 0;
	lie_alg_mat(2, 2) = 0;
	warp_mat = lie_alg_mat.exp();
}

void LieAffine::getStateFromWarp(VectorXd &ssm_state,
	const Matrix3d& warp_mat){
	validate_ssm_state(ssm_state);

	double warp_det = warp_mat.determinant();
	Matrix3d norm_warp_mat = warp_mat / cbrt(warp_det);

	//double norm_warp_det = norm_warp_mat.determinant();
	//printf("warp_det: %f\n", warp_det);
	//printf("norm_warp_det: %f\n", norm_warp_det);

	Matrix3d lie_alg_mat = norm_warp_mat.log();
	ssm_state(0) = (lie_alg_mat(0, 0) + lie_alg_mat(1, 1)) / 2.0;
	ssm_state(1) = (lie_alg_mat(0, 0) - lie_alg_mat(1, 1)) / 2.0;
	ssm_state(2) = (lie_alg_mat(1, 0) - lie_alg_mat(0, 1)) / 2.0;
	ssm_state(3) = (lie_alg_mat(1, 0) + lie_alg_mat(0, 1)) / 2.0;
	ssm_state(4) = lie_alg_mat(0, 2);
	ssm_state(5) = lie_alg_mat(1, 2);
}

void LieAffine::getInitPixGrad(Matrix2Xd &ssm_grad, int pt_id) {
	double x = init_pts(0, pt_id);
	double y = init_pts(1, pt_id);

	ssm_grad <<
		x, x, -y, y, 1, 0,
		y, -y, x, x, 0, 1;
}


void LieAffine::getCurrPixGrad(Matrix2Xd &ssm_grad, int pt_id) {
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

void LieAffine::cmptInitPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dw){
	validate_ssm_jacobian(dI_dp, dI_dw);

	unsigned int ch_pt_id = 0;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		for(unsigned int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = dI_dw(ch_pt_id, 0);
			double Iy = dI_dw(ch_pt_id, 1);

			double Ixx = Ix * x;
			double Iyy = Iy * y;
			double Ixy = Ix * y;
			double Iyx = Iy * x;

			dI_dp(ch_pt_id, 0) = Ixx + Iyy;
			dI_dp(ch_pt_id, 1) = Ixx - Iyy;
			dI_dp(ch_pt_id, 2) = Iyx - Ixy;
			dI_dp(ch_pt_id, 3) = Iyx + Ixy;
			dI_dp(ch_pt_id, 4) = Ix;
			dI_dp(ch_pt_id, 5) = Iy;
			++ch_pt_id;
		}
	}
}

void LieAffine::cmptWarpedPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dw) {
	validate_ssm_jacobian(dI_dp, dI_dw);

	double a00 = curr_warp(0, 0);
	double a01 = curr_warp(0, 1);
	double a10 = curr_warp(1, 0);
	double a11 = curr_warp(1, 1);

	unsigned int ch_pt_id = 0;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id) {
		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		for(unsigned int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = a00*dI_dw(ch_pt_id, 0) + a10*dI_dw(ch_pt_id, 1);
			double Iy = a01*dI_dw(ch_pt_id, 0) + a11*dI_dw(ch_pt_id, 1);

			double Ixx = Ix * x;
			double Ixy = Ix * y;
			double Iyy = Iy * y;
			double Iyx = Iy * x;

			dI_dp(ch_pt_id, 0) = Ixx + Iyy;
			dI_dp(ch_pt_id, 1) = Ixx - Iyy;
			dI_dp(ch_pt_id, 2) = Iyx - Ixy;
			dI_dp(ch_pt_id, 3) = Iyx + Ixy;
			dI_dp(ch_pt_id, 4) = Ix;
			dI_dp(ch_pt_id, 5) = Iy;

			++ch_pt_id;
		}
	}
}

void LieAffine::cmptPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dw){
	validate_ssm_jacobian(dI_dp, dI_dw);

	MatrixXd dw_dp_t(12, n_pts);
	computeJacobian(dw_dp_t, init_pts_hm);

	unsigned int ch_pt_id = 0;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){

		//Matrix2Xd dw_dp(2, 8);
		//getCurrPixGrad(dw_dp, pt_id);

		for(unsigned int ch_id = 0; ch_id < n_channels; ++ch_id){
			//dI_dp.row(ch_pt_id) = dI_dw.row(ch_pt_id)*dw_dp;

			double Ix = dI_dw(ch_pt_id, 0);
			double Iy = dI_dw(ch_pt_id, 1);

			dI_dp(ch_pt_id, 0) = Ix * dw_dp_t(0, pt_id) + Iy * dw_dp_t(1, pt_id);
			dI_dp(ch_pt_id, 1) = Ix * dw_dp_t(2, pt_id) + Iy * dw_dp_t(3, pt_id);
			dI_dp(ch_pt_id, 2) = Ix * dw_dp_t(4, pt_id) + Iy * dw_dp_t(5, pt_id);
			dI_dp(ch_pt_id, 3) = Ix * dw_dp_t(6, pt_id) + Iy * dw_dp_t(7, pt_id);
			dI_dp(ch_pt_id, 4) = Ix * dw_dp_t(8, pt_id) + Iy * dw_dp_t(9, pt_id);
			dI_dp(ch_pt_id, 5) = Ix * dw_dp_t(10, pt_id) + Iy * dw_dp_t(11, pt_id);

			++ch_pt_id;
		}
	}
	//jacobian_prod.array().colwise() /= curr_pts_hm.array().row(2).transpose();
}

// numerically compute the jacobian of the grid points w.r.t. x, y coordinates of the grid corners
void LieAffine::computeJacobian(MatrixXd &dw_dp, Matrix3Xd &pts_hm){
	VectorXd inc_state(6), dec_state(6);
	Matrix3d inc_warp, dec_warp;
	Matrix2Xd inc_pts(2, n_pts), dec_pts(2, n_pts);
	inc_state = dec_state = curr_state;
	for(unsigned int state_id = 0; state_id < 6; ++state_id){
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


void LieAffine::cmptApproxPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dw) {
	validate_ssm_jacobian(dI_dp, dI_dw);

	MatrixXd dw_dp_t(12, n_pts);
	computeJacobian(dw_dp_t, init_pts_hm);

	double h00_plus_1 = curr_warp(0, 0);
	double h01 = curr_warp(0, 1);
	double h10 = curr_warp(1, 0);
	double h11_plus_1 = curr_warp(1, 1);
	double a = h00_plus_1;
	double b = h01;
	double c = h10;
	double d = h11_plus_1;
	double inv_det = 1.0 / (a*d - b*c);

	unsigned int ch_pt_id = 0;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
		//double x = init_pts(0, pt_id);
		//double y = init_pts(1, pt_id);

		//double curr_x = curr_pts(0, pt_id);
		//double curr_y = curr_pts(1, pt_id);

		for(unsigned int ch_id = 0; ch_id < n_channels; ++ch_id){

			double Ix = dI_dw(ch_pt_id, 0)*inv_det;
			double Iy = dI_dw(ch_pt_id, 1)*inv_det;

			dI_dp(ch_pt_id, 0) = Ix * (d*dw_dp_t(0, pt_id) - b*dw_dp_t(1, pt_id)) +
				Iy * (a*dw_dp_t(1, pt_id) - c*dw_dp_t(0, pt_id));
			dI_dp(ch_pt_id, 1) = Ix * (d*dw_dp_t(2, pt_id) - b*dw_dp_t(3, pt_id)) +
				Iy * (a*dw_dp_t(3, pt_id) - c*dw_dp_t(2, pt_id));
			dI_dp(ch_pt_id, 2) = Ix * (d*dw_dp_t(4, pt_id) - b*dw_dp_t(5, pt_id)) +
				Iy * (a*dw_dp_t(5, pt_id) - c*dw_dp_t(4, pt_id));
			dI_dp(ch_pt_id, 3) = Ix * (d*dw_dp_t(6, pt_id) - b*dw_dp_t(7, pt_id)) +
				Iy * (a*dw_dp_t(7, pt_id) - c*dw_dp_t(6, pt_id));
			dI_dp(ch_pt_id, 4) = Ix * (d*dw_dp_t(8, pt_id) - b*dw_dp_t(9, pt_id)) +
				Iy * (a*dw_dp_t(9, pt_id) - c*dw_dp_t(8, pt_id));
			dI_dp(ch_pt_id, 5) = Ix * (d*dw_dp_t(10, pt_id) - b*dw_dp_t(11, pt_id)) +
				Iy * (a*dw_dp_t(11, pt_id) - c*dw_dp_t(10, pt_id));
	
			++ch_pt_id;
		}
	}
}

void LieAffine::estimateWarpFromCorners(VectorXd &state_update, const Matrix24d &in_corners,
	const Matrix24d &out_corners){
	validate_ssm_state(state_update);
	Matrix3d warp_update_mat = utils::computeAffineDLT(in_corners, out_corners);
	getStateFromWarp(state_update, warp_update_mat);
}

void LieAffine::estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
	const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
	const EstimatorParams &est_params){
	cv::Mat warp_mat_cv = estimateAffine(in_pts, out_pts, mask, est_params);
	utils::copyCVToEigen<double, Matrix3d>(warp_mat, warp_mat_cv);
	getStateFromWarp(state_update, warp_mat);
}

_MTF_END_NAMESPACE

