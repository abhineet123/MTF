#include "mtf/SSM/LieAffine.h"
#include "mtf/Utilities/warpUtils.h"
#include "mtf/Utilities/miscUtils.h"
#include <unsupported/Eigen/MatrixFunctions>
#include "opencv2/calib3d/calib3d.hpp"

_MTF_BEGIN_NAMESPACE

//! value constructor
LieAffineParams::LieAffineParams(const SSMParams *ssm_params,
bool _init_as_basis, bool _debug_mode) :
SSMParams(ssm_params),
normalized_init(_init_as_basis),
debug_mode(_debug_mode){}

//! copy/default constructor
LieAffineParams::LieAffineParams(LieAffineParams *params = nullptr) :
normalized_init(LAFF_NORMALIZED_BASIS),
debug_mode(LAFF_DEBUG_MODE){
	if(params){
		normalized_init = params->normalized_init;
		debug_mode = params->debug_mode;
	}
}

LieAffine::LieAffine(
LieAffineParams *_params) : 
ProjectiveBase(_params), params(_params){

	printf("\n");
	printf("Using Lie Affine SSM with:\n");
	printf("resx: %d\n", resx);
	printf("resy: %d\n", resy);
	printf("debug_mode: %d\n", params.debug_mode);
	printf("init_as_basis: %d\n", params.normalized_init);


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
	ssm_state(0) = (lie_alg_mat(0, 0) + lie_alg_mat(1, 1))/2.0;
	ssm_state(1) = (lie_alg_mat(0, 0) - lie_alg_mat(1, 1)) / 2.0;
	ssm_state(2) = (lie_alg_mat(1, 0) - lie_alg_mat(0, 1)) / 2.0;
	ssm_state(3) = (lie_alg_mat(1, 0) + lie_alg_mat(0, 1)) / 2.0;
	ssm_state(4) = lie_alg_mat(0, 2);
	ssm_state(5) = lie_alg_mat(1, 2);
}

void LieAffine::cmptInitPixJacobian(MatrixXd &jacobian_prod,
	const PixGradT &pix_jacobian){
	validate_ssm_jacobian(jacobian_prod, pix_jacobian);

	for (int i = 0; i < n_pts; i++){
		double x = init_pts(0, i);
		double y = init_pts(1, i);
		double Ix = pix_jacobian(i, 0);
		double Iy = pix_jacobian(i, 1);

		double Ixx = Ix * x;
		double Iyy = Iy * y;
		double Ixy = Ix * y;
		double Iyx = Iy * x;

		jacobian_prod(i, 0) = Ixx - Iyy;
		jacobian_prod(i, 1) = Ixy;
		jacobian_prod(i, 2) = Ix;
		jacobian_prod(i, 3) = Iyx;
		jacobian_prod(i, 4) = Ixx + 2 * Iyy;
		jacobian_prod(i, 5) = Iy;
	}
}

void LieAffine::getInitPixGrad(Matrix2Xd &ssm_grad, int pt_id) {
	double x = init_pts(0, pt_id);
	double y = init_pts(1, pt_id);

	ssm_grad <<
		 x, y, 1, 0,   x, 0, -x*x, -x*y,
		-y, 0, 0, x, 2*y, 1, -y*x, -y*y;
}

void LieAffine::cmptPixJacobian(MatrixXd &jacobian_prod,
	const PixGradT &pix_jacobian){
	validate_ssm_jacobian(jacobian_prod, pix_jacobian);

	for (int pt_id = 0; pt_id < n_pts; pt_id++){
		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		double Ix = pix_jacobian(pt_id, 0);
		double Iy = pix_jacobian(pt_id, 1);

		double curr_x = curr_pts(0, pt_id);
		double curr_y = curr_pts(1, pt_id);
		double inv_d = 1.0 / curr_pts_hm(2, pt_id);

		double Ixx = Ix * x;
		double Iyy = Iy * y;
		double Ixy = Ix * y;
		double Iyx = Iy * x;

		jacobian_prod(pt_id, 0) = (Ixx - Iyy) * inv_d;
		jacobian_prod(pt_id, 1) = Ixy * inv_d;
		jacobian_prod(pt_id, 2) = Ix * inv_d;
		jacobian_prod(pt_id, 3) = Iyx * inv_d;
		jacobian_prod(pt_id, 4) = (Ix*curr_x + Iy*(y + curr_y)) * inv_d;
		jacobian_prod(pt_id, 5) = Iy * inv_d;
		jacobian_prod(pt_id, 6) = (-Ixx*curr_x - Iyx*curr_y) * inv_d;
		jacobian_prod(pt_id, 7) = (-Ixy*curr_x - Iyy*curr_y) * inv_d;
	}
	//jacobian_prod.array().colwise() /= curr_pts_hm.array().row(2).transpose();
}

void LieAffine::getCurrPixGrad(Matrix2Xd &ssm_grad, int pt_id) {
	double x = init_pts(0, pt_id);
	double y = init_pts(1, pt_id);

	double curr_x = curr_pts(0, pt_id);
	double curr_y = curr_pts(1, pt_id);
	double inv_d = 1.0 / curr_pts_hm(2, pt_id);

	ssm_grad <<
		x, y, 1, 0, curr_x, 0, -x*curr_x, -y*curr_x,
		-y, 0, 0, x, y + curr_y, 1, -x*curr_y, -y*curr_y;
	ssm_grad *= inv_d;
}

void LieAffine::cmptApproxPixJacobian(MatrixXd &jacobian_prod,
	const PixGradT &pix_jacobian) {
	validate_ssm_jacobian(jacobian_prod, pix_jacobian);

	curr_warp /= curr_warp(2, 2);

	double h00_plus_1 = curr_warp(0, 0);
	double h01 = curr_warp(0, 1);
	double h10 = curr_warp(1, 0);
	double h11_plus_1 = curr_warp(1, 1);
	double h20 = curr_warp(2, 0);
	double h21 = curr_warp(2, 1);

	for(int i = 0; i < n_pts; i++){

		double Nx = curr_pts_hm(0, i);
		double Ny = curr_pts_hm(1, i);
		double D = curr_pts_hm(2, i);
		double D_sqr_inv = 1.0 / (D*D);

		double a = (h00_plus_1*D - h21*Nx) * D_sqr_inv;
		double b = (h01*D - h21*Nx) * D_sqr_inv;
		double c = (h10*D - h20*Ny) * D_sqr_inv;
		double d = (h11_plus_1*D - h21*Ny) * D_sqr_inv;
		double inv_det = 1.0 / ((a*d - b*c)*D);

		double x = init_pts(0, i);
		double y = init_pts(1, i);

		double curr_x = curr_pts(0, i);
		double curr_y = curr_pts(1, i);

		double Ix = pix_jacobian(i, 0);
		double Iy = pix_jacobian(i, 1);

		double Ixx = Ix * x;
		double Ixy = Ix * y;
		double Iyy = Iy * y;
		double Iyx = Iy * x;

		double factor1 = b*curr_y - d*curr_x;
		double factor2 = c*curr_x - a*curr_y;

		jacobian_prod(i, 0) = (Ixx*d + Ixy*b - Iyx*c - Iyy*a) * inv_det;
		jacobian_prod(i, 1) = (Ixy*d - Iyy*c) * inv_det;
		jacobian_prod(i, 2) = (Ix*d - Iy*c) * inv_det;
		jacobian_prod(i, 3) = (Iyx*a - Ixx*b) * inv_det;
		jacobian_prod(i, 4) = (Iyy*a - Ix*factor1 - Ixy*b - Iy*factor2) * inv_det;
		jacobian_prod(i, 5) = (Iy*a - Ix*b) * inv_det;
		jacobian_prod(i, 6) = (Ixx*factor1 + Iyx*factor2) * inv_det;
		jacobian_prod(i, 7) = (Ixy*factor1 + Iyy*factor2) * inv_det;

		//jacobian_prod(i, 0) = (Ix*(d*x + b*y) - Iy*(c*x + a*y)) * inv_det;
		//jacobian_prod(i, 1) = (Ix*d*y - Iy*c*y) * inv_det;
		//jacobian_prod(i, 2) = (Ix*d - Iy*c) * inv_det;
		//jacobian_prod(i, 3) = (Iy*a*x - Ix*b*x) * inv_det;
		//jacobian_prod(i, 4) = (Ix*(d*curr_x - b*(y + curr_y)) + Iy*(-c*curr_x + a*(y + curr_y))) * inv_det;
		//jacobian_prod(i, 5) = (Iy*a - Ix*b) * inv_det;
		//jacobian_prod(i, 6) = (Ix*(-d*x*curr_x + b*x*curr_y) + Iy*(c*x*curr_x - a*x*curr_y)) * inv_det;
		//jacobian_prod(i, 7) = (Ix*(-d*y*curr_x + b*y*curr_y) + Iy*(c*y*curr_x - a*y*curr_y)) * inv_det;
	}
}

void LieAffine::estimateWarpFromCorners(VectorXd &state_update, const Matrix24d &in_corners,
	const Matrix24d &out_corners){
	validate_ssm_state(state_update);
	Matrix3d warp_update_mat = utils::computeHomographyDLT(in_corners, out_corners);
	getStateFromWarp(state_update, warp_update_mat);
}

void LieAffine::estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
	const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
	int estimation_method, double ransac_reproj_thresh){
	cv::Mat warp_mat_cv = cv::findHomography(in_pts, out_pts, estimation_method,
		ransac_reproj_thresh, mask);
	utils::copyCVToEigen<double, Matrix3d>(warp_mat, warp_mat_cv);
	getStateFromWarp(state_update, warp_mat);
}


_MTF_END_NAMESPACE

