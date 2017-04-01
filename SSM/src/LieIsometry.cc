#include "mtf/SSM/LieIsometry.h"
#include "mtf/Utilities/warpUtils.h"
#include "mtf/Utilities/miscUtils.h"
#include <unsupported/Eigen/MatrixFunctions>

_MTF_BEGIN_NAMESPACE

LieIsometry::LieIsometry( LieIsometryParams *_params) : 
ProjectiveBase(_params), 
params(_params){
	printf("\n");
	printf("Using Lie Isometry SSM with:\n");
	printf("resx: %d\n", resx);
	printf("resy: %d\n", resy);
	printf("c: %f\n", params.c);
	printf("debug_mode: %d\n", params.debug_mode);
	printf("init_as_basis: %d\n", params.init_as_basis);


	name = "lie isometry";
	state_size = 3;
	curr_state.resize(state_size);

	zero_vec = RowVector3d::Zero();
	lie_alg_mat = Matrix3d::Zero();
	warp_mat = Matrix3d::Identity();
}

void LieIsometry::compositionalUpdate(const VectorXd& state_update){
	validate_ssm_state(state_update);

	getWarpFromState(warp_update_mat, state_update);
	curr_warp = curr_warp * warp_update_mat;
	//curr_warp /= curr_warp(2, 2);

	curr_pts_hm.noalias() = curr_warp * init_pts_hm;
	curr_corners_hm.noalias() = curr_warp * init_corners_hm;

	utils::dehomogenize(curr_pts_hm, curr_pts);
	utils::dehomogenize(curr_corners_hm, curr_corners);
}

void LieIsometry::getWarpFromState(Matrix3d &warp_mat,
	const VectorXd& ssm_state){
	validate_ssm_state(ssm_state);

	assert(ssm_state.size() == state_size);
	//for(int i=0;i<state_size;i++){
	//	lie_alg_mat += ssm_state(i) * lieAlgBasis[i];
	//}
	lie_alg_mat(0, 1) = -ssm_state(2);
	lie_alg_mat(0, 2) = ssm_state(0);
	lie_alg_mat(1, 0) = ssm_state(2);
	lie_alg_mat(1, 2) = ssm_state(1);
	warp_mat = lie_alg_mat.exp();
}

void LieIsometry::getStateFromWarp(VectorXd &ssm_state,
	const Matrix3d& warp_mat){
	validate_ssm_state(ssm_state);

	Matrix3d lie_alg_mat = warp_mat.log();

	ssm_state(0) = lie_alg_mat(0, 2);
	ssm_state(1) = lie_alg_mat(1, 2);
	ssm_state(2) = lie_alg_mat(1, 0);
}

void LieIsometry::invertState(VectorXd& inv_state, const VectorXd& state){
	inv_state = -state;
}

void LieIsometry::cmptInitPixJacobian(MatrixXd &jacobian_prod,
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
		jacobian_prod(i, 6) = -Ixx*x - Iyy*x;
		jacobian_prod(i, 7) = -Ixx*y - Iyy*y;
	}
}

void LieIsometry::cmptPixJacobian(MatrixXd &jacobian_prod,
	const PixGradT &pix_jacobian){
	validate_ssm_jacobian(jacobian_prod, pix_jacobian);

	for (int i = 0; i < n_pts; i++){
		double x = init_pts(0, i);
		double y = init_pts(1, i);
		double Ix = pix_jacobian(i, 0);
		double Iy = pix_jacobian(i, 1);

		double curr_x = curr_pts(0, i);
		double curr_y = curr_pts(1, i);

		double Ixx = Ix * x;
		double Iyy = Iy * y;
		double Ixy = Ix * y;
		double Iyx = Iy * x;

		jacobian_prod(i, 0) = Ixx - Iyy;
		jacobian_prod(i, 1) = Ixy;
		jacobian_prod(i, 2) = Ix;
		jacobian_prod(i, 3) = Iyx;
		jacobian_prod(i, 4) = Ix*curr_x + Iy*(y + curr_y);
		jacobian_prod(i, 5) = Iy;
		jacobian_prod(i, 6) = -Ixx*curr_x - Iyx*curr_y;
		jacobian_prod(i, 7) = -Ixy*curr_x - Iyy*curr_y;
	}
	jacobian_prod.array().colwise() /= curr_pts_hm.array().row(2).transpose();
}

void LieIsometry::cmptApproxPixJacobian(MatrixXd &jacobian_prod,
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


void LieIsometry::estimateWarpFromCorners(VectorXd &state_update, const Matrix24d &in_corners,
	const Matrix24d &out_corners){
	validate_ssm_state(state_update);
	Matrix3d warp_update_mat = utils::computeHomographyDLT(in_corners, out_corners);
	getStateFromWarp(state_update, warp_update_mat);
}

void LieIsometry::estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
	const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
	int estimation_method, double ransac_reproj_thresh){
	cv::Mat warp_mat_cv = cv::findHomography(in_pts, out_pts, estimation_method,
		ransac_reproj_thresh, mask);
	utils::copyCVToEigen<double, Matrix3d>(warp_mat, warp_mat_cv);
	getStateFromWarp(state_update, warp_mat);
}

void LieIsometry::updateGradPts(double grad_eps){
	Vector3d diff_vec_x_warped = curr_warp.col(0) * grad_eps;
	Vector3d diff_vec_y_warped = curr_warp.col(1) * grad_eps;

	Vector3d pt_inc_warped, pt_dec_warped;
	for(int pix_id = 0; pix_id < n_pts; pix_id++){
		pt_inc_warped = curr_pts_hm.col(pix_id) + diff_vec_x_warped;
		grad_pts(0, pix_id) = pt_inc_warped(0) / pt_inc_warped(2);
		grad_pts(1, pix_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pix_id) - diff_vec_x_warped;
		grad_pts(2, pix_id) = pt_dec_warped(0) / pt_dec_warped(2);
		grad_pts(3, pix_id) = pt_dec_warped(1) / pt_dec_warped(2);

		pt_inc_warped = curr_pts_hm.col(pix_id) + diff_vec_y_warped;
		grad_pts(4, pix_id) = pt_inc_warped(0) / pt_inc_warped(2);
		grad_pts(5, pix_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pix_id) - diff_vec_y_warped;
		grad_pts(6, pix_id) = pt_dec_warped(0) / pt_dec_warped(2);
		grad_pts(7, pix_id) = pt_dec_warped(1) / pt_dec_warped(2);
	}
}

void LieIsometry::updateHessPts(double hess_eps){
	double hess_eps2 = 2 * hess_eps;

	Vector3d diff_vec_xx_warped = curr_warp.col(0) * hess_eps2;
	Vector3d diff_vec_yy_warped = curr_warp.col(1) * hess_eps2;
	Vector3d diff_vec_xy_warped = (curr_warp.col(0) + curr_warp.col(1)) * hess_eps;
	Vector3d diff_vec_yx_warped = (curr_warp.col(0) - curr_warp.col(1)) * hess_eps;

	Vector3d pt_inc_warped, pt_dec_warped;

	for(int pix_id = 0; pix_id < n_pts; pix_id++){

		pt_inc_warped = curr_pts_hm.col(pix_id) + diff_vec_xx_warped;
		hess_pts(0, pix_id) = pt_inc_warped(0) / pt_inc_warped(2);
		hess_pts(1, pix_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pix_id) - diff_vec_xx_warped;
		hess_pts(2, pix_id) = pt_dec_warped(0) / pt_dec_warped(2);
		hess_pts(3, pix_id) = pt_dec_warped(1) / pt_dec_warped(2);

		pt_inc_warped = curr_pts_hm.col(pix_id) + diff_vec_yy_warped;
		hess_pts(4, pix_id) = pt_inc_warped(0) / pt_inc_warped(2);
		hess_pts(5, pix_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pix_id) - diff_vec_yy_warped;
		hess_pts(6, pix_id) = pt_dec_warped(0) / pt_dec_warped(2);
		hess_pts(7, pix_id) = pt_dec_warped(1) / pt_dec_warped(2);

		pt_inc_warped = curr_pts_hm.col(pix_id) + diff_vec_xy_warped;
		hess_pts(8, pix_id) = pt_inc_warped(0) / pt_inc_warped(2);
		hess_pts(9, pix_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pix_id) - diff_vec_xy_warped;
		hess_pts(10, pix_id) = pt_dec_warped(0) / pt_dec_warped(2);
		hess_pts(11, pix_id) = pt_dec_warped(1) / pt_dec_warped(2);

		pt_inc_warped = curr_pts_hm.col(pix_id) + diff_vec_yx_warped;
		hess_pts(12, pix_id) = pt_inc_warped(0) / pt_inc_warped(2);
		hess_pts(13, pix_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pix_id) - diff_vec_yx_warped;
		hess_pts(14, pix_id) = pt_dec_warped(0) / pt_dec_warped(2);
		hess_pts(15, pix_id) = pt_dec_warped(1) / pt_dec_warped(2);
	}
}

void LieIsometry::applyWarpToCorners(Matrix24d &warped_corners, const Matrix24d &orig_corners,
	const VectorXd &ssm_state){
	getWarpFromState(warp_mat, ssm_state);
	for(int i = 0; i < 4; i++){
		double discr = warp_mat(2, 0)*orig_corners(0, i) + warp_mat(2, 1)*orig_corners(1, i) + warp_mat(2, 2);
		warped_corners(0, i) = (warp_mat(0, 0)*orig_corners(0, i) + warp_mat(0, 1)*orig_corners(1, i) +
			warp_mat(0, 2)) / discr;
		warped_corners(1, 0) = (warp_mat(1, 0)*orig_corners(0, i) + warp_mat(1, 1)*orig_corners(1, i) +
			warp_mat(1, 2)) / discr;
	}
}

void LieIsometry::applyWarpToPts(Matrix2Xd &warped_pts, const Matrix2Xd &orig_pts,
	const VectorXd &ssm_state){
	getWarpFromState(warp_mat, ssm_state);
	int n_pts = orig_pts.cols();
	for(int i = 0; i < n_pts; i++){
		double discr = warp_mat(2, 0)*orig_pts(0, i) + warp_mat(2, 1)*orig_pts(1, i) + warp_mat(2, 2);
		warped_pts(0, i) = (warp_mat(0, 0)*orig_pts(0, i) + warp_mat(0, 1)*orig_pts(1, i) +
			warp_mat(0, 2)) / discr;
		warped_pts(1, i) = (warp_mat(1, 0)*orig_pts(0, i) + warp_mat(1, 1)*orig_pts(1, i) +
			warp_mat(1, 2)) / discr;
	}
}


_MTF_END_NAMESPACE

