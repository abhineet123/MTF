#include "mtf/SSM/CornerHomography.h"
#include "mtf/SSM/SSMEstimator.h"
#include "mtf/Utilities/warpUtils.h"
#include "mtf/Utilities/miscUtils.h"
#include <time.h>
#include "opencv2/calib3d/calib3d.hpp"

_MTF_BEGIN_NAMESPACE

CornerHomographyParams::CornerHomographyParams(const SSMParams *ssm_params,
bool _normalized_init, double _grad_eps, bool _debug_mode):
SSMParams(ssm_params),
normalized_init(_normalized_init),
grad_eps(_grad_eps),
debug_mode(_debug_mode){}
CornerHomographyParams::CornerHomographyParams(const CornerHomographyParams *params) :
SSMParams(params),
normalized_init(CHOM_NORMALIZED_BASIS),
grad_eps(CHOM_GRAD_EPS),
debug_mode(CHOM_DEBUG_MODE){
	if(params){
		normalized_init = params->normalized_init;
		grad_eps = params->grad_eps;
		debug_mode = params->debug_mode;
	}
}

CornerHomography::CornerHomography(
	const ParamType *_params) :
ProjectiveBase(_params), hom_den(0, 0),
params(_params){

	printf("\n");
	printf("Using Corner based Homography SSM with:\n");
	printf("resx: %d\n", resx);
	printf("resy: %d\n", resy);
	printf("normalized_init: %d\n", params.normalized_init);
	printf("grad_eps: %e\n", params.grad_eps);
	printf("debug_mode: %d\n", params.debug_mode);

	name = "corner_homography";
	state_size = 8;

	curr_state.resize(state_size);
	dw_dp_0.resize(16, n_pts);
	dw_dp_t.resize(16, n_pts);

	inc_pts.resize(NoChange, n_pts);
	dec_pts.resize(NoChange, n_pts);	
}

void CornerHomography::setCorners(const CornersT& corners){
	curr_corners = corners;
	utils::homogenize(curr_corners, curr_corners_hm);

	getPtsFromCorners(curr_warp, curr_pts, curr_pts_hm, curr_corners);

	if(params.normalized_init){
		init_corners = getNormCorners();
		init_corners_hm = getHomNormCorners();
		init_pts = getNormPts();
		init_pts_hm = getHomNormPts();
		getStateFromWarp(curr_state, curr_warp);
		if(!is_initialized.pts){
			computeJacobian(dw_dp_0, init_corners, init_pts_hm);
		}
	} else{
		init_corners = curr_corners;
		init_corners_hm = curr_corners_hm;
		init_pts = curr_pts;
		init_pts_hm = curr_pts_hm;
		curr_warp = Matrix3d::Identity();
		curr_state.fill(0);
		computeJacobian(dw_dp_0, init_corners, init_pts_hm);
	}
}


void CornerHomography::compositionalUpdate(const VectorXd& state_update){
	validate_ssm_state(state_update);

	//utils::printMatrix(curr_corners, "old_corners");
	//utils::printMatrix(state_update, "state_update");
	//curr_state += state_update;

	//curr_corners(0, 0) += state_update(0);
	//curr_corners(1, 0) += state_update(1);

	//curr_corners(0, 1) += state_update(2);
	//curr_corners(1, 1) += state_update(3);

	//curr_corners(0, 2) += state_update(4);
	//curr_corners(1, 2) += state_update(5);

	//curr_corners(0, 3) += state_update(6);
	//curr_corners(1, 3) += state_update(7);

	//curr_warp = utils::computeHomographyDLT(init_corners, curr_corners);
	//curr_warp /= curr_warp(2, 2);

	//curr_pts_hm.noalias() = curr_warp * init_pts_hm;
	//utils::dehomogenize(curr_pts_hm, curr_pts);
	//utils::homogenize(curr_corners, curr_corners_hm);

	getWarpFromState(warp_update_mat, state_update);
	curr_warp = curr_warp * warp_update_mat;
	curr_warp /= curr_warp(2, 2);

	getStateFromWarp(curr_state, curr_warp);

	curr_pts_hm.noalias() = curr_warp * init_pts_hm;
	curr_corners_hm.noalias() = curr_warp * init_corners_hm;

	utils::dehomogenize(curr_pts_hm, curr_pts);
	utils::dehomogenize(curr_corners_hm, curr_corners);

	//utils::printMatrix(curr_corners, "curr_corners");
}

void CornerHomography::getWarpFromState(Matrix3d &warp_mat,
	const VectorXd& ssm_state){
	validate_ssm_state(ssm_state);

	// loop unrolling for maximum speed
	updated_corners(0, 0) = init_corners(0, 0) + ssm_state(0);
	updated_corners(1, 0) = init_corners(1, 0) + ssm_state(1);

	updated_corners(0, 1) = init_corners(0, 1) + ssm_state(2);
	updated_corners(1, 1) = init_corners(1, 1) + ssm_state(3);

	updated_corners(0, 2) = init_corners(0, 2) + ssm_state(4);
	updated_corners(1, 2) = init_corners(1, 2) + ssm_state(5);

	updated_corners(0, 3) = init_corners(0, 3) + ssm_state(6);
	updated_corners(1, 3) = init_corners(1, 3) + ssm_state(7);

	warp_mat = utils::computeHomographyDLT(init_corners, updated_corners);

}
void CornerHomography::getStateFromWarp(VectorXd &state_vec,
	const Matrix3d& warp_mat){
	validate_ssm_state(state_vec);

	utils::dehomogenize(warp_mat * init_corners_hm, updated_corners);

	// loop unrolling for maximum speed

	state_vec(0) = updated_corners(0, 0) - init_corners(0, 0);
	state_vec(1) = updated_corners(1, 0) - init_corners(1, 0);

	state_vec(2) = updated_corners(0, 1) - init_corners(0, 1);
	state_vec(3) = updated_corners(1, 1) - init_corners(1, 1);

	state_vec(4) = updated_corners(0, 2) - init_corners(0, 2);
	state_vec(5) = updated_corners(1, 2) - init_corners(1, 2);

	state_vec(6) = updated_corners(0, 3) - init_corners(0, 3);
	state_vec(7) = updated_corners(1, 3) - init_corners(1, 3);
}

void CornerHomography::invertState(VectorXd& inv_state, const VectorXd& state){
	//inv_state = -state;
	getWarpFromState(warp_mat, state);
	Matrix3d inv_warp_mat = warp_mat.inverse();
	getStateFromWarp(inv_state, inv_warp_mat);
}

void CornerHomography::cmptInitPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dw){
	validate_ssm_jacobian(dI_dp, dI_dw);

	//MatrixXd jacobian_prod2(n_pts, state_size);
	//clock_t start_time=clock();
	//for(int i = 0; i < n_pts; i++){
	//	Eigen::Map< Matrix<double, 2, 8> > ssm_jacobian(init_jacobian.col(i).data());
	//	jacobian_prod2.row(i) = pix_jacobian.row(i) * ssm_jacobian;
	//}
	//clock_t end_time = clock();
	//double prod2_delay = ((double)(end_time - start_time)) / CLOCKS_PER_SEC;
	//utils::printScalar(prod2_delay, "prod2_delay");
	//utils::printMatrixToFile(jacobian_prod2, "ssm jacobian_prod2", "log/mtf_log.txt", "%15.9f", "a");

	//INIT_TIMER(start_time);
	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; pt_id++){
		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = dI_dw(ch_pt_id, 0);
			double Iy = dI_dw(ch_pt_id, 1);
			dI_dp(ch_pt_id, 0) = Ix * dw_dp_0(0, pt_id) + Iy * dw_dp_0(1, pt_id);
			dI_dp(ch_pt_id, 1) = Ix * dw_dp_0(2, pt_id) + Iy * dw_dp_0(3, pt_id);
			dI_dp(ch_pt_id, 2) = Ix * dw_dp_0(4, pt_id) + Iy * dw_dp_0(5, pt_id);
			dI_dp(ch_pt_id, 3) = Ix * dw_dp_0(6, pt_id) + Iy * dw_dp_0(7, pt_id);
			dI_dp(ch_pt_id, 4) = Ix * dw_dp_0(8, pt_id) + Iy * dw_dp_0(9, pt_id);
			dI_dp(ch_pt_id, 5) = Ix * dw_dp_0(10, pt_id) + Iy * dw_dp_0(11, pt_id);
			dI_dp(ch_pt_id, 6) = Ix * dw_dp_0(12, pt_id) + Iy * dw_dp_0(13, pt_id);
			dI_dp(ch_pt_id, 7) = Ix * dw_dp_0(14, pt_id) + Iy * dw_dp_0(15, pt_id);
			++ch_pt_id;
		}
	}
	//end_time = clock();
	//double prod_delay = ((double)(end_time - start_time)) / CLOCKS_PER_SEC;
	//utils::printScalar(prod_delay, "prod_delay");
	//utils::printMatrixToFile(jacobian_prod, "ssm jacobian_prod", "log/mtf_log.txt", "%15.9f", "a");
}

void CornerHomography::cmptWarpedPixJacobian(MatrixXd &dI_dp,
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

		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = (dwx_dx*dI_dw(ch_pt_id, 0) + dwy_dx*dI_dw(ch_pt_id, 1))*inv_det;
			double Iy = (dwx_dy*dI_dw(ch_pt_id, 0) + dwy_dy*dI_dw(ch_pt_id, 1))*inv_det;

			dI_dp(pt_id, 0) = Ix * dw_dp_0(0, pt_id) + Iy * dw_dp_0(1, pt_id);
			dI_dp(pt_id, 1) = Ix * dw_dp_0(2, pt_id) + Iy * dw_dp_0(3, pt_id);
			dI_dp(pt_id, 2) = Ix * dw_dp_0(4, pt_id) + Iy * dw_dp_0(5, pt_id);
			dI_dp(pt_id, 3) = Ix * dw_dp_0(6, pt_id) + Iy * dw_dp_0(7, pt_id);
			dI_dp(pt_id, 4) = Ix * dw_dp_0(8, pt_id) + Iy * dw_dp_0(9, pt_id);
			dI_dp(pt_id, 5) = Ix * dw_dp_0(10, pt_id) + Iy * dw_dp_0(11, pt_id);
			dI_dp(pt_id, 6) = Ix * dw_dp_0(12, pt_id) + Iy * dw_dp_0(13, pt_id);
			dI_dp(pt_id, 7) = Ix * dw_dp_0(14, pt_id) + Iy * dw_dp_0(15, pt_id);

			++ch_pt_id;
		}
	}
}

void CornerHomography::cmptPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dw){
	validate_ssm_jacobian(dI_dp, dI_dw);

	computeJacobian(dw_dp_t, curr_corners, curr_pts_hm);

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; pt_id++){
		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
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
}
void CornerHomography::cmptApproxPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dw) {
	validate_ssm_jacobian(dI_dp, dI_dw);

	computeJacobian(dw_dp_t, curr_corners, curr_pts_hm);

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
		double inv_det = 1.0 / (a*d - b*c);

		for(int ch_id = 0; ch_id < n_channels; ++ch_id){

			double Ix = dI_dw(ch_pt_id, 0);
			double Iy = dI_dw(ch_pt_id, 1);

			dI_dp(ch_pt_id, 0) = (Ix * (d*dw_dp_t(0, pt_id) - b*dw_dp_t(1, pt_id)) + Iy * (a*dw_dp_t(1, pt_id) - c*dw_dp_t(0, pt_id)))*inv_det;
			dI_dp(ch_pt_id, 1) = (Ix * (d*dw_dp_t(2, pt_id) - b*dw_dp_t(3, pt_id)) + Iy * (a*dw_dp_t(3, pt_id) - c*dw_dp_t(2, pt_id)))*inv_det;
			dI_dp(ch_pt_id, 2) = (Ix * (d*dw_dp_t(4, pt_id) - b*dw_dp_t(5, pt_id)) + Iy * (a*dw_dp_t(5, pt_id) - c*dw_dp_t(4, pt_id)))*inv_det;
			dI_dp(ch_pt_id, 3) = (Ix * (d*dw_dp_t(6, pt_id) - b*dw_dp_t(7, pt_id)) + Iy * (a*dw_dp_t(7, pt_id) - c*dw_dp_t(6, pt_id)))*inv_det;
			dI_dp(ch_pt_id, 4) = (Ix * (d*dw_dp_t(8, pt_id) - b*dw_dp_t(9, pt_id)) + Iy * (a*dw_dp_t(9, pt_id) - c*dw_dp_t(8, pt_id)))*inv_det;
			dI_dp(ch_pt_id, 5) = (Ix * (d*dw_dp_t(10, pt_id) - b*dw_dp_t(11, pt_id)) + Iy * (a*dw_dp_t(11, pt_id) - c*dw_dp_t(10, pt_id)))*inv_det;
			dI_dp(ch_pt_id, 6) = (Ix * (d*dw_dp_t(12, pt_id) - b*dw_dp_t(13, pt_id)) + Iy * (a*dw_dp_t(13, pt_id) - c*dw_dp_t(12, pt_id)))*inv_det;
			dI_dp(ch_pt_id, 7) = (Ix * (d*dw_dp_t(14, pt_id) - b*dw_dp_t(15, pt_id)) + Iy * (a*dw_dp_t(15, pt_id) - c*dw_dp_t(14, pt_id)))*inv_det;
			
			++ch_pt_id;
		}
	}
}

// numerically compute the jacobian of the grid points w.r.t. x, y coordinates of the grid corners
void CornerHomography::computeJacobian(MatrixXd &jacobian, Matrix24d &corners,
	Matrix3Xd &pts_hm){
	int state_id = 0;
	inc_corners = dec_corners = corners;
	for(int corner_id = 0; corner_id < 4; ++corner_id){
		inc_corners(0, corner_id) += params.grad_eps;
		inc_warp = utils::computeHomographyDLT(corners, inc_corners);
		utils::dehomogenize(inc_warp * pts_hm, inc_pts);
		inc_corners(0, corner_id) = corners(0, corner_id);

		dec_corners(0, corner_id) -= params.grad_eps;
		dec_warp = utils::computeHomographyDLT(corners, dec_corners);
		utils::dehomogenize(dec_warp * pts_hm, dec_pts);
		dec_corners(0, corner_id) = corners(0, corner_id);

		jacobian.middleRows(state_id * 2, 2) = (inc_pts - dec_pts) / (2 * params.grad_eps);
		++state_id;

		inc_corners(1, corner_id) += params.grad_eps;
		inc_warp = utils::computeHomographyDLT(corners, inc_corners);
		utils::dehomogenize(inc_warp * pts_hm, inc_pts);
		inc_corners(1, corner_id) = corners(1, corner_id);

		dec_corners(1, corner_id) -= params.grad_eps;
		dec_warp = utils::computeHomographyDLT(corners, dec_corners);
		utils::dehomogenize(dec_warp * pts_hm, dec_pts);
		dec_corners(1, corner_id) = corners(1, corner_id);

		jacobian.middleRows(state_id * 2, 2) = (inc_pts - dec_pts) / (2 * params.grad_eps);
		++state_id;
	}
}

void CornerHomography::getPixGrad(Matrix2Xd &ssm_grad, int pt_id,
	const PtsT &pts, const CornersT &corners) {
	double x = pts(0, pt_id);
	double y = pts(1, pt_id);

	double inc_x, inc_y;
	double dec_x, dec_y;

	int state_id = 0;
	inc_corners = dec_corners = corners;
	for(int corner_id = 0; corner_id < 4; corner_id++){
		inc_corners(0, corner_id) += params.grad_eps;
		inc_warp = utils::computeHomographyDLT(corners, inc_corners);
		applyWarpToPt(inc_x, inc_y, x, y, inc_warp);
		inc_corners(0, corner_id) = corners(0, corner_id);

		dec_corners(0, corner_id) -= params.grad_eps;
		dec_warp = utils::computeHomographyDLT(corners, dec_corners);
		applyWarpToPt(dec_x, dec_y, x, y, dec_warp);
		dec_corners(0, corner_id) = corners(0, corner_id);

		ssm_grad(0, state_id) = (inc_x - dec_x) / (2 * params.grad_eps);
		ssm_grad(1, state_id) = (inc_y - dec_y) / (2 * params.grad_eps);

		state_id++;

		inc_corners(1, corner_id) += params.grad_eps;
		inc_warp = utils::computeHomographyDLT(corners, inc_corners);
		applyWarpToPt(inc_x, inc_y, x, y, inc_warp);
		inc_corners(1, corner_id) = corners(1, corner_id);

		dec_corners(1, corner_id) -= params.grad_eps;
		dec_warp = utils::computeHomographyDLT(corners, dec_corners);
		applyWarpToPt(dec_x, dec_y, x, y, dec_warp);
		dec_corners(1, corner_id) = corners(1, corner_id);

		ssm_grad(0, state_id) = (inc_x - dec_x) / (2 * params.grad_eps);
		ssm_grad(1, state_id) = (inc_y - dec_y) / (2 * params.grad_eps);

		state_id++;
	}
}

void CornerHomography::estimateWarpFromCorners(VectorXd &state_update, const Matrix24d &in_corners,
	const Matrix24d &out_corners){
	validate_ssm_state(state_update);
	//state_update(0) = out_corners(0, 0) - in_corners(0, 0);
	//state_update(1) = out_corners(1, 0) - in_corners(1, 0);

	//state_update(2) = out_corners(0, 1) - in_corners(0, 1);
	//state_update(3) = out_corners(1, 1) - in_corners(1, 1);

	//state_update(4) = out_corners(0, 2) - in_corners(0, 2);
	//state_update(5) = out_corners(1, 2) - in_corners(1, 2);

	//state_update(6) = out_corners(0, 3) - in_corners(0, 3);
	//state_update(7) = out_corners(1, 3) - in_corners(1, 3);
	Matrix3d warp_update_mat = utils::computeHomographyDLT(in_corners, out_corners);
	getStateFromWarp(state_update, warp_update_mat);
}

void CornerHomography::estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
	const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
	const EstimatorParams &est_params){
	cv::Mat warp_mat_cv = estimateHomography(in_pts, out_pts, mask, est_params);
	utils::copyCVToEigen<double, Matrix3d>(warp_mat, warp_mat_cv);
	getStateFromWarp(state_update, warp_mat);
}


_MTF_END_NAMESPACE

