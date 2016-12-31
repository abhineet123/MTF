#include "mtf/SSM/Transcaling.h"
#include "mtf/SSM/SSMEstimator.h"
#include "mtf/Utilities/warpUtils.h"
#include "mtf/Utilities/miscUtils.h"

_MTF_BEGIN_NAMESPACE

TranscalingParams::TranscalingParams(const SSMParams *ssm_params,
bool _debug_mode) :
SSMParams(ssm_params),
debug_mode(_debug_mode){}

TranscalingParams::TranscalingParams(const TranscalingParams *params) :
SSMParams(params),
debug_mode(TRANSCALING_DEBUG_MODE){
	if(params){
		debug_mode = params->debug_mode;
	}
}

Transcaling::Transcaling( const ParamType *_params) :
ProjectiveBase(_params), params(_params){

	printf("\n");
	printf("Using Transcaling SSM with:\n");
	printf("resx: %d\n", resx);
	printf("resy: %d\n", resy);
	printf("debug_mode: %d\n", params.debug_mode);

	name = "Transcaling";
	state_size = 3;
	curr_state.resize(state_size);
}

void Transcaling::setState(const VectorXd &ssm_state){
	validate_ssm_state(ssm_state);
	curr_state = ssm_state;
	getWarpFromState(curr_warp, curr_state);
	curr_pts.noalias() = curr_warp.topRows<2>() * init_pts_hm;
	curr_corners.noalias() = curr_warp.topRows<2>() * init_corners_hm;
}

void Transcaling::compositionalUpdate(const VectorXd& state_update){
	validate_ssm_state(state_update);

	getWarpFromState(warp_update_mat, state_update);
	curr_warp = curr_warp * warp_update_mat;

	getStateFromWarp(curr_state, curr_warp);

	curr_pts.noalias() = curr_warp.topRows<2>() * init_pts_hm;
	curr_corners.noalias() = curr_warp.topRows<2>() * init_corners_hm;
}

void Transcaling::getWarpFromState(Matrix3d &warp_mat,
	const VectorXd& ssm_state){
	validate_ssm_state(ssm_state);

	warp_mat = Matrix3d::Identity();
	warp_mat(0, 0) = warp_mat(1, 1) = 1 + ssm_state(2);
	warp_mat(0, 2) = ssm_state(0);
	warp_mat(1, 2) = ssm_state(1);
}

void Transcaling::getStateFromWarp(VectorXd &state_vec,
	const Matrix3d& sim_mat){
	validate_ssm_state(state_vec);
	VALIDATE_TRS_WARP(sim_mat);

	state_vec(0) = sim_mat(0, 2);
	state_vec(1) = sim_mat(1, 2);
	state_vec(2) = sim_mat(0, 0) - 1;
}

void Transcaling::getInitPixGrad(Matrix2Xd &dw_dp, int pt_id) {
	double x = init_pts(0, pt_id);
	double y = init_pts(1, pt_id);
	dw_dp <<
		1, 0, x,
		0, 1, y;
}

void Transcaling::cmptInitPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dx){
	validate_ssm_jacobian(dI_dp, dI_dx);

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; pt_id++){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = dI_dx(ch_pt_id, 0);
			double Iy = dI_dx(ch_pt_id, 1);

			dI_dp(ch_pt_id, 0) = Ix;
			dI_dp(ch_pt_id, 1) = Iy;
			dI_dp(ch_pt_id, 2) = Ix*x + Iy*y;
			++ch_pt_id;
		}
	}
}

void Transcaling::cmptApproxPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dx){
	validate_ssm_jacobian(dI_dp, dI_dx);
	double s_plus_1_inv = 1.0 / (curr_state(2) + 1);
	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; pt_id++){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		for(int ch_id = 0; ch_id < n_channels; ++ch_id){

			double Ix = dI_dx(ch_pt_id, 0);
			double Iy = dI_dx(ch_pt_id, 1);

			dI_dp(ch_pt_id, 0) = Ix * s_plus_1_inv;
			dI_dp(ch_pt_id, 1) = Iy * s_plus_1_inv;
			dI_dp(ch_pt_id, 2) = (Ix*x + Iy*y) * s_plus_1_inv;
			++ch_pt_id;
		}
	}
}

void Transcaling::cmptWarpedPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dx){
	validate_ssm_jacobian(dI_dp, dI_dx);
	double s = curr_state(2) + 1;

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; pt_id++){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = s*dI_dx(ch_pt_id, 0);
			double Iy = s*dI_dx(ch_pt_id, 1);

			dI_dp(ch_pt_id, 0) = Ix;
			dI_dp(ch_pt_id, 1) = Iy;
			dI_dp(ch_pt_id, 2) = Ix*x + Iy*y;
			dI_dp(ch_pt_id, 3) = Iy*x - Ix*y;
			++ch_pt_id;
		}
	}
}

void Transcaling::cmptInitPixHessian(MatrixXd &d2I_dp2, const PixHessT &d2I_dw2,
	const PixGradT &dI_dw){
	validate_ssm_hessian(d2I_dp2, d2I_dw2, dI_dw);

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; pt_id++){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		Matrix23d dw_dp;
		dw_dp <<
			1, 0, x,
			0, 1, y;
		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			Map<Matrix3d>(d2I_dp2.col(ch_pt_id).data()) = dw_dp.transpose()*Map<const Matrix2d>(d2I_dw2.col(ch_pt_id).data())*dw_dp;
			++ch_pt_id;
		}
	}
}
void Transcaling::cmptWarpedPixHessian(MatrixXd &d2I_dp2, const PixHessT &d2I_dw2,
	const PixGradT &dI_dw) {
	validate_ssm_hessian(d2I_dp2, d2I_dw2, dI_dw);
	double s = curr_state(2) + 1;
	double s2 = s*s;

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; ++pt_id) {
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		Matrix23d dw_dp;
		dw_dp <<
			1, 0, x,
			0, 1, y;

		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			Map<Matrix3d>(d2I_dp2.col(ch_pt_id).data()) = s2*dw_dp.transpose()*
				Map<const Matrix2d>(d2I_dw2.col(ch_pt_id).data())*dw_dp;
			++ch_pt_id;
		}
	}
}
void Transcaling::estimateWarpFromCorners(VectorXd &state_update, const Matrix24d &in_corners,
	const Matrix24d &out_corners){
	validate_ssm_state(state_update);

	Matrix3d warp_update_mat = utils::computeTranscalingDLT(in_corners, out_corners);
	getStateFromWarp(state_update, warp_update_mat);
}

void Transcaling::estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
	const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
	const EstimatorParams &est_params){
	cv::Mat trs_params = estimateTranscaling(in_pts, out_pts, mask, est_params);
	state_update(0) = trs_params.at<double>(0, 0);
	state_update(1) = trs_params.at<double>(0, 1);
	state_update(2) = trs_params.at<double>(0, 2) - 1;
}

void Transcaling::updateGradPts(double grad_eps){
	double scaled_eps = curr_warp(0, 0) * grad_eps;
	for(int pt_id = 0; pt_id < n_pts; pt_id++){
		spi_pt_check(spi_mask, pt_id);

		grad_pts(0, pt_id) = curr_pts(0, pt_id) + scaled_eps;
		grad_pts(1, pt_id) = curr_pts(1, pt_id);

		grad_pts(2, pt_id) = curr_pts(0, pt_id) - scaled_eps;
		grad_pts(3, pt_id) = curr_pts(1, pt_id);

		grad_pts(4, pt_id) = curr_pts(0, pt_id);
		grad_pts(5, pt_id) = curr_pts(1, pt_id) + scaled_eps;

		grad_pts(6, pt_id) = curr_pts(0, pt_id);
		grad_pts(7, pt_id) = curr_pts(1, pt_id) - scaled_eps;
	}
}


void Transcaling::updateHessPts(double hess_eps){
	double scaled_eps = curr_warp(0, 0) * hess_eps;
	double scaled_eps2 = 2 * scaled_eps;

	for(int pt_id = 0; pt_id < n_pts; pt_id++){

		spi_pt_check(spi_mask, pt_id);

		hess_pts(0, pt_id) = curr_pts(0, pt_id) + scaled_eps2;
		hess_pts(1, pt_id) = curr_pts(1, pt_id);

		hess_pts(2, pt_id) = curr_pts(0, pt_id) - scaled_eps2;
		hess_pts(3, pt_id) = curr_pts(1, pt_id);

		hess_pts(4, pt_id) = curr_pts(0, pt_id);
		hess_pts(5, pt_id) = curr_pts(1, pt_id) + scaled_eps2;

		hess_pts(6, pt_id) = curr_pts(0, pt_id);
		hess_pts(7, pt_id) = curr_pts(1, pt_id) - scaled_eps2;

		hess_pts(8, pt_id) = curr_pts(0, pt_id) + scaled_eps;
		hess_pts(9, pt_id) = curr_pts(1, pt_id) + scaled_eps;

		hess_pts(10, pt_id) = curr_pts(0, pt_id) - scaled_eps;
		hess_pts(11, pt_id) = curr_pts(1, pt_id) - scaled_eps;

		hess_pts(12, pt_id) = curr_pts(0, pt_id) + scaled_eps;
		hess_pts(13, pt_id) = curr_pts(1, pt_id) - scaled_eps;

		hess_pts(14, pt_id) = curr_pts(0, pt_id) - scaled_eps;
		hess_pts(15, pt_id) = curr_pts(1, pt_id) + scaled_eps;
	}
}

void Transcaling::applyWarpToCorners(Matrix24d &warped_corners, const Matrix24d &orig_corners,
	const VectorXd &state_update){
	warped_corners.noalias() = (orig_corners * (state_update(2) + 1)).colwise() + state_update.head<2>();
}
void Transcaling::applyWarpToPts(Matrix2Xd &warped_pts, const Matrix2Xd &orig_pts,
	const VectorXd &state_update){
	warped_pts.noalias() = (orig_pts * (state_update(2) + 1)).colwise() + state_update.head<2>();
}

_MTF_END_NAMESPACE

