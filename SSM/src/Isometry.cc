#include "mtf/SSM/Isometry.h"
#include "mtf/SSM/SSMEstimator.h"
#include "mtf/Utilities/warpUtils.h"
#include "mtf/Utilities/miscUtils.h"
#define _USE_MATH_DEFINES
#include <math.h>

_MTF_BEGIN_NAMESPACE

IsometryParams::IsometryParams(const SSMParams *ssm_params,
int _pt_based_sampling) :
SSMParams(ssm_params),
pt_based_sampling(_pt_based_sampling){}

IsometryParams::IsometryParams(const IsometryParams *params) :
SSMParams(params),
pt_based_sampling(ISO_PT_BASED_SAMPLING){
	if(params){
		pt_based_sampling = params->pt_based_sampling;
	}
}

Isometry::Isometry(const ParamType *_params) :
ProjectiveBase(_params), params(_params){

	printf("\n");
	printf("Using Isometry SSM with:\n");
	printf("resx: %d\n", resx);
	printf("resy: %d\n", resy);
	printf("pt_based_sampling: %d\n", params.pt_based_sampling);

	name = "isometry";
	state_size = 3;
	curr_state.resize(state_size);
}

void Isometry::compositionalUpdate(const VectorXd& state_update){
	validate_iso_state(state_update);

	getWarpFromState(warp_update_mat, state_update);
	curr_warp = curr_warp * warp_update_mat;

	getStateFromWarp(curr_state, curr_warp);

	curr_pts.noalias() = curr_warp.topRows<2>() * init_pts_hm;
	curr_corners.noalias() = curr_warp.topRows<2>() * init_corners_hm;
}

void Isometry::getWarpFromState(Matrix3d &warp_mat,
	const VectorXd& ssm_state){
	validate_iso_state(ssm_state);

	double tx = ssm_state(0);
	double ty = ssm_state(1);
	double cos_theta = cos(ssm_state(2));
	double sin_theta = sin(ssm_state(2));

	warp_mat(0, 0) = cos_theta;
	warp_mat(0, 1) = -sin_theta;
	warp_mat(0, 2) = tx;
	warp_mat(1, 0) = sin_theta;
	warp_mat(1, 1) = cos_theta;
	warp_mat(1, 2) = ty;
	warp_mat(2, 0) = 0;
	warp_mat(2, 1) = 0;
	warp_mat(2, 2) = 1;
}

void Isometry::getStateFromWarp(VectorXd &state_vec,
	const Matrix3d& iso_mat){
	validate_iso_state(state_vec);
	validate_iso_warp(iso_mat);

	state_vec(0) = iso_mat(0, 2);
	state_vec(1) = iso_mat(1, 2);
	//state_vec(2) = getAngleOfRotation(iso_mat(1, 0), iso_mat(0, 0));
	state_vec(2) = atan2(iso_mat(1, 0), iso_mat(0, 0));
}

void Isometry::getInitPixGrad(Matrix2Xd &ssm_grad, int pt_id) {
	double x = init_pts(0, pt_id);
	double y = init_pts(1, pt_id);
	ssm_grad <<
		1, 0, -y,
		0, 1, x;
}

void Isometry::getCurrPixGrad(Matrix2Xd &ssm_grad, int pt_id) {
	double curr_x = curr_pts(0, pt_id);
	double curr_y = curr_pts(1, pt_id);
	ssm_grad <<
		1, 0, -curr_y,
		0, 1, curr_x;
}

void Isometry::cmptInitPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dx){
	validate_ssm_jacobian(dI_dp, dI_dx);

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = dI_dx(ch_pt_id, 0);
			double Iy = dI_dx(ch_pt_id, 1);

			dI_dp(ch_pt_id, 0) = Ix;
			dI_dp(ch_pt_id, 1) = Iy;
			dI_dp(ch_pt_id, 2) = Iy*x - Ix*y;
			++ch_pt_id;
		}
	}
}

void Isometry::cmptPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dx){
	validate_ssm_jacobian(dI_dp, dI_dx);

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);
		//double x = init_pts(0, i);
		//double y = init_pts(1, i);

		double curr_x = curr_pts(0, pt_id);
		double curr_y = curr_pts(1, pt_id);

		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = dI_dx(ch_pt_id, 0);
			double Iy = dI_dx(ch_pt_id, 1);

			dI_dp(ch_pt_id, 0) = Ix;
			dI_dp(ch_pt_id, 1) = Iy;
			dI_dp(ch_pt_id, 2) = Iy*curr_x - Ix*curr_y;
			++ch_pt_id;
		}
	}
}

void Isometry::cmptWarpedPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dx){
	validate_ssm_jacobian(dI_dp, dI_dx);
	double cos_theta = curr_warp(0, 0);
	double sin_theta = curr_warp(1, 0);

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = cos_theta*dI_dx(ch_pt_id, 0) + sin_theta*dI_dx(ch_pt_id, 1);
			double Iy = cos_theta*dI_dx(ch_pt_id, 1) - sin_theta*dI_dx(ch_pt_id, 0);

			dI_dp(ch_pt_id, 0) = Ix;
			dI_dp(ch_pt_id, 1) = Iy;
			dI_dp(ch_pt_id, 2) = Iy*x - Ix*y;
			++ch_pt_id;
		}
	}
}

void Isometry::cmptApproxPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dx) {
	validate_ssm_jacobian(dI_dp, dI_dx);
	double cos_theta = curr_warp(0, 0);
	double sin_theta = curr_warp(1, 0);

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double curr_x = curr_pts(0, pt_id);
		double curr_y = curr_pts(1, pt_id);

		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = cos_theta*dI_dx(ch_pt_id, 0) - sin_theta*dI_dx(ch_pt_id, 1);
			double Iy = sin_theta*dI_dx(ch_pt_id, 0) + cos_theta*dI_dx(ch_pt_id, 1);

			dI_dp(ch_pt_id, 0) = Ix;
			dI_dp(ch_pt_id, 1) = Iy;
			dI_dp(ch_pt_id, 2) = Iy*curr_x - Ix*curr_y;
			++ch_pt_id;
		}
	}
}

void Isometry::cmptInitPixHessian(MatrixXd &d2I_dp2, const PixHessT &d2I_dx2,
	const PixGradT &dI_dx){
	validate_ssm_hessian(d2I_dp2, d2I_dx2, dI_dx);

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		Matrix23d dw_dp;
		dw_dp <<
			1, 0, -y,
			0, 1, x;
		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			Map<Matrix3d>(d2I_dp2.col(ch_pt_id).data()) = dw_dp.transpose()*
				Map<const Matrix2d>(d2I_dx2.col(ch_pt_id).data())*dw_dp;
			++ch_pt_id;
		}
	}
}

void Isometry::cmptPixHessian(MatrixXd &d2I_dp2, const PixHessT &d2I_dx2,
	const PixGradT &dI_dx){
	validate_ssm_hessian(d2I_dp2, d2I_dx2, dI_dx);

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double curr_x = curr_pts(0, pt_id);
		double curr_y = curr_pts(1, pt_id);
		Matrix23d dw_dp;
		dw_dp <<
			1, 0, -curr_y,
			0, 1, curr_x;
		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = dI_dx(ch_pt_id, 0);
			double Iy = dI_dx(ch_pt_id, 1);
			Map<Matrix3d> curr_pix_hess_ssm(d2I_dp2.col(ch_pt_id).data());
			curr_pix_hess_ssm = dw_dp.transpose()*Map<const Matrix2d>(d2I_dx2.col(ch_pt_id).data())*dw_dp;
			curr_pix_hess_ssm(2, 2) += Iy*curr_y - Ix*curr_x;
			++ch_pt_id;
		}
	}
}

void Isometry::cmptWarpedPixHessian(MatrixXd &d2I_dp2, const PixHessT &d2I_dw2,
	const PixGradT &dI_dw) {
	validate_ssm_hessian(d2I_dp2, d2I_dw2, dI_dw);
	double cos_theta = curr_warp(0, 0);
	double sin_theta = curr_warp(1, 0);
	Matrix2d dw_dx;
	dw_dx <<
		cos_theta, -sin_theta,
		sin_theta, cos_theta;

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; ++pt_id) {
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		Matrix23d dw_dp;
		dw_dp <<
			1, 0, -y,
			0, 1, x;

		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = dI_dw(ch_pt_id, 0);
			double Iy = dI_dw(ch_pt_id, 1);
			Map<Matrix3d> _d2I_dp2(d2I_dp2.col(pt_id).data());
			_d2I_dp2 = dw_dp.transpose()*dw_dx.transpose()*
				Map<const Matrix2d>(d2I_dw2.col(ch_pt_id).data())
				*dw_dx*dw_dp;

			double Ixx = Ix * x;
			double Iyy = Iy * y;
			_d2I_dp2(2, 2) -= (Iyy + Ixx);
			++ch_pt_id;
		}
	}
}

void Isometry::estimateWarpFromCorners(VectorXd &state_update, const Matrix24d &in_corners,
	const Matrix24d &out_corners){
	validate_ssm_state(state_update);

	Matrix3d warp_update_mat = utils::computeSimilitudeDLT(in_corners, out_corners);
	double a_plus_1 = warp_update_mat(0, 0);
	double b = warp_update_mat(1, 0);

	double s_plus_1 = sqrt(a_plus_1*a_plus_1 + b*b);
	double cos_theta = a_plus_1 / s_plus_1;
	double sin_theta = b / s_plus_1;

	//double theta = getAngleOfRotation(sin_theta, cos_theta);
	double theta = atan2(sin_theta, cos_theta);

	//double cos_theta_final = cos(theta);
	//double sin_theta_final = sin(theta);
	//if(cos_theta_final != sin_theta_final){
	//	printf("cos and sin theta are not same for theta:  %15.9f : \n", theta);
	//	printf("cos_theta: %15.9f\n", cos_theta_final);
	//	printf("sin_theta: %15.9f\n", sin_theta_final);
	//}

	state_update(0) = warp_update_mat(0, 2);
	state_update(1) = warp_update_mat(1, 2);
	state_update(2) = theta;
}

void Isometry::estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
	const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
	const EstimatorParams &est_params){
	cv::Mat sim_params = estimateIsometry(in_pts, out_pts, mask, est_params);
	state_update(0) = sim_params.at<double>(0, 0);
	state_update(1) = sim_params.at<double>(0, 1);
	state_update(2) = sim_params.at<double>(0, 2);
}

double Isometry::getAngleOfRotation(double sin_theta, double cos_theta){
	double theta = 0;
	if(cos_theta < 0){
		if(sin_theta < 0){// 3rd quadrant
			return 2 * M_PI - acos(cos_theta);
		} else{// 2nd quadrant
			return acos(cos_theta);
		}
	} else{// 1st or 4th quadrant

		return asin(sin_theta);
	}

}

void Isometry::updateGradPts(double grad_eps){
	Vector2d diff_vec_x_warped = curr_warp.topRows<2>().col(0) * grad_eps;
	Vector2d diff_vec_y_warped = curr_warp.topRows<2>().col(1) * grad_eps;

	for(int pt_id = 0; pt_id < n_pts; ++pt_id){
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


void Isometry::updateHessPts(double hess_eps){
	double hess_eps2 = 2 * hess_eps;

	Vector2d diff_vec_xx_warped = curr_warp.topRows<2>().col(0) * hess_eps2;
	Vector2d diff_vec_yy_warped = curr_warp.topRows<2>().col(1) * hess_eps2;
	Vector2d diff_vec_xy_warped = (curr_warp.topRows<2>().col(0) + curr_warp.topRows<2>().col(1)) * hess_eps;
	Vector2d diff_vec_yx_warped = (curr_warp.topRows<2>().col(0) - curr_warp.topRows<2>().col(1)) * hess_eps;

	for(int pt_id = 0; pt_id < n_pts; ++pt_id){
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

void Isometry::setState(const VectorXd &ssm_state){
	validate_ssm_state(ssm_state);
	curr_state = ssm_state;
	getWarpFromState(curr_warp, curr_state);
	curr_pts.noalias() = curr_warp.topRows<2>() * init_pts_hm;
	curr_corners.noalias() = curr_warp.topRows<2>() * init_corners_hm;
}

void Isometry::applyWarpToCorners(Matrix24d &warped_corners, const Matrix24d &orig_corners,
	const VectorXd &ssm_state){
	getWarpFromState(warp_mat, ssm_state);
	for(int corner_id = 0; corner_id < 4; ++corner_id){
		warped_corners(0, corner_id) = warp_mat(0, 0)*orig_corners(0, corner_id) + warp_mat(0, 1)*orig_corners(1, corner_id) +
			warp_mat(0, 2);
		warped_corners(1, corner_id) = warp_mat(1, 0)*orig_corners(0, corner_id) + warp_mat(1, 1)*orig_corners(1, corner_id) +
			warp_mat(1, 2);
	}
}

void Isometry::applyWarpToPts(Matrix2Xd &warped_pts, const Matrix2Xd &orig_pts,
	const VectorXd &ssm_state){
	getWarpFromState(warp_mat, ssm_state);
	int n_pts = orig_pts.cols();
	for(int pt_id = 0; pt_id < n_pts; ++pt_id){
		warped_pts(0, pt_id) = warp_mat(0, 0)*orig_pts(0, pt_id) + warp_mat(0, 1)*orig_pts(1, pt_id) +
			warp_mat(0, 2);
		warped_pts(1, pt_id) = warp_mat(1, 0)*orig_pts(0, pt_id) + warp_mat(1, 1)*orig_pts(1, pt_id) +
			warp_mat(1, 2);
	}
}

void Isometry::generatePerturbation(VectorXd &perturbation){
	assert(perturbation.size() == state_size);
	if(params.pt_based_sampling){
		PtsT orig_pts, perturbed_pts;
		orig_pts.resize(Eigen::NoChange, 2);
		perturbed_pts.resize(Eigen::NoChange, 2);
		orig_pts.col(0) = init_corners.col(0);
		orig_pts.col(1) = init_corners.col(2);
		if(params.pt_based_sampling == 1){
			perturbed_pts(0, 0) = orig_pts(0, 0) + rand_dist[0](rand_gen[0]);
			perturbed_pts(1, 0) = orig_pts(1, 0) + rand_dist[0](rand_gen[0]);
			perturbed_pts(0, 1) = orig_pts(0, 1) + rand_dist[0](rand_gen[0]);
			perturbed_pts(1, 1) = orig_pts(1, 1) + rand_dist[0](rand_gen[0]);
		} else {
			//! different perturbation for x,y coordinates of each corner
			//! followed by consistent translational perturbation to all corners
			perturbed_pts(0, 0) = orig_pts(0, 0) + rand_dist[1](rand_gen[1]);
			perturbed_pts(1, 0) = orig_pts(1, 0) + rand_dist[1](rand_gen[1]);
			perturbed_pts(0, 1) = orig_pts(0, 1) + rand_dist[1](rand_gen[1]);
			perturbed_pts(1, 1) = orig_pts(1, 1) + rand_dist[1](rand_gen[1]);
			perturbed_pts = perturbed_pts.colwise() + Vector2d(rand_dist[0](rand_gen[0]), rand_dist[0](rand_gen[0]));
		}
		perturbation = utils::computeIsometryDLT(orig_pts, perturbed_pts);

		//PtsT perturbed_pts_rec(2, 2);
		//applyWarpToPts(perturbed_pts_rec, orig_pts, perturbation);
		//double perturbed_rec_norm = (perturbed_pts_rec - orig_pts).norm();
		//double perturbed_norm = (perturbed_pts - orig_pts).norm();
		//double rec_norm = (perturbed_pts - perturbed_pts_rec).norm();

		//printf("perturbed_norm: %12.8f perturbed_rec_norm: %12.8f rec_norm: %12.8f\n",
		//	perturbed_norm, perturbed_rec_norm, rec_norm);

		//utils::printMatrix(init_corners, "init_corners");
	} else{
		ProjectiveBase::generatePerturbation(perturbation);
	}
}

_MTF_END_NAMESPACE

