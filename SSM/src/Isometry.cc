#include "mtf/SSM/Isometry.h"
#include "mtf/Utilities/warpUtils.h"
#include "mtf/Utilities/miscUtils.h"
#include "opencv2/core/core_c.h"
#include "opencv2/calib3d/calib3d.hpp"
#define _USE_MATH_DEFINES
#include <math.h>

#ifdef _WIN32
# define M_PI 3.14159265358979323846  /* pi */
#endif

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
			dI_dp(ch_pt_id, 2) = Iy*x - Ix*y;
			++ch_pt_id;
		}
	}
}

void Isometry::cmptPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dx){
	validate_ssm_jacobian(dI_dp, dI_dx);

	unsigned int ch_pt_id = 0;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);
		//double x = init_pts(0, i);
		//double y = init_pts(1, i);

		double curr_x = curr_pts(0, pt_id);
		double curr_y = curr_pts(1, pt_id);

		for(unsigned int ch_id = 0; ch_id < n_channels; ++ch_id){
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

	unsigned int ch_pt_id = 0;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		for(unsigned int ch_id = 0; ch_id < n_channels; ++ch_id){
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

	unsigned int ch_pt_id = 0;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double curr_x = curr_pts(0, pt_id);
		double curr_y = curr_pts(1, pt_id);

		for(unsigned int ch_id = 0; ch_id < n_channels; ++ch_id){
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

	unsigned int ch_pt_id = 0;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		Matrix23d dw_dp;
		dw_dp <<
			1, 0, -y,
			0, 1, x;
		for(unsigned int ch_id = 0; ch_id < n_channels; ++ch_id){
			Map<Matrix3d>(d2I_dp2.col(ch_pt_id).data()) = dw_dp.transpose()*
				Map<const Matrix2d>(d2I_dx2.col(ch_pt_id).data())*dw_dp;
			++ch_pt_id;
		}
	}
}

void Isometry::cmptPixHessian(MatrixXd &d2I_dp2, const PixHessT &d2I_dx2,
	const PixGradT &dI_dx){
	validate_ssm_hessian(d2I_dp2, d2I_dx2, dI_dx);

	unsigned int ch_pt_id = 0;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double curr_x = curr_pts(0, pt_id);
		double curr_y = curr_pts(1, pt_id);
		Matrix23d dw_dp;
		dw_dp <<
			1, 0, -curr_y,
			0, 1, curr_x;
		for(unsigned int ch_id = 0; ch_id < n_channels; ++ch_id){
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

	unsigned int ch_pt_id = 0;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id) {
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		Matrix23d dw_dp;
		dw_dp <<
			1, 0, -y,
			0, 1, x;

		for(unsigned int ch_id = 0; ch_id < n_channels; ++ch_id){
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


void Isometry::updateHessPts(double hess_eps){
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
	for(unsigned int corner_id = 0; corner_id < 4; ++corner_id){
		warped_corners(0, corner_id) = warp_mat(0, 0)*orig_corners(0, corner_id) + warp_mat(0, 1)*orig_corners(1, corner_id) +
			warp_mat(0, 2);
		warped_corners(1, corner_id) = warp_mat(1, 0)*orig_corners(0, corner_id) + warp_mat(1, 1)*orig_corners(1, corner_id) +
			warp_mat(1, 2);
	}
}

void Isometry::applyWarpToPts(Matrix2Xd &warped_pts, const Matrix2Xd &orig_pts,
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


cv::Mat Isometry::estimateIsometry(cv::InputArray _in_pts, cv::InputArray _out_pts,
	cv::OutputArray _mask, const SSMEstimatorParams &params){
	cv::Mat in_pts = _in_pts.getMat(), out_pts = _out_pts.getMat();
	int n_pts = in_pts.checkVector(2);
	CV_Assert(n_pts >= 0 && out_pts.checkVector(2) == n_pts &&
		in_pts.type() == out_pts.type());

	cv::Mat H(1, 3, CV_64F);
	CvMat _pt1 = in_pts, _pt2 = out_pts;
	CvMat matH = H, c_mask, *p_mask = 0;
	if(_mask.needed()){
		_mask.create(n_pts, 1, CV_8U, -1, true);
		p_mask = &(c_mask = _mask.getMat());
	}
	bool ok = estimateIsometry(&_pt1, &_pt2, &matH, p_mask, params) > 0;
	if(!ok)
		H = cv::Scalar(0);
	return H;
}

int	Isometry::estimateIsometry(const CvMat* in_pts, const CvMat* out_pts,
	CvMat* __H, CvMat* mask, const SSMEstimatorParams &params) {
	bool result = false;
	cv::Ptr<CvMat> out_pts_hm, in_pts_hm, tempMask;

	double H[3];
	CvMat matH = cvMat(1, 3, CV_64FC1, H);

	CV_Assert(CV_IS_MAT(out_pts) && CV_IS_MAT(in_pts));

	int n_pts = MAX(out_pts->cols, out_pts->rows);
	CV_Assert(n_pts >= params.n_model_pts);

	out_pts_hm = cvCreateMat(1, n_pts, CV_64FC2);
	cvConvertPointsHomogeneous(out_pts, out_pts_hm);

	in_pts_hm = cvCreateMat(1, n_pts, CV_64FC2);
	cvConvertPointsHomogeneous(in_pts, in_pts_hm);

	if(mask) {
		CV_Assert(CV_IS_MASK_ARR(mask) && CV_IS_MAT_CONT(mask->type) &&
			(mask->rows == 1 || mask->cols == 1) &&
			mask->rows * mask->cols == n_pts);
	}
	if(mask || n_pts > params.n_model_pts)
		tempMask = cvCreateMat(1, n_pts, CV_8U);
	if(!tempMask.empty())
		cvSet(tempMask, cvScalarAll(1.));

	IsometryEstimator estimator(params.n_model_pts, params.use_boost_rng);

	int method = n_pts == params.n_model_pts ? 0 : params.method_cv;

	if(method == CV_LMEDS)
		result = estimator.runLMeDS(in_pts_hm, out_pts_hm, &matH, tempMask, params.confidence,
		params.max_iters, params.max_subset_attempts);
	else if(method == CV_RANSAC)
		result = estimator.runRANSAC(in_pts_hm, out_pts_hm, &matH, tempMask, params.ransac_reproj_thresh,
		params.confidence, params.max_iters, params.max_subset_attempts);
	else
		result = estimator.runKernel(in_pts_hm, out_pts_hm, &matH) > 0;

	if(result && n_pts > params.n_model_pts) {
		utils::icvCompressPoints((CvPoint2D64f*)in_pts_hm->data.ptr, tempMask->data.ptr, 1, n_pts);
		n_pts = utils::icvCompressPoints((CvPoint2D64f*)out_pts_hm->data.ptr, tempMask->data.ptr, 1, n_pts);
		in_pts_hm->cols = out_pts_hm->cols = n_pts;
		if(method == CV_RANSAC)
			estimator.runKernel(in_pts_hm, out_pts_hm, &matH);
		if(params.refine){
			estimator.refine(in_pts_hm, out_pts_hm, &matH, params.lm_max_iters);
		}
	}

	if(result)
		cvConvert(&matH, __H);

	if(mask && tempMask) {
		if(CV_ARE_SIZES_EQ(mask, tempMask))
			cvCopy(tempMask, mask);
		else
			cvTranspose(tempMask, mask);
	}

	return (int)result;
}



IsometryEstimator::IsometryEstimator(int _modelPoints, bool _use_boost_rng)
	: SSMEstimator(_modelPoints, cvSize(3, 1), 1, _use_boost_rng) {
	assert(_modelPoints >= 2);
	checkPartialSubsets = false;
}

int IsometryEstimator::runKernel(const CvMat* m1, const CvMat* m2, CvMat* H) {
	int n_pts = m1->rows * m1->cols;

	//if(n_pts != 3) {
	//    throw invalid_argument(cv::format("Invalid no. of points: %d provided", n_pts));
	//}
	const CvPoint2D64f* M = (const CvPoint2D64f*)m1->data.ptr;
	const CvPoint2D64f* m = (const CvPoint2D64f*)m2->data.ptr;

	Matrix2Xd in_pts, out_pts;
	in_pts.resize(Eigen::NoChange, n_pts);
	out_pts.resize(Eigen::NoChange, n_pts);
	for(int pt_id = 0; pt_id < n_pts; pt_id++) {
		in_pts(0, pt_id) = M[pt_id].x;
		in_pts(1, pt_id) = M[pt_id].y;

		out_pts(0, pt_id) = m[pt_id].x;
		out_pts(1, pt_id) = m[pt_id].y;
	}
	Vector3d iso_params = utils::computeIsometryDLT(in_pts, out_pts);

	double *H_ptr = H->data.db;
	H_ptr[0] = iso_params(0);
	H_ptr[1] = iso_params(1);
	H_ptr[2] = iso_params(2);
	return 1;
}


void IsometryEstimator::computeReprojError(const CvMat* m1, const CvMat* m2,
	const CvMat* model, CvMat* _err) {
	int n_pts = m1->rows * m1->cols;
	const CvPoint2D64f* M = (const CvPoint2D64f*)m1->data.ptr;
	const CvPoint2D64f* m = (const CvPoint2D64f*)m2->data.ptr;
	const double* H = model->data.db;
	float* err = _err->data.fl;

	double cos_theta = cos(H[2]), sin_theta = sin(H[2]);
	for(int pt_id = 0; pt_id < n_pts; pt_id++) {
		double dx = (cos_theta * M[pt_id].x - sin_theta * M[pt_id].y + H[0]) - m[pt_id].x;
		double dy = (sin_theta * M[pt_id].x + cos_theta * M[pt_id].y + H[1]) - m[pt_id].y;
		err[pt_id] = (float)(dx * dx + dy * dy);
	}
}

bool IsometryEstimator::refine(const CvMat* m1, const CvMat* m2,
	CvMat* model, int maxIters) {
	LevMarq solver(3, 0, cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, maxIters, DBL_EPSILON));
	int n_pts = m1->rows * m1->cols;
	const CvPoint2D64f* M = (const CvPoint2D64f*)m1->data.ptr;
	const CvPoint2D64f* m = (const CvPoint2D64f*)m2->data.ptr;
	CvMat modelPart = cvMat(solver.param->rows, solver.param->cols, model->type, model->data.ptr);
	cvCopy(&modelPart, solver.param);

	for(;;)	{
		const CvMat* _param = 0;
		CvMat *_JtJ = 0, *_JtErr = 0;
		double* _errNorm = 0;

		if(!solver.updateAlt(_param, _JtJ, _JtErr, _errNorm))
			break;
		const double* h = _param->data.db;
		double cos_theta = cos(h[2]), sin_theta = sin(h[2]);
		for(int pt_id = 0; pt_id < n_pts; pt_id++)	{
			double Mx = M[pt_id].x, My = M[pt_id].y;
			double _xi = (cos_theta * Mx - sin_theta * My + h[0]);
			double _yi = (sin_theta * Mx + cos_theta * My + h[1]);
			double err[] = { _xi - m[pt_id].x, _yi - m[pt_id].y };
			if(_JtJ || _JtErr) {
				double J[][3] = {
					{ 1, 0, -_yi },
					{ 0, 1, _xi }
				};
				for(unsigned int j = 0; j < 3; j++) {
					for(unsigned int k = j; k < 3; k++)
						_JtJ->data.db[j * 3 + k] += J[0][j] * J[0][k] + J[1][j] * J[1][k];
					_JtErr->data.db[j] += J[0][j] * err[0] + J[1][j] * err[1];
				}
			}
			if(_errNorm)
				*_errNorm += err[0] * err[0] + err[1] * err[1];
		}
	}

	cvCopy(solver.param, &modelPart);
	return true;
}

_MTF_END_NAMESPACE

