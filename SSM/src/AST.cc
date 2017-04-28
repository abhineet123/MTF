#include "mtf/SSM/AST.h"
#include "mtf/SSM/SSMEstimator.h"
#include "mtf/Utilities/warpUtils.h"
#include "mtf/Utilities/miscUtils.h"
#include "opencv2/core/core_c.h"
#include "opencv2/calib3d/calib3d.hpp"


_MTF_BEGIN_NAMESPACE

ASTParams::ASTParams(const SSMParams *ssm_params,
bool _debug_mode) :
SSMParams(ssm_params),
debug_mode(_debug_mode){}

ASTParams::ASTParams(const ASTParams *params) :
SSMParams(params),
debug_mode(AST_DEBUG_MODE){
	if(params){
		debug_mode = params->debug_mode;
	}
}

AST::AST( const ParamType *_params) :
ProjectiveBase(_params), params(_params){

	printf("\n");
	printf("Using Anisotropic Scaling and Translation SSM with:\n");
	printf("resx: %d\n", resx);
	printf("resy: %d\n", resy);
	printf("debug_mode: %d\n", params.debug_mode);

	name = "ast";
	state_size = 4;
	curr_state.resize(state_size);
}

void AST::setState(const VectorXd &ssm_state){
	validate_ssm_state(ssm_state);
	curr_state = ssm_state;
	getWarpFromState(curr_warp, curr_state);
	curr_pts.noalias() = curr_warp.topRows<2>() * init_pts_hm;
	curr_corners.noalias() = curr_warp.topRows<2>() * init_corners_hm;
}

void AST::compositionalUpdate(const VectorXd& state_update){
	validate_ssm_state(state_update);

	getWarpFromState(warp_update_mat, state_update);
	curr_warp = curr_warp * warp_update_mat;

	getStateFromWarp(curr_state, curr_warp);

	curr_pts.noalias() = curr_warp.topRows<2>() * init_pts_hm;
	curr_corners.noalias() = curr_warp.topRows<2>() * init_corners_hm;
}

void AST::getWarpFromState(Matrix3d &warp_mat,
	const VectorXd& ssm_state){
	validate_ssm_state(ssm_state);

	warp_mat = Matrix3d::Identity();
	warp_mat(0, 0) =  1 + ssm_state(2);
	warp_mat(1, 1) = 1 + ssm_state(3);
	warp_mat(0, 2) = ssm_state(0);
	warp_mat(1, 2) = ssm_state(1);
}

void AST::getStateFromWarp(VectorXd &state_vec,
	const Matrix3d& ast_mat){
	validate_ssm_state(state_vec);
	VALIDATE_AST_WARP(ast_mat);

	state_vec(0) = ast_mat(0, 2);
	state_vec(1) = ast_mat(1, 2);
	state_vec(2) = ast_mat(0, 0) - 1;
	state_vec(3) = ast_mat(1, 1) - 1;
}

void AST::getInitPixGrad(Matrix2Xd &dw_dp, int pt_id) {
	double x = init_pts(0, pt_id);
	double y = init_pts(1, pt_id);
	dw_dp <<
		1, 0, x, 0,
		0, 1, 0, y;
}

void AST::cmptInitPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dx){
	validate_ssm_jacobian(dI_dp, dI_dx);

	unsigned int ch_pt_id = 0;
	for(unsigned int pt_id = 0; pt_id < n_pts; pt_id++){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		for(unsigned int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = dI_dx(ch_pt_id, 0);
			double Iy = dI_dx(ch_pt_id, 1);

			dI_dp(ch_pt_id, 0) = Ix;
			dI_dp(ch_pt_id, 1) = Iy;
			dI_dp(ch_pt_id, 2) = Ix*x;
			dI_dp(ch_pt_id, 3) = Iy*y;
			++ch_pt_id;
		}
	}
}

void AST::cmptApproxPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dx){
	validate_ssm_jacobian(dI_dp, dI_dx);
	double sx_plus_1_inv = 1.0 / (curr_state(2) + 1);
	double sy_plus_1_inv = 1.0 / (curr_state(3) + 1);
	unsigned int ch_pt_id = 0;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		for(unsigned int ch_id = 0; ch_id < n_channels; ++ch_id){

			double Ix = sx_plus_1_inv * dI_dx(ch_pt_id, 0);
			double Iy = sy_plus_1_inv * dI_dx(ch_pt_id, 1);

			dI_dp(ch_pt_id, 0) = Ix;
			dI_dp(ch_pt_id, 1) = Iy;
			dI_dp(ch_pt_id, 2) = Ix*x;
			dI_dp(ch_pt_id, 3) = Iy*y;
			++ch_pt_id;
		}
	}
}

void AST::cmptWarpedPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dx){
	validate_ssm_jacobian(dI_dp, dI_dx);
	double sx = curr_state(2) + 1;
	double sy = curr_state(3) + 1;

	unsigned int ch_pt_id = 0;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		for(unsigned int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = sx*dI_dx(ch_pt_id, 0);
			double Iy = sy*dI_dx(ch_pt_id, 1);

			dI_dp(ch_pt_id, 0) = Ix;
			dI_dp(ch_pt_id, 1) = Iy;
			dI_dp(ch_pt_id, 2) = Ix*x;
			dI_dp(ch_pt_id, 3) = Iy*y;
			++ch_pt_id;
		}
	}
}

void AST::cmptInitPixHessian(MatrixXd &d2I_dp2, const PixHessT &d2I_dw2,
	const PixGradT &dI_dw){
	validate_ssm_hessian(d2I_dp2, d2I_dw2, dI_dw);

	unsigned int ch_pt_id = 0;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		Matrix24d dw_dp;
		dw_dp <<
			1, 0, x, 0,
			0, 1, 0, y;
		for(unsigned int ch_id = 0; ch_id < n_channels; ++ch_id){
			Map<Matrix4d>(d2I_dp2.col(ch_pt_id).data()) = dw_dp.transpose()*Map<const Matrix2d>(d2I_dw2.col(ch_pt_id).data())*dw_dp;
			++ch_pt_id;
		}
	}
}
void AST::cmptWarpedPixHessian(MatrixXd &d2I_dp2, const PixHessT &d2I_dw2,
	const PixGradT &dI_dw) {
	validate_ssm_hessian(d2I_dp2, d2I_dw2, dI_dw);
	double s = curr_state(2) + 1;
	double s2 = s*s;

	unsigned int ch_pt_id = 0;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id) {
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		Matrix24d dw_dp;
		dw_dp <<
			1, 0, x, 0,
			0, 1, 0, y;

		for(unsigned int ch_id = 0; ch_id < n_channels; ++ch_id){
			Map<Matrix4d>(d2I_dp2.col(ch_pt_id).data()) = s2*dw_dp.transpose()*
				Map<const Matrix2d>(d2I_dw2.col(ch_pt_id).data())*dw_dp;
			++ch_pt_id;
		}
	}
}
void AST::estimateWarpFromCorners(VectorXd &state_update, const Matrix24d &in_corners,
	const Matrix24d &out_corners){
	validate_ssm_state(state_update);

	Matrix3d warp_update_mat = utils::computeASTDLT(in_corners, out_corners);
	getStateFromWarp(state_update, warp_update_mat);
}

void AST::estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
	const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
	const EstimatorParams &est_params){
	assert(state_update.size() == state_size);
	cv::Mat ast_params = estimateAST(in_pts, out_pts, mask, est_params);
	state_update(0) = ast_params.at<double>(0, 0);
	state_update(1) = ast_params.at<double>(0, 1);
	state_update(2) = ast_params.at<double>(0, 2) - 1;
	state_update(3) = ast_params.at<double>(0, 3) - 1;
}

void AST::updateGradPts(double grad_eps){
	double scaled_eps_x = curr_warp(0, 0) * grad_eps;
	double scaled_eps_y = curr_warp(1, 1) * grad_eps;

	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check(spi_mask, pt_id);

		grad_pts(0, pt_id) = curr_pts(0, pt_id) + scaled_eps_x;
		grad_pts(1, pt_id) = curr_pts(1, pt_id);

		grad_pts(2, pt_id) = curr_pts(0, pt_id) - scaled_eps_x;
		grad_pts(3, pt_id) = curr_pts(1, pt_id);

		grad_pts(4, pt_id) = curr_pts(0, pt_id);
		grad_pts(5, pt_id) = curr_pts(1, pt_id) + scaled_eps_y;

		grad_pts(6, pt_id) = curr_pts(0, pt_id);
		grad_pts(7, pt_id) = curr_pts(1, pt_id) - scaled_eps_y;
	}
}


void AST::updateHessPts(double hess_eps){
	double scaled_eps_x = curr_warp(0, 0) * hess_eps;
	double scaled_eps_y = curr_warp(1, 1) * hess_eps;
	double scaled_eps_x2 = 2 * scaled_eps_x;
	double scaled_eps_y2 = 2 * scaled_eps_y;


	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){

		spi_pt_check(spi_mask, pt_id);

		hess_pts(0, pt_id) = curr_pts(0, pt_id) + scaled_eps_x2;
		hess_pts(1, pt_id) = curr_pts(1, pt_id);

		hess_pts(2, pt_id) = curr_pts(0, pt_id) - scaled_eps_x2;
		hess_pts(3, pt_id) = curr_pts(1, pt_id);

		hess_pts(4, pt_id) = curr_pts(0, pt_id);
		hess_pts(5, pt_id) = curr_pts(1, pt_id) + scaled_eps_y2;

		hess_pts(6, pt_id) = curr_pts(0, pt_id);
		hess_pts(7, pt_id) = curr_pts(1, pt_id) - scaled_eps_y2;

		hess_pts(8, pt_id) = curr_pts(0, pt_id) + scaled_eps_x;
		hess_pts(9, pt_id) = curr_pts(1, pt_id) + scaled_eps_y;

		hess_pts(10, pt_id) = curr_pts(0, pt_id) - scaled_eps_x;
		hess_pts(11, pt_id) = curr_pts(1, pt_id) - scaled_eps_y;

		hess_pts(12, pt_id) = curr_pts(0, pt_id) + scaled_eps_x;
		hess_pts(13, pt_id) = curr_pts(1, pt_id) - scaled_eps_y;

		hess_pts(14, pt_id) = curr_pts(0, pt_id) - scaled_eps_x;
		hess_pts(15, pt_id) = curr_pts(1, pt_id) + scaled_eps_y;
	}
}

void AST::applyWarpToCorners(Matrix24d &warped_corners, const Matrix24d &orig_corners,
	const VectorXd &state_update){

	warped_corners.row(0) = (orig_corners.row(0).array() * (state_update(2) + 1)) + state_update(0);
	warped_corners.row(1) = (orig_corners.row(1).array() * (state_update(3) + 1)) + state_update(1);
}
void AST::applyWarpToPts(Matrix2Xd &warped_pts, const Matrix2Xd &orig_pts,
	const VectorXd &state_update){
	warped_pts.row(0) = (orig_pts.row(0).array() * (state_update(2) + 1)) + state_update(0);
	warped_pts.row(1) = (orig_pts.row(1).array() * (state_update(3) + 1)) + state_update(1);
}


cv::Mat AST::estimateAST(cv::InputArray _in_pts, cv::InputArray _out_pts,
	cv::OutputArray _mask, const SSMEstimatorParams &params){
	cv::Mat in_pts = _in_pts.getMat(), out_pts = _out_pts.getMat();
	int n_pts = in_pts.checkVector(2);
	CV_Assert(n_pts >= 0 && out_pts.checkVector(2) == n_pts &&
		in_pts.type() == out_pts.type());

	cv::Mat H(1, 4, CV_64F);
	CvMat _pt1 = in_pts, _pt2 = out_pts;
	CvMat matH = H, c_mask, *p_mask = 0;
	if(_mask.needed()){
		_mask.create(n_pts, 1, CV_8U, -1, true);
		p_mask = &(c_mask = _mask.getMat());
	}
	bool ok = estimateAST(&_pt1, &_pt2, &matH, p_mask, params) > 0;
	if(!ok)
		H = cv::Scalar(0);
	return H;
}

int	AST::estimateAST(const CvMat* in_pts, const CvMat* out_pts,
	CvMat* __H, CvMat* mask, const SSMEstimatorParams &params) {
	bool result = false;
	cv::Ptr<CvMat> out_pts_hm, in_pts_hm, tempMask;

	double H[4];
	CvMat matH = cvMat(1, 4, CV_64FC1, H);

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

	ASTEstimator estimator(params.n_model_pts, params.use_boost_rng);

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



ASTEstimator::ASTEstimator(int _modelPoints, bool _use_boost_rng)
	: SSMEstimator(_modelPoints, cvSize(4, 1), 1, _use_boost_rng) {
	assert(_modelPoints >= 2);
	checkPartialSubsets = false;
}

int ASTEstimator::runKernel(const CvMat* m1, const CvMat* m2, CvMat* H) {
	int n_pts = m1->rows * m1->cols;

	//if(n_pts != 3) {
	//    throw invalid_argument(cv::format("Invalid no. of points: %d provided", n_pts));
	//}
	const CvPoint2D64f* M = (const CvPoint2D64f*)m1->data.ptr;
	const CvPoint2D64f* m = (const CvPoint2D64f*)m2->data.ptr;

	Matrix2Xd in_pts, out_pts;
	in_pts.resize(Eigen::NoChange, n_pts);
	out_pts.resize(Eigen::NoChange, n_pts);
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id) {
		in_pts(0, pt_id) = M[pt_id].x;
		in_pts(1, pt_id) = M[pt_id].y;

		out_pts(0, pt_id) = m[pt_id].x;
		out_pts(1, pt_id) = m[pt_id].y;
	}
	Matrix3d ast_mat = utils::computeASTDLT(in_pts, out_pts);

	double *H_ptr = H->data.db;
	H_ptr[0] = ast_mat(0, 2);
	H_ptr[1] = ast_mat(1, 2);
	H_ptr[2] = ast_mat(0, 0);
	H_ptr[3] = ast_mat(1, 1);
	return 1;
}


void ASTEstimator::computeReprojError(const CvMat* m1, const CvMat* m2,
	const CvMat* model, CvMat* _err) {
	int n_pts = m1->rows * m1->cols;
	const CvPoint2D64f* M = (const CvPoint2D64f*)m1->data.ptr;
	const CvPoint2D64f* m = (const CvPoint2D64f*)m2->data.ptr;
	const double* H = model->data.db;
	float* err = _err->data.fl;

	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id) {
		double dx = (H[2] * M[pt_id].x + H[0]) - m[pt_id].x;
		double dy = (H[2] * M[pt_id].y + H[1]) - m[pt_id].y;
		err[pt_id] = (float)(dx * dx + dy * dy);
	}
}

bool ASTEstimator::refine(const CvMat* m1, const CvMat* m2,
	CvMat* model, int maxIters) {
	LevMarq solver(4, 0, cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, maxIters, DBL_EPSILON));
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

		for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id)	{
			const double* h = _param->data.db;
			double Mx = M[pt_id].x, My = M[pt_id].y;
			double _xi = (h[2] * Mx + h[0]);
			double _yi = (h[3] * My + h[1]);
			double err[] = { _xi - m[pt_id].x, _yi - m[pt_id].y };
			if(_JtJ || _JtErr) {
				double J[][4] = {
					{ 1, 0, Mx, 0 },
					{ 0, 1, 0, My }
				};
				for(unsigned int j = 0; j < 4; j++) {
					for(unsigned int k = j; k < 4; k++)
						_JtJ->data.db[j * 4 + k] += J[0][j] * J[0][k] + J[1][j] * J[1][k];
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

