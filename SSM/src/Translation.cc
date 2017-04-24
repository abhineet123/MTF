#include "mtf/SSM/Translation.h"
#include "mtf/SSM/SSMEstimator.h"
#include "mtf/Utilities/warpUtils.h"
#include "mtf/Utilities/miscUtils.h"
#include "opencv2/core/core_c.h"
#include "opencv2/calib3d/calib3d.hpp"

#ifndef CV_LMEDS
#define CV_LMEDS 4
#endif
#ifndef CV_RANSAC
#define CV_RANSAC 8
#endif

_MTF_BEGIN_NAMESPACE

TranslationParams::TranslationParams(const SSMParams *ssm_params,
bool _debug_mode) :
SSMParams(ssm_params),
debug_mode(_debug_mode){}

TranslationParams::TranslationParams(const TranslationParams *params) :
SSMParams(params),
debug_mode(TRANS_DEBUG_MODE){
	if(params){
		debug_mode = params->debug_mode;
	}
}

Translation::Translation(
const ParamType *_params) : ProjectiveBase(_params),
params(_params){

	printf("\n");
	printf("Using Translation SSM with:\n");
	printf("resx: %d\n", resx);
	printf("resy: %d\n", resy);
	printf("debug_mode: %d\n", params.debug_mode);

	name = "translation";
	state_size = 2;

	curr_state.resize(state_size);

	identity_jacobian = true;
}

void Translation::setCorners(const CornersT& corners){
	curr_corners = corners;
	getPtsFromCorners(curr_warp, curr_pts, curr_pts_hm, curr_corners);

	init_corners = curr_corners;
	init_pts = curr_pts;

	curr_warp = Matrix3d::Identity();
	curr_state.fill(0);
}

void Translation::setState(const VectorXd &ssm_state){
	validate_ssm_state(ssm_state);
	curr_state = ssm_state;
	getWarpFromState(curr_warp, curr_state);

	curr_pts = init_pts.colwise() + curr_state;
	curr_corners = init_corners.colwise() + curr_state;
}

void Translation::compositionalUpdate(const VectorXd& state_update){
	validate_ssm_state(state_update);

	curr_state += state_update;
	curr_warp(0, 2) = curr_state(0);
	curr_warp(1, 2) = curr_state(1);

	curr_pts = curr_pts.colwise() + state_update;
	curr_corners = curr_corners.colwise() + state_update;
}

void Translation::getWarpFromState(Matrix3d &warp_mat,
	const VectorXd& ssm_state){
	validate_ssm_state(ssm_state);

	warp_mat.setIdentity();
	warp_mat(0, 2) = ssm_state(0);
	warp_mat(1, 2) = ssm_state(1);
}

void Translation::getStateFromWarp(VectorXd &state_vec,
	const Matrix3d& warp_mat){
	VALIDATE_TRANS_WARP(warp_mat);

	state_vec(0) = warp_mat(0, 2);
	state_vec(1) = warp_mat(1, 2);
}

void Translation::invertState(VectorXd& inv_state, const VectorXd& state){
	inv_state = -state;
}

void Translation::updateGradPts(double grad_eps){

	for(int pix_id = 0; pix_id < n_pts; pix_id++){
		grad_pts(0, pix_id) = curr_pts(0, pix_id) + grad_eps;
		grad_pts(1, pix_id) = curr_pts(1, pix_id);

		grad_pts(2, pix_id) = curr_pts(0, pix_id) - grad_eps;
		grad_pts(3, pix_id) = curr_pts(1, pix_id);

		grad_pts(4, pix_id) = curr_pts(0, pix_id);
		grad_pts(5, pix_id) = curr_pts(1, pix_id) + grad_eps;

		grad_pts(6, pix_id) = curr_pts(0, pix_id);
		grad_pts(7, pix_id) = curr_pts(1, pix_id) - grad_eps;
	}
}

void Translation::updateHessPts(double hess_eps){
	double hess_eps2 = 2 * hess_eps;

	for(int pix_id = 0; pix_id < n_pts; pix_id++){

		hess_pts(0, pix_id) = curr_pts(0, pix_id) + hess_eps2;
		hess_pts(1, pix_id) = curr_pts(1, pix_id);

		hess_pts(2, pix_id) = curr_pts(0, pix_id) - hess_eps2;
		hess_pts(3, pix_id) = curr_pts(1, pix_id);

		hess_pts(4, pix_id) = curr_pts(0, pix_id);
		hess_pts(5, pix_id) = curr_pts(1, pix_id) + hess_eps2;

		hess_pts(6, pix_id) = curr_pts(0, pix_id);
		hess_pts(7, pix_id) = curr_pts(1, pix_id) - hess_eps2;

		hess_pts(8, pix_id) = curr_pts(0, pix_id) + hess_eps;
		hess_pts(9, pix_id) = curr_pts(1, pix_id) + hess_eps;

		hess_pts(10, pix_id) = curr_pts(0, pix_id) - hess_eps;
		hess_pts(11, pix_id) = curr_pts(1, pix_id) - hess_eps;

		hess_pts(12, pix_id) = curr_pts(0, pix_id) + hess_eps;
		hess_pts(13, pix_id) = curr_pts(1, pix_id) - hess_eps;

		hess_pts(14, pix_id) = curr_pts(0, pix_id) - hess_eps;
		hess_pts(15, pix_id) = curr_pts(1, pix_id) + hess_eps;
	}
}

void Translation::applyWarpToCorners(Matrix24d &warped_corners, const Matrix24d &orig_corners,
	const VectorXd &state_update){
	warped_corners = orig_corners.colwise() + state_update;
}
void Translation::applyWarpToPts(Matrix2Xd &warped_pts, const Matrix2Xd &orig_pts,
	const VectorXd &state_update){
	warped_pts = orig_pts.colwise() + state_update;
}

void Translation::estimateWarpFromCorners(VectorXd &state_update, const Matrix24d &in_corners,
	const Matrix24d &out_corners){
	validate_ssm_state(state_update);

	Vector2d out_centroid = out_corners.rowwise().mean();
	Vector2d in_centroid = in_corners.rowwise().mean();
	state_update = out_centroid - in_centroid;
}

// use Random Walk model to generate perturbed sample
void Translation::compositionalRandomWalk(VectorXd &perturbed_state,
	const VectorXd &base_state){
	generatePerturbation(state_perturbation);
	perturbed_state = base_state + state_perturbation;
}
// use first order Auto Regressive model to generate perturbed sample
void Translation::compositionalAutoRegression1(VectorXd &perturbed_state, VectorXd &perturbed_ar,
	const VectorXd &base_state, const VectorXd &base_ar, double a){
	generatePerturbation(state_perturbation);
	perturbed_state = base_state + base_ar + state_perturbation;
	perturbed_ar = a*(perturbed_state - base_state);
}

void Translation::estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
	const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
	const EstimatorParams &est_params){
	assert(in_pts.size() == out_pts.size() && in_pts.size() == mask.size());
	if(est_params.max_iters > 0){
		cv::Mat trans_params = estimateTranslation(in_pts, out_pts, mask, est_params);	
		state_update(0) = trans_params.at<double>(0, 0);
		state_update(1) = trans_params.at<double>(0, 1);
	} else{
		int n_pts = in_pts.size();
		vector<cv::Point2f> diff_between_pts(n_pts);
		vector<uchar> test_mask(n_pts);
		for(int pt_id = 0; pt_id < n_pts; ++pt_id){
			diff_between_pts[pt_id].x = out_pts[pt_id].x - in_pts[pt_id].x;
			diff_between_pts[pt_id].y = out_pts[pt_id].y - in_pts[pt_id].y;
		}
		int max_inliers = -1, best_pt_id = -1;
		switch(est_params.method){
		case EstType::LeastMedian:
			throw std::domain_error("translation::Least median estimator is not implemented yet");
		case EstType::RANSAC:
			for(int test_pt_id = 0; test_pt_id < n_pts; ++test_pt_id){
				float tx = out_pts[test_pt_id].x - in_pts[test_pt_id].x;
				float ty = out_pts[test_pt_id].y - in_pts[test_pt_id].y;
				int n_inliers = 0;
				for(int pt_id = 0; pt_id < n_pts; ++pt_id){
					float dx = diff_between_pts[pt_id].x - diff_between_pts[test_pt_id].x;
					float dy = diff_between_pts[pt_id].y - diff_between_pts[test_pt_id].y;
					float err = dx*dx + dy*dy;
					n_inliers += test_mask[pt_id] = err <= est_params.ransac_reproj_thresh;
				}
				if(n_inliers > max_inliers){
					max_inliers = n_inliers;
					best_pt_id = test_pt_id;
					memcpy(mask.data(), test_mask.data(), sizeof(uchar)*n_pts);
				}
			}
			break;
		default:
			std::fill(mask.begin(), mask.end(), 1);
		}

		state_update[0] = 0, state_update[1] = 0;
		int pts_found = 0;
		for(int pt_id = 0; pt_id < n_pts; ++pt_id){
			if(!mask[pt_id]){ continue; }
			++pts_found;
			state_update[0] += (diff_between_pts[pt_id].x - state_update[0]) / pts_found;
			state_update[1] += (diff_between_pts[pt_id].y - state_update[1]) / pts_found;
		}
	}
}


TranslationEstimator::TranslationEstimator(int _modelPoints, bool _use_boost_rng)
	: SSMEstimator(_modelPoints, cvSize(2, 1), 1, _use_boost_rng) {
	assert(_modelPoints >= 1);
	checkPartialSubsets = false;
}

int TranslationEstimator::runKernel(const CvMat* m1, const CvMat* m2, CvMat* H) {
	int n_pts = m1->rows * m1->cols;

	//if(n_pts != 3) {
	//    throw invalid_argument(cv::format("Invalid no. of points: %d provided", n_pts));
	//}
	const CvPoint2D64f* M = (const CvPoint2D64f*)m1->data.ptr;
	const CvPoint2D64f* m = (const CvPoint2D64f*)m2->data.ptr;

	double mean_tx = 0, mean_ty = 0;

	for(int pt_id = 0; pt_id < n_pts; pt_id++) {
		double tx = m[pt_id].x - M[pt_id].x;
		double ty = m[pt_id].y - M[pt_id].y;
		mean_tx += (tx - mean_tx) / (pt_id + 1);
		mean_ty += (ty - mean_ty) / (pt_id + 1);

	}
	double *H_ptr = H->data.db;
	H_ptr[0] = mean_tx;
	H_ptr[1] = mean_ty;
	return 1;
}


void TranslationEstimator::computeReprojError(const CvMat* m1, const CvMat* m2,
	const CvMat* model, CvMat* _err) {
	int n_pts = m1->rows * m1->cols;
	const CvPoint2D64f* M = (const CvPoint2D64f*)m1->data.ptr;
	const CvPoint2D64f* m = (const CvPoint2D64f*)m2->data.ptr;
	const double* H = model->data.db;
	float* err = _err->data.fl;

	for(int pt_id = 0; pt_id < n_pts; pt_id++) {
		double dx = (H[0] + M[pt_id].x) - m[pt_id].x;
		double dy = (H[1] + M[pt_id].y) - m[pt_id].y;
		err[pt_id] = (float)(dx * dx + dy * dy);
	}
}

bool TranslationEstimator::refine(const CvMat* m1, const CvMat* m2,
	CvMat* model, int maxIters) {
	LevMarq solver(2, 0, cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, maxIters, DBL_EPSILON));
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

		for(int pt_id = 0; pt_id < n_pts; pt_id++)	{
			const double* h = _param->data.db;
			double Mx = M[pt_id].x, My = M[pt_id].y;
			double _xi = (h[0] + Mx);
			double _yi = (h[1] + My);
			double err[] = { _xi - m[pt_id].x, _yi - m[pt_id].y };
			if(_JtJ || _JtErr) {
				double J[][2] = {
					{ 1, 0 },
					{ 0, 1 }
				};
				for(int j = 0; j < 2; j++) {
					for(int k = j; k < 2; k++)
						_JtJ->data.db[j * 2 + k] += J[0][j] * J[0][k] + J[1][j] * J[1][k];
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

cv::Mat Translation::estimateTranslation(cv::InputArray _in_pts, cv::InputArray _out_pts,
	cv::OutputArray _mask, const SSMEstimatorParams &params){
	cv::Mat in_pts = _in_pts.getMat(), out_pts = _out_pts.getMat();
	int n_pts = in_pts.checkVector(2);
	CV_Assert(n_pts >= 0 && out_pts.checkVector(2) == n_pts &&
		in_pts.type() == out_pts.type());

	cv::Mat H(1, 2, CV_64F);
	CvMat _pt1 = in_pts, _pt2 = out_pts;
	CvMat matH = H, c_mask, *p_mask = 0;
	if(_mask.needed()){
		_mask.create(n_pts, 1, CV_8U, -1, true);
		p_mask = &(c_mask = _mask.getMat());
	}
	bool ok = estimateTranslation(&_pt1, &_pt2, &matH, p_mask, params) > 0;
	if(!ok)
		H = cv::Scalar(0);
	return H;
}

int	Translation::estimateTranslation(const CvMat* in_pts, const CvMat* out_pts,
	CvMat* __H, CvMat* mask, const SSMEstimatorParams &params) {
	bool result = false;
	cv::Ptr<CvMat> out_pts_hm, in_pts_hm, tempMask;

	double H[2];
	CvMat matH = cvMat(1, 2, CV_64FC1, H);

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

	TranslationEstimator estimator(params.n_model_pts, params.use_boost_rng);

	int method = n_pts == params.n_model_pts ? 0 : params.method_cv;

	if(method == CV_LMEDS)
		result = estimator.runLMeDS(in_pts_hm, out_pts_hm, &matH, tempMask, params.confidence,
		params.max_iters, params.max_subset_attempts);
	else if(method == CV_RANSAC)
		result = estimator.runRANSAC(in_pts_hm, out_pts_hm, &matH, tempMask, params.ransac_reproj_thresh,
		params.confidence, params.max_iters, params.max_subset_attempts);
	else
		result = estimator.runKernel(in_pts_hm, out_pts_hm, &matH) > 0; 0;

	if(result && n_pts > params.n_model_pts) {
		icvCompressPoints((CvPoint2D64f*)in_pts_hm->data.ptr, tempMask->data.ptr, 1, n_pts);
		n_pts = icvCompressPoints((CvPoint2D64f*)out_pts_hm->data.ptr, tempMask->data.ptr, 1, n_pts);
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


_MTF_END_NAMESPACE

