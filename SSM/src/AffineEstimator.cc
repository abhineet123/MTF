#include "mtf/SSM//AffineEstimator.h"
#include "mtf/Utilities/warpUtils.h"
#include "mtf/Utilities/miscUtils.h"
#include "opencv2/core/core_c.h"
#include "opencv2/calib3d/calib3d.hpp"
#include <boost/random/uniform_int_distribution.hpp>


_MTF_BEGIN_NAMESPACE

AffineEstimator::AffineEstimator(int _modelPoints, bool _use_boost_rng)
	: SSMEstimator(_modelPoints, cvSize(3, 2), 1, _use_boost_rng) {
	assert(_modelPoints >= 3);
	checkPartialSubsets = false;
}

int AffineEstimator::runKernel(const CvMat* m1, const CvMat* m2, CvMat* H) {
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
	Matrix3d affine_mat = utils::computeAffineDLT(in_pts, out_pts);

	double *H_ptr = H->data.db;
	H_ptr[0] = affine_mat(0, 0);
	H_ptr[1] = affine_mat(0, 1);
	H_ptr[2] = affine_mat(0, 2);
	H_ptr[3] = affine_mat(1, 0);
	H_ptr[4] = affine_mat(1, 1);
	H_ptr[5] = affine_mat(1, 2);
	return 1;
}


void AffineEstimator::computeReprojError(const CvMat* m1, const CvMat* m2,
	const CvMat* model, CvMat* _err) {
	int n_pts = m1->rows * m1->cols;
	const CvPoint2D64f* M = (const CvPoint2D64f*)m1->data.ptr;
	const CvPoint2D64f* m = (const CvPoint2D64f*)m2->data.ptr;
	const double* H = model->data.db;
	float* err = _err->data.fl;

	for(int pt_id = 0; pt_id < n_pts; pt_id++) {
		double dx = (H[0] * M[pt_id].x + H[1] * M[pt_id].y + H[2]) - m[pt_id].x;
		double dy = (H[3] * M[pt_id].x + H[4] * M[pt_id].y + H[5]) - m[pt_id].y;
		err[pt_id] = (float)(dx * dx + dy * dy);
	}
}

bool AffineEstimator::refine(const CvMat* m1, const CvMat* m2,
	CvMat* model, int maxIters) {
	LevMarq solver(6, 0, cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, maxIters, DBL_EPSILON));
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
			double _xi = (h[0] * Mx + h[1] * My + h[2]);
			double _yi = (h[3] * Mx + h[4] * My + h[5]);
			double err[] = { _xi - m[pt_id].x, _yi - m[pt_id].y };
			if(_JtJ || _JtErr) {
				double J[][6] = {
					{ Mx, My, 1, 0, 0, 0 },
					{ 0, 0, 0, Mx, My, 1 }
				};
				for(int j = 0; j < 6; j++) {
					for(int k = j; k < 6; k++)
						_JtJ->data.db[j * 6 + k] += J[0][j] * J[0][k] + J[1][j] * J[1][k];
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

cv::Mat estimateAffine(cv::InputArray _points1, cv::InputArray _points2,
	cv::OutputArray _mask, const SSMEstimatorParams &params){
	cv::Mat points1 = _points1.getMat(), points2 = _points2.getMat();
	int npoints = points1.checkVector(2);
	CV_Assert(npoints >= 0 && points2.checkVector(2) == npoints &&
		points1.type() == points2.type());

	cv::Mat H(2, 3, CV_64F);
	CvMat _pt1 = points1, _pt2 = points2;
	CvMat matH = H, c_mask, *p_mask = 0;
	if(_mask.needed()){
		_mask.create(npoints, 1, CV_8U, -1, true);
		p_mask = &(c_mask = _mask.getMat());
	}
	bool ok = estimateAffine(&_pt1, &_pt2, &matH, p_mask, params) > 0;
	if(!ok)
		H = cv::Scalar(0);
	return H;
}

int	estimateAffine(const CvMat* in_pts, const CvMat* out_pts,
	CvMat* __H, CvMat* mask, const SSMEstimatorParams &params) {
	bool result = false;
	cv::Ptr<CvMat> out_pts_hm, in_pts_hm, tempMask;

	double H[6];
	CvMat matH = cvMat(2, 3, CV_64FC1, H);

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

	AffineEstimator estimator(params.n_model_pts, params.use_boost_rng);

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

//AffineEstimator::AffineEstimator(int _modelPoints)
//	: SSMEstimator(_modelPoints, cvSize(3, 3), 1)
//{
//	assert(_modelPoints == 4 || _modelPoints == 5);
//	checkPartialSubsets = false;
//}

//int AffineEstimator::runKernel(const CvMat* m1, const CvMat* m2, CvMat* H)
//{
//	int i, count = m1->rows*m1->cols;
//	const CvPoint2D64f* M = (const CvPoint2D64f*)m1->data.ptr;
//	const CvPoint2D64f* m = (const CvPoint2D64f*)m2->data.ptr;

//	double LtL[9][9], W[9][1], V[9][9];
//	CvMat _LtL = cvMat(9, 9, CV_64F, LtL);
//	CvMat matW = cvMat(9, 1, CV_64F, W);
//	CvMat matV = cvMat(9, 9, CV_64F, V);
//	CvMat _H0 = cvMat(3, 3, CV_64F, V[8]);
//	CvMat _Htemp = cvMat(3, 3, CV_64F, V[7]);
//	CvPoint2D64f cM = { 0, 0 }, cm = { 0, 0 }, sM = { 0, 0 }, sm = { 0, 0 };

//	for(i = 0; i < count; i++)
//	{
//		cm.x += m[i].x; cm.y += m[i].y;
//		cM.x += M[i].x; cM.y += M[i].y;
//	}

//	cm.x /= count; cm.y /= count;
//	cM.x /= count; cM.y /= count;

//	for(i = 0; i < count; i++)
//	{
//		sm.x += fabs(m[i].x - cm.x);
//		sm.y += fabs(m[i].y - cm.y);
//		sM.x += fabs(M[i].x - cM.x);
//		sM.y += fabs(M[i].y - cM.y);
//	}

//	if(fabs(sm.x) < DBL_EPSILON || fabs(sm.y) < DBL_EPSILON ||
//		fabs(sM.x) < DBL_EPSILON || fabs(sM.y) < DBL_EPSILON)
//		return 0;
//	sm.x = count / sm.x; sm.y = count / sm.y;
//	sM.x = count / sM.x; sM.y = count / sM.y;

//	double invHnorm[9] = { 1. / sm.x, 0, cm.x, 0, 1. / sm.y, cm.y, 0, 0, 1 };
//	double Hnorm2[9] = { sM.x, 0, -cM.x*sM.x, 0, sM.y, -cM.y*sM.y, 0, 0, 1 };
//	CvMat _invHnorm = cvMat(3, 3, CV_64FC1, invHnorm);
//	CvMat _Hnorm2 = cvMat(3, 3, CV_64FC1, Hnorm2);

//	cvZero(&_LtL);
//	for(i = 0; i < count; i++)
//	{
//		double x = (m[i].x - cm.x)*sm.x, y = (m[i].y - cm.y)*sm.y;
//		double X = (M[i].x - cM.x)*sM.x, Y = (M[i].y - cM.y)*sM.y;
//		double Lx[] = { X, Y, 1, 0, 0, 0, -x*X, -x*Y, -x };
//		double Ly[] = { 0, 0, 0, X, Y, 1, -y*X, -y*Y, -y };
//		int j, k;
//		for(j = 0; j < 9; j++)
//			for(k = j; k < 9; k++)
//				LtL[j][k] += Lx[j] * Lx[k] + Ly[j] * Ly[k];
//	}
//	cvCompleteSymm(&_LtL);

//	//cvSVD( &_LtL, &matW, 0, &matV, CV_SVD_MODIFY_A + CV_SVD_V_T );
//	cvEigenVV(&_LtL, &matV, &matW);
//	cvMatMul(&_invHnorm, &_H0, &_Htemp);
//	cvMatMul(&_Htemp, &_Hnorm2, &_H0);
//	cvConvertScale(&_H0, H, 1. / _H0.data.db[8]);

//	return 1;
//}


//void AffineEstimator::computeReprojError(const CvMat* m1, const CvMat* m2,
//	const CvMat* model, CvMat* _err)
//{
//	int i, count = m1->rows*m1->cols;
//	const CvPoint2D64f* M = (const CvPoint2D64f*)m1->data.ptr;
//	const CvPoint2D64f* m = (const CvPoint2D64f*)m2->data.ptr;
//	const double* H = model->data.db;
//	float* err = _err->data.fl;

//	for(i = 0; i < count; i++)
//	{
//		double ww = 1. / (H[6] * M[i].x + H[7] * M[i].y + 1.);
//		double dx = (H[0] * M[i].x + H[1] * M[i].y + H[2])*ww - m[i].x;
//		double dy = (H[3] * M[i].x + H[4] * M[i].y + H[5])*ww - m[i].y;
//		err[i] = (float)(dx*dx + dy*dy);
//	}
//}

//bool AffineEstimator::refine(const CvMat* m1, const CvMat* m2, CvMat* model, int maxIters)
//{
//	CvLevMarq solver(8, 0, cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, maxIters, DBL_EPSILON));
//	int i, j, k, count = m1->rows*m1->cols;
//	const CvPoint2D64f* M = (const CvPoint2D64f*)m1->data.ptr;
//	const CvPoint2D64f* m = (const CvPoint2D64f*)m2->data.ptr;
//	CvMat modelPart = cvMat(solver.param->rows, solver.param->cols, model->type, model->data.ptr);
//	cvCopy(&modelPart, solver.param);

//	for(;;)
//	{
//		const CvMat* _param = 0;
//		CvMat *_JtJ = 0, *_JtErr = 0;
//		double* _errNorm = 0;

//		if(!solver.updateAlt(_param, _JtJ, _JtErr, _errNorm))
//			break;

//		for(i = 0; i < count; i++)
//		{
//			const double* h = _param->data.db;
//			double Mx = M[i].x, My = M[i].y;
//			double ww = h[6] * Mx + h[7] * My + 1.;
//			ww = fabs(ww) > DBL_EPSILON ? 1. / ww : 0;
//			double _xi = (h[0] * Mx + h[1] * My + h[2])*ww;
//			double _yi = (h[3] * Mx + h[4] * My + h[5])*ww;
//			double err[] = { _xi - m[i].x, _yi - m[i].y };
//			if(_JtJ || _JtErr)
//			{
//				double J[][8] =
//				{
//					{ Mx*ww, My*ww, ww, 0, 0, 0, -Mx*ww*_xi, -My*ww*_xi },
//					{ 0, 0, 0, Mx*ww, My*ww, ww, -Mx*ww*_yi, -My*ww*_yi }
//				};

//				for(j = 0; j < 8; j++)
//				{
//					for(k = j; k < 8; k++)
//						_JtJ->data.db[j * 8 + k] += J[0][j] * J[0][k] + J[1][j] * J[1][k];
//					_JtErr->data.db[j] += J[0][j] * err[0] + J[1][j] * err[1];
//				}
//			}
//			if(_errNorm)
//				*_errNorm += err[0] * err[0] + err[1] * err[1];
//		}
//	}

//	cvCopy(solver.param, &modelPart);
//	return true;
//}

_MTF_END_NAMESPACE
