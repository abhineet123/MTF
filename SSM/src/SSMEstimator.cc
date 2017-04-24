#include "mtf/SSM//SSMEstimator.h"
#include "mtf/Utilities/warpUtils.h"
#include "opencv2/core/core_c.h"
#if CV_MAJOR_VERSION < 3
#include "opencv2/core/internal.hpp"
#else
#include "mtf/SM//opencv2/core/internal.hpp"
#endif
#include "opencv2/calib3d/calib3d.hpp"
#include <iostream>
#include <boost/random/random_device.hpp>
#include <boost/random/uniform_int_distribution.hpp>


_MTF_BEGIN_NAMESPACE

SSMEstimator::SSMEstimator(int _modelPoints, CvSize _modelSize,
int _maxBasicSolutions, bool _use_boost_rng):
modelPoints(_modelPoints),
modelSize(_modelSize),
maxBasicSolutions(_maxBasicSolutions),
checkPartialSubsets(true),
use_boost_rng(_use_boost_rng){
	boost::random_device seed_gen;     
	boost_rng = BoostRNG(seed_gen());
	cv_rng = cvRNG(seed_gen());
}

SSMEstimator::~SSMEstimator() {}

void SSMEstimator::setSeed(int64 seed) {
	cv_rng = cvRNG(seed);
	boost_rng.seed(seed);
}


int SSMEstimator::findInliers(const CvMat* m1, const CvMat* m2,
	const CvMat* model, CvMat* _err,
	CvMat* _mask, double threshold) {
	int i, count = _err->rows * _err->cols, goodCount = 0;
	const float* err = _err->data.fl;
	uchar* mask = _mask->data.ptr;

	computeReprojError(m1, m2, model, _err);
	threshold *= threshold;
	for(i = 0; i < count; i++)
		goodCount += mask[i] = err[i] <= threshold;
	return goodCount;
}


int cvRANSACUpdateNumIters(double p, double ep,
	int model_points, int max_iters) {
	if(model_points <= 0)
		CV_Error(CV_StsOutOfRange, "the number of model points should be positive");

	p = MAX(p, 0.);
	p = MIN(p, 1.);
	ep = MAX(ep, 0.);
	ep = MIN(ep, 1.);

	// avoid inf's & nan's
	double num = MAX(1. - p, DBL_MIN);
	double denom = 1. - pow(1. - ep, model_points);
	if(denom < DBL_MIN)
		return 0;

	num = log(num);
	denom = log(denom);

	return denom >= 0 || -num >= max_iters * (-denom) ?
	max_iters : cvRound(num / denom);
}

bool SSMEstimator::runRANSAC(const CvMat* m1, const CvMat* m2, CvMat* model,
	CvMat* mask0, double reprojThreshold,double confidence, int maxIters,
	int maxAttempts) {
	bool result = false;
	cv::Ptr<CvMat> mask = cvCloneMat(mask0);
	cv::Ptr<CvMat> models, err, tmask;
	cv::Ptr<CvMat> ms1, ms2;

	int iter, niters = maxIters;
	int count = m1->rows * m1->cols, maxGoodCount = 0;
	CV_Assert(CV_ARE_SIZES_EQ(m1, m2) && CV_ARE_SIZES_EQ(m1, mask));

	if(count < modelPoints)
		return false;

	models = cvCreateMat(modelSize.height * maxBasicSolutions, modelSize.width, CV_64FC1);
	err = cvCreateMat(1, count, CV_32FC1);
	tmask = cvCreateMat(1, count, CV_8UC1);

	if(count > modelPoints) {
		ms1 = cvCreateMat(1, modelPoints, m1->type);
		ms2 = cvCreateMat(1, modelPoints, m2->type);
	} else {
		niters = 1;
		ms1 = cvCloneMat(m1);
		ms2 = cvCloneMat(m2);
	}

	for(iter = 0; iter < niters; iter++) {
		int i, goodCount, nmodels;
		if(count > modelPoints) {
			bool found = getSubset(m1, m2, ms1, ms2, maxAttempts);
			if(!found) {
				if(iter == 0)
					return false;
				break;
			}
		}

		nmodels = runKernel(ms1, ms2, models);
		if(nmodels <= 0)
			continue;
		for(i = 0; i < nmodels; i++) {
			CvMat model_i;
			cvGetRows(models, &model_i, i * modelSize.height, (i + 1)*modelSize.height);
			goodCount = findInliers(m1, m2, &model_i, err, tmask, reprojThreshold);

			if(goodCount > MAX(maxGoodCount, modelPoints - 1)) {
				std::swap(tmask, mask);
				cvCopy(&model_i, model);
				maxGoodCount = goodCount;
				niters = cvRANSACUpdateNumIters(confidence,
					(double)(count - goodCount) / count, modelPoints, niters);
			}
		}
	}

	//printf("SSMEstimator::runRANSAC :: iter: %d\n", iter);

	if(maxGoodCount > 0) {
		if(mask != mask0)
			cvCopy(mask, mask0);
		result = true;
	}

	return result;
}


static CV_IMPLEMENT_QSORT(icvSortDistances, int, CV_LT)
bool SSMEstimator::runLMeDS(const CvMat* m1, const CvMat* m2, CvMat* model,
CvMat* mask, double confidence, int maxIters, int maxAttempts) {
	const double outlierRatio = 0.45;
	bool result = false;
	cv::Ptr<CvMat> models;
	cv::Ptr<CvMat> ms1, ms2;
	cv::Ptr<CvMat> err;

	int iter, niters = maxIters;
	int count = m1->rows * m1->cols;
	double minMedian = DBL_MAX, sigma;

	CV_Assert(CV_ARE_SIZES_EQ(m1, m2) && CV_ARE_SIZES_EQ(m1, mask));

	if(count < modelPoints)
		return false;

	models = cvCreateMat(modelSize.height * maxBasicSolutions, modelSize.width, CV_64FC1);
	err = cvCreateMat(1, count, CV_32FC1);

	if(count > modelPoints) {
		ms1 = cvCreateMat(1, modelPoints, m1->type);
		ms2 = cvCreateMat(1, modelPoints, m2->type);
	} else {
		niters = 1;
		ms1 = cvCloneMat(m1);
		ms2 = cvCloneMat(m2);
	}

	niters = cvRound(log(1 - confidence) / log(1 - pow(1 - outlierRatio, (double)modelPoints)));
	niters = MIN(MAX(niters, 3), maxIters);

	for(iter = 0; iter < niters; iter++) {
		int i, nmodels;
		if(count > modelPoints) {
			bool found = getSubset(m1, m2, ms1, ms2, maxAttempts);
			if(!found) {
				if(iter == 0)
					return false;
				break;
			}
		}

		nmodels = runKernel(ms1, ms2, models);
		if(nmodels <= 0)
			continue;
		for(i = 0; i < nmodels; i++) {
			CvMat model_i;
			cvGetRows(models, &model_i, i * modelSize.height, (i + 1)*modelSize.height);
			computeReprojError(m1, m2, &model_i, err);
			icvSortDistances(err->data.i, count, 0);

			double median = count % 2 != 0 ?
				err->data.fl[count / 2] : (err->data.fl[count / 2 - 1] + err->data.fl[count / 2]) * 0.5;

			if(median < minMedian) {
				minMedian = median;
				cvCopy(&model_i, model);
			}
		}
	}

	//printf("SSMEstimator::runLMeDS :: iter: %d\n", iter);

	if(minMedian < DBL_MAX) {
		sigma = 2.5 * 1.4826 * (1 + 5. / (count - modelPoints)) * sqrt(minMedian);
		sigma = MAX(sigma, 0.001);

		count = findInliers(m1, m2, model, err, mask, sigma);
		result = count >= modelPoints;
	}

	return result;
}


bool SSMEstimator::getSubset(const CvMat* m1, const CvMat* m2,
	CvMat* ms1, CvMat* ms2, int maxAttempts) {
	cv::AutoBuffer<int> _idx(modelPoints);
	int* idx = _idx;
	int i = 0, j, k, idx_i, iters = 0;
	int type = CV_MAT_TYPE(m1->type), elemSize = CV_ELEM_SIZE(type);
	const int *m1ptr = m1->data.i, *m2ptr = m2->data.i;
	int *ms1ptr = ms1->data.i, *ms2ptr = ms2->data.i;
	int count = m1->cols * m1->rows;

	assert(CV_IS_MAT_CONT(m1->type & m2->type) && (elemSize % sizeof(int) == 0));
	elemSize /= sizeof(int);

	boost::random::uniform_int_distribution<int> uni(0, count-1);

	for(; iters < maxAttempts; iters++) {
		for(i = 0; i < modelPoints && iters < maxAttempts;) {
			idx[i] = idx_i = use_boost_rng ? uni(boost_rng) : cvRandInt(&cv_rng) % count;
			for(j = 0; j < i; j++)
				if(idx_i == idx[j])
					break;
			if(j < i)
				continue;
			for(k = 0; k < elemSize; k++) {
				ms1ptr[i * elemSize + k] = m1ptr[idx_i * elemSize + k];
				ms2ptr[i * elemSize + k] = m2ptr[idx_i * elemSize + k];
			}
			if(checkPartialSubsets && (!checkSubset(ms1, i + 1) || !checkSubset(ms2, i + 1))) {
				iters++;
				continue;
			}
			i++;
		}
		if(!checkPartialSubsets && i == modelPoints &&
			(!checkSubset(ms1, i) || !checkSubset(ms2, i)))
			continue;
		break;
	}

	return i == modelPoints && iters < maxAttempts;
}


bool SSMEstimator::checkSubset(const CvMat* m, int count) {
	if(count <= 2)
		return true;

	int j, k, i, i0, i1;
	CvPoint2D64f* ptr = (CvPoint2D64f*)m->data.ptr;

	assert(CV_MAT_TYPE(m->type) == CV_64FC2);

	if(checkPartialSubsets)
		i0 = i1 = count - 1;
	else
		i0 = 0, i1 = count - 1;

	for(i = i0; i <= i1; i++) {
		// check that the i-th selected point does not belong
		// to a line connecting some previously selected points
		for(j = 0; j < i; j++) {
			double dx1 = ptr[j].x - ptr[i].x;
			double dy1 = ptr[j].y - ptr[i].y;
			for(k = 0; k < j; k++) {
				double dx2 = ptr[k].x - ptr[i].x;
				double dy2 = ptr[k].y - ptr[i].y;
				if(fabs(dx2 * dy1 - dy2 * dx1) <= FLT_EPSILON * (fabs(dx1) + fabs(dy1) + fabs(dx2) + fabs(dy2)))
					break;
			}
			if(k < j)
				break;
		}
		if(j < i)
			break;
	}

	return i > i1;
}

HomographyEstimator::HomographyEstimator(int _modelPoints, bool _use_boost_rng)
	: SSMEstimator(_modelPoints, cvSize(3, 3), 1, _use_boost_rng) {
	assert(_modelPoints >= 4);
	checkPartialSubsets = false;
}

int HomographyEstimator::runKernel(const CvMat* m1, const CvMat* m2, CvMat* H) {
	int i, count = m1->rows * m1->cols;
	const CvPoint2D64f* M = (const CvPoint2D64f*)m1->data.ptr;
	const CvPoint2D64f* m = (const CvPoint2D64f*)m2->data.ptr;

	double LtL[9][9], W[9][1], V[9][9];
	CvMat _LtL = cvMat(9, 9, CV_64F, LtL);
	CvMat matW = cvMat(9, 1, CV_64F, W);
	CvMat matV = cvMat(9, 9, CV_64F, V);
	CvMat _H0 = cvMat(3, 3, CV_64F, V[8]);
	CvMat _Htemp = cvMat(3, 3, CV_64F, V[7]);
	CvPoint2D64f cM = { 0, 0 }, cm = { 0, 0 }, sM = { 0, 0 }, sm = { 0, 0 };

	for(i = 0; i < count; i++) {
		cm.x += m[i].x;
		cm.y += m[i].y;
		cM.x += M[i].x;
		cM.y += M[i].y;
	}

	cm.x /= count;
	cm.y /= count;
	cM.x /= count;
	cM.y /= count;

	for(i = 0; i < count; i++) {
		sm.x += fabs(m[i].x - cm.x);
		sm.y += fabs(m[i].y - cm.y);
		sM.x += fabs(M[i].x - cM.x);
		sM.y += fabs(M[i].y - cM.y);
	}

	if(fabs(sm.x) < DBL_EPSILON || fabs(sm.y) < DBL_EPSILON ||
		fabs(sM.x) < DBL_EPSILON || fabs(sM.y) < DBL_EPSILON)
		return 0;
	sm.x = count / sm.x;
	sm.y = count / sm.y;
	sM.x = count / sM.x;
	sM.y = count / sM.y;

	double invHnorm[9] = { 1. / sm.x, 0, cm.x, 0, 1. / sm.y, cm.y, 0, 0, 1 };
	double Hnorm2[9] = { sM.x, 0, -cM.x * sM.x, 0, sM.y, -cM.y * sM.y, 0, 0, 1 };
	CvMat _invHnorm = cvMat(3, 3, CV_64FC1, invHnorm);
	CvMat _Hnorm2 = cvMat(3, 3, CV_64FC1, Hnorm2);

	cvZero(&_LtL);
	for(i = 0; i < count; i++) {
		double x = (m[i].x - cm.x) * sm.x, y = (m[i].y - cm.y) * sm.y;
		double X = (M[i].x - cM.x) * sM.x, Y = (M[i].y - cM.y) * sM.y;
		double Lx[] = { X, Y, 1, 0, 0, 0, -x * X, -x * Y, -x };
		double Ly[] = { 0, 0, 0, X, Y, 1, -y * X, -y * Y, -y };
		int j, k;
		for(j = 0; j < 9; j++)
			for(k = j; k < 9; k++)
				LtL[j][k] += Lx[j] * Lx[k] + Ly[j] * Ly[k];
	}
	cvCompleteSymm(&_LtL);

	//cvSVD( &_LtL, &matW, 0, &matV, CV_SVD_MODIFY_A + CV_SVD_V_T );
	cvEigenVV(&_LtL, &matV, &matW);
	cvMatMul(&_invHnorm, &_H0, &_Htemp);
	cvMatMul(&_Htemp, &_Hnorm2, &_H0);
	cvConvertScale(&_H0, H, 1. / _H0.data.db[8]);

	return 1;
}


void HomographyEstimator::computeReprojError(const CvMat* m1, const CvMat* m2,
	const CvMat* model, CvMat* _err) {
	int i, count = m1->rows * m1->cols;
	const CvPoint2D64f* M = (const CvPoint2D64f*)m1->data.ptr;
	const CvPoint2D64f* m = (const CvPoint2D64f*)m2->data.ptr;
	const double* H = model->data.db;
	float* err = _err->data.fl;

	for(i = 0; i < count; i++) {
		double ww = 1. / (H[6] * M[i].x + H[7] * M[i].y + 1.);
		double dx = (H[0] * M[i].x + H[1] * M[i].y + H[2]) * ww - m[i].x;
		double dy = (H[3] * M[i].x + H[4] * M[i].y + H[5]) * ww - m[i].y;
		err[i] = (float)(dx * dx + dy * dy);
	}
}

bool HomographyEstimator::refine(const CvMat* m1, const CvMat* m2,
	CvMat* model, int maxIters) {
	LevMarq solver(8, 0, cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, maxIters, DBL_EPSILON));
	int i, j, k, count = m1->rows * m1->cols;
	const CvPoint2D64f* M = (const CvPoint2D64f*)m1->data.ptr;
	const CvPoint2D64f* m = (const CvPoint2D64f*)m2->data.ptr;
	CvMat modelPart = cvMat(solver.param->rows, solver.param->cols, model->type, model->data.ptr);
	cvCopy(&modelPart, solver.param);

	for(;;) {
		const CvMat* _param = 0;
		CvMat *_JtJ = 0, *_JtErr = 0;
		double* _errNorm = 0;

		if(!solver.updateAlt(_param, _JtJ, _JtErr, _errNorm))
			break;

		for(i = 0; i < count; i++) {
			const double* h = _param->data.db;
			double Mx = M[i].x, My = M[i].y;
			double ww = h[6] * Mx + h[7] * My + 1.;
			ww = fabs(ww) > DBL_EPSILON ? 1. / ww : 0;
			double _xi = (h[0] * Mx + h[1] * My + h[2]) * ww;
			double _yi = (h[3] * Mx + h[4] * My + h[5]) * ww;
			double err[] = { _xi - m[i].x, _yi - m[i].y };
			if(_JtJ || _JtErr) {
				double J[][8] = {
					{ Mx * ww, My * ww, ww, 0, 0, 0, -Mx*ww * _xi, -My*ww * _xi },
					{ 0, 0, 0, Mx * ww, My * ww, ww, -Mx*ww * _yi, -My*ww * _yi }
				};

				for(j = 0; j < 8; j++) {
					for(k = j; k < 8; k++)
						_JtJ->data.db[j * 8 + k] += J[0][j] * J[0][k] + J[1][j] * J[1][k];
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


LevMarq::LevMarq() {
	mask = prevParam = param = J = err = JtJ = JtJN = JtErr = JtJV = JtJW = cv::Ptr<CvMat>();
	lambdaLg10 = 0;
	state = DONE;
	criteria = cvTermCriteria(0, 0, 0);
	iters = 0;
	completeSymmFlag = false;
}

LevMarq::LevMarq(int nparams, int nerrs, CvTermCriteria criteria0, bool _completeSymmFlag) {
	mask = prevParam = param = J = err = JtJ = JtJN = JtErr = JtJV = JtJW = cv::Ptr<CvMat>();
	init(nparams, nerrs, criteria0, _completeSymmFlag);
}

void LevMarq::clear() {
	mask.release();
	prevParam.release();
	param.release();
	J.release();
	err.release();
	JtJ.release();
	JtJN.release();
	JtErr.release();
	JtJV.release();
	JtJW.release();
}

LevMarq::~LevMarq() {
	clear();
}

void LevMarq::init(int nparams, int nerrs, CvTermCriteria criteria0, bool _completeSymmFlag) {
	if(!param || param->rows != nparams || nerrs != (err ? err->rows : 0))
		clear();
	mask = cvCreateMat(nparams, 1, CV_8U);
	cvSet(mask, cvScalarAll(1));
	prevParam = cvCreateMat(nparams, 1, CV_64F);
	param = cvCreateMat(nparams, 1, CV_64F);
	JtJ = cvCreateMat(nparams, nparams, CV_64F);
	JtJN = cvCreateMat(nparams, nparams, CV_64F);
	JtJV = cvCreateMat(nparams, nparams, CV_64F);
	JtJW = cvCreateMat(nparams, 1, CV_64F);
	JtErr = cvCreateMat(nparams, 1, CV_64F);
	if(nerrs > 0) {
		J = cvCreateMat(nerrs, nparams, CV_64F);
		err = cvCreateMat(nerrs, 1, CV_64F);
	}
	prevErrNorm = DBL_MAX;
	lambdaLg10 = -3;
	criteria = criteria0;
	if(criteria.type & CV_TERMCRIT_ITER)
		criteria.max_iter = MIN(MAX(criteria.max_iter, 1), 1000);
	else
		criteria.max_iter = 30;
	if(criteria.type & CV_TERMCRIT_EPS)
		criteria.epsilon = MAX(criteria.epsilon, 0);
	else
		criteria.epsilon = DBL_EPSILON;
	state = STARTED;
	iters = 0;
	completeSymmFlag = _completeSymmFlag;
}

bool LevMarq::update(const CvMat*& _param, CvMat*& matJ, CvMat*& _err) {
	double change;

	matJ = _err = 0;

	assert(!err.empty());
	if(state == DONE) {
		_param = param;
		return false;
	}

	if(state == STARTED) {
		_param = param;
		cvZero(J);
		cvZero(err);
		matJ = J;
		_err = err;
		state = CALC_J;
		return true;
	}

	if(state == CALC_J) {
		cvMulTransposed(J, JtJ, 1);
		cvGEMM(J, err, 1, 0, 0, JtErr, CV_GEMM_A_T);
		cvCopy(param, prevParam);
		step();
		if(iters == 0)
			prevErrNorm = cvNorm(err, 0, CV_L2);
		_param = param;
		cvZero(err);
		_err = err;
		state = CHECK_ERR;
		return true;
	}

	assert(state == CHECK_ERR);
	errNorm = cvNorm(err, 0, CV_L2);
	if(errNorm > prevErrNorm) {
		if(++lambdaLg10 <= 16) {
			step();
			_param = param;
			cvZero(err);
			_err = err;
			state = CHECK_ERR;
			return true;
		}
	}

	lambdaLg10 = MAX(lambdaLg10 - 1, -16);
	if(++iters >= criteria.max_iter ||
		(change = cvNorm(param, prevParam, CV_RELATIVE_L2)) < criteria.epsilon) {
		_param = param;
		state = DONE;
		return true;
	}

	prevErrNorm = errNorm;
	_param = param;
	cvZero(J);
	matJ = J;
	_err = err;
	state = CALC_J;
	return true;
}


bool LevMarq::updateAlt(const CvMat*& _param, CvMat*& _JtJ, CvMat*& _JtErr, double*& _errNorm) {
	double change;

	CV_Assert(err.empty());
	if(state == DONE) {
		_param = param;
		return false;
	}

	if(state == STARTED) {
		_param = param;
		cvZero(JtJ);
		cvZero(JtErr);
		errNorm = 0;
		_JtJ = JtJ;
		_JtErr = JtErr;
		_errNorm = &errNorm;
		state = CALC_J;
		return true;
	}

	if(state == CALC_J) {
		cvCopy(param, prevParam);
		step();
		_param = param;
		prevErrNorm = errNorm;
		errNorm = 0;
		_errNorm = &errNorm;
		state = CHECK_ERR;
		return true;
	}

	assert(state == CHECK_ERR);
	if(errNorm > prevErrNorm) {
		if(++lambdaLg10 <= 16) {
			step();
			_param = param;
			errNorm = 0;
			_errNorm = &errNorm;
			state = CHECK_ERR;
			return true;
		}
	}

	lambdaLg10 = MAX(lambdaLg10 - 1, -16);
	if(++iters >= criteria.max_iter ||
		(change = cvNorm(param, prevParam, CV_RELATIVE_L2)) < criteria.epsilon) {
		_param = param;
		state = DONE;
		return false;
	}

	prevErrNorm = errNorm;
	cvZero(JtJ);
	cvZero(JtErr);
	_param = param;
	_JtJ = JtJ;
	_JtErr = JtErr;
	state = CALC_J;
	return true;
}

void LevMarq::step() {
	const double LOG10 = log(10.);
	double lambda = exp(lambdaLg10 * LOG10);
	int i, j, nparams = param->rows;

	for(i = 0; i < nparams; i++)
		if(mask->data.ptr[i] == 0) {
			double *row = JtJ->data.db + i * nparams, *col = JtJ->data.db + i;
			for(j = 0; j < nparams; j++)
				row[j] = col[j * nparams] = 0;
			JtErr->data.db[i] = 0;
		}

	if(!err)
		cvCompleteSymm(JtJ, completeSymmFlag);
#if 1
	cvCopy(JtJ, JtJN);
	for(i = 0; i < nparams; i++)
		JtJN->data.db[(nparams + 1)*i] *= 1. + lambda;
#else
	cvSetIdentity(JtJN, cvRealScalar(lambda));
	cvAdd(JtJ, JtJN, JtJN);
#endif
	cvSVD(JtJN, JtJW, 0, JtJV, CV_SVD_MODIFY_A + CV_SVD_U_T + CV_SVD_V_T);
	cvSVBkSb(JtJW, JtJV, JtJV, JtErr, param, CV_SVD_U_T + CV_SVD_V_T);
	for(i = 0; i < nparams; i++)
		param->data.db[i] = prevParam->data.db[i] - (mask->data.ptr[i] ? param->data.db[i] : 0);
}


cv::Mat estimateHomography(cv::InputArray _points1, cv::InputArray _points2,
	cv::OutputArray _mask, const SSMEstimatorParams &params){
	cv::Mat points1 = _points1.getMat(), points2 = _points2.getMat();
	int n_pts = points1.checkVector(2);
	CV_Assert(n_pts >= 0 && points2.checkVector(2) == n_pts &&
		points1.type() == points2.type());

	cv::Mat H(3, 3, CV_64F);
	CvMat _pt1 = points1, _pt2 = points2;
	CvMat matH = H, c_mask, *p_mask = 0;
	if(_mask.needed()){
		_mask.create(n_pts, 1, CV_8U, -1, true);
		p_mask = &(c_mask = _mask.getMat());
	}
	bool ok = estimateHomography(&_pt1, &_pt2, &matH, p_mask, params) > 0;
	if(!ok)
		H = cv::Scalar(0);
	return H;
}

int estimateHomography(const CvMat* objectPoints, const CvMat* imagePoints,
	CvMat* __H, CvMat* mask, const SSMEstimatorParams &params){
	bool result = false;
	cv::Ptr<CvMat> m, M, tempMask;

	double H[9];
	CvMat matH = cvMat(3, 3, CV_64FC1, H);

	CV_Assert(CV_IS_MAT(imagePoints) && CV_IS_MAT(objectPoints));

	int n_pts = MAX(imagePoints->cols, imagePoints->rows);
	CV_Assert(n_pts >= params.n_model_pts);

	m = cvCreateMat(1, n_pts, CV_64FC2);
	cvConvertPointsHomogeneous(imagePoints, m);

	M = cvCreateMat(1, n_pts, CV_64FC2);
	cvConvertPointsHomogeneous(objectPoints, M);

	if(mask){
		CV_Assert(CV_IS_MASK_ARR(mask) && CV_IS_MAT_CONT(mask->type) &&
			(mask->rows == 1 || mask->cols == 1) &&
			mask->rows*mask->cols == n_pts);
	}
	if(mask || n_pts > params.n_model_pts)
		tempMask = cvCreateMat(1, n_pts, CV_8U);
	if(!tempMask.empty())
		cvSet(tempMask, cvScalarAll(1.));

	HomographyEstimator estimator(params.n_model_pts, params.use_boost_rng);
	int method = n_pts == params.n_model_pts ? 0 : params.method_cv;
	if(method == CV_LMEDS)
		result = estimator.runLMeDS(M, m, &matH, tempMask, params.confidence, 
		params.max_iters, params.max_subset_attempts);
	else if(method == CV_RANSAC)
		result = estimator.runRANSAC(M, m, &matH, tempMask, params.ransac_reproj_thresh,
		params.confidence, params.max_iters, params.max_subset_attempts);
	else
		result = estimator.runKernel(M, m, &matH) > 0;

	if(result && n_pts > params.n_model_pts){
		icvCompressPoints((CvPoint2D64f*)M->data.ptr, tempMask->data.ptr, 1, n_pts);
		n_pts = icvCompressPoints((CvPoint2D64f*)m->data.ptr, tempMask->data.ptr, 1, n_pts);
		M->cols = m->cols = n_pts;
		if(method == CV_RANSAC)
			estimator.runKernel(M, m, &matH);
		if(params.refine){
			estimator.refine(M, m, &matH, params.lm_max_iters);
		}
	}

	if(result)
		cvConvert(&matH, __H);

	if(mask && tempMask){
		if(CV_ARE_SIZES_EQ(mask, tempMask))
			cvCopy(tempMask, mask);
		else
			cvTranspose(tempMask, mask);
	}

	return (int)result;
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

cv::Mat estimateSimilitude(cv::InputArray _in_pts, cv::InputArray _out_pts,
	cv::OutputArray _mask, const SSMEstimatorParams &params){
	cv::Mat in_pts = _in_pts.getMat(), out_pts = _out_pts.getMat();
	int n_pts = in_pts.checkVector(2);
	CV_Assert(n_pts >= 0 && out_pts.checkVector(2) == n_pts &&
		in_pts.type() == out_pts.type());

	cv::Mat H(2, 2, CV_64F);
	CvMat _pt1 = in_pts, _pt2 = out_pts;
	CvMat matH = H, c_mask, *p_mask = 0;
	if(_mask.needed()){
		_mask.create(n_pts, 1, CV_8U, -1, true);
		p_mask = &(c_mask = _mask.getMat());
	}
	bool ok = estimateSimilitude(&_pt1, &_pt2, &matH, p_mask, params) > 0;
	if(!ok)
		H = cv::Scalar(0);
	return H;
}

int	estimateSimilitude(const CvMat* in_pts, const CvMat* out_pts,
	CvMat* __H, CvMat* mask, const SSMEstimatorParams &params) {
	bool result = false;
	cv::Ptr<CvMat> out_pts_hm, in_pts_hm, tempMask;

	double H[4];
	CvMat matH = cvMat(2, 2, CV_64FC1, H);

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

	SimilitudeEstimator estimator(params.n_model_pts, params.use_boost_rng);

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


cv::Mat estimateIsometry(cv::InputArray _in_pts, cv::InputArray _out_pts,
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

int	estimateIsometry(const CvMat* in_pts, const CvMat* out_pts,
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


cv::Mat estimateAST(cv::InputArray _in_pts, cv::InputArray _out_pts,
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

int	estimateAST(const CvMat* in_pts, const CvMat* out_pts,
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

//int compressPoints(HomPtsT &pts, const VectorXb &mask,
//	int mstep, int count){
//	int i, j;
//	for(i = j = 0; i < count; i++)
//		if(mask[i*mstep]){
//			if(i > j)
//				pts.col(j) = pts.col(i);
//			j++;
//		}
//	return j;
//}


//int	cvFindHomography(const PtsT &objectPoints, const PtsT &imagePoints,
//	ProjWarpT & __H, EstimatorMethod method, double ransacReprojThreshold,
//	VectorXb &mask){
//	int count = objectPoints.cols();
//	assert(count >= 4);
//	assert(mask.size() == count);

//	const double confidence = 0.995;
//	const int maxIters = 2000;
//	const double defaultRANSACReprojThreshold = 3;
//	bool result = false;

//	HomPtsT m, M;
//	ProjWarpT matH;
//	if(ransacReprojThreshold <= 0)
//		ransacReprojThreshold = defaultRANSACReprojThreshold;

//	homogenize(imagePoints, m);
//	homogenize(objectPoints, M);
//
//	mask.fill(1);

//	AffineEstimator estimator(4);
//	if(count == 4)
//		method = EstimatorMethod::LSTSQR;

//	if(method == EstimatorMethod::LMEDS)
//		result = estimator.runLMeDS(M, m, matH, mask, confidence, maxIters);
//	else if(method == EstimatorMethod::RANSAC)
//		result = estimator.runRANSAC(M, m, matH, mask, ransacReprojThreshold, confidence, maxIters);
//	else
//		result = estimator.runKernel(M, m, matH) > 0;

//	if(result && count > 4){
//		compressPoints(m, mask, 1, count);
//		count = compressPoints(M, mask, 1, count);
//		if(method == EstimatorMethod::RANSAC)
//			estimator.runKernel(M, m, matH);
//		estimator.refine(M, m, matH, 10);
//	}

//	if(result)
//		__H = matH;

//	return result;
//}

_MTF_END_NAMESPACE
