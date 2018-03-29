#include "mtf/SSM//SSMEstimator.h"
#include "mtf/SSM/internal/cv2_q_sort.h"
#include "opencv2/core/core_c.h"
//#if CV_MAJOR_VERSION < 3
//#include "opencv2/core/internal.hpp"
//#else
////#include "mtf/SM//opencv2/core/internal.hpp"
//#endif
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
	boost_rng.seed(static_cast<const uint32_t>(seed));
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

_MTF_END_NAMESPACE
