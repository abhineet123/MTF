#ifndef MTF_SSM_ESTIMATOR_H
#define MTF_SSM_ESTIMATOR_H

#include "mtf/Macros/common.h"
#include "SSMEstimatorParams.h"

#include <boost/random/mersenne_twister.hpp>

_MTF_BEGIN_NAMESPACE

//! Base class for robust estimators for different SSMs
//! adapted from CvModelEstimator2 defined in _modeltest.h inside
//! calib3d module

class SSMEstimator{
public:
	SSMEstimator(int _modelPoints, CvSize _modelSize, 
		int _maxBasicSolutions, bool _use_boost_rng);
	virtual ~SSMEstimator();

	virtual int runKernel(const CvMat* m1, const CvMat* m2, CvMat* model) = 0;
	virtual bool runLMeDS(const CvMat* m1, const CvMat* m2, CvMat* model,
		CvMat* mask, double confidence = 0.99, int maxIters = 2000, int maxAttempts = 300);
	virtual bool runRANSAC(const CvMat* m1, const CvMat* m2, CvMat* model,
		CvMat* mask, double threshold, double confidence = 0.99, int maxIters = 2000,
		int maxAttempts=300);
	virtual bool refine(const CvMat*, const CvMat*, CvMat*, int) { return true; }
	virtual void setSeed(int64 seed);

protected:
	virtual void computeReprojError(const CvMat* m1, const CvMat* m2,
		const CvMat* model, CvMat* error) = 0;
	virtual int findInliers(const CvMat* m1, const CvMat* m2,
		const CvMat* model, CvMat* error,
		CvMat* mask, double threshold);
	virtual bool getSubset(const CvMat* m1, const CvMat* m2,
		CvMat* ms1, CvMat* ms2, int maxAttempts = 1000);
	virtual bool checkSubset(const CvMat* ms1, int count);

	typedef boost::mt19937 BoostRNG;

	CvRNG cv_rng;
	BoostRNG boost_rng;

	int modelPoints;
	CvSize modelSize;
	int maxBasicSolutions;
	bool checkPartialSubsets;
	const bool use_boost_rng;
};

class HomographyEstimator : public SSMEstimator{
public:
	HomographyEstimator(int modelPoints, bool _use_boost_rng);

	int runKernel(const CvMat* m1, const CvMat* m2, CvMat* model) override;
	bool refine(const CvMat* m1, const CvMat* m2,
		CvMat* model, int maxIters) override;
protected:
	void computeReprojError(const CvMat* m1, const CvMat* m2,
		const CvMat* model, CvMat* error) override;
};

class AffineEstimator : public SSMEstimator{
public:
	AffineEstimator(int modelPoints, bool _use_boost_rng);

	int runKernel(const CvMat* m1, const CvMat* m2, CvMat* model) override;
	bool refine(const CvMat* m1, const CvMat* m2,
		CvMat* model, int maxIters) override;
protected:
	void computeReprojError(const CvMat* m1, const CvMat* m2,
		const CvMat* model, CvMat* error) override;
};

//! Levenberg Marquardt Solver for refining estimated SSM paarameters
//! copied from CvLevMarq in calib3d module of OpenCV
class LevMarq{
public:
	LevMarq();
	LevMarq(int nparams, int nerrs, CvTermCriteria criteria =
		cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, DBL_EPSILON),
		bool completeSymmFlag = false);
	~LevMarq();
	void init(int nparams, int nerrs, CvTermCriteria criteria =
		cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, DBL_EPSILON),
		bool completeSymmFlag = false);
	bool update(const CvMat*& param, CvMat*& J, CvMat*& err);
	bool updateAlt(const CvMat*& param, CvMat*& JtJ, CvMat*& JtErr, double*& errNorm);

	void clear();
	void step();
	enum { DONE = 0, STARTED = 1, CALC_J = 2, CHECK_ERR = 3 };

	cv::Ptr<CvMat> mask;
	cv::Ptr<CvMat> prevParam;
	cv::Ptr<CvMat> param;
	cv::Ptr<CvMat> J;
	cv::Ptr<CvMat> err;
	cv::Ptr<CvMat> JtJ;
	cv::Ptr<CvMat> JtJN;
	cv::Ptr<CvMat> JtErr;
	cv::Ptr<CvMat> JtJV;
	cv::Ptr<CvMat> JtJW;
	double prevErrNorm, errNorm;
	int lambdaLg10;
	CvTermCriteria criteria;
	int state;
	int iters;
	bool completeSymmFlag;
};


cv::Mat estimateHomography(cv::InputArray _points1, cv::InputArray _points2,
	cv::OutputArray _mask, const SSMEstimatorParams &est_params);

//! modified version of cv:: cvFindHomography defined in fundam.cpp as part of calib3d module of OpenCV
int estimateHomography(const CvMat* objectPoints, const CvMat* imagePoints,
	CvMat* __H, CvMat* mask, const SSMEstimatorParams &est_params);

cv::Mat estimateAffine(cv::InputArray _points1, cv::InputArray _points2,
	cv::OutputArray _mask, const SSMEstimatorParams &est_params);

int	estimateAffine(const CvMat* in_pts, const CvMat* imagePoints,
	CvMat* __H, CvMat* mask, const SSMEstimatorParams &est_params);

cv::Mat estimateSimilitude(cv::InputArray _points1, cv::InputArray _points2,
	cv::OutputArray _mask, const SSMEstimatorParams &est_params);

int	estimateSimilitude(const CvMat* in_pts, const CvMat* out_pts,
	CvMat* __H, CvMat* mask, const SSMEstimatorParams &est_params);

cv::Mat estimateIsometry(cv::InputArray _points1, cv::InputArray _points2,
	cv::OutputArray _mask, const SSMEstimatorParams &est_params);

int	estimateIsometry(const CvMat* in_pts, const CvMat* out_pts,
	CvMat* __H, CvMat* mask, const SSMEstimatorParams &est_params);

cv::Mat estimateAST(cv::InputArray _points1, cv::InputArray _points2,
	cv::OutputArray _mask, const SSMEstimatorParams &est_params);

int	estimateAST(const CvMat* in_pts, const CvMat* out_pts,
	CvMat* __H, CvMat* mask, const SSMEstimatorParams &est_params);

_MTF_END_NAMESPACE
#endif
