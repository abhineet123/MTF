#ifndef MTF_HOMOGRAPHY_ESTIMATOR_H
#define MTF_HOMOGRAPHY_ESTIMATOR_H

#include "SSMEstimator.h"
#include "SSMEstimatorParams.h"


_MTF_BEGIN_NAMESPACE

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

cv::Mat estimateHomography(cv::InputArray _points1, cv::InputArray _points2,
	cv::OutputArray _mask, const SSMEstimatorParams &est_params);

//! modified version of cv:: cvFindHomography defined in fundam.cpp as part of calib3d module of OpenCV
int estimateHomography(const CvMat* objectPoints, const CvMat* imagePoints,
	CvMat* __H, CvMat* mask, const SSMEstimatorParams &est_params);

_MTF_END_NAMESPACE

#endif
