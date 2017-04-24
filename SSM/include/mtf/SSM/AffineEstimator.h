#ifndef MTF_AFFINE_ESTIMATOR_H
#define MTF_AFFINE_ESTIMATOR_H

#include "SSMEstimator.h"

_MTF_BEGIN_NAMESPACE


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

cv::Mat estimateAffine(cv::InputArray _points1, cv::InputArray _points2,
	cv::OutputArray _mask, const SSMEstimatorParams &est_params);

int	estimateAffine(const CvMat* in_pts, const CvMat* imagePoints,
	CvMat* __H, CvMat* mask, const SSMEstimatorParams &est_params);

_MTF_END_NAMESPACE

#endif
