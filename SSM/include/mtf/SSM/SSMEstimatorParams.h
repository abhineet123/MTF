#ifndef MTF_SSM_ESTIMATOR_PARAMS_H
#define MTF_SSM_ESTIMATOR_PARAMS_H

#include "mtf/Macros/common.h"

_MTF_BEGIN_NAMESPACE


struct SSMEstimatorParams{

	enum class EstType { RANSAC, LeastMedian, LeastSquares };

	static const char* toString(EstType est_type);
	static int toCV(EstType est_type);

	EstType method;
	int method_cv;
	double ransac_reproj_thresh;
	int n_model_pts;
	int max_iters;
	int max_subset_attempts;
	bool  use_boost_rng;
	double confidence;
	bool refine;
	int lm_max_iters;
	
	SSMEstimatorParams(EstType _method, double _ransac_reproj_thresh, 
		int _n_model_pts, bool _refine, int _max_iters, int _max_subset_attempts,
		bool  _use_boost_rng, double _confidence, int _lm_max_iters);
	SSMEstimatorParams(const SSMEstimatorParams *params = nullptr);

	void print() const;
};

_MTF_END_NAMESPACE
#endif
