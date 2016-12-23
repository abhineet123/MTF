#include "mtf/SSM/SSMEstimatorParams.h"
#include "opencv2/calib3d/calib3d.hpp"

#define SSM_EST_METHOD EstType::RANSAC
#define SSM_EST_MAX_ITERS 2000
#define SSM_MAX_SUBSET_ATTEMPTS 300
#define SSM_USE_BOOST_RNG false
#define SSM_EST_RANSAC_REPROJ_THRESH 10.0
#define SSM_EST_N_MODEL_PTS 4
#define SSM_EST_REFINE true
#define SSM_EST_LM_MAX_ITERS 10
#define SSM_EST_CONFIDENCE 0.995

_MTF_BEGIN_NAMESPACE

const char* SSMEstimatorParams::toString(EstType est_type){
	switch(est_type){
	case EstType::LeastSquares:
		return "LeastSquares";
	case EstType::RANSAC:
		return "RANSAC";
	case EstType::LeastMedian:
		return "LeastMedian";
	default:
		throw std::invalid_argument("Invalid estimation method specified");
	}
}
int SSMEstimatorParams::toCV(EstType est_type){
	switch(est_type){
	case EstType::LeastSquares:
		return 0;
	case EstType::RANSAC:
		return CV_RANSAC;
	case EstType::LeastMedian:
		return CV_LMEDS;
	default:
		throw std::invalid_argument("Invalid estimation method specified");
	}
}

SSMEstimatorParams::SSMEstimatorParams(EstType _method, double _ransac_reproj_thresh,
	int _n_model_pts, bool _refine, int _max_iters, int _max_subset_attempts,
	bool  _use_boost_rng, double _confidence, int _lm_max_iters) :
method(_method),
ransac_reproj_thresh(_ransac_reproj_thresh),
n_model_pts(_n_model_pts),
max_iters(_max_iters),
max_subset_attempts(_max_subset_attempts),
use_boost_rng(_use_boost_rng),
confidence(_confidence),
refine(_refine),
lm_max_iters(_lm_max_iters){
	if(ransac_reproj_thresh <= 0){
		ransac_reproj_thresh = 3;
	}
	method_cv = toCV(method);
}

SSMEstimatorParams::SSMEstimatorParams(const SSMEstimatorParams *params) :
method(SSM_EST_METHOD),
ransac_reproj_thresh(SSM_EST_RANSAC_REPROJ_THRESH),
n_model_pts(SSM_EST_N_MODEL_PTS),
max_iters(SSM_EST_MAX_ITERS),
max_subset_attempts(SSM_MAX_SUBSET_ATTEMPTS),
use_boost_rng(SSM_USE_BOOST_RNG),
confidence(SSM_EST_CONFIDENCE),
refine(SSM_EST_REFINE),
lm_max_iters(SSM_EST_LM_MAX_ITERS){
	if(params){
		method = params->method;
		ransac_reproj_thresh = params->ransac_reproj_thresh;
		n_model_pts = params->n_model_pts;
		max_iters = params->max_iters;
		max_subset_attempts = params->max_subset_attempts;
		use_boost_rng = params->use_boost_rng;
		confidence = params->confidence;
		refine = params->refine;
		lm_max_iters = params->lm_max_iters;
	}
	if(ransac_reproj_thresh <= 0){
		ransac_reproj_thresh = 3;
	}
	method_cv = toCV(method);
}
void SSMEstimatorParams::print() const{
	printf("method: %s\n", toString(method));
	printf("ransac_reproj_thresh: %f\n", ransac_reproj_thresh);
	printf("n_model_pts: %d\n", n_model_pts);
	printf("max_iters: %d\n", max_iters);
	printf("max_subset_attempts: %d\n", max_subset_attempts);
	printf("use_boost_rng: %d\n", use_boost_rng);
	printf("confidence: %f\n", confidence);
	printf("refine: %d\n", refine);
	printf("lm_max_iters: %d\n", lm_max_iters);
}
_MTF_END_NAMESPACE
