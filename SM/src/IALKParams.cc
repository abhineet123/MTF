#include "mtf/SM/IALKParams.h"

#define IALK_MAX_ITERS 10
#define IALK_EPSILON 0.01
#define IALK_HESS_TYPE 0
#define IALK_SEC_ORD_HESS false
#define IALK_LEVEN_MARQ false
#define IALK_LM_DELTA_INIT 0.01
#define IALK_LM_DELTA_UPDATE 10
#define IALK_DEBUG_MODE false

_MTF_BEGIN_NAMESPACE

IALKParams::IALKParams(int _max_iters, double _epsilon,
HessType _hess_type, bool _sec_ord_hess,
bool _leven_marq, double _lm_delta_init,
double _lm_delta_update, bool _debug_mode) :
max_iters(_max_iters),
epsilon(_epsilon),
hess_type(_hess_type),
sec_ord_hess(_sec_ord_hess),
leven_marq(_leven_marq),
lm_delta_init(_lm_delta_init),
lm_delta_update(_lm_delta_update),
debug_mode(_debug_mode){}

IALKParams::IALKParams(const IALKParams *params) :
max_iters(IALK_MAX_ITERS),
epsilon(IALK_EPSILON),
hess_type(static_cast<HessType>(IALK_HESS_TYPE)),
sec_ord_hess(IALK_SEC_ORD_HESS),
leven_marq(IALK_LEVEN_MARQ),
lm_delta_init(IALK_LM_DELTA_INIT),
lm_delta_update(IALK_LM_DELTA_UPDATE),
debug_mode(IALK_DEBUG_MODE){
	if(params){
		max_iters = params->max_iters;
		epsilon = params->epsilon;
		hess_type = params->hess_type;
		sec_ord_hess = params->sec_ord_hess;
		leven_marq = params->leven_marq;
		lm_delta_init = params->lm_delta_init;
		lm_delta_update = params->lm_delta_update;
		debug_mode = params->debug_mode;
	}
}

const char* IALKParams::toString(HessType hess_type){
	switch(hess_type){
	case HessType::InitialSelf:
		return "Initial Self";
	case HessType::CurrentSelf:
		return "Current Self";
	case HessType::Std:
		return "Standard";
	default:
		throw std::invalid_argument(
			cv::format("IALKParams :: Invalid hessian type provided: %d", hess_type));
	}
}


_MTF_END_NAMESPACE

