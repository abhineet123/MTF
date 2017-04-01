#include "mtf/SM/ICLKParams.h"

#define IC_MAX_ITERS 30
#define IC_EPSILON 1e-4
#define IC_HESS_TYPE 0
#define IC_SEC_ORD_HESS false
#define IC_UPDATE_SSM false
#define IC_CHAINED_WARP true
#define IC_LEVEN_MARQ true
#define IC_LM_DELTA_INIT 0.01
#define IC_LM_DELTA_UPDATE 10
#define IC_ENABLE_LEARNING 0
#define IC_DEBUG_MODE false

_MTF_BEGIN_NAMESPACE

ICLKParams::ICLKParams(int _max_iters, double _epsilon,
HessType _hess_type, bool _sec_ord_hess,
bool _update_ssm, bool _chained_warp,
bool _leven_marq, double _lm_delta_init,
double _lm_delta_update, bool _enable_learning, 
bool _debug_mode) :
max_iters(_max_iters),
epsilon(_epsilon),
hess_type(_hess_type),
sec_ord_hess(_sec_ord_hess),
update_ssm(_update_ssm),
chained_warp(_chained_warp),
leven_marq(_leven_marq),
lm_delta_init(_lm_delta_init),
lm_delta_update(_lm_delta_update),
enable_learning(_enable_learning),
debug_mode(_debug_mode){}

ICLKParams::ICLKParams(const ICLKParams *params) :
max_iters(IC_MAX_ITERS),
epsilon(IC_EPSILON),
hess_type(static_cast<HessType>(IC_HESS_TYPE)),
sec_ord_hess(IC_SEC_ORD_HESS),
update_ssm(IC_UPDATE_SSM),
chained_warp(IC_CHAINED_WARP),
leven_marq(IC_LEVEN_MARQ),
lm_delta_init(IC_LM_DELTA_INIT),
lm_delta_update(IC_LM_DELTA_UPDATE),
enable_learning(IC_ENABLE_LEARNING),
debug_mode(IC_DEBUG_MODE){
	if(params){
		max_iters = params->max_iters;
		epsilon = params->epsilon;
		hess_type = params->hess_type;
		sec_ord_hess = params->sec_ord_hess;
		update_ssm = params->update_ssm;
		chained_warp = params->chained_warp;
		leven_marq = params->leven_marq;
		lm_delta_init = params->lm_delta_init;
		lm_delta_update = params->lm_delta_update;
		enable_learning = params->enable_learning;
		debug_mode = params->debug_mode;
	}
}
const char*  ICLKParams::toString(HessType hess_type){
	switch(hess_type){
	case HessType::InitialSelf:
		return "Initial Self";
	case HessType::CurrentSelf:
		return "Current Self";
	case HessType::Std:
		return "Standard";
	default:
		throw std::invalid_argument(
			cv::format("ICLKParams :: Invalid hessian type provided: %d", hess_type));
	}
}

_MTF_END_NAMESPACE


