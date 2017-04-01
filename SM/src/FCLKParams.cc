#include "mtf/SM/FCLKParams.h"

#define FC_MAX_ITERS 30
#define FC_EPSILON 1e-4
#define FC_HESS_TYPE 1
#define FC_SECOND_ORDER_HESS false
#define FC_CHAINED_WARP true
#define FC_LEVEN_MARQ true
#define FC_LM_DELTA_INIT 0.01
#define FC_LM_DELTA_UPDATE 10
#define FC_ENABLE_LEARNING false
#define FC_WRITE_SSM_UPDATES false
#define FC_SHOW_GRID false
#define FC_SHOW_PATCH false
#define FC_PATCH_RESIZE_FACTOR 1.0
#define FC_DEBUG_MODE false

_MTF_BEGIN_NAMESPACE

FCLKParams::FCLKParams(int _max_iters, double _epsilon,
HessType _hess_type, bool _sec_ord_hess,
bool _chained_warp, bool _leven_marq, 
double _lm_delta_init, double _lm_delta_update, 
bool _enable_learning, bool _write_ssm_updates,
bool _show_grid, bool _show_patch,
double _patch_resize_factor, bool _debug_mode) :
max_iters(_max_iters),
epsilon(_epsilon),
hess_type(_hess_type),
sec_ord_hess(_sec_ord_hess),
chained_warp(_chained_warp),
leven_marq(_leven_marq),
lm_delta_init(_lm_delta_init),
lm_delta_update(_lm_delta_update),
enable_learning(_enable_learning),
write_ssm_updates(_write_ssm_updates),
show_grid(_show_grid),
show_patch(_show_patch),
patch_resize_factor(_patch_resize_factor),
debug_mode(_debug_mode){}

FCLKParams::FCLKParams(const FCLKParams *params) :
max_iters(FC_MAX_ITERS),
epsilon(FC_EPSILON),
hess_type(static_cast<HessType>(FC_HESS_TYPE)),
sec_ord_hess(FC_SECOND_ORDER_HESS),
chained_warp(FC_CHAINED_WARP),
leven_marq(FC_LEVEN_MARQ),
lm_delta_init(FC_LM_DELTA_INIT),
lm_delta_update(FC_LM_DELTA_UPDATE),
enable_learning(FC_ENABLE_LEARNING),
write_ssm_updates(FC_WRITE_SSM_UPDATES),
show_grid(FC_SHOW_GRID),
show_patch(FC_SHOW_PATCH),
patch_resize_factor(FC_PATCH_RESIZE_FACTOR),
debug_mode(FC_DEBUG_MODE){
	if(params){
		max_iters = params->max_iters;
		epsilon = params->epsilon;
		hess_type = params->hess_type;
		sec_ord_hess = params->sec_ord_hess;
		enable_learning = params->enable_learning;
		chained_warp = params->chained_warp;
		leven_marq = params->leven_marq;
		lm_delta_init = params->lm_delta_init;
		lm_delta_update = params->lm_delta_update;
		write_ssm_updates = params->write_ssm_updates;
		show_grid = params->show_grid;
		show_patch = params->show_patch;
		patch_resize_factor = params->patch_resize_factor;
		debug_mode = params->debug_mode;
	}
}

const char* FCLKParams::toString(HessType hess_type){
	switch(hess_type){
	case HessType::InitialSelf:
		return "Initial Self";
	case HessType::CurrentSelf:
		return "Current Self";
	case HessType::Std:
		return "Standard";
	default:
		throw std::invalid_argument(
			cv::format("FCLKParams :: Invalid hessian type provided: %d", hess_type));
	}
}

_MTF_END_NAMESPACE