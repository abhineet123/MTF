#include "mtf/SM/FALKParams.h"

#define FALK_MAX_ITERS 10
#define FALK_EPSILON 0.01
#define FALK_HESS_TYPE 0
#define FALK_SECOND_ORDER_HESS false
#define FALK_SHOW_GRID false
#define FALK_SHOW_PATCH false
#define FALK_PATCH_RESIZE_FACTOR 1.0
#define FALK_WRITE_FRAMES false
#define FALK_ENABLE_LEARNING false
#define FALK_LEVEN_MARQ false
#define FALK_LM_DELTA_INIT 0.01
#define FALK_LM_DELTA_UPDATE 10
#define FALK_DEBUG_MODE false

_MTF_BEGIN_NAMESPACE

FALKParams::FALKParams(int _max_iters, double _epsilon,
HessType _hess_type, bool _sec_ord_hess,
bool _show_grid, bool _show_patch,
double _patch_resize_factor,
bool _write_frames, bool _leven_marq,
double _lm_delta_init, double _lm_delta_update,
bool _enable_learning, bool _debug_mode) :
max_iters(_max_iters),
epsilon(_epsilon),
hess_type(_hess_type),
sec_ord_hess(_sec_ord_hess),
show_grid(_show_grid),
show_patch(_show_patch),
patch_resize_factor(_patch_resize_factor),
write_frames(_write_frames),
enable_learning(_enable_learning),
leven_marq(_leven_marq),
lm_delta_init(_lm_delta_init),
lm_delta_update(_lm_delta_update),
debug_mode(_debug_mode){}

FALKParams::FALKParams(const FALKParams *params) :
max_iters(FALK_MAX_ITERS),
epsilon(FALK_EPSILON),
hess_type(static_cast<HessType>(FALK_HESS_TYPE)),
sec_ord_hess(FALK_SECOND_ORDER_HESS),
show_grid(FALK_SHOW_GRID),
show_patch(FALK_SHOW_PATCH),
patch_resize_factor(FALK_PATCH_RESIZE_FACTOR),
write_frames(FALK_WRITE_FRAMES),
enable_learning(FALK_ENABLE_LEARNING),
leven_marq(FALK_LEVEN_MARQ),
lm_delta_init(FALK_LM_DELTA_INIT),
lm_delta_update(FALK_LM_DELTA_UPDATE),
debug_mode(FALK_DEBUG_MODE){
	if(params){
		max_iters = params->max_iters;
		epsilon = params->epsilon;
		hess_type = params->hess_type;
		sec_ord_hess = params->sec_ord_hess;
		show_grid = params->show_grid;
		show_patch = params->show_patch;
		patch_resize_factor = params->patch_resize_factor;
		write_frames = params->write_frames;
		enable_learning = params->enable_learning;
		leven_marq = params->leven_marq;
		lm_delta_init = params->lm_delta_init;
		lm_delta_update = params->lm_delta_update;
		debug_mode = params->debug_mode;
	}
}

const char* FALKParams::toString(HessType hess_type){
	switch(hess_type){
	case HessType::InitialSelf:
		return "Initial Self";
	case HessType::CurrentSelf:
		return "Current Self";
	case HessType::Std:
		return "Standard";
	default:
		throw std::invalid_argument(
			cv::format("FALKParams :: Invalid hessian type provided: %d", hess_type));
	}
}

_MTF_END_NAMESPACE

