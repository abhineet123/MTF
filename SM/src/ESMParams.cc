#include "mtf/SM/ESMParams.h"

#define ESM_MAX_ITERS 10
#define ESM_EPSILON 0.01
#define ESM_JAC_TYPE 0
#define ESM_HESS_TYPE 0
#define ESM_SEC_ORD_HESS false
#define ESM_CHAINED_WARP false
#define ESM_LEVEN_MARQ false
#define ESM_LM_DELTA_INIT 0.01
#define ESM_LM_DELTA_UPDATE 10
#define ESM_ENABLE_SPI false
#define ESM_SPI_THRESH 10
#define ESM_DEBUG_MODE false

_MTF_BEGIN_NAMESPACE

ESMParams::ESMParams(int _max_iters, double _epsilon,
JacType _jac_type, HessType _hess_type, bool _sec_ord_hess,
bool _chained_warp, bool _leven_marq, double _lm_delta_init,
double _lm_delta_update, bool _enable_spi, double _spi_thresh,
bool _debug_mode) :
max_iters(_max_iters),
epsilon(_epsilon),
jac_type(_jac_type),
hess_type(_hess_type),
sec_ord_hess(_sec_ord_hess),
chained_warp(_chained_warp),
leven_marq(_leven_marq),
lm_delta_init(_lm_delta_init),
lm_delta_update(_lm_delta_update),
enable_spi(_enable_spi),
spi_thresh(_spi_thresh),
debug_mode(_debug_mode){}

// default and copy constructor
ESMParams::ESMParams(const ESMParams *params) :
max_iters(ESM_MAX_ITERS), epsilon(ESM_EPSILON),
jac_type(static_cast<JacType>(ESM_JAC_TYPE)),
hess_type(static_cast<HessType>(ESM_HESS_TYPE)),
sec_ord_hess(ESM_SEC_ORD_HESS),
chained_warp(ESM_CHAINED_WARP),
leven_marq(ESM_LEVEN_MARQ),
lm_delta_init(ESM_LM_DELTA_INIT),
lm_delta_update(ESM_LM_DELTA_UPDATE),
enable_spi(ESM_ENABLE_SPI),
spi_thresh(ESM_SPI_THRESH),
debug_mode(ESM_DEBUG_MODE){
	if(params){
		max_iters = params->max_iters;
		epsilon = params->epsilon;
		jac_type = params->jac_type;
		hess_type = params->hess_type;
		sec_ord_hess = params->sec_ord_hess;
		chained_warp = params->chained_warp;
		leven_marq = params->leven_marq;
		lm_delta_init = params->lm_delta_init;
		lm_delta_update = params->lm_delta_update;
		enable_spi = params->enable_spi;
		spi_thresh = params->spi_thresh;
		debug_mode = params->debug_mode;
	}
}

const char* ESMParams::toString(JacType _jac_type){
	switch(_jac_type){
	case JacType::Original:
		return "Original ESM Jacobian";
	case JacType::DiffOfJacs:
		return "Difference of Jacobians";
	default:
		throw std::invalid_argument(
			cv::format("ESMParams :: Invalid jacobian type provided: %d", _jac_type));
	}
}

const char* ESMParams::toString(HessType _hess_type){
	switch(_hess_type){
	case HessType::InitialSelf:
		return "Initial Self Hessian";
	case HessType::CurrentSelf:
		return "Current Self Hessians";
	case HessType::Std:
		return "Standard Hessians";
	case HessType::Original:
		return "Original ESM Hessian";
	case HessType::SumOfStd:
		return "Sum of Standard Hessians";
	case HessType::SumOfSelf:
		return "Sum of Self Hessians";
	default:
		throw std::invalid_argument(
			cv::format("ESMParams :: Invalid hessian type provided: %d", _hess_type));
	}
}

_MTF_END_NAMESPACE


