#include "mtf/AM/NSSD.h"

_MTF_BEGIN_NAMESPACE

//! value constructor
NSSDParams::NSSDParams(const AMParams *am_params,
double _norm_pix_max, double _norm_pix_min,
bool _debug_mode) :
AMParams(am_params){
	norm_pix_max = _norm_pix_max;
	norm_pix_min = _norm_pix_min;
	debug_mode = _debug_mode;
}
//! default/copy constructor
NSSDParams::NSSDParams(const NSSDParams *params) :
AMParams(params),
norm_pix_min(NSSD_NORM_MIN),
norm_pix_max(NSSD_NORM_MAX),
debug_mode(NSSD_DEBUG){
	if(params){
		norm_pix_max = params->norm_pix_max;
		norm_pix_min = params->norm_pix_min;
		debug_mode = params->debug_mode;
	}
}
NSSD::NSSD(const ParamType *nssd_params) : 
SSDBase(nssd_params), params(nssd_params){
	printf("\n");
	printf("Using  Normalized Sum of Squared Differences AM with:\n");
	printf("norm_pix_min: %f\n", params.norm_pix_min);
	printf("norm_pix_max: %f\n", params.norm_pix_max);

	name = "normalized ssd";

	pix_norm_mult = (params.norm_pix_max - params.norm_pix_min) / (PIX_MAX - PIX_MIN);
	pix_norm_add = params.norm_pix_min;
}

_MTF_END_NAMESPACE

