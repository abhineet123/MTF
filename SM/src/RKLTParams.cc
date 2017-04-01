#include "mtf/SM/RKLTParams.h"

#define RKLT_ENABLE_SPI true
#define RKLT_ENABLE_FEEDBACK true
#define RKLT_FAILURE_DETECTION true
#define RKLT_FAILURE_THRESH 15.0
#define RKLT_DEBUG_MODE false

_MTF_BEGIN_NAMESPACE

RKLTParams::RKLTParams(
bool _enable_spi, bool _enable_feedback,
bool _failure_detection, double _failure_thresh,
bool _debug_mode){
	enable_spi = _enable_spi;
	enable_feedback = _enable_feedback;
	failure_detection = _failure_detection;
	failure_thresh = _failure_thresh;
	debug_mode = _debug_mode;
}
RKLTParams::RKLTParams(const RKLTParams *params) :
enable_spi(RKLT_ENABLE_SPI),
enable_feedback(RKLT_ENABLE_FEEDBACK),
failure_detection(RKLT_FAILURE_DETECTION),
failure_thresh(RKLT_FAILURE_THRESH),
debug_mode(RKLT_DEBUG_MODE){
	if(params){
		enable_spi = params->enable_spi;
		enable_feedback = params->enable_feedback;
		failure_detection = params->failure_detection;
		failure_thresh = params->failure_thresh;
		debug_mode = params->debug_mode;
	}
}

_MTF_END_NAMESPACE
