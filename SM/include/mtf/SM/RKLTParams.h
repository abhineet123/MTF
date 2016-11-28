#ifndef MTF_RKLT_PARAMS_H
#define MTF_RKLT_PARAMS_H

#include "mtf/Macros/common.h"

_MTF_BEGIN_NAMESPACE

struct RKLTParams{
	bool enable_spi; 
	bool enable_feedback;
	bool failure_detection;
	double failure_thresh;
	bool debug_mode;

	RKLTParams(bool _enable_spi, bool _enable_feedback,
		bool _failure_detection, double _failure_thresh,
		bool _debug_mode);
	RKLTParams(const RKLTParams *params = nullptr);
};

_MTF_END_NAMESPACE

#endif

