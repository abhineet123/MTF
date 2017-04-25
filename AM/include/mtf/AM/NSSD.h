#ifndef MTF_NSSD_H
#define MTF_NSSD_H

#include "SSDBase.h"

#define NSSD_NORM_MIN 0.0
#define NSSD_NORM_MAX 1.0
#define NSSD_DEBUG false

_MTF_BEGIN_NAMESPACE

struct NSSDParams : AMParams{
	//! minimum pix value after normalization
	double norm_pix_min;
	//! maximum pix value after normalization
	double norm_pix_max;
	//! decides whether logging data will be printed for debugging purposes; 
	//! only matters if logging is enabled at compile time
	bool debug_mode;

	//! value constructor
	NSSDParams(const AMParams *am_params,
		double _norm_pix_max, double _norm_pix_min,
		bool _debug_mode);
	//! default/copy constructor
	NSSDParams(const NSSDParams *params = nullptr);
};
class NSSD : public SSDBase{
public:

	typedef NSSDParams ParamType; 

	NSSD(const ParamType *nssd_params = nullptr);

protected:
	ParamType params;

};

_MTF_END_NAMESPACE

#endif