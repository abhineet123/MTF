#ifndef MTF_ZNCC_H
#define MTF_ZNCC_H

#include "SSDBase.h"

_MTF_BEGIN_NAMESPACE

struct ZNCCParams : AMParams{
	//! decides whether logging data will be printed for debugging purposes; 
	//! only matters if logging is enabled at compile time
	bool debug_mode;
	//! value constructor
	ZNCCParams(const AMParams *am_params,
		 bool _debug_mode);
	//! default/copy constructor
	ZNCCParams(const ZNCCParams *params = nullptr);
};
// Zero mean Normalized Cross Correlation
class ZNCC : public SSDBase{
public:

	typedef ZNCCParams ParamType;
	ParamType params;
	//! mean, variance and standard deviation of the initial pixel values
	double I0_mean, I0_var, I0_std;
	//! mean, variance and standard deviation of the current pixel values
	double It_mean, It_var, It_std;

	ZNCC(const ParamType *ncc_params = nullptr, const int _n_channels = 1);
	void initializePixVals(const Matrix2Xd& curr_pts) override;
	void updatePixVals(const Matrix2Xd& curr_pts) override;
	double getLikelihood() const override;
};

_MTF_END_NAMESPACE

#endif