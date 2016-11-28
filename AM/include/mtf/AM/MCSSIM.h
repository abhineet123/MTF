#ifndef MTF_MC_SSIM_H
#define MTF_MC_SSIM_H

#include "SSIM.h"

_MTF_BEGIN_NAMESPACE

// Multi Channel Normalized Cross Correlation
class MCSSIM : public SSIM{
public:
	MCSSIM(const ParamType *ssim_params = nullptr);
};

_MTF_END_NAMESPACE

#endif