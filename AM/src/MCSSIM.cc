#include "mtf/AM/MCSSIM.h"

_MTF_BEGIN_NAMESPACE

MCSSIM::MCSSIM(const ParamType *ssim_params) :
SSIM(ssim_params, 3){
	name = "mcssim";
	printf("Using Multi Channel variant\n");
}

_MTF_END_NAMESPACE

