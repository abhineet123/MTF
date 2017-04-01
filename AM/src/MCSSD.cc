#include "mtf/AM/MCSSD.h"

_MTF_BEGIN_NAMESPACE

MCSSD::MCSSD(const ParamType *ssd_params) : 
SSD(ssd_params, 3){
	printf("Using Multi Channel variant\n");
	name = "mcssd";
}

_MTF_END_NAMESPACE

