#include "mtf/AM/MCRSCV.h"

_MTF_BEGIN_NAMESPACE

MCRSCV::MCRSCV(const ParamType *rscv_params) : 
RSCV(rscv_params, 3){
	printf("Using Multi Channel variant\n");
	name = "mcrscv";
}

_MTF_END_NAMESPACE

