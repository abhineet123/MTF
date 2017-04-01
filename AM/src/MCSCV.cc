#include "mtf/AM/MCSCV.h"

_MTF_BEGIN_NAMESPACE

MCSCV::MCSCV(const ParamType *scv_params) : 
SCV(scv_params, 3){
	printf("Using Multi Channel variant\n");
	name = "mcscv";
}

_MTF_END_NAMESPACE

