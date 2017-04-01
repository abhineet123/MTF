#include "mtf/AM/MCSAD.h"

_MTF_BEGIN_NAMESPACE

MCSAD::MCSAD(const ParamType *ssd_params) : 
SAD(ssd_params, 3){
	printf("Using Multi Channel variant\n");
	name = "mcsad";
}

_MTF_END_NAMESPACE

