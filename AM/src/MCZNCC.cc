#include "mtf/AM/MCZNCC.h"

_MTF_BEGIN_NAMESPACE

MCZNCC::MCZNCC(const ParamType *zncc_params) : 
ZNCC(zncc_params, 3){
	printf("Using Multi Channel variant\n");
	name = "mczncc";
}

_MTF_END_NAMESPACE

