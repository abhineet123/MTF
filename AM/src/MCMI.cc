#include "mtf/AM/MCMI.h"

_MTF_BEGIN_NAMESPACE

MCMI::MCMI(const ParamType *mi_params) :
MI(mi_params, 3){
	name = "mcmi";
	printf("Using Multi Channel variant\n");
}

_MTF_END_NAMESPACE

