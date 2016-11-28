#include "mtf/AM/MCNCC.h"

_MTF_BEGIN_NAMESPACE

MCNCC::MCNCC(const ParamType *ncc_params) : 
NCC(ncc_params, 3){
	name = "mcncc";
	printf("Using Multi Channel variant\n");
}

_MTF_END_NAMESPACE

