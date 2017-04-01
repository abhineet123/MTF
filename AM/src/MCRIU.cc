#include "mtf/AM/MCRIU.h"

_MTF_BEGIN_NAMESPACE

MCRIU::MCRIU(const ParamType *riu_params) :
RIU(riu_params, 3){
	name = "mcriu";
	printf("Using Multi Channel variant\n");
}

_MTF_END_NAMESPACE

