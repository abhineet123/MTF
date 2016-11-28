#include "mtf/AM/MCSPSS.h"

_MTF_BEGIN_NAMESPACE

MCSPSS::MCSPSS(const ParamType *spss_params) :
SPSS(spss_params, 3){
	name = "mcspss";
	printf("Using Multi Channel variant\n");
}

_MTF_END_NAMESPACE

