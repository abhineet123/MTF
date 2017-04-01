#include "mtf/AM/MCCCRE.h"

_MTF_BEGIN_NAMESPACE

MCCCRE::MCCCRE(const ParamType *ccre_params) : 
CCRE(ccre_params, 3){
	name = "mcccre";
	printf("Using Multi Channel variant\n");
}

_MTF_END_NAMESPACE

