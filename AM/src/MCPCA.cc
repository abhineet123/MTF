#include "mtf/AM/MCPCA.h"

_MTF_BEGIN_NAMESPACE

MCPCA::MCPCA(const ParamType *pca_params) :
PCA(pca_params, 3){
	printf("Using Multi Channel variant\n");
	name = "mcpca";
}

_MTF_END_NAMESPACE

