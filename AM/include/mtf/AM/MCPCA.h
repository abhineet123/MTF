#ifndef MTF_MC_PCA_H
#define MTF_MC_PCA_H

#include "PCA.h"

_MTF_BEGIN_NAMESPACE

class MCPCA : public PCA{
public:
	MCPCA(const ParamType *pca_params = nullptr);
};

_MTF_END_NAMESPACE

#endif