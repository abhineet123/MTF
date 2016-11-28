#ifndef MTF_MC_SPSS_H
#define MTF_MC_SPSS_H

#include "SPSS.h"

_MTF_BEGIN_NAMESPACE

// Multi Channel Sum of Pixelwise Structural Similarity
class MCSPSS : public SPSS{
public:
	MCSPSS(const ParamType *spss_params = nullptr);
};

_MTF_END_NAMESPACE

#endif