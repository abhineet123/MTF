#ifndef MTF_MC_CCRE_H
#define MTF_MC_CCRE_H

#include "CCRE.h"

_MTF_BEGIN_NAMESPACE

//! Multi Channel Cross Cumulative Residual Entropy
class MCCCRE : public CCRE{
public:
	MCCCRE(const ParamType *ccre_params = nullptr);
};

_MTF_END_NAMESPACE

#endif