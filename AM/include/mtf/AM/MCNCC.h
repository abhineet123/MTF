#ifndef MTF_MC_NCC_H
#define MTF_MC_NCC_H

#include "NCC.h"

_MTF_BEGIN_NAMESPACE

// Multi Channel Normalized Cross Correlation
class MCNCC : public NCC{
public:
	MCNCC(const ParamType *ncc_params = nullptr);
};

_MTF_END_NAMESPACE

#endif