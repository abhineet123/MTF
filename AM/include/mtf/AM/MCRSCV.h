#ifndef MTF_MC_RSCV_H
#define MTF_MC_RSCV_H

#include "RSCV.h"

_MTF_BEGIN_NAMESPACE

class MCRSCV : public RSCV{
public:
	MCRSCV(const ParamType *rscv_params = nullptr);
};

_MTF_END_NAMESPACE

#endif