#ifndef MTF_MC_SCV_H
#define MTF_MC_SCV_H

#include "SCV.h"

_MTF_BEGIN_NAMESPACE

class MCSCV : public SCV{
public:
	MCSCV(const ParamType *scv_params = nullptr);
};

_MTF_END_NAMESPACE

#endif