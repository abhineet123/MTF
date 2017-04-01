#ifndef MTF_MC_RIU_H
#define MTF_MC_RIU_H

#include "RIU.h"

_MTF_BEGIN_NAMESPACE

// Multi Channel Ratio Image Uniformity
class MCRIU : public RIU{
public:
	MCRIU(const ParamType *riu_params = nullptr);
};

_MTF_END_NAMESPACE

#endif