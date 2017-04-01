#ifndef MTF_MC_MI_H
#define MTF_MC_MI_H

#include "MI.h"

_MTF_BEGIN_NAMESPACE

// Multi Channel Mutual Information
class MCMI : public MI{
public:
	MCMI(const ParamType *mi_params = nullptr);
};

_MTF_END_NAMESPACE

#endif