#ifndef MTF_MC_SAD_H
#define MTF_MC_SAD_H

#include "SAD.h"

_MTF_BEGIN_NAMESPACE

class MCSAD : public SAD{
public:
	MCSAD(const ParamType *ssd_params = nullptr);
};

_MTF_END_NAMESPACE

#endif