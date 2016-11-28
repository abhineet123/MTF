#ifndef MTF_MC_ZNCC_H
#define MTF_MC_ZNCC_H

#include "ZNCC.h"

_MTF_BEGIN_NAMESPACE

class MCZNCC : public ZNCC{
public:
	MCZNCC(const ParamType *zncc_params = nullptr);
};

_MTF_END_NAMESPACE

#endif