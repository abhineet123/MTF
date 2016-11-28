#ifndef MTF_MC_SSD_H
#define MTF_MC_SSD_H

#include "SSD.h"

_MTF_BEGIN_NAMESPACE

class MCSSD : public SSD{
public:
	MCSSD(const ParamType *ssd_params = nullptr);
};

_MTF_END_NAMESPACE

#endif