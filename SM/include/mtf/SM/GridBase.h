#ifndef MTF_GRID_BASE_H
#define MTF_GRID_BASE_H

#include "CompositeBase.h"

_MTF_BEGIN_NAMESPACE

class GridBase : public CompositeBase{
public:
	GridBase(const vector<TrackerBase*> _trackers):
		CompositeBase(_trackers){}
	GridBase(){}
	virtual const uchar* getPixMask() = 0;
	virtual int getResX() = 0;
	virtual int getResY() = 0;
};

_MTF_END_NAMESPACE

#endif

