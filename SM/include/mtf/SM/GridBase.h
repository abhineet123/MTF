#ifndef MTF_GRID_BASE_H
#define MTF_GRID_BASE_H

#include "CompositeBase.h"

_MTF_BEGIN_NAMESPACE

class GridBase : public CompositeBase{
public:
	GridBase(const vector<TrackerBase*> _trackers) :
		CompositeBase(_trackers), pix_mask_needed(false){}
	GridBase(){}
	virtual void initPixMask(){ pix_mask_needed = true; }
	virtual const uchar* getPixMask() = 0;
	virtual int getResX() = 0;
	virtual int getResY() = 0;
protected:
	bool pix_mask_needed;
};

_MTF_END_NAMESPACE

#endif

