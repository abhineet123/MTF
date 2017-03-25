#ifndef MTF_FEATURE_BASE_H
#define MTF_FEATURE_BASE_H

#include "mtf/TrackerBase.h"

_MTF_BEGIN_NAMESPACE

class FeatureBase : public TrackerBase{
public:
	FeatureBase() : TrackerBase(), pix_mask_needed(false){}
	virtual void initPixMask(){ pix_mask_needed = true; }
	virtual const uchar* getPixMask() = 0;
	virtual int getResX() = 0;
	virtual int getResY() = 0;
	virtual bool detect(const cv::Mat &mask, cv::Mat &obj_location) = 0;
	virtual bool detect(const cv::Mat &img, const cv::Mat &mask, cv::Mat &obj_location){
		setImage(img);
		return detect(mask, obj_location);
	}
protected:
	bool pix_mask_needed;
};

_MTF_END_NAMESPACE

#endif

