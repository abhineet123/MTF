#ifndef MTF_PYRAMIDAL_TRACKER_H
#define MTF_PYRAMIDAL_TRACKER_H

#include "CompositeBase.h"
#include "mtf/SM/PyramidalParams.h"

_MTF_BEGIN_NAMESPACE

// run multiple trackers on a Gaussian image pyramid
class PyramidalTracker : public CompositeBase {

public:
	typedef PyramidalParams ParamType;
	ParamType params;

	vector<cv::Size> img_sizes;
	vector<cv::Mat> img_pyramid;
	double overall_scale_factor;
	bool external_img_pyramid;

	PyramidalTracker(const vector<TrackerBase*> _trackers, const ParamType *parl_params);
	void setImage(const cv::Mat &img) override;
	void initialize(const cv::Mat &corners) override;
	void update() override;
	const cv::Mat& getRegion() override { 
		return trackers[0]->getRegion();
	}
	void setRegion(const cv::Mat& corners)  override;
	int inputType() const override{
		return trackers[0]->inputType();
	}
	void setImagePyramid(const vector<cv::Mat> &_img_pyramid);
	const vector<cv::Mat>& getImagePyramid() const{ return img_pyramid; }
	void updateImagePyramid();
	void showImagePyramid();
};

_MTF_END_NAMESPACE

#endif

