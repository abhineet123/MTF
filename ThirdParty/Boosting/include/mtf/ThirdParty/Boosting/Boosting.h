#ifndef MTF_BOOSTING_H
#define MTF_BOOSTING_H

#include "object_tracker.h"
#include "mtf/TrackerBase.h"
#include <memory>

struct BoostingParams{	
	BoostingParams(int _algorithm, int _num_classifiers, float _overlap, float _search_factor,
		float _pos_radius_train, int _neg_num_train, int _num_features);
	BoostingParams(const BoostingParams *params = nullptr);
	const cv::BoostingTrackerParams& get() const{ return _params; }
private:
	cv::BoostingTrackerParams _params;
};


class Boosting : public mtf::TrackerBase{
public:
	typedef BoostingParams ParamType;
	typedef std::unique_ptr<cv::BoostingTracker> BoostingTrackerPtr;

	Boosting(const ParamType *mil_params = nullptr);
	~Boosting(){
		/**
		ObjectTracker object cannot be safely deleted
		(or enclosed in unique_ptr) due to a bug in libmil 
		that causes segmentation fault
		*/
	}
	void setImage(const cv::Mat &img) override{ curr_img = img; }
	int inputType() const override{ return CV_8UC1; }
	void initialize(const cv::Mat& corners) override;
	void update() override;
private:
	ParamType params;
	cv::BoostingTracker *tracker;
	cv::Mat curr_img;
	cv::Rect curr_location;
	CvRect init_bb;
	void updateCVCorners();
};

#endif 
