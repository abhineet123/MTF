#ifndef MTF_MIL_H
#define MTF_MIL_H

#include "object_tracker.h"
#include "mtf/TrackerBase.h"
#include <memory>

struct MILParams{	
	MILParams(int _algorithm, int _num_classifiers, float _overlap, float _search_factor,
		float _pos_radius_train, int _neg_num_train, int _num_features);
	MILParams(const MILParams *params = nullptr);
	const cv::MILTrackerParams& get() const{ return _params; }
private:
	cv::MILTrackerParams _params;
};


class MIL : public mtf::TrackerBase{
public:
	typedef MILParams ParamType;
	typedef std::unique_ptr<cv::MILTracker> MILTrackerPtr;

	MIL(const ParamType *mil_params = nullptr);
	~MIL(){
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
	cv::MILTracker *tracker;
	cv::Mat curr_img;
	cv::Rect curr_location;
	CvRect init_bb;
	void updateCVCorners();
};

#endif 
