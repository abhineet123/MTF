#ifndef MTF_PARALLEL_TRACKER_H
#define MTF_PARALLEL_TRACKER_H

#include "CompositeBase.h"
#include "mtf/SM/ParallelParams.h"

_MTF_BEGIN_NAMESPACE

//! run multiple trackers in parallel
class ParallelTracker : public CompositeBase {

public:
	typedef ParallelParams ParamType;
	ParamType params;

	bool failure_detected;
	vector<cv::Mat> img_buffer, corners_buffer;
	int buffer_id;
	bool buffer_filled;
	cv::Mat curr_img;

	typedef ParamType::PrlEstMethod EstimationMethod;

	cv::Mat mean_corners_cv;

	ParallelTracker(const vector<TrackerBase*> _trackers, const ParamType *parl_params);
	void setImage(const cv::Mat &img) override;
	void initialize(const cv::Mat &corners) override;
	void update() override;
	const cv::Mat& getRegion() override {
		return mean_corners_cv;
	}
	void setRegion(const cv::Mat& corners)  override;
};

_MTF_END_NAMESPACE

#endif

