#ifndef MTF_CASCADE_TRACKER_H
#define MTF_CASCADE_TRACKER_H

#include "mtf/SM/CompositeBase.h"
#include "mtf/SM/CascadeParams.h"

_MTF_BEGIN_NAMESPACE


class CascadeTracker : public CompositeBase{

public:
	typedef CascadeParams ParamType;
	ParamType params;

	CascadeTracker(const vector<TrackerBase*> _trackers,
		const ParamType *casc_params=nullptr);
	void initialize(const cv::Mat &corners) override;
	void update() override;
	using CompositeBase::update;
	using CompositeBase::initialize;
	void setRegion(const cv::Mat& corners)  override;
	const cv::Mat& getRegion()  override{ return trackers[n_trackers - 1]->getRegion(); }
	void setImage(const cv::Mat &img) override;

protected:
	bool failure_detected;
	vector<cv::Mat> img_buffer, corners_buffer;
	int buffer_id;
	bool buffer_filled;
	cv::Mat curr_img;

	void updateTrackers(const cv::Mat &img);

};
_MTF_END_NAMESPACE

#endif

