//#if CV_MAJOR_VERSION >= 3

#ifndef MTF_CV3_H
#define MTF_CV3_H

#include "mtf/TrackerBase.h"
#include <opencv2/tracking.hpp>

struct CV3Params{
	enum class TrackerType{
		MIL,
		BOOSTING,
		MEDIANFLOW,
		TLD,
		KCF,
		GOTURN
	};
	TrackerType tracker_type;
	CV3Params(
		TrackerType _tracker_type);

	CV3Params(const CV3Params *params = nullptr);
	std::string toString(TrackerType _detector_type);

};

class CV3 : public mtf::TrackerBase{
public:

	typedef CV3Params ParamType;
	typedef ParamType::TrackerType TrackerType;

	CV3(const ParamType *cv3_params);	
	void initialize(const cv::Mat &corners) override;
	void update() override;
	const cv::Mat& getRegion() override{ return cv_corners_mat; }
	void setImage(const cv::Mat &img) override;
	int inputType() const override{ return CV_8UC3; }

protected:
	ParamType params;
	cv::Ptr<cv::Tracker> tracker;
	cv::Mat curr_img_cv;
	
private:
	int input_type;
	virtual ~CV3();
};
#endif
//#endif
