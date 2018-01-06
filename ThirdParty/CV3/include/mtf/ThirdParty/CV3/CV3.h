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
		KCF
	};
	TrackerType tracker_type;

	int max_iters;
	int resx, resy;
	double lambda;
	double thresh_grad;
	int pyr_n_levels;
	int pyr_level_to_stop;

	CV3Params(TrackerType _sm_type,
		int _max_iters,
		int _resx,
		int _resy,
		double _lambda,
		double _thresh_grad,
		int _pyr_n_levels,
		int _pyr_level_to_stop);

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
	void setRegion(const cv::Mat& corners) override;
	void setImage(const cv::Mat &img) override;
	int inputType() const override{ return CV_8UC1; }
	void updateCorners();

protected:
	ParamType params;
	cv::Ptr<cv::Tracker> tracker;
	cv::Mat curr_img_cv;
	
private:
	virtual ~CV3(){}
};
#endif
//#endif
