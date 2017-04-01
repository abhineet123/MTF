#ifndef MTF_CASCADE_SM_H
#define MTF_CASCADE_SM_H

#include "CompositeSM.h"
#include "mtf/SM/CascadeParams.h"

_MTF_BEGIN_NAMESPACE

/**
run multiple search methods in cascade with the same AM/SSM; although the code here
is currently identical to CascadeTracker, a seperate module exists for future extensions
that take advantage of the additional information that is available to the Cascade through
direct access to the underlying AM and SSM
*/
template<class AM, class SSM>
class CascadeSM : public CompositeSM < AM, SSM > {

public:
	typedef CascadeParams ParamType;

	using typename CompositeSM<AM, SSM>::SM;
	typedef typename SM::SSMParams SSMParams;
	using CompositeSM<AM, SSM>::name;
	using CompositeSM<AM, SSM>::cv_corners_mat;
	using CompositeSM<AM, SSM>::trackers;
	using CompositeSM<AM, SSM>::n_trackers;
	using CompositeSM<AM, SSM>::input_type;

	CascadeSM(const vector<SM*> _trackers, const ParamType *casc_params);
	void initialize(const cv::Mat &corners) override;
	void update() override;
	void setImage(const cv::Mat &img) override;
	const cv::Mat& getRegion()  override{ return trackers[n_trackers - 1]->getRegion(); }
	void setRegion(const cv::Mat& corners)  override;

protected:

	ParamType params;

	bool failure_detected;
	vector<cv::Mat> img_buffer, corners_buffer;
	int buffer_id;
	bool buffer_filled;
	cv::Mat curr_img;

	void updateTrackers(const cv::Mat &img);
};
_MTF_END_NAMESPACE

#endif

