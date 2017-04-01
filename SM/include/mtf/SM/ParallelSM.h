#ifndef MTF_PARALLEL_SM_H
#define MTF_PARALLEL_SM_H

#include "CompositeSM.h"
#include "mtf/SM/ParallelParams.h"

_MTF_BEGIN_NAMESPACE

/**
run multiple search methods in parallel with the same AM/SSM; although the code here
is currently identical to ParallelTracker, a seperate module exists for future extensions
that take advantage of the additional information that is available to this class through
direct access to the underlying AM and SSM
*/
template<class AM, class SSM>
class ParallelSM : public CompositeSM<AM, SSM> {

public:
	typedef ParallelParams ParamType;
	typedef ParamType::PrlEstMethod PrlEstMethod;

	using typename CompositeSM<AM, SSM>::SM;
	typedef typename SM::SSMParams SSMParams;
	using CompositeSM<AM, SSM>::name;
	using CompositeSM<AM, SSM>::cv_corners_mat;
	using CompositeSM<AM, SSM>::trackers;
	using CompositeSM<AM, SSM>::n_trackers;
	using CompositeSM<AM, SSM>::input_type;

	ParallelSM(const vector<SM*> _trackers, const ParamType *parl_params,
		 const SSMParams *ssm_params);
	void setImage(const cv::Mat &img) override;
	void initialize(const cv::Mat &corners) override;
	void update() override;
	const cv::Mat& getRegion() override { return cv_corners_mat; }
	void setRegion(const cv::Mat& corners)  override;

protected:

	SSM ssm;
	ParamType params;

	bool failure_detected;

	int buffer_id;
	bool buffer_filled;
	vector<cv::Mat> img_buffer, corners_buffer;

	cv::Mat curr_img;
	cv::Mat mean_corners_cv;

	int ssm_state_size;
	std::vector<VectorXd> ssm_states;
	VectorXd mean_state;

};


_MTF_END_NAMESPACE

#endif

