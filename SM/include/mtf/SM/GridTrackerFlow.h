#ifndef MTF_GRID_TRACKER_FLOW_H
#define MTF_GRID_TRACKER_FLOW_H

#include "GridBase.h"
#include "GridTrackerFlowParams.h"
#include <vector>

_MTF_BEGIN_NAMESPACE

template<class AM, class SSM>
class GridTrackerFlow : public GridBase{

public:

	typedef GridTrackerFlowParams ParamType;
	typedef typename  AM::ParamType AMParams;
	typedef typename SSM::ParamType SSMParams;
	typedef typename SSM::EstimatorParams EstimatorParams;


	GridTrackerFlow(
		const ParamType *grid_params = nullptr,
		const EstimatorParams *_est_params = nullptr,
		const AMParams *am_params = nullptr,
		const SSMParams *ssm_params = nullptr);

	void initialize(const cv::Mat &corners) override;
	void update() override;
	void setImage(const cv::Mat &img) override;
	void setRegion(const cv::Mat& corners) override;
	const uchar* getPixMask() override{ return pix_mask.data(); }
	int getResX() override{ return params.grid_size_x; }
	int getResY() override{ return params.grid_size_y; }
	const cv::Mat& getRegion() override{ return cv_corners_mat; }
	int inputType() const override{ return am.inputType(); }
	SSM& getSSM() { return ssm; }
	AM& getAM() { return am; }

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

	AM am;
	SSM ssm;
	ParamType params;
	EstimatorParams est_params;

	cv::Mat curr_img, prev_img;
	cv::Mat curr_pts_mat, prev_pts_mat;

	std::vector<cv::Point2f> curr_pts, prev_pts;
	int n_pts;
	cv::Size search_window;
	cv::TermCriteria lk_termination_criteria;

	cv::Mat warp_mat;
	cv::Mat patch_corners;
	std::vector<uchar> lk_status, pix_mask;
	std::vector<float> lk_error;
	int lk_flags;

	VectorXd ssm_update;

	cv::Mat curr_img_disp;

	Matrix2Xd centroid_offset;

	char* patch_win_name;

	MatrixXi _linear_idx;//used for indexing the sub region locations
	int pause_seq;

	~GridTrackerFlow(){}
	void showTrackers();
};

_MTF_END_NAMESPACE

#endif

