#ifndef MTF_GRID_TRACKER2_H
#define MTF_GRID_TRACKER2_H

#include "GridBase.h"

#include <vector>

#define GT_GRID_SIZE_X 10
#define GT_GRID_SIZE_Y 10
#define GT_PATCH_SIZE_X 10
#define GT_PATCH_SIZE_Y 10
#define GT_RESET_AT_EACH_FRAME 1
#define GT_DYN_PATCH_SIZE 0
#define GT_PATCH_CENTROID_INSIDE true
#define GT_FB_ERR_THRESH 0
#define GT_FB_REINIT true
#define GT_USE_TBB true
#define GT_MAX_ITERS 1
#define GT_EPSILON 0.01
#define GT_ENABLE_PYR 0
#define GT_SHOW_TRACKERS 0
#define GT_SHOW_TRACKER_EDGES 0
#define GT_DEBUG_MODE 0

_MTF_BEGIN_NAMESPACE

struct GridTrackerParams{

	int grid_size_x, grid_size_y;
	int patch_size_x, patch_size_y;

	int reset_at_each_frame;
	bool dyn_patch_size;

	bool patch_centroid_inside;

	double fb_err_thresh;
	bool fb_reinit;

	bool use_tbb;

	int max_iters; //! maximum iterations of the GridTracker algorithm to run for each frame
	double epsilon;
	bool enable_pyr;

	bool show_trackers;// show the locations of individual patch trackers
	bool show_tracker_edges;

	bool debug_mode; //! decides whether logging data will be printed for debugging purposes; 
	//! only matters if logging is enabled at compile time

	GridTrackerParams(
		int _grid_size_x, int _grid_size_y,
		int _patch_size_x, int _patch_size_y,
		int _reset_at_each_frame, bool _dyn_patch_size,
		bool _patch_centroid_inside, double fb_err_thresh,
		bool _fb_reinit, bool _use_tbb, int _max_iters, double _epsilon,
		bool _enable_pyr,bool _show_trackers, 
		bool _show_tracker_edges,bool _debug_mode);
	GridTrackerParams(const GridTrackerParams *params = nullptr);

	int getResX() const{ return resx; }
	int getResY() const{ return resy; }

private:
	int resx, resy;
	void updateRes();
};

template<class SSM>
class GridTracker : public GridBase{

public:

	typedef typename SSM::ParamType SSMParams;
	typedef typename SSM::EstimatorParams EstimatorParams;

	typedef GridTrackerParams ParamType;

	GridTracker(const vector<TrackerBase*> _trackers, const ParamType *grid_params,
		const EstimatorParams *_est_params, const SSMParams *ssm_params);
	void initialize(const cv::Mat &corners) override;
	void update() override;
	void setImage(const cv::Mat &img) override;
	void setRegion(const cv::Mat& corners) override;
	const cv::Mat& getRegion() override{ return cv_corners_mat; }
	const uchar* getPixMask() override{ return pix_mask.data(); }
	int getResX() override{ return params.grid_size_x; }
	int getResY() override{ return params.grid_size_y; }
	SSM& getSSM() { return ssm; }

private:

	SSM ssm;
	ParamType params;
	EstimatorParams est_params;

	double centrod_dist_x, centrod_dist_y;

	std::vector<cv::Point2f> prev_pts;
	std::vector<cv::Point2f> curr_pts;

	cv::Mat patch_corners;
	std::vector<uchar> pix_mask;
	VectorXd ssm_update;

	cv::Mat curr_img, prev_img;
	cv::Mat curr_img_disp;

	Matrix2Xd centroid_offset;

	char* patch_win_name;
	//! used for indexing the sub region locations
	MatrixXi _linear_idx;
	int pause_seq;
	bool reinit_at_each_frame;

	std::vector<cv::Point2f> fb_prev_pts;
	VectorXb fb_err_mask;
	bool enable_fb_err_est;

	~GridTracker(){}
	void resetTrackers(bool reinit=true);
	void backwardEstimation();
	void showTrackers();

};

_MTF_END_NAMESPACE

#endif

