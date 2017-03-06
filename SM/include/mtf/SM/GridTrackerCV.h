#ifndef MTF_GRID_TRACKER_CV_H
#define MTF_GRID_TRACKER_CV_H

#include "GridBase.h"
#include <vector>

#define GTCV_GRID_SIZE_X 10
#define GTCV_GRID_SIZE_Y 10
#define GTCV_SEARCH_WINDOW_X 10
#define GTCV_SEARCH_WINDOW_Y 10
#define GTCV_RESET_AT_EACH_FRAME 1
#define GTCV_PATCH_CENTROID_INSIDE 0
#define GTCV_FB_ERR_THRESH 0
#define GTCV_PYRAMID_LEVELS 0
#define GTCV_USE_MIN_EIG_VALS 0
#define GTCV_MIN_EIG_THRESH 1e-4
#define GTCV_MAX_ITERS 30
#define GTCV_UCHAR_INPUT 1
#define GTCV_EPSILON 0.01
#define GTCV_SHOW_PTS 0
#define GTCV_DEBUG_MODE 0

_MTF_BEGIN_NAMESPACE

struct GridTrackerCVParams{

	int grid_size_x, grid_size_y;
	int search_window_x, search_window_y;
	bool reset_at_each_frame;
	bool patch_centroid_inside;

	double fb_err_thresh;

	int pyramid_levels;
	bool use_min_eig_vals;
	double min_eig_thresh;

	int max_iters; //! maximum iterations of the GridTrackerCV algorithm to run for each frame
	double epsilon;

	bool uchar_input;

	bool show_pts;// show the locations of individual patch trackers

	bool debug_mode; //! decides whether logging data will be printed for debugging purposes; 
	//! only matters if logging is enabled at compile time

	GridTrackerCVParams(
		int _grid_size_x, int _grid_size_y,
		int _search_window_x, int _search_window_y,
		bool reset_at_each_frame, bool _patch_centroid_inside,
		double _fb_err_thresh,
		int _pyramid_levels, bool _use_min_eig_vals,
		double _min_eig_thresh, int _max_iters, 
		double _epsilon, bool _uchar_input, bool _show_pts,
		bool _debug_mode);
	GridTrackerCVParams(const GridTrackerCVParams *params = nullptr);
	
	int getResX() const{ return resx; }
	int getResY() const{ return resy; }

private:
	int resx, resy;
	void updateRes();
};

template<class SSM>
class GridTrackerCV : public GridBase{

public:
	typedef GridTrackerCVParams ParamType;
	typedef typename SSM::ParamType SSMParams;
	typedef typename SSM::EstimatorParams EstimatorParams;


	GridTrackerCV(const ParamType *grid_params, const EstimatorParams *_est_params,
		const SSMParams *ssm_params);

	void initialize(const cv::Mat &corners) override;
	void update() override;
	void setImage(const cv::Mat &img) override;
	void setRegion(const cv::Mat& corners) override;
	const uchar* getPixMask() override{ return pix_mask.data(); }
	int getResX() override{ return params.grid_size_x; }
	int getResY() override{ return params.grid_size_y; }
	const cv::Mat& getRegion() override{ return cv_corners_mat; }
	int inputType() const override{ 
		return params.uchar_input ? CV_8UC1 : CV_32FC1;
	}
	SSM& getSSM() { return ssm; }

private:

	SSM ssm;
	ParamType params;
	EstimatorParams est_params;

	cv::Mat curr_img_in, curr_img, prev_img;
	cv::Mat curr_pts_mat, prev_pts_mat;

	std::vector<cv::Point2f> curr_pts, prev_pts;
	int n_pts;
	cv::Size search_window;
	cv::TermCriteria lk_term_criteria;

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

	bool enable_fb_err_est;
	std::vector<cv::Point2f> fb_prev_pts;
	VectorXb fb_err_mask;

	~GridTrackerCV(){}
	void resetPts();
	void backwardEstimation();
	void showPts();
};

_MTF_END_NAMESPACE

#endif

