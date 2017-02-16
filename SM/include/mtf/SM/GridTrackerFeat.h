#ifndef MTF_GRID_TRACKER_FEAT_H
#define MTF_GRID_TRACKER_FEAT_H

#include "GridBase.h"
#include <vector>
#include "opencv2/nonfree/nonfree.hpp"
#include <flann/flann.hpp>
#include <memory>
#include "FLANNParams.h"

_MTF_BEGIN_NAMESPACE

struct SIFTParams{

	int n_features;
	int n_octave_layers;
	double contrast_thresh;
	double edge_thresh;
	double sigma;

	SIFTParams(
		int _n_features,
		int _n_octave_layers,
		double _contrast_thresh,
		double _edge_thresh,
		double _sigma);
	SIFTParams(const SIFTParams *params = nullptr);
	void print() const;

};

struct GridTrackerFeatParams{

	int grid_size_x, grid_size_y;
	int search_window_x, search_window_y;

	bool init_at_each_frame;
	bool detect_keypoints;

	bool rebuild_index;

	//! maximum iterations of the GridTracker algorithm to run for each frame
	int max_iters;
	double epsilon;
	bool enable_pyr;

	//! show the locations of individual key points
	bool show_keypoints;
	//! show the matches between keypoints
	bool show_matches;

	bool debug_mode;

	GridTrackerFeatParams(
		int _grid_size_x, int _grid_size_y,
		int _search_win_x, int _search_win_y,
		bool _init_at_each_frame,
		bool _detect_keypoints, bool _rebuild_index,
		int _max_iters, double _epsilon, bool _enable_pyr,
		bool _show_keypoints, bool _show_matches,
		bool _debug_mode);
	GridTrackerFeatParams(const GridTrackerFeatParams *params = nullptr);

	int getResX() const{ return grid_size_x; }
	int getResY() const{ return grid_size_y; }

};
template<class SSM>
class GridTrackerFeat : public GridBase{

public:
	typedef GridTrackerFeatParams ParamType;
	typedef typename SSM::ParamType SSMParams;
	typedef typename SSM::EstimatorParams EstimatorParams;

	typedef FLANNParams::IdxType IdxType;
	typedef FLANNParams::SearchType SearchType;

	//typedef std::unique_ptr<cv::Feature2D> Feature2DPtr;
	typedef std::unique_ptr<cv::SIFT> SIFTPtr;
	typedef flann::Index<flann::L2<float> > flannIdxT;
	typedef flann::Matrix<float> flannMatT;
	typedef flann::Matrix<int> flannResultT;
	typedef std::shared_ptr<flannMatT> FlannMatPtr;
	typedef std::shared_ptr<flannIdxT> FlannIdxPtr;

	GridTrackerFeat(
		const ParamType *grid_params = nullptr,
		const SIFTParams *_sift_params = nullptr,
		const FLANNParams *_flann_params = nullptr,
		const EstimatorParams *_est_params = nullptr,
		const SSMParams *ssm_params = nullptr);

	void initialize(const cv::Mat &corners) override;
	void update() override;
	void setImage(const cv::Mat &img) override;
	void setRegion(const cv::Mat& corners) override;
	const uchar* getPixMask() override{ return pix_mask.data(); }
	int getResX() override{ return params.grid_size_x; }
	int getResY() override{ return params.grid_size_y; }
	const cv::Mat& getRegion() override{ return cv_corners_mat; }
	int inputType() const override{ return CV_32FC1; }
	SSM& getSSM() { return ssm; }

private:

	SSM ssm;
	ParamType params;	
	SIFTParams sift_params;
	FLANNParams flann_params;
	EstimatorParams est_params;

	SIFTPtr feat;
	flannIdxT *flann_idx;

	FlannMatPtr flann_dataset, flann_query;
	cv::Mat best_idices;
	cv::Mat best_distances;

	cv::Mat curr_img_float, curr_img, prev_img;
	std::vector<cv::KeyPoint> curr_key_pts, prev_key_pts;
	std::vector<cv::Point2f> curr_pts, prev_pts;

	int n_pts, n_key_pts, n_good_key_pts;
	cv::Size search_window;
	cv::Mat curr_descriptors, prev_descriptors;
	std::vector<uchar> pix_mask;
	std::vector<int> good_indices;

	cv::Mat warp_mat;
	VectorXd ssm_update;

	cv::Mat curr_img_uchar;

	char* patch_win_name;

	MatrixXi _linear_idx;//used for indexing the sub region locations
	int pause_seq;

	~GridTrackerFeat(){}
	void showKeyPoints();

};

_MTF_END_NAMESPACE

#endif

