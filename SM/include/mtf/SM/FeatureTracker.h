#ifndef MTF_FEATURE_TRACKER_H
#define MTF_FEATURE_TRACKER_H

#include "FeatureBase.h"
#ifndef DISABLE_FLANN
#include <flann/flann.hpp>
#include "FLANNParams.h"
#else
#include "FLANNCVParams.h"
#endif
#if CV_MAJOR_VERSION < 3
#include "opencv2/nonfree/nonfree.hpp"
#else
#include "opencv2/xfeatures2d/nonfree.hpp"
#endif

#include <memory>
#include <vector>
#include <boost/any.hpp>

_MTF_BEGIN_NAMESPACE

struct SIFT{
	int n_features;
	int n_octave_layers;
	double contrast_thresh;
	double edge_thresh;
	double sigma;
	SIFT(const vector<boost::any> &params,
		std::string _type = "detector");
	void create(cv::Ptr<cv::Feature2D> &ptr);
};
struct SURF{
	double hessian_threshold;
	int n_octaves;
	int n_octave_layers;
	bool extended;
	bool upright;
	SURF(const vector<boost::any> &params,
		std::string _type = "detector");
	void create(cv::Ptr<cv::Feature2D> &ptr);
};
struct FAST{
	int threshold;
	bool non_max_suppression;
	int type;
	FAST(const vector<boost::any> &params);
	void create(cv::Ptr<cv::Feature2D> &ptr);
};
struct BRISK{
	int thresh = 30;
	int octaves = 3;
	float pattern_scale = 1.0f;
	BRISK(const vector<boost::any> &params,
		std::string _type = "detector");
	void create(cv::Ptr<cv::Feature2D> &ptr);
};
struct MSER{
	int delta = 5;
	int min_area = 60;
	int max_area = 14400;
	double max_variation = 0.25;
	double min_diversity = .2;
	int max_evolution = 200;
	double area_threshold = 1.01;
	double min_margin = 0.003;
	int edge_blur_size = 5;
	MSER(const vector<boost::any> &params);
	void create(cv::Ptr<cv::Feature2D> &ptr);
};
struct ORB{
	int n_features = 500;
	float scale_factor = 1.2f;
	int n_levels = 8;
	int edge_threshold = 31;
	int first_level = 0;
	int WTA_K = 2;
	int score_type =cv:: ORB::HARRIS_SCORE;
	int patch_size = 31;
	int fast_threshold = 20;
	ORB(const vector<boost::any> &params);
	void create(cv::Ptr<cv::Feature2D> &ptr);
};

struct FeatureTrackerParams{
	enum class DetectorType { NONE, SIFT, SURF, FAST,
		BRISK, MSER, ORB, AGAST, GFTT };
	enum class DescriptorType { SIFT, SURF, BRIEF, ORB };

	int grid_size_x, grid_size_y;
	int search_window_x, search_window_y;
	bool init_at_each_frame;

	DetectorType detector_type;
	DescriptorType descriptor_type;

	bool rebuild_index;
	double max_dist_ratio;

	int min_matches;

	//! maximum iterations of the GridTracker algorithm to run for each frame
	int max_iters;
	double epsilon;
	bool enable_pyr;

	bool use_cv_flann;

	bool uchar_input;

	//! show the locations of individual key points
	bool show_keypoints;
	//! show the matches between keypoints
	bool show_matches;

	bool debug_mode;

	FeatureTrackerParams(
		int _grid_size_x, int _grid_size_y,
		int _search_win_x, int _search_win_y,
		bool _init_at_each_frame, DetectorType detector_type,
		DescriptorType descriptor_type, bool _rebuild_index,
		int _max_iters, double _epsilon, bool _enable_pyr,
		bool _use_cv_flann,
		double _max_dist_ratio, int _min_matches, bool _uchar_input,
		bool _show_keypoints, bool _show_matches, bool _debug_mode);
	FeatureTrackerParams(const FeatureTrackerParams *params = nullptr);

	int getResX() const{ return grid_size_x; }
	int getResY() const{ return grid_size_y; }

};
template<class SSM>
class FeatureTracker : public FeatureBase{

public:
	typedef FeatureTrackerParams ParamType;
	typedef std::vector<boost::any> FeatParamsType;
	typedef ParamType::DetectorType DetectorType;
	typedef ParamType::DescriptorType DescriptorType;
	typedef typename SSM::ParamType SSMParams;
	typedef typename SSM::EstimatorParams EstimatorParams;
#ifndef DISABLE_FLANN
	typedef FLANNParams::IdxType IdxType;
	typedef FLANNParams::SearchType SearchType;
	typedef flann::Index<flann::L2<float> > flannIdxT;
	typedef flann::Matrix<float> flannMatT;
	typedef flann::Matrix<int> flannResultT;
	typedef std::shared_ptr<flannMatT> FlannMatPtr;
	typedef std::shared_ptr<flannIdxT> FlannIdxPtr;
#else
	typedef FLANNCVParams FLANNParams;
#endif
	
	typedef cv::Ptr<cv::Feature2D> FeatPtr;

	FeatureTracker(
		const ParamType *grid_params = nullptr,
		const FLANNParams *_flann_params = nullptr,
		const EstimatorParams *_est_params = nullptr,
		const SSMParams *ssm_params = nullptr,
		const FeatParamsType &feat_params);
	~FeatureTracker(){}

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
	bool detect(const cv::Mat &mask, cv::Mat &obj_location) override;

	using FeatureBase::initialize;
	using FeatureBase::update;
	using FeatureBase::setRegion;

	SSM& getSSM() { return ssm; }
	
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW


private:
	SSM ssm;
	ParamType params;	
	FLANNParams flann_params;
	EstimatorParams est_params;
	cv::FlannBasedMatcher matcher;
	FeatPtr detector;

#ifndef DISABLE_FLANN
	FlannIdxPtr flann_idx;
	FlannMatPtr flann_dataset, flann_query;
#endif
	cv::Mat curr_img_in, curr_img, prev_img;
	std::vector<cv::KeyPoint> curr_key_pts, prev_key_pts;
	std::vector<cv::Point2f> curr_pts, prev_pts;
	cv::Mat best_idices, best_distances;

	int n_pts, n_key_pts, n_good_key_pts;
	cv::Size search_window;
	cv::Mat curr_descriptors, prev_descriptors;
	std::vector<uchar> pix_mask;
	std::vector<int> good_indices;

	CornersT opt_warped_corners;
	VectorXd ssm_update;

	cv::Mat curr_img_disp;

	char* patch_win_name;

	MatrixXi _linear_idx;//used for indexing the sub region locations
	int pause_seq;
	bool use_feature_detector;

	void matchKeyPoints();
	void cmptWarpedCorners();
	void showKeyPoints();
};

_MTF_END_NAMESPACE

#endif

