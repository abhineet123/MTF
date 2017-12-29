#include "mtf/SM/FeatureTracker.h"
#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/imgUtils.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "mtf/Utilities/excpUtils.h"

#if CV_MAJOR_VERSION < 3
#include "opencv2/nonfree/nonfree.hpp"
#else
#include "opencv2/xfeatures2d/nonfree.hpp"
#endif
#include <stdexcept>

#define FEAT_GRID_SIZE_X 10
#define FEAT_GRID_SIZE_Y 10
#define FEAT_SEARCH_WINDOW_X 10
#define FEAT_SEARCH_WINDOW_Y 10
#define FEAT_INIT_AT_EACH_FRAME 1
#define FEAT_DETECTOR_TYPE 1
#define FEAT_DESCRIPTOR_TYPE 0
#define FEAT_REBUILD_INDEX 1
#define FEAT_MAX_ITERS 1
#define FEAT_EPSILON 0.01
#define FEAT_ENABLE_PYR 0
#define FEAT_USE_CV_FLANN 1
#define FEAT_MAX_DIST_RATIO 0.75
#define FEAT_MIN_MATCHES 10
#define FEAT_UCHAR_INPUT 1
#define FEAT_SHOW_TRACKERS 0
#define FEAT_SHOW_TRACKER_EDGES 0
#define FEAT_DEBUG_MODE 0

#define parse_feat_param(name, data_type, param, feat_type)\
try{\
	name = boost::any_cast<data_type>(param);\
} catch(const boost::bad_any_cast &){\
	throw utils::InvalidArgument(cv::format("%s :: Invalid parameter type provided for %s", #feat_type, #name));\
}


#define SIFT_N_FEATURES 0
#define SIFT_N_OCTAVE_LAYERS 3
#define SIFT_CONTRAST_THRESH 0.04
#define SIFT_EDGE_THRESH 10
#define SIFT_SIGMA 1.6

#define SURF_HESSIAN_THRESHOLD 400
#define SURF_N_OCTAVES 4
#define SURF_N_OCTAVE_LAYERS 2
#define SURF_EXTENDED true
#define SURF_UPRIGHT false

#define FAST_THRESHOLD 10
#define FAST_NON_MAX_SUPPRESSION true
#define FAST_TYPE cv::FastFeatureDetector::TYPE_9_16


_MTF_BEGIN_NAMESPACE

SIFT::SIFT(const vector<boost::any> &params, std::string _type) :
n_features(SIFT_N_FEATURES),
n_octave_layers(SIFT_N_OCTAVE_LAYERS),
contrast_thresh(SIFT_CONTRAST_THRESH),
edge_thresh(SIFT_EDGE_THRESH),
sigma(SIFT_SIGMA){
	parse_feat_param(n_features, int, params[0], SIFTParams);
	parse_feat_param(n_octave_layers, int, params[1], SIFTParams);
	parse_feat_param(contrast_thresh, double, params[2], SIFTParams);
	parse_feat_param(edge_thresh, double, params[3], SIFTParams);
	parse_feat_param(sigma, double, params[4], SIFTParams);
	printf("Using SIFT %s with:\n", _type.c_str());
	printf("contrast_thresh: %f\n", contrast_thresh);
	printf("n_features: %d\n", n_features);
	printf("n_octave_layers: %d\n", n_octave_layers);
	printf("edge_thresh: %f\n", edge_thresh);
	printf("sigma: %f\n", sigma);
	printf("\n");
}

void SIFT::create(cv::Ptr<cv::Feature2D> &ptr){
#if CV_MAJOR_VERSION < 3
	ptr.reset(new cv::SIFT(
		n_features,
		n_octave_layers, 
		contrast_thresh,
		edge_thresh, 
		sigma
		));
#else
	ptr = cv::xfeatures2d::SIFT::create(
		n_features, 
		n_octave_layers,
		contrast_thresh,
		edge_thresh,
		sigma);
#endif
}
SURF::SURF(const vector<boost::any> &params, std::string _type) :
hessian_threshold(SURF_HESSIAN_THRESHOLD),
n_octaves(SURF_N_OCTAVES),
n_octave_layers(SURF_N_OCTAVE_LAYERS),
extended(SURF_EXTENDED),
upright(SURF_UPRIGHT){
	parse_feat_param(hessian_threshold, double, params[0], SURFParams);
	parse_feat_param(n_octaves, int, params[1], SURFParams);
	parse_feat_param(n_octave_layers, int, params[2], SURFParams);
	parse_feat_param(extended, bool, params[3], SURFParams);
	parse_feat_param(upright, bool, params[4], SURFParams);
	printf("Using SURF %s with:\n", _type.c_str());
	printf("hessian_threshold: %f\n", hessian_threshold);
	printf("n_octaves: %d\n", n_octaves);
	printf("n_octave_layers: %d\n", n_octave_layers);
	printf("extended: %d\n", extended);
	printf("upright: %d\n", upright);
	printf("\n");
}
void SURF::create(cv::Ptr<cv::Feature2D> &ptr){
#if CV_MAJOR_VERSION < 3
	ptr.reset(new cv::SURF(
		hessian_threshold,
		n_octaves, 
		n_octave_layers,
		extended, 
		upright
		));
#else
	ptr = cv::xfeatures2d::SURF::create(
		hessian_threshold,
		n_octaves, 
		n_octave_layers,
		extended, 
		upright);
#endif
}

FAST::FAST(const vector<boost::any> &params) :
threshold(FAST_THRESHOLD),
non_max_suppression(FAST_NON_MAX_SUPPRESSION),
type(FAST_TYPE){
	parse_feat_param(threshold, int, params[0], FASTParams);
	parse_feat_param(non_max_suppression, bool, params[1], FASTParams);
	parse_feat_param(type, int, params[2], FASTParams);
	printf("Using FAST detector with:\n");
	printf("threshold: %d\n", threshold);
	printf("non_max_suppression: %d\n", non_max_suppression);
	printf("type: %d\n", type);
	printf("\n");
}
void FAST::create(cv::Ptr<cv::Feature2D> &ptr){
#if CV_MAJOR_VERSION < 3
	ptr.reset(new cv::FastFeatureDetector(
		threshold,
		non_max_suppression, 
		type));
#else
	ptr = cv::FastFeatureDetector::create(
		threshold, non_max_suppression, type);
#endif
}
void BRISK::create(cv::Ptr<cv::Feature2D> &ptr){
#if CV_MAJOR_VERSION < 3
	ptr.reset(new cv::BRISK(
		thresh,
		octaves, 
		patternScale));
#else
	ptr = cv::BRISK::create(
		thresh,
		octaves, 
		patternScale);
#endif
}
void MSER::create(cv::Ptr<cv::Feature2D> &ptr){
#if CV_MAJOR_VERSION < 3
	ptr.reset(new cv::MSER(
		delta,
		min_area,
		max_area,
		max_variation,
		min_diversity,
		max_evolution,
		area_threshold,
		min_margin,
		edge_blur_size
		));
#else
	ptr = cv::MSER::create(
		delta,
		min_area,
		max_area,
		max_variation,
		min_diversity,
		max_evolution,
		area_threshold,
		min_margin,
		edge_blur_size
		);
#endif
}
void ORB::create(cv::Ptr<cv::Feature2D> &ptr){
#if CV_MAJOR_VERSION < 3
	ptr.reset(new cv::ORB(
		n_features,
		scale_factor,
		n_levels,
		edge_threshold,
		first_level,
		WTA_K,
		score_type,
		patch_size,
		fast_threshold
		));
#else
	ptr = cv::ORB::create(
		n_features,
		scale_factor,
		n_levels,
		edge_threshold,
		first_level,
		WTA_K,
		score_type,
		patch_size,
		fast_threshold
		);
#endif
}
FeatureTrackerParams::FeatureTrackerParams(
	int _grid_size_x, int _grid_size_y,
	int _search_win_x, int _search_win_y,
	bool _init_at_each_frame,
	DetectorType _detector_type,
	DescriptorType _descriptor_type,
	bool _rebuild_index, int _max_iters, double _epsilon,
	bool _enable_pyr, bool _use_cv_flann,
	double _max_dist_ratio,
	int _min_matches, bool _uchar_input,
	bool _show_keypoints, bool _show_matches,
	bool _debug_mode) :
	grid_size_x(_grid_size_x),
	grid_size_y(_grid_size_y),
	search_window_x(_search_win_x),
	search_window_y(_search_win_y),
	init_at_each_frame(_init_at_each_frame),
	detector_type(_detector_type),
	descriptor_type(_descriptor_type),
	rebuild_index(_rebuild_index),
	max_iters(_max_iters),
	epsilon(_epsilon),
	enable_pyr(_enable_pyr),
	use_cv_flann(_use_cv_flann),
	max_dist_ratio(_max_dist_ratio),
	min_matches(_min_matches),
	uchar_input(_uchar_input),
	show_keypoints(_show_keypoints),
	show_matches(_show_matches),
	debug_mode(_debug_mode){}

FeatureTrackerParams::FeatureTrackerParams(const FeatureTrackerParams *params) :
grid_size_x(FEAT_GRID_SIZE_X),
grid_size_y(FEAT_GRID_SIZE_Y),
search_window_x(FEAT_SEARCH_WINDOW_X),
search_window_y(FEAT_SEARCH_WINDOW_Y),
init_at_each_frame(FEAT_INIT_AT_EACH_FRAME),
detector_type(static_cast<DetectorType>(FEAT_DETECTOR_TYPE)),
descriptor_type(static_cast<DescriptorType>(FEAT_DESCRIPTOR_TYPE)),
rebuild_index(FEAT_REBUILD_INDEX),
max_iters(FEAT_MAX_ITERS),
epsilon(FEAT_EPSILON),
enable_pyr(FEAT_ENABLE_PYR),
use_cv_flann(FEAT_USE_CV_FLANN),
max_dist_ratio(FEAT_MAX_DIST_RATIO),
min_matches(FEAT_MIN_MATCHES),
uchar_input(FEAT_UCHAR_INPUT),
show_keypoints(FEAT_SHOW_TRACKERS),
show_matches(FEAT_SHOW_TRACKER_EDGES),
debug_mode(FEAT_DEBUG_MODE){
	if(params){
		grid_size_x = params->grid_size_x;
		grid_size_y = params->grid_size_y;
		search_window_x = params->search_window_x;
		search_window_y = params->search_window_y;
		init_at_each_frame = params->init_at_each_frame;
		detector_type = params->detector_type;
		descriptor_type = params->descriptor_type;
		rebuild_index = params->rebuild_index;
		max_iters = params->max_iters;
		epsilon = params->epsilon;
		max_dist_ratio = params->max_dist_ratio;
		min_matches = params->min_matches;
		enable_pyr = params->enable_pyr;
		use_cv_flann = params->use_cv_flann;
		uchar_input = params->uchar_input;
		show_keypoints = params->show_keypoints;
		show_matches = params->show_matches;
		debug_mode = params->debug_mode;
	}
}
template<class SSM>
FeatureTracker<SSM>::FeatureTracker(
	const ParamType *grid_params,
	const FLANNParams *_flann_params,
	const EstimatorParams *_est_params,
	const SSMParams *_ssm_params,
	const FeatParamsType &feat_params) :
	FeatureBase(), ssm(_ssm_params), params(grid_params),
	flann_params(_flann_params), est_params(_est_params),
	use_feature_detector(true){
	printf("\n");
	printf("Using Feature tracker with:\n");
	printf("grid_size: %d x %d\n", params.grid_size_x, params.grid_size_y);
	printf("search window size: %d x %d\n", params.search_window_x, params.search_window_y);
	printf("flann_index_type: %s\n", FLANNParams::toString(flann_params.index_type));
	printf("init_at_each_frame: %d\n", params.init_at_each_frame);
	printf("detector_type: %d\n", params.detector_type);
	printf("descriptor_type: %d\n", params.descriptor_type);
	printf("max_dist_ratio: %f\n", params.max_dist_ratio);
	printf("min_matches: %d\n", params.min_matches);
	printf("use_cv_flann: %d\n", params.use_cv_flann);
	printf("uchar_input: %d\n", params.uchar_input);
	printf("rebuild_index: %d\n", params.rebuild_index);
	printf("show_keypoints: %d\n", params.show_keypoints);
	printf("debug_mode: %d\n", params.debug_mode);
	printf("\n");

	printf("Using %s estimator with:\n", ssm.name.c_str());
	est_params.print();
	printf("\n");

	name = "feat";

	if(ssm.getResX() != params.getResX() || ssm.getResY() != params.getResY()){
		throw utils::InvalidArgument(
			cv::format("FeatureTracker: SSM has invalid sampling resolution: %d x %d",
			ssm.getResX(), ssm.getResY()));
	}
	if(params.detector_type == DetectorType::NONE){
		printf("Feature detection is disabled.\n");
		use_feature_detector = false;
	} else if(params.detector_type == DetectorType::SIFT){
		SIFT sift_params(feat_params);
	} else if(params.detector_type == DetectorType::SURF){
		SURF surf_params(feat_params);
	} else if(params.detector_type == DetectorType::FAST){
		FAST fast_params(feat_params);
	} else if(params.detector_type == DetectorType::BRISK){
		BRISK brisk_params(feat_params);
		printf("Using BRISK feature detector with:\n");
		brisk_params.print();
		printf("\n");
	} else if(params.detector_type == DetectorType::MSER){
		MSER mser_params(feat_params);
		printf("Using MSER feature detector with:\n");
		mser_params.print();
		printf("\n");
	} else if(params.detector_type == DetectorType::ORB){
		ORBarams orb_params(feat_params);
		printf("Using ORB feature detector with:\n");
		orb_params.print();
		printf("\n");
	} else{
		throw utils::FunctonNotImplemented(cv::format(
			"Feature detector %s is not implemented yet", toString(detector_type)));
	}
	n_pts = params.grid_size_x *params.grid_size_y;
	search_window = cv::Size(params.search_window_x, params.search_window_y);

	cv_corners_mat.create(2, 4, CV_64FC1);

	prev_key_pts.resize(n_pts);
	curr_key_pts.resize(n_pts);

	ssm_update.resize(ssm.getStateSize());
	pix_mask.resize(n_pts);
	std::fill(pix_mask.begin(), pix_mask.end(), 1);
	pause_seq = 0;

	if(params.show_keypoints){
		patch_win_name = "Matched Keypoints";
		cv::namedWindow(patch_win_name);
	}
}

template<class SSM>
void FeatureTracker<SSM>::setImage(const cv::Mat &img){
	params.uchar_input = img.type() == CV_8UC1;
	if(img.type() != inputType()){
		throw utils::InvalidArgument(
			cv_format("FeatureTracker::Input image type: %s does not match the required type: %s",
			utils::getType(img), utils::typeToString(inputType())));
	}
	if(params.uchar_input){
		curr_img = img;
	} else if(curr_img.empty()){
		curr_img.create(img.rows, img.cols, CV_8UC1);
	}
	if(prev_img.empty()){
		prev_img.create(img.rows, img.cols, CV_8UC1);
	}
	if(params.show_keypoints && curr_img_disp.empty()){
		curr_img_disp.create(img.rows, img.cols, CV_8UC3);
	}
	curr_img_in = img;
}

template<class SSM>
void FeatureTracker<SSM>::initialize(const cv::Mat &corners) {
	if(!params.uchar_input){
		curr_img_in.convertTo(curr_img, curr_img.type());
	}

	ssm.initialize(corners);
	cv::Mat mask = cv::Mat::zeros(curr_img.rows, curr_img.cols, CV_8U); // all 0
	mask(utils::getBestFitRectangle<int>(corners)) = 255;
	if(use_feature_detector){
		detector->detect(curr_img, prev_key_pts, mask);
	} else{
		for(int pt_id = 0; pt_id < n_pts; ++pt_id){
			Vector2d patch_centroid = ssm.getPts().col(pt_id);
			prev_key_pts[pt_id].pt.x = static_cast<float>(patch_centroid(0));
			prev_key_pts[pt_id].pt.y = static_cast<float>(patch_centroid(1));
		}
	}
	detector->compute(curr_img, prev_key_pts, prev_descriptors);
	//printf("prev_descriptors.type: %s\n", utils::getType(prev_descriptors));

	if(params.debug_mode){
		printf("n_init_key_pts: %d\n", prev_descriptors.rows);
	}
	//printf("descriptor_size: %d\n", prev_descriptors.cols);	
	//printf("prev_descriptors: %s\n", utils::getType(prev_descriptors));
#ifndef DISABLE_FLANN
	if(!params.use_cv_flann){
		flann_idx.reset(new flannIdxT(flann_params.getIndexParams(flann_params.index_type)));
		if(params.rebuild_index){
			flann_query.reset(new flannMatT((float*)(prev_descriptors.data),
				prev_descriptors.rows, prev_descriptors.cols));
			n_key_pts = prev_descriptors.rows;
			assert(prev_key_pts.size() == n_key_pts);
		} else{
			flann_idx->buildIndex(flannMatT((float*)(prev_descriptors.data),
				prev_descriptors.rows, prev_descriptors.cols));
		}
	}
#endif
	curr_img.copyTo(prev_img);
	ssm.getCorners(cv_corners_mat);
}
template<class SSM>
void FeatureTracker<SSM>::update() {
	if(!params.uchar_input){
		curr_img_in.convertTo(curr_img, curr_img.type());
	}

	//! create mask of search region for keypoints
	cv::Mat mask = cv::Mat::zeros(curr_img.rows, curr_img.cols, CV_8U);
	cv::Rect curr_location_rect = utils::getBestFitRectangle<int>(cv_corners_mat);
	cv::Rect search_region;
	search_region.x = max(curr_location_rect.x - params.search_window_x, 0);
	search_region.y = max(curr_location_rect.y - params.search_window_y, 0);
	search_region.width = min(curr_location_rect.width + 2 * params.search_window_x, curr_img.cols - search_region.x - 1);
	search_region.height = min(curr_location_rect.height + 2 * params.search_window_y, curr_img.rows - search_region.y - 1);
	mask(search_region) = 255;

	detector->detect(curr_img, curr_key_pts, mask);
	detector->compute(curr_img, curr_key_pts, curr_descriptors);

	matchKeyPoints();
	if(n_good_key_pts < params.min_matches){
		printf("FeatureTracker :: Insufficient matching keypoints found\n");
		return;
	}
	cmptWarpedCorners();
	ssm.setCorners(opt_warped_corners);
	ssm.getCorners(cv_corners_mat);
	if(params.show_keypoints){
		utils::drawRegion(mask, cv_corners_mat, CV_RGB(255, 255, 255));
		cv::imshow("Mask", mask);
		showKeyPoints();
	}
	if(params.init_at_each_frame){
		prev_key_pts = curr_key_pts;
		prev_descriptors = curr_descriptors.clone();
#ifndef DISABLE_FLANN
		if(!params.use_cv_flann){
			if(params.rebuild_index){
				flann_query.reset(new flannMatT((float*)(prev_descriptors.data),
					prev_descriptors.rows, prev_descriptors.cols));
				n_key_pts = prev_descriptors.rows;
			} else{
				flann_idx->buildIndex(flannMatT((float*)(prev_descriptors.data),
					prev_descriptors.rows, prev_descriptors.cols));
			}
		}
#endif
		curr_img.copyTo(prev_img);
	}
}

template<class SSM>
bool FeatureTracker<SSM>::detect(const cv::Mat &mask, cv::Mat &obj_location) {
	if(!params.uchar_input){
		curr_img_in.convertTo(curr_img, curr_img.type());
	}

	detector->detect(curr_img, curr_key_pts, mask);
	detector->compute(curr_img, curr_key_pts, curr_descriptors);

	matchKeyPoints();

	if(params.show_keypoints){ showKeyPoints(); }

	if(n_good_key_pts < params.min_matches){ return false; }

	cmptWarpedCorners();

	if((opt_warped_corners.array() < 0).any() || !opt_warped_corners.allFinite()){
		return false;
	}
	if(params.debug_mode){
		utils::printMatrix(opt_warped_corners, "opt_warped_corners");
	}
	obj_location = utils::Corners(opt_warped_corners).mat();
	return true;
}

template<class SSM>
void FeatureTracker<SSM>::matchKeyPoints() {
	n_key_pts = curr_descriptors.rows;
	assert(curr_key_pts.size() == n_key_pts);

	if(params.debug_mode){
		printf("n_key_pts: %d\n", n_key_pts);
	}

	best_idices.create(n_key_pts, 2, CV_32S);
	best_distances.create(n_key_pts, 2, CV_32F);

#ifndef DISABLE_FLANN
	if(!params.use_cv_flann){
		if(params.rebuild_index){
			flann_idx->buildIndex(flannMatT((float*)(curr_descriptors.data),
				curr_descriptors.rows, curr_descriptors.cols));
		} else{
			flann_query.reset(new flannMatT((float*)(curr_descriptors.data),
				curr_descriptors.rows, curr_descriptors.cols));

		}
		flannResultT flann_result((int*)(best_idices.data), n_key_pts, 2);
		flannMatT flann_dists((float*)(best_distances.data), n_key_pts, 2);

		switch(flann_params.search_type){
		case SearchType::KNN:
			flann_idx->knnSearch(*flann_query, flann_result, flann_dists, 2, flann_params.search);
			break;
		case SearchType::Radius:
			flann_idx->radiusSearch(*flann_query, flann_result, flann_dists, 2, flann_params.search);
			break;
		default: throw invalid_argument(
			cv_format("Invalid search type specified: %d....\n", flann_params.search_type));
		}
		if(params.debug_mode){
			utils::printMatrixToFile<int>(best_idices, "best_idices", "log/FeatureTracker.txt", "%d");
			utils::printMatrixToFile<int>(best_distances, "best_distances", "log/FeatureTracker.txt", "%d");
		}
	} else{
#endif
		std::vector<std::vector<cv::DMatch>> matches;
		if(params.rebuild_index){
			matcher.knnMatch(prev_descriptors, curr_descriptors, matches, 2);
		} else{
			matcher.knnMatch(curr_descriptors, prev_descriptors, matches, 2);
		}
		for(unsigned int match_id = 0; match_id < matches.size(); ++match_id){
			best_distances.at<float>(match_id, 0) = matches[match_id][0].distance;
			best_distances.at<float>(match_id, 1) = matches[match_id][1].distance;
			best_idices.at<int>(match_id, 0) = matches[match_id][0].trainIdx;
			best_idices.at<int>(match_id, 1) = matches[match_id][1].trainIdx;
		}
#ifndef DISABLE_FLANN
	}
#endif
	prev_pts.clear();
	curr_pts.clear();
	good_indices.clear();
	if(params.rebuild_index){
		for(int pt_id = 0; pt_id < n_key_pts; ++pt_id){
			if(params.max_dist_ratio < 0 ||
				best_distances.at<float>(pt_id, 0) < params.max_dist_ratio*best_distances.at<float>(pt_id, 1)){
				curr_pts.push_back(curr_key_pts[best_idices.at<int>(pt_id, 0)].pt);
				prev_pts.push_back(prev_key_pts[pt_id].pt);
				good_indices.push_back(pt_id);
			}
		}
	} else{
		for(int pt_id = 0; pt_id < n_key_pts; ++pt_id){
			//if(params.debug_mode){
			//	printf("pt_id: %d best_distances: %f, %f\n", pt_id, best_distances.at<float>(pt_id, 0),
			//		best_distances.at<float>(pt_id, 1));
			//}
			if(params.max_dist_ratio < 0 ||
				best_distances.at<float>(pt_id, 0) < params.max_dist_ratio*best_distances.at<float>(pt_id, 1)){
				prev_pts.push_back(prev_key_pts[best_idices.at<int>(pt_id, 0)].pt);
				curr_pts.push_back(curr_key_pts[pt_id].pt);
				good_indices.push_back(pt_id);
			}
		}
	}
	n_good_key_pts = curr_pts.size();
	if(params.debug_mode){
		printf("n_good_key_pts: %d\n", n_good_key_pts);
	}
}
template<class SSM>
void FeatureTracker<SSM>::cmptWarpedCorners() {
	//! compute warp from matching keypoints 
	if(params.rebuild_index){
		ssm.estimateWarpFromPts(ssm_update, pix_mask, prev_pts, curr_pts, est_params);
	} else{
		VectorXd inv_update(ssm.getStateSize());
		ssm.estimateWarpFromPts(inv_update, pix_mask, curr_pts, prev_pts, est_params);
		ssm.invertState(ssm_update, inv_update);
	}
	ssm.applyWarpToCorners(opt_warped_corners, ssm.getCorners(), ssm_update);
}

template<class SSM>
void FeatureTracker<SSM>::setRegion(const cv::Mat& corners) {
	ssm.setCorners(corners);
	if(use_feature_detector){
		detector->detect(curr_img, prev_key_pts, mask);
	} else{
		for(int pt_id = 0; pt_id < n_pts; pt_id++){
			Vector2d patch_centroid = ssm.getPts().col(pt_id);
			prev_key_pts[pt_id].pt.x = static_cast<float>(patch_centroid(0));
			prev_key_pts[pt_id].pt.y = static_cast<float>(patch_centroid(1));
		}
	}
	cv::Mat mask = cv::Mat::zeros(curr_img.rows, curr_img.cols, CV_8U);
	mask(utils::getBestFitRectangle<int>(corners)) = 1;	

	detector->compute(curr_img, prev_key_pts, prev_descriptors);

#ifndef DISABLE_FLANN
	if(!params.use_cv_flann){
		flann_idx->buildIndex(flannMatT((float*)(prev_descriptors.data),
			prev_descriptors.rows, prev_descriptors.cols));
	}
#endif
	ssm.getCorners(cv_corners_mat);
	if(params.show_keypoints){ showKeyPoints(); }

}

template<class SSM>
void FeatureTracker<SSM>::showKeyPoints(){
	curr_img_in.convertTo(curr_img_disp, curr_img_disp.type());
	cv::cvtColor(curr_img_disp, curr_img_disp, CV_GRAY2BGR);
	utils::drawRegion(curr_img_disp, cv_corners_mat, CV_RGB(0, 0, 255), 2);
	vector<cv::DMatch> matches;
	for(int pt_id = 0; pt_id < n_good_key_pts; pt_id++) {
		circle(curr_img_disp, curr_pts[pt_id], 2,
			pix_mask[pt_id] ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
		matches.push_back(cv::DMatch(pt_id,
			best_idices.at<int>(good_indices[pt_id], 0),
			best_distances.at<float>(good_indices[pt_id], 0)));
	}
	imshow(patch_win_name, curr_img_disp);
	if(params.show_matches){
		try{
			cv::Mat img_matches;
			if(params.rebuild_index){
				drawMatches(prev_img, prev_key_pts, curr_img, curr_key_pts, matches, img_matches);
			} else{
				drawMatches(curr_img, curr_key_pts, prev_img, prev_key_pts, matches, img_matches);
			}
			imshow("Matches", img_matches);
		} catch(const cv::Exception &err){
			printf("Error in drawMatches: %s\n", err.what());
		}
	}
	//int key = cv::waitKey(1 - pause_seq);
	//if(key == 32){
	//	pause_seq = 1 - pause_seq;
	//}
}

_MTF_END_NAMESPACE

#ifndef HEADER_ONLY_MODE
#include "mtf/Macros/register.h"
_REGISTER_TRACKERS_SSM(FeatureTracker);
#endif