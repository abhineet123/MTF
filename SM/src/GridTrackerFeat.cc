#include "mtf/SM/GridTrackerFeat.h"
#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/imgUtils.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <stdexcept>

#define SIFT_N_FEATURES 0
#define SIFT_N_OCTAVE_LAYERS 3
#define SIFT_CONTRAST_THRESH 0.04
#define SIFT_EDGE_THRESH 10
#define SIFT_SIGMA 1.6

#define GTF_GRID_SIZE_X 10
#define GTF_GRID_SIZE_Y 10
#define GTF_SEARCH_WINDOW_X 10
#define GTF_SEARCH_WINDOW_Y 10
#define GTF_INIT_AT_EACH_FRAME 1
#define GTF_DETECT_KEYPOINTS 0
#define GTF_REBUILD_INDEX 1
#define GTF_MAX_ITERS 1
#define GTF_EPSILON 0.01
#define GTF_ENABLE_PYR 0
#define GTF_SHOW_TRACKERS 0
#define GTF_SHOW_TRACKER_EDGES 0
#define GTF_DEBUG_MODE 0

_MTF_BEGIN_NAMESPACE

SIFTParams::SIFTParams(
int _n_features,
int _n_octave_layers,
double _contrast_thresh,
double _edge_thresh,
double _sigma) :
n_features(_n_features),
n_octave_layers(_n_octave_layers),
contrast_thresh(_contrast_thresh),
edge_thresh(_edge_thresh),
sigma(_sigma){}

SIFTParams::SIFTParams(const SIFTParams *params) :
n_features(SIFT_N_FEATURES),
n_octave_layers(SIFT_N_OCTAVE_LAYERS),
contrast_thresh(SIFT_CONTRAST_THRESH),
edge_thresh(SIFT_EDGE_THRESH),
sigma(SIFT_SIGMA){
	if(params){
		n_features = params->n_features;
		n_octave_layers = params->n_octave_layers;
		contrast_thresh = params->contrast_thresh;
		edge_thresh = params->edge_thresh;
		sigma = params->sigma;
	}
}

void SIFTParams::print() const{
	printf("contrast_thresh: %f\n", contrast_thresh);
	printf("n_features: %d\n", n_features);
	printf("n_octave_layers: %d\n", n_octave_layers);
	printf("edge_thresh: %f\n", edge_thresh);
	printf("sigma: %f\n", sigma);
}

GridTrackerFeatParams::GridTrackerFeatParams(
int _grid_size_x, int _grid_size_y,
int _search_win_x, int _search_win_y,
bool _init_at_each_frame,
bool _detect_keypoints, bool _rebuild_index,
int _max_iters, double _epsilon, bool _enable_pyr,
bool _show_keypoints, bool _show_matches,
bool _debug_mode) :
grid_size_x(_grid_size_x),
grid_size_y(_grid_size_y),
search_window_x(_search_win_x),
search_window_y(_search_win_y),
init_at_each_frame(_init_at_each_frame),
detect_keypoints(_detect_keypoints),
rebuild_index(_rebuild_index),
max_iters(_max_iters),
epsilon(_epsilon),
enable_pyr(_enable_pyr),
show_keypoints(_show_keypoints),
show_matches(_show_matches),
debug_mode(_debug_mode){}

GridTrackerFeatParams::GridTrackerFeatParams(const GridTrackerFeatParams *params) :
grid_size_x(GTF_GRID_SIZE_X),
grid_size_y(GTF_GRID_SIZE_Y),
search_window_x(GTF_SEARCH_WINDOW_X),
search_window_y(GTF_SEARCH_WINDOW_Y),
init_at_each_frame(GTF_INIT_AT_EACH_FRAME),
detect_keypoints(GTF_DETECT_KEYPOINTS),
rebuild_index(GTF_REBUILD_INDEX),
max_iters(GTF_MAX_ITERS),
epsilon(GTF_EPSILON),
enable_pyr(GTF_ENABLE_PYR),
show_keypoints(GTF_SHOW_TRACKERS),
show_matches(GTF_SHOW_TRACKER_EDGES),
debug_mode(GTF_DEBUG_MODE){
	if(params){
		grid_size_x = params->grid_size_x;
		grid_size_y = params->grid_size_y;
		search_window_x = params->search_window_x;
		search_window_y = params->search_window_y;
		init_at_each_frame = params->init_at_each_frame;
		detect_keypoints = params->detect_keypoints;
		rebuild_index = params->rebuild_index;
		max_iters = params->max_iters;
		epsilon = params->epsilon;
		enable_pyr = params->enable_pyr;
		show_keypoints = params->show_keypoints;
		show_matches = params->show_matches;
		debug_mode = params->debug_mode;
	}
}
template<class SSM>
GridTrackerFeat<SSM>::GridTrackerFeat(const ParamType *grid_params,
	const SIFTParams *_sift_params, const FLANNParams *_flann_params, 
	const EstimatorParams *_est_params,	const SSMParams *_ssm_params) :
	GridBase(), ssm(_ssm_params), params(grid_params), sift_params(_sift_params),
	flann_params(_flann_params), est_params(_est_params){
	printf("\n");
	printf("Using Feature based Grid tracker with:\n");
	printf("grid_size: %d x %d\n", params.grid_size_x, params.grid_size_y);
	printf("search window size: %d x %d\n", params.search_window_x, params.search_window_y);
	printf("flann_index_type: %s\n", FLANNParams::toString(flann_params.index_type));
	printf("init_at_each_frame: %d\n", params.init_at_each_frame);
	printf("detect_keypoints: %d\n", params.detect_keypoints);
	printf("rebuild_index: %d\n", params.rebuild_index);
	printf("show_keypoints: %d\n", params.show_keypoints);
	printf("debug_mode: %d\n", params.debug_mode);
	printf("\n");

	printf("Using SIFT feature detector with:\n");
	sift_params.print();
	printf("\n");

	printf("Using %s estimator with:\n", ssm.name.c_str());
	est_params.print();
	printf("\n");

	name = "grid_feat";

	if(ssm.getResX() != params.getResX() || ssm.getResY() != params.getResY()){
		throw std::invalid_argument(
			cv::format("GridTrackerFeat: SSM has invalid sampling resolution: %d x %d",
			ssm.getResX(), ssm.getResY()));
	}

	feat.reset(new cv::SIFT(sift_params.n_features,
		sift_params.n_octave_layers, sift_params.contrast_thresh,
		sift_params.edge_thresh, sift_params.sigma));

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
void GridTrackerFeat<SSM>::initialize(const cv::Mat &corners) {
	curr_img_float.convertTo(curr_img, curr_img.type());

	ssm.initialize(corners);
	cv::Mat mask = cv::Mat::zeros(curr_img.rows, curr_img.cols, CV_8U); // all 0
	mask(utils::getBestFitRectangle<int>(corners)) = 255;
	if(!params.detect_keypoints){
		for(int pt_id = 0; pt_id < n_pts; pt_id++){
			Vector2d patch_centroid = ssm.getPts().col(pt_id);
			prev_key_pts[pt_id].pt.x = patch_centroid(0);
			prev_key_pts[pt_id].pt.y = patch_centroid(1);
		}
	}
	(*feat)(curr_img, mask, prev_key_pts, prev_descriptors, !params.detect_keypoints);
	printf("n_key_pts: %d\n", prev_descriptors.rows);
	//printf("descriptor_size: %d\n", prev_descriptors.cols);	
	//printf("prev_descriptors: %s\n", utils::getType(prev_descriptors));

	flann_idx = new flannIdxT(flann_params.getIndexParams(flann_params.index_type));
	if(params.rebuild_index){
		flann_query.reset(new flannMatT((float*)(prev_descriptors.data),
			prev_descriptors.rows, prev_descriptors.cols));
		n_key_pts = prev_descriptors.rows;
		assert(prev_key_pts.size() == n_key_pts);
	} else{
		flann_idx->buildIndex(flannMatT((float*)(prev_descriptors.data),
			prev_descriptors.rows, prev_descriptors.cols));
	}
	curr_img.copyTo(prev_img);

	ssm.getCorners(cv_corners_mat);
}
template<class SSM>
void GridTrackerFeat<SSM>::update() {
	curr_img_float.convertTo(curr_img, curr_img.type());

	cv::Mat mask = cv::Mat::zeros(curr_img.rows, curr_img.cols, CV_8U); 
	
	cv::Rect curr_location_rect = utils::getBestFitRectangle<int>(cv_corners_mat);
	cv::Rect search_reigon;
	search_reigon.x = max(curr_location_rect.x - params.search_window_x, 0);
	search_reigon.y = max(curr_location_rect.y - params.search_window_y, 0);
	search_reigon.width = min(curr_location_rect.width + 2*params.search_window_x, curr_img.cols - search_reigon.x - 1);
	search_reigon.height = min(curr_location_rect.height + 2*params.search_window_y, curr_img.rows - search_reigon.y - 1);
	mask(search_reigon) = 255;

	(*feat)(curr_img, mask, curr_key_pts, curr_descriptors, false);

	if(params.rebuild_index){
		flann_idx->buildIndex(flannMatT((float*)(curr_descriptors.data),
			curr_descriptors.rows, curr_descriptors.cols));
	} else{
		flann_query.reset(new flannMatT((float*)(curr_descriptors.data),
			curr_descriptors.rows, curr_descriptors.cols));
		n_key_pts = curr_descriptors.rows;
		assert(curr_key_pts.size() == n_key_pts);
	}

	printf("n_key_pts: %d\n", n_key_pts);
	//printf("curr_key_pts.size: %ld\n", curr_key_pts.size());
	//printf("curr_descriptors_type: %s\n", utils::getType(curr_descriptors));

	best_idices.create(n_key_pts, 2, CV_32S);
	best_distances.create(n_key_pts, 2, CV_32F);
	flannResultT flann_result((int*)(best_idices.data), n_key_pts, 2);
	flannMatT flann_dists((float*)(best_distances.data), n_key_pts, 2);

	switch(flann_params.search_type){
	case SearchType::KNN:
		flann_idx->knnSearch(*flann_query, flann_result, flann_dists, 2, flann_params.search);
		record_event("flann_idx->knnSearch");
		break;
	case SearchType::Radius:
		flann_idx->radiusSearch(*flann_query, flann_result, flann_dists, 2, flann_params.search);
		record_event("flann_idx->radiusSearch");
		break;
	default: throw invalid_argument(
		cv::format("Invalid search type specified: %d....\n", flann_params.search_type));
	}

	utils::printMatrixToFile<int>(best_idices, "best_idices", "log/GridTrackerFeat.txt", "%d");
	utils::printMatrixToFile<int>(best_distances, "best_distances", "log/GridTrackerFeat.txt", "%d");

	prev_pts.clear();
	curr_pts.clear();
	good_indices.clear();
	if(params.rebuild_index){
		for(int pt_id = 0; pt_id < n_key_pts; ++pt_id){
			if(best_distances.at<float>(pt_id, 0) < 0.75*best_distances.at<float>(pt_id, 1)){
				curr_pts.push_back(curr_key_pts[best_idices.at<int>(pt_id, 0)].pt);
				prev_pts.push_back(prev_key_pts[pt_id].pt);
				good_indices.push_back(pt_id);
			}
		}
		n_good_key_pts = curr_pts.size();
		printf("n_good_key_pts: %d\n", n_good_key_pts);
		ssm.estimateWarpFromPts(ssm_update, pix_mask, prev_pts, curr_pts, est_params);
	} else{
		for(int pt_id = 0; pt_id < n_key_pts; ++pt_id){
			if(best_distances.at<float>(pt_id, 0) < 0.75*best_distances.at<float>(pt_id, 1)){
				prev_pts.push_back(prev_key_pts[best_idices.at<int>(pt_id, 0)].pt);
				curr_pts.push_back(curr_key_pts[pt_id].pt);
				good_indices.push_back(pt_id);
			}
		}
		VectorXd inv_update(ssm.getStateSize());
		ssm.estimateWarpFromPts(inv_update, pix_mask, curr_pts, prev_pts, est_params);
		ssm.invertState(ssm_update, inv_update);		
	}


	Matrix24d opt_warped_corners;
	ssm.applyWarpToCorners(opt_warped_corners, ssm.getCorners(), ssm_update);
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
		if(params.rebuild_index){
			flann_query.reset(new flannMatT((float*)(prev_descriptors.data),
				prev_descriptors.rows, prev_descriptors.cols));
			n_key_pts = prev_descriptors.rows;
		} else{
			flann_idx->buildIndex(flannMatT((float*)(curr_descriptors.data),
				curr_descriptors.rows, curr_descriptors.cols));
		}
		curr_img.copyTo(prev_img);
	}

}
template<class SSM>
void GridTrackerFeat<SSM>::setImage(const cv::Mat &img){
	if(curr_img_float.empty()){
		prev_img.create(img.rows, img.cols, CV_8UC1);
		curr_img.create(img.rows, img.cols, CV_8UC1);
	}
	if(params.show_keypoints && curr_img_uchar.empty()){
		curr_img_uchar.create(img.rows, img.cols, CV_8UC3);
	}
	curr_img_float = img;
}

template<class SSM>
void GridTrackerFeat<SSM>::setRegion(const cv::Mat& corners) {
	ssm.setCorners(corners);
	if(!params.detect_keypoints){
		for(int pt_id = 0; pt_id < n_pts; pt_id++){
			Vector2d patch_centroid = ssm.getPts().col(pt_id);
			prev_key_pts[pt_id].pt.x = patch_centroid(0);
			prev_key_pts[pt_id].pt.y = patch_centroid(1);
		}
	}
	cv::Mat mask = cv::Mat::zeros(curr_img.rows, curr_img.cols, CV_8U); 
	mask(utils::getBestFitRectangle<int>(corners)) = 1;
	(*feat)(curr_img, mask, prev_key_pts, prev_descriptors, !params.detect_keypoints);
	flann_idx->buildIndex(flannMatT((float*)(prev_descriptors.data),
		prev_descriptors.rows, prev_descriptors.cols));
	ssm.getCorners(cv_corners_mat);
	if(params.show_keypoints){ showKeyPoints(); }

}

template<class SSM>
void GridTrackerFeat<SSM>::showKeyPoints(){
	curr_img_float.convertTo(curr_img_uchar, curr_img_uchar.type());
	cv::cvtColor(curr_img_uchar, curr_img_uchar, CV_GRAY2BGR);
	utils::drawRegion(curr_img_uchar, cv_corners_mat, CV_RGB(0, 0, 255), 2);
	vector<cv::DMatch> matches;
	for(int pt_id = 0; pt_id < n_good_key_pts; pt_id++) {
		circle(curr_img_uchar, curr_pts[pt_id], 2,
			pix_mask[pt_id] ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
		matches.push_back(cv::DMatch(pt_id, 
			best_idices.at<int>(good_indices[pt_id], 0), 
			best_distances.at<int>(good_indices[pt_id], 0)));
	}
	imshow(patch_win_name, curr_img_uchar);	
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
_REGISTER_TRACKERS_SSM(GridTrackerFeat);
#endif