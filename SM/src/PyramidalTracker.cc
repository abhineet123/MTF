#include "mtf/SM/PyramidalTracker.h"
#include "mtf/Utilities/miscUtils.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

_MTF_BEGIN_NAMESPACE

PyramidalTracker::PyramidalTracker(const vector<TrackerBase*> _trackers,
 const ParamType *parl_params) : CompositeBase(_trackers),
	params(parl_params), external_img_pyramid(false){
	if(n_trackers != params.no_of_levels){
		throw std::domain_error(
			cv::format("PyramidalTracker :: no. of trackers: %d does not match the no. of levels in the pyramid: %d",
			n_trackers, params.no_of_levels));
	}
	if(input_type == HETEROGENEOUS_INPUT){
		throw std::domain_error("PyramidalTracker :: Heterogeneous input is currently not supported");
	}
	printf("\n");
	printf("Using Pyramidal tracker with:\n");
	printf("no_of_levels: %d\n", params.no_of_levels);
	printf("scale_factor: %f\n", params.scale_factor);
	printf("show_levels: %d\n", params.show_levels);
	printf("trackers: ");
	printf("1: %s ", trackers[0]->name.c_str());
	name = "pyr: " + trackers[0]->name;
	for(int tracker_id = 1; tracker_id < n_trackers; tracker_id++) {
		if(trackers[tracker_id]->name != trackers[0]->name){
			name = name + trackers[tracker_id]->name + " ";
			printf("%d: %s ", tracker_id + 1, trackers[tracker_id]->name.c_str());
		}
	}
	printf("\n");
	img_pyramid.resize(params.no_of_levels);
	img_sizes.resize(params.no_of_levels);
	overall_scale_factor = pow(params.scale_factor, params.no_of_levels - 1);
}
void PyramidalTracker::setImage(const cv::Mat &img){	
	if(img_pyramid[0].empty()){
		img_sizes[0] = cv::Size(img.cols, img.rows);
		printf("Level %d: size: %dx%d\n",
			0, img_sizes[0].width, img_sizes[0].height);
	}

	img_pyramid[0] = img;
	trackers[0]->setImage(img_pyramid[0]);
	for(int pyr_level = 1; pyr_level < params.no_of_levels; ++pyr_level){
		if(img_pyramid[pyr_level].empty()){
			int n_rows = img_pyramid[pyr_level - 1].rows*params.scale_factor;
			int n_cols = img_pyramid[pyr_level - 1].cols*params.scale_factor;
			img_sizes[pyr_level] = cv::Size(n_cols, n_rows);
			img_pyramid[pyr_level].create(img_sizes[pyr_level], img.type());
			printf("Level %d: size: %dx%d\n",
				pyr_level, img_sizes[pyr_level].width, img_sizes[pyr_level].height);
		}
		trackers[pyr_level]->setImage(img_pyramid[pyr_level]);
	}
	external_img_pyramid = false;
}

void PyramidalTracker::setImagePyramid(const vector<cv::Mat> &_img_pyramid){
	if(_img_pyramid.size() != params.no_of_levels){
		throw std::invalid_argument(
			cv::format(
			"PyramidalTracker::setImagePyramid :: Mismatch between the no. of levels in the provided pyramid: %d and the expected: %d", 
			_img_pyramid.size(), params.no_of_levels)
			);
	}
	for(int pyr_level = 0; pyr_level < params.no_of_levels; ++pyr_level){
		img_pyramid[pyr_level] = _img_pyramid[pyr_level];
		img_sizes[pyr_level] = cv::Size(img_pyramid[pyr_level].cols, img_pyramid[pyr_level].rows);
		trackers[pyr_level]->setImage(img_pyramid[pyr_level]);
	}
	external_img_pyramid = true;
}

void PyramidalTracker::updateImagePyramid(){
	for(int pyr_level = 1; pyr_level < params.no_of_levels; ++pyr_level){
		if(params.scale_factor == 0.5){
			cv::pyrDown(img_pyramid[pyr_level - 1], img_pyramid[pyr_level], img_sizes[pyr_level]);
		} else{
			cv::resize(img_pyramid[pyr_level - 1], img_pyramid[pyr_level], img_sizes[pyr_level]);
			cv::GaussianBlur(img_pyramid[pyr_level], img_pyramid[pyr_level], cv::Size(5, 5), 3);
		}		
	}
}
void PyramidalTracker::initialize(const cv::Mat &corners)  {
	if(!external_img_pyramid){
		updateImagePyramid();
	}	
	trackers[0]->initialize(corners);
	cv::Mat scaled_corners = corners.clone();
	for(int pyr_level = 1; pyr_level < params.no_of_levels; ++pyr_level){
		scaled_corners *= params.scale_factor;
		trackers[pyr_level]->initialize(scaled_corners);
	}
	if(params.show_levels){
		for(int pyr_level = 1; pyr_level < params.no_of_levels; ++pyr_level){
			cv::namedWindow(cv::format("Level %d", pyr_level));
		}
		showImagePyramid();
	}
}
void PyramidalTracker::update()  {
	if(!external_img_pyramid){
		updateImagePyramid();
	}
	trackers[n_trackers-1]->update();
	for(int tracker_id = n_trackers-2; tracker_id >= 0; --tracker_id) {
		cv::Mat scaled_up_corners = trackers[tracker_id + 1]->getRegion() / params.scale_factor;
		trackers[tracker_id]->setRegion(scaled_up_corners);
		trackers[tracker_id]->update();
	}
	cv::Mat scaled_down_corners = trackers[0]->getRegion() * overall_scale_factor;
	trackers[n_trackers - 1]->setRegion(scaled_down_corners);
	if(params.show_levels){	showImagePyramid(); }
}
void PyramidalTracker::setRegion(const cv::Mat& corners)   {
	trackers[0]->setRegion(corners);
	cv::Mat scaled_down_corners = corners.clone();
	for(int pyr_level = 1; pyr_level < params.no_of_levels; ++pyr_level){
		scaled_down_corners *= params.scale_factor;
		trackers[pyr_level]->setRegion(scaled_down_corners);
	}
	if(params.show_levels){
		if(!external_img_pyramid){
			updateImagePyramid();
		}
		showImagePyramid();
	}
}

void PyramidalTracker::showImagePyramid(){
	for(int pyr_level = 1; pyr_level < params.no_of_levels; ++pyr_level){
		cv::Mat curr_level_img(img_sizes[pyr_level], CV_8UC3);
		img_pyramid[pyr_level].convertTo(curr_level_img, curr_level_img.type());
		cv::cvtColor(curr_level_img, curr_level_img, CV_GRAY2BGR);
		cv::Scalar line_color(255, 0, 0);
		cv_corners_mat = trackers[pyr_level]->getRegion();
		cv::Point2d ul(cv_corners_mat.at<double>(0, 0), cv_corners_mat.at<double>(1, 0));
		cv::Point2d ur(cv_corners_mat.at<double>(0, 1), cv_corners_mat.at<double>(1, 1));
		cv::Point2d lr(cv_corners_mat.at<double>(0, 2), cv_corners_mat.at<double>(1, 2));
		cv::Point2d ll(cv_corners_mat.at<double>(0, 3), cv_corners_mat.at<double>(1, 3));
		line(curr_level_img, ul, ur, line_color, 2);
		line(curr_level_img, ur, lr, line_color, 2);
		line(curr_level_img, lr, ll, line_color, 2);
		line(curr_level_img, ll, ul, line_color, 2);
		cv::imshow(cv::format("Level %d", pyr_level).c_str(), curr_level_img);
	}
}

_MTF_END_NAMESPACE
