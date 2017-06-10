#include "mtf/mtf.h"
#include "mtf/Config/parameters.h"
#include "mtf/Config/datasets.h"
#include "mtf/Utilities/miscUtils.h"
//! tools for reading in images from various sources like image sequences, 
//! videos and cameras as well as for pre processing them
#include "mtf/Tools/pipeline.h"

#include <time.h>
#include <string.h>
#include <vector>
#include <map>
#include <memory>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <mex.h>

#define MEX_TRACKER_CREATE 1
#define MEX_TRACKER_INITIALIZE 2
#define MEX_TRACKER_UPDATE 3
#define MEX_TRACKER_SET_REGION 4

using namespace std;
using namespace mtf::params;

std::map<std::string, const int> cmd_list = {
	{ "create", MEX_TRACKER_CREATE },
	{ "initialize", MEX_TRACKER_INITIALIZE },
	{ "update", MEX_TRACKER_UPDATE },
	{ "set_region", MEX_TRACKER_SET_REGION }
};


double* out_corners_data;

typedef unique_ptr<mtf::TrackerBase> Tracker_;
Tracker_ tracker;
PreProc_ pre_proc;

double min_x, min_y, max_x, max_y;
double size_x, size_y;
cv::Mat init_img_cv, init_corners_cv, curr_img_cv;

int img_height, img_width;
vector<cv::Scalar> obj_cols;

cv::Point fps_origin(10, 20);
double fps_font_size = 0.50;
cv::Scalar fps_color(0, 255, 0);
int frame_id;
char* config_root_dir = nullptr;
bool using_input_pipeline = false;

bool tracker_created = false, tracker_initialized = false;

/* ==== create tracker ==== */
bool createTracker() {
#ifdef USE_TBB
	Eigen::initParallel();
#endif
	if(!config_root_dir){
		config_root_dir = "C++/MTF/Config";
		printf("Using default configuration folder: %s\n", config_root_dir);
	} else{
		printf("Reading MTF configuration files from: %s\n", config_root_dir);
	}
	config_dir = std::string(config_root_dir);
	if(!readParams(0, nullptr)){ return false; }

	if(res_from_size){
		printf("Getting sampling resolution from object size...\n");
	}
	try{
		if(res_from_size){
			resx = static_cast<unsigned int>(size_x / res_from_size);
			resy = static_cast<unsigned int>(size_y / res_from_size);
		}
		tracker.reset(mtf::getTracker(mtf_sm, mtf_am, mtf_ssm, mtf_ilm));
		if(!tracker){
			printf("Tracker could not be created successfully\n");
			return false;
		}
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while creating the tracker: %s\n",
			err.type(), err.what());
		return false;
	}
	try{
		pre_proc = getPreProc(tracker->inputType(), pre_proc_type);
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while creating the pre processor: %s\n",
			err.type(), err.what());
		return false;
	}
	return true;
}

bool initializeTracker(const cv::Mat &init_img, const cv::Mat &init_corners) {
	img_height = init_img.rows;
	img_width = init_img.cols;

	printf("img_height: %d\n", img_height);
	printf("img_width: %d\n", img_width);

	printf("init_corners:\n");
	for(unsigned int corner_id = 0; corner_id < 4; ++corner_id) {
		printf("%d: (%f, %f)\n", corner_id, init_corners.at<double>(0, corner_id), init_corners.at<double>(1, corner_id));
	}
	min_x = init_corners_cv.at<double>(0, 0);
	min_y = init_corners_cv.at<double>(1, 0);
	max_x = init_corners_cv.at<double>(0, 2);
	max_y = init_corners_cv.at<double>(1, 2);
	size_x = max_x - min_x;
	size_y = max_y - min_y;
	try{		
		pre_proc->initialize(init_img_cv);
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while initializing the pre processor: %s\n",
			err.type(), err.what());
		return false;
	}
	try{
		for(PreProc_ curr_obj = pre_proc; curr_obj; curr_obj = curr_obj->next){
			tracker->setImage(curr_obj->getFrame());
		}
		printf("Initializing tracker with object of size %f x %f\n", size_x, size_y);
		tracker->initialize(init_corners_cv);
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while initializing the tracker: %s\n",
			err.type(), err.what());
		return false;
	}
	if(show_cv_window) {
		cv::namedWindow("mexMTF", cv::WINDOW_AUTOSIZE);
		obj_cols.push_back(cv::Scalar(0, 0, 255));
		obj_cols.push_back(cv::Scalar(0, 255, 0));
		obj_cols.push_back(cv::Scalar(255, 0, 0));
		obj_cols.push_back(cv::Scalar(255, 255, 0));
		obj_cols.push_back(cv::Scalar(255, 0, 255));
		obj_cols.push_back(cv::Scalar(0, 255, 255));
		obj_cols.push_back(cv::Scalar(255, 255, 255));
		obj_cols.push_back(cv::Scalar(0, 0, 0));
	}
	frame_id = 0;
	return true;
}

bool updateTracker(const cv::Mat &curr_img) {
	double fps = 0, fps_win = 0;
	double tracking_time, tracking_time_with_input;
	++frame_id;
	mtf_clock_get(start_time_with_input);
	//update trackers
	for(unsigned int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
		try{
			//! update pre processor
			pre_proc[tracker_id]->update(curr_img);
			mtf_clock_get(start_time);
			//! update tracker
			tracker->update();
			mtf_clock_get(end_time);
			mtf_clock_measure(start_time, end_time, tracking_time);
			mtf_clock_measure(start_time_with_input, end_time, tracking_time_with_input);
			fps = 1.0 / tracking_time;
			fps_win = 1.0 / tracking_time_with_input;
			if(reset_template){
				tracker->initialize(tracker->getRegion());
			}
		} catch(const mtf::utils::Exception &err){
			printf("Exception of type %s encountered while updating the tracker: %s\n",
				err.type(), err.what());
			return false;
		}
	}
	if(show_cv_window) {
		/* draw tracker positions to OpenCV window */
			int col_id = 0;
			cv::Point2d corners[4];
			mtf::utils::Corners(tracker->getRegion()).points(corners);
			line(curr_img_cv, corners[0], corners[1], obj_cols[col_id], line_thickness);
			line(curr_img_cv, corners[1], corners[2], obj_cols[col_id], line_thickness);
			line(curr_img_cv, corners[2], corners[3], obj_cols[col_id], line_thickness);
			line(curr_img_cv, corners[3], corners[0], obj_cols[col_id], line_thickness);
			putText(curr_img_cv, tracker->name, corners[0],
				cv::FONT_HERSHEY_SIMPLEX, fps_font_size, obj_cols[col_id]);
		std::string fps_text = cv::format("frame: %d c: %12.6f cw: %12.6f", frame_id, fps, fps_win);
		putText(curr_img_cv, fps_text, fps_origin, cv::FONT_HERSHEY_SIMPLEX, fps_font_size, fps_color);
		imshow("mexMTF", curr_img_cv);
		cv::waitKey(1);
	}
	cv::Mat out_corners = tracker->getRegion();
	for(unsigned int corner_id = 0; corner_id < 4; ++corner_id) {
		out_corners_data[corner_id] = out_corners.at<double>(0, corner_id);
		out_corners_data[corner_id + 4] = out_corners.at<double>(1, corner_id);
	}
	return true;
}
bool setRegion(const cv::Mat &corners) {
	try{
		tracker->setRegion(corners);
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while resetting the tracker: %s\n",
			err.type(), err.what());
		return false;
	}
	return true;
}
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
	if(nrhs < 1){
		mexErrMsgTxt("Not enough input arguments.");
	}
	if(!mxIsChar(prhs[0])){
		mexErrMsgTxt("The first argument must be a string.");
	}
	int cmd_str_len = mxGetM(prhs[0])*mxGetN(prhs[0]) + 1;
	char *cmd_str = (char *)mxMalloc(cmd_str_len);
	mxGetString(prhs[0], cmd_str, cmd_str_len);
	auto cmd_iter = cmd_list.find[std::string(cmd_str)];
	if(cmd_iter == cmd_list.end()){
		mexErrMsgTxt(cv::format("Invalid comand provided: %s.", cmd_str).c_str());
	}
	const int cmd_id = cmd_iter->second();
	plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
	double *ret_val = mxGetPr(plhs[0]);
	switch(cmd_id) {
	case MEX_TRACKER_CREATE:
		if(nrhs > 1){
			int cfg_str_len = mxGetM(prhs[1])*mxGetN(prhs[1]) + 1;
			config_root_dir = (char *)mxMalloc(cfg_str_len);
			mxGetString(prhs[1], config_root_dir, cfg_str_len);
		}
		if(!createTracker()){
			*ret_val = 0;
			return;
		}
		tracker_created = true;
		*ret_val = 1;
		return;
	case MEX_TRACKER_INITIALIZE:
		double *init_img_ptr = mxGetPr(prhs[1]);
		int img_n_dims = mxGetNumberOfDimensions(prhs[1]);
		if(img_n_dims<2 || img_n_dims>3){
			mexErrMsgTxt("Input image must have 2 or 3 dimensions");
		}
		int img_type = img_n_dims == 2 ? CV_8UC1 : CV_8UC3;
		const mwSize *img_dims = mxGetDimensions(prhs[1]);
		int width = img_dims[0];
		int height = img_dims[1];		
		cv::Mat init_img(width, height, img_type, init_img_ptr);

		int corners_n_dims = mxGetNumberOfDimensions(prhs[2]);
		if(img_n_dims != 2){
			mexErrMsgTxt("Input corner array must have 2 dimensions");
		}
		const mwSize *corners_dims = mxGetDimensions(prhs[2]);
		if(img_dims[0] != 4 || img_dims[1] != 2){
			mexErrMsgTxt("Input corner array must be of size 2 x 4");
		}
		double *corners_ptr = mxGetPr(prhs[2]);
		cv::Mat init_corners(width, height, CV_64FC1, corners_ptr);
		if(!initializeTracker(init_img, init_corners)){
			*ret_val = 0;
			return;
		}
		return;
	case MEX_TRACKER_UPDATE:
		if(!updateTracker()){
			*ret_val = 0;
			return;
		}
		return;
	case MEX_TRACKER_SET_REGION:
		if(!setRegion()){
			*ret_val = 0;
			return;
		}
		return;
	default:
		mexErrMsgTxt(cv::format("Invalid comand provided: %s.", cmd_str).c_str());
	}

}
