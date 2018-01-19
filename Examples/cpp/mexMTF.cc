#include "mtf/mtf.h"
#include "mtf/pipeline.h"
#include "mtf/Utilities/mexUtils.h"

#include <vector>
#include <map>
#include <memory>

#define MEX_INPUT_CREATE 1
#define MEX_INPUT_UPDATE 2
#define MEX_INPUT_REMOVE 3
#define MEX_TRACKER_CREATE 4
#define MEX_TRACKER_INITIALIZE 5
#define MEX_TRACKER_UPDATE 6
#define MEX_TRACKER_SET_REGION 7
#define MEX_TRACKER_REMOVE 8
#define MEX_CLEAR 9

using namespace std;
using namespace mtf::params;
typedef std::shared_ptr<mtf::utils::InputBase> Input;
typedef std::shared_ptr<mtf::TrackerBase> Tracker;
typedef PreProc_ PreProc;

static std::map<std::string, const int> cmd_list = {
	{ "create_input", MEX_INPUT_CREATE },
	{ "update_input", MEX_INPUT_UPDATE },
	{ "remove_input", MEX_INPUT_REMOVE },
	{ "create_tracker", MEX_TRACKER_CREATE },
	{ "initialize_tracker", MEX_TRACKER_INITIALIZE },
	{ "update_tracker", MEX_TRACKER_UPDATE },
	{ "set_region", MEX_TRACKER_SET_REGION },
	{ "remove_tracker", MEX_TRACKER_REMOVE },
	{ "clear", MEX_CLEAR }
};

struct TrackerStruct{
	Tracker tracker;
	PreProc pre_proc;
	TrackerStruct(Tracker _tracker, PreProc _pre_proc) :
		tracker(_tracker), pre_proc(_pre_proc){}
};

static std::map<int, Input> input_pipelines;
static std::map<int, TrackerStruct> trackers;

static double min_x, min_y, max_x, max_y;
static double size_x, size_y;

static int img_height, img_width;
static vector<cv::Scalar> obj_cols;

static int frame_id;
static unsigned int _tracker_id = 0, _input_id = 0;

bool createInput() {
	Input input;
	try{
		input.reset(mtf::getInput(pipeline));
		if(!input->initialize()){
			printf("Pipeline could not be initialized successfully. Exiting...\n");
			return false;
		}
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while initializing the input pipeline: %s\n",
			err.type(), err.what());
		return false;
	}
	++_input_id;
	input_pipelines.insert(std::pair<int, Input>(_input_id, input));
	return true;
}

bool updateInput(unsigned int input_id, mxArray* &plhs) {
	std::map<int, Input>::iterator it = input_pipelines.find(input_id);
	if(it == input_pipelines.end()){
		printf("Invalid input ID: %d\n", input_id);
		return false;
	}
	it->second->update();
	const cv::Mat frame = it->second->getFrame();
	int n_channels = 3, n_dims = 3;
	if(frame.type() == CV_8UC1){
		n_channels = 1;
		n_dims = 2;
	}	
	mwSize *dims = new mwSize[n_dims];
	dims[0] = frame.rows;
	dims[1] = frame.cols;
	if(n_channels == 3){
		dims[2] = n_channels;
	}
	plhs = mxCreateNumericArray(n_dims, dims, mxUINT8_CLASS, mxREAL);
	mtf::utils::copyMatrixToMatlab<unsigned char>(frame, (unsigned char*)mxGetPr(plhs), n_channels);
	delete(dims);
	return true;
}
bool createTracker() {
	Tracker tracker;
	try{
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
	PreProc pre_proc;
	try{
		pre_proc = mtf::getPreProc(tracker->inputType(), pre_proc_type);
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while creating the pre processor: %s\n",
			err.type(), err.what());
		return false;
	}
	++_tracker_id;
	trackers.insert(std::pair<int, TrackerStruct>(_tracker_id, TrackerStruct(tracker, pre_proc)));
	return true;
}

bool initializeTracker(unsigned int tracker_id, const cv::Mat &init_img, const cv::Mat &init_corners) {
	std::map<int, TrackerStruct>::iterator it = trackers.find(tracker_id);
	if(it == trackers.end()){
		printf("Invalid tracker ID: %d\n", tracker_id);
		return false;
	}
	img_height = init_img.rows;
	img_width = init_img.cols;

	//printf("img_height: %d\n", img_height);
	//printf("img_width: %d\n", img_width);
	//printf("init_corners:\n");
	//for(unsigned int corner_id = 0; corner_id < 4; ++corner_id) {
	//	printf("%d: (%f, %f)\n", corner_id, init_corners.at<double>(0, corner_id), init_corners.at<double>(1, corner_id));
	//}
	min_x = init_corners.at<double>(0, 0);
	min_y = init_corners.at<double>(1, 0);
	max_x = init_corners.at<double>(0, 2);
	max_y = init_corners.at<double>(1, 2);
	size_x = max_x - min_x;
	size_y = max_y - min_y;
	try{
		it->second.pre_proc->initialize(init_img);
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while initializing the pre processor: %s\n",
			err.type(), err.what());
		return false;
	}
	try{
		for(PreProc curr_obj = it->second.pre_proc; curr_obj; curr_obj = curr_obj->next){
			it->second.tracker->setImage(curr_obj->getFrame());
		}
		printf("Initializing tracker with object of size %f x %f\n", size_x, size_y);
		it->second.tracker->initialize(init_corners);
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while initializing the tracker: %s\n",
			err.type(), err.what());
		return false;
	}
	frame_id = 0;
	return true;
}

bool updateTracker(unsigned int tracker_id, const cv::Mat &curr_img, mxArray* &plhs) {
	std::map<int, TrackerStruct>::iterator it = trackers.find(tracker_id);
	if(it == trackers.end()){
		printf("Invalid tracker ID: %d\n", tracker_id);
		return false;
	}
	double fps = 0, fps_win = 0;
	double tracking_time, tracking_time_with_input;
	++frame_id;
	mtf_clock_get(start_time_with_input);
	try{
		//! update pre processor
		it->second.pre_proc->update(curr_img);
		mtf_clock_get(start_time);
		//! update tracker
		it->second.tracker->update();
		if(print_fps){
			mtf_clock_get(end_time);
			mtf_clock_measure(start_time, end_time, tracking_time);
			mtf_clock_measure(start_time_with_input, end_time, tracking_time_with_input);
			fps = 1.0 / tracking_time;
			fps_win = 1.0 / tracking_time_with_input;
			printf("fps: %f\t fps_win=%f\n", fps, fps_win);
		}
		if(reset_template){
			it->second.tracker->initialize(it->second.tracker->getRegion());
		}
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while updating the tracker: %s\n",
			err.type(), err.what());
		return false;
	}
	plhs = mtf::utils::setCorners(it->second.tracker->getRegion());
	return true;
}
bool setRegion(unsigned int tracker_id, const cv::Mat &corners, mxArray* &plhs) {
	std::map<int, TrackerStruct>::iterator it = trackers.find(tracker_id);
	if(it == trackers.end()){
		printf("Invalid tracker ID: %d\n", tracker_id);
		return false;
	}
	try{
		it->second.tracker->setRegion(corners);
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while resetting the tracker: %s\n",
			err.type(), err.what());
		return false;
	}
	plhs = mtf::utils::setCorners(it->second.tracker->getRegion());
	return true;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){
	mwSize dims[2] = { 1, 1 };
	plhs[0] = mxCreateNumericArray(2, dims, mxUINT32_CLASS, mxREAL);
	//plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
	unsigned int *ret_val = (unsigned int*)mxGetPr(plhs[0]);
	if(nrhs < 1){
		mexErrMsgTxt("Not enough input arguments.");
	}
	if(!mxIsChar(prhs[0])){
		mexErrMsgTxt("The first argument must be a string.");
	}
	int cmd_str_len = mxGetM(prhs[0])*mxGetN(prhs[0]) + 1;
	char *cmd_str = (char *)mxMalloc(cmd_str_len);
	mxGetString(prhs[0], cmd_str, cmd_str_len);

	//printf("cmd_str: %s\n", cmd_str);
	//int k;
	//scanf("Press any key to continue: %d", &k);

	auto cmd_iter = cmd_list.find(std::string(cmd_str));
	if(cmd_iter == cmd_list.end()){
		mexErrMsgTxt(cv::format("Invalid command provided: %s.", cmd_str).c_str());
	}
	const int cmd_id = cmd_iter->second;

	switch(cmd_id) {
	case MEX_INPUT_CREATE:
	{
		//if(nlhs != 2){
		//	mexErrMsgTxt("2 output arguments (input_id, n_frames) are needed to create input pipeline.");
		//}
		if(nrhs > 1){
			if(!mxIsChar(prhs[1])){
				mexErrMsgTxt("Second input argument for creating input pipeline must be a string.");
			}
			if(!readParams(mtf::utils::toString(prhs[1]))) {
				mexErrMsgTxt("Parameters could not be parsed");
			}
		}
		if(!createInput()){
			*ret_val = 0;
			return;
		}
		if(nlhs >= 2){
			std::map<int, Input>::iterator it = input_pipelines.find(_input_id);
			mwSize dims[2] = { 1, 1 };
			plhs[1] = mxCreateNumericArray(2, dims, mxUINT32_CLASS, mxREAL);
			unsigned int *n_frames = (unsigned int*)mxGetPr(plhs[1]);
			*n_frames = static_cast<unsigned int>(it->second->getNFrames());
		}
		*ret_val = _input_id;
		return;
	}
	case MEX_INPUT_UPDATE:
	{
		if(nrhs < 2){
			mexErrMsgTxt("At least 2 input arguments are needed to update tracker.");
		}
		if(nlhs != 2){
			mexErrMsgTxt("2 output arguments are needed to update input pipeline.");
		}
		unsigned int input_id = mtf::utils::getID(prhs[1]);
		if(!updateInput(input_id, plhs[1])){
			*ret_val = 0;
			return;
		}
		*ret_val = 1;
		return;
	}
	case MEX_INPUT_REMOVE:
	{
		if(nrhs < 2){
			mexErrMsgTxt("At least 2 input arguments are needed to remove input pipeline.");
		}
		if(nlhs != 1){
			mexErrMsgTxt("1 output argument is needed to remove input pipeline.");
		}
		unsigned int input_id = mtf::utils::getID(prhs[1]);
		std::map<int, Input>::iterator it = input_pipelines.find(input_id);
		if(it == input_pipelines.end()){
			printf("Invalid input ID: %d\n", input_id);
			*ret_val = 0;
			return;
		}
		input_pipelines.erase(it);
		*ret_val = 1;
		return;
	}
	case MEX_TRACKER_CREATE:
	{
		if(nrhs > 1){
			if(!mxIsChar(prhs[1])){
				mexErrMsgTxt("Second input argument for creating tracker must be a string.");
			}
			if(!readParams(mtf::utils::toString(prhs[1]))) {
				mexErrMsgTxt("Parameters could not be parsed");
			}
		}
		if(!createTracker()){
			*ret_val = 0;
			return;
		}
		*ret_val = _tracker_id;
		return;
	}
	case MEX_TRACKER_INITIALIZE:
	{
		if(nrhs < 3){
			mexErrMsgTxt("At least 3 input arguments are needed to initialize tracker.");
		}
		unsigned int tracker_id = mtf::utils::getID(prhs[1]);
		cv::Mat init_img = mtf::utils::getImage(prhs[2]);
		cv::Mat init_corners;
		if(nrhs > 3){
			init_corners = mtf::utils::getCorners(prhs[3]);
		} else{
			mtf::utils::ObjUtils obj_utils;
			try{
				if(mex_live_init){
					if(input_pipelines.empty()){
						printf("At least one input pipeline must exist for live initialization: %d\n", tracker_id);
						*ret_val = 0;
						return;
					}
					if(!obj_utils.selectObjects(input_pipelines.begin()->second.get(), 1, 
						patch_size, line_thickness, write_objs, sel_quad_obj,
						write_obj_fname.c_str())){
						mexErrMsgTxt("Object(s) to be tracked could not be obtained.\n");
					}
				} else{
					if(!obj_utils.selectObjects(init_img, 1, patch_size, line_thickness,
						write_objs, sel_quad_obj, write_obj_fname.c_str())){
						mexErrMsgTxt("Object(s) to be tracked could not be obtained.\n");
					}
				}

			} catch(const mtf::utils::Exception &err){
				mexErrMsgTxt(cv::format("Exception of type %s encountered while obtaining the objects to track: %s\n",
					err.type(), err.what()).c_str());
			}
			init_corners = obj_utils.getObj().corners.clone();
		}
		if(!initializeTracker(tracker_id, init_img, init_corners)){
			*ret_val = 0;
			return;
		}
		*ret_val = 1;
		return;
	}
	case MEX_TRACKER_UPDATE:
	{
		if(nrhs <= 2){
			mexErrMsgTxt("At least 3 input arguments are needed to update tracker.");
		}
		if(nlhs != 2){
			mexErrMsgTxt("2 output arguments are needed to update tracker.");
		}
		unsigned int tracker_id = mtf::utils::getID(prhs[1]);
		cv::Mat curr_img = mtf::utils::getImage(prhs[2]);
		if(!updateTracker(tracker_id, curr_img, plhs[1])){
			*ret_val = 0;
			return;
		}
		*ret_val = 1;
		return;
	}
	case MEX_TRACKER_SET_REGION:
	{
		if(nrhs < 2){
			mexErrMsgTxt("At least 2 input arguments are needed to reset tracker.");
		}
		if(nlhs != 2){
			mexErrMsgTxt("2 output arguments are needed to reset tracker.");
		}
		unsigned int tracker_id = mtf::utils::getID(prhs[1]);
		cv::Mat curr_corners = mtf::utils::getCorners(prhs[2]);
		if(!setRegion(tracker_id, curr_corners, plhs[1])){
			*ret_val = 0;
			return;
		}
		
		*ret_val = 1;
		return;
	}
	case MEX_TRACKER_REMOVE:
	{
		if(nrhs < 2){
			mexErrMsgTxt("At least 2 input arguments are needed to reset tracker.");
		}
		if(nlhs != 1){
			mexErrMsgTxt("1 output argument is needed to reset tracker.");
		}
		unsigned int tracker_id = mtf::utils::getID(prhs[1]);
		std::map<int, TrackerStruct>::iterator it = trackers.find(tracker_id);
		if(it == trackers.end()){
			printf("Invalid tracker ID: %d\n", tracker_id);
			*ret_val = 0;
			return;
		}
		trackers.erase(it);
		*ret_val = 1;
		return;
	}
	case MEX_CLEAR:
	{
		printf("Clearing up...");
		input_pipelines.clear();
		trackers.clear();		
		*ret_val = 1;
		printf("Done\n");
		return;
	}
	default:
		mexErrMsgTxt(cv::format("Invalid command provided: %s.", cmd_str).c_str());
	}
}
