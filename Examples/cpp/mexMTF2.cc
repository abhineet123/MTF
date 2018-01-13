#include "mtf/mtf.h"
//! tools for reading in images from various sources like image sequences, 
//! videos and cameras as well as for pre processing them
#include "mtf/pipeline.h"
#include "mtf/Config/parameters.h"
#include "mtf/Utilities/miscUtils.h"

#include <time.h>
#include <string.h>
#include <vector>
#include <map>
#include <memory>
#include <sstream>

#include "opencv2/core/core.hpp"
#include <boost/thread.hpp>
#include <mex.h>

#define MEX_CREATE_INPUT 1
#define MEX_GET_FRAME 2
#define MEX_REMOVE_INPUT 3
#define MEX_CREATE_TRACKER 4
#define MEX_GET_REGION 5
#define MEX_SET_REGION 6
#define MEX_REMOVE_TRACKER 7
#define MEX_CLEAR 8

#define _A3D_IDX_COLUMN_MAJOR(i,j,k,nrows,ncols) ((i)+((j)+(k)*ncols)*nrows)
// interleaved row-major indexing for 2-D OpenCV images
//#define _A3D_IDX_OPENCV(x,y,c,mat) (((y)*mat.step[0]) + ((x)*mat.step[1]) + (c))
#define _A3D_IDX_OPENCV(i,j,k,nrows,ncols,nchannels) (((i*ncols + j)*nchannels) + (k))

using namespace std;
using namespace mtf::params;
typedef std::shared_ptr<mtf::utils::InputBase> Input;
typedef std::shared_ptr<mtf::TrackerBase> Tracker;
typedef PreProc_ PreProc;

static std::map<std::string, const int> cmd_list = {
	{ "create_input", MEX_CREATE_INPUT },
	{ "get_frame", MEX_GET_FRAME },
	{ "remove_input", MEX_REMOVE_INPUT },
	{ "create_tracker", MEX_CREATE_TRACKER },
	{ "get_region", MEX_GET_REGION },
	{ "set_region", MEX_SET_REGION },
	{ "remove_tracker", MEX_REMOVE_TRACKER },
	{ "clear", MEX_CLEAR }
};

struct InputStruct{
	Input input;
	boost::thread t;
	InputStruct(Input _input) :
		input(_input){
		 t = boost::thread{ RunInput(input) };
	}

};
struct RunInput{
	RunInput(Input _input) : input(_input){}
	void operator()(){
		while(input->update()){
			if(input->destroy){return;}	
			boost::this_thread::interruption_point();
		}
		input->destroy = true;
	}

private:
	Input input;	
};

struct TrackerStruct{
	Tracker tracker;
	PreProc pre_proc;
	boost::thread t;
	TrackerStruct(Tracker &_tracker, PreProc &_pre_proc, Input &_input) :
		tracker(_tracker), pre_proc(_pre_proc){
		t = boost::thread{ RunTracker(tracker, pre_proc, _input) };
	}

};
struct RunTracker{
	RunTracker(Tracker _tracker, PreProc _pre_proc, Input _input) :
		tracker(_tracker), pre_proc(_pre_proc), input(_input){}
	void operator()(){
		int frame_id = 0;
		while(!input->destroy){
			if(frame_id == input->getFrameID()){
				//! do not process the same image multiple times
				continue;
			}

			frame_id = input->getFrameID();

			double tracking_time, tracking_time_with_input;
			mtf_clock_get(start_time_with_input);
			try{
				//! update pre processor
				pre_proc->update(input->getFrame());
				mtf_clock_get(start_time);
				//! update tracker
				tracker->update();
				if(print_fps){
					mtf_clock_get(end_time);
					mtf_clock_measure(start_time, end_time, tracking_time);
					mtf_clock_measure(start_time_with_input, end_time, tracking_time_with_input);
					double fps = 1.0 / tracking_time;
					double fps_win = 1.0 / tracking_time_with_input;
					printf("fps: %f\t fps_win=%f\n", fps, fps_win);
				}
				if(reset_template){
					tracker->initialize(tracker->getRegion());
				}
			} catch(const mtf::utils::Exception &err){
				printf("Exception of type %s encountered while updating the tracker: %s\n",
					err.type(), err.what());
				return;
			}
			boost::this_thread::interruption_point();
		}
	}

private:
	Input input;
	PreProc pre_proc;
	Tracker tracker;	
};

static std::map<int, InputStruct> input_pipelines;
static std::map<int, TrackerStruct> trackers;

static double min_x, min_y, max_x, max_y;
static double size_x, size_y;

static int img_height, img_width;
static vector<cv::Scalar> obj_cols;

static int frame_id;
static unsigned int _tracker_id = 0, _input_id = 0;

/**
* Copy the (image) data from Matlab-algorithm compatible (column-major) representation to cv::Mat.
* The information about the image are taken from the OpenCV cv::Mat structure.
adapted from OpenCV-Matlab package available at: https://se.mathworks.com/matlabcentral/fileexchange/41530-opencv-matlab
*/
template <typename T>
inline void
copyMatrixFromMatlab(const T* from, cv::Mat& to, int n_channels){

	const int n_rows = to.rows;
	const int n_cols = to.cols;

	T* pdata = (T*)to.data;

	for(int c = 0; c < n_channels; ++c){
		for(int x = 0; x < n_cols; ++x){
			for(int y = 0; y < n_rows; ++y){
				const T element = from[_A3D_IDX_COLUMN_MAJOR(y, x, c, n_rows, n_cols)];
				pdata[_A3D_IDX_OPENCV(y, x, c, rows, n_cols, n_channels)] = element;
			}
		}
	}
}
template <typename T>
inline void
copyMatrixToMatlab(const cv::Mat& from, T* to, int n_channels)
{
	assert(from.dims == 2); // =2 <=> 2-D image

	const int n_rows = from.rows;
	const int n_cols = from.cols;

	const T* pdata = (T*)from.data;

	for(int c = 0; c < n_channels; ++c){
		for(int x = 0; x < n_cols; ++x){
			for(int y = 0; y < n_rows; ++y){
				//const T element = pdata[_A3D_IDX_OPENCV(x,y,c,from)];
				const T element = pdata[_A3D_IDX_OPENCV(y, x, c, rows, n_cols, n_channels)];
				to[_A3D_IDX_COLUMN_MAJOR(y, x, c, n_rows, n_cols)] = element;
			}
		}
	}
}
unsigned int getID(const mxArray *mx_id){
	if(!mxIsClass(mx_id, "uint32")){
		mexErrMsgTxt("ID must be of 32 bit unsigned integral type");
	}
	unsigned int* id_ptr = (unsigned int*)mxGetData(mx_id);
	return *id_ptr;
}
cv::Mat getImage(const mxArray *mx_img){
	int img_n_dims = mxGetNumberOfDimensions(mx_img);
	if(!mxIsClass(mx_img, "uint8")){
		mexErrMsgTxt("Input image must be of 8 bit unsigned integral type");
	}
	if(img_n_dims < 2 || img_n_dims > 3){
		mexErrMsgTxt("Input image must have 2 or 3 dimensions");
	}
	int img_type = img_n_dims == 2 ? CV_8UC1 : CV_8UC3;
	const mwSize *img_dims = mxGetDimensions(mx_img);
	int height = img_dims[0];
	int width = img_dims[1];
	//printf("width: %d\t height=%d\t img_n_dims: %d\n", width, height, img_n_dims);
	unsigned char *img_ptr = (unsigned char*)mxGetData(mx_img);
	cv::Mat img(height, width, img_type);
	if(img_n_dims == 2){
		cv::Mat img_transpose(width, height, img_type, img_ptr);
		cv::transpose(img_transpose, img);
	} else{
		copyMatrixFromMatlab(img_ptr, img, 3);
	}
	return img;
}
cv::Mat getCorners(const mxArray *mx_corners){
	int corners_n_dims = mxGetNumberOfDimensions(mx_corners);
	if(!mxIsClass(mx_corners, "double")){
		mexErrMsgTxt("Input corner array must be of 64 bit floating point type");
	}
	if(corners_n_dims != 2){
		mexErrMsgTxt("Input corner array must have 2 dimensions");
	}
	const mwSize *corners_dims = mxGetDimensions(mx_corners);
	if(corners_dims[0] != 2 || corners_dims[1] != 4){
		mexErrMsgTxt("Input corner array must be of size 2 x 4");
	}
	double *corners_ptr = mxGetPr(mx_corners);
	cv::Mat corners_transposed(4, 2, CV_64FC1, corners_ptr);
	//cout << "corners_transposed: \n" << corners_transposed << "\n";
	cv::Mat corners(2, 4, CV_64FC1);
	cv::transpose(corners_transposed, corners);
	//cout << "corners: \n" << corners << "\n";
	return corners;
}
void parseParams(const mxArray *prhs){
	int param_str_len = mxGetM(prhs)*mxGetN(prhs) + 1;
	char* param_str = (char *)mxMalloc(param_str_len);
	mxGetString(prhs, param_str, param_str_len);
	std::vector<char*> fargv;
	fargv.push_back(nullptr);
	std::string _param_str = std::string(param_str);
	std::istringstream iss(_param_str);
	do{
		string subs;
		iss >> subs;

		if(subs.empty()){ continue; }

		//printf("subs: %s\n", subs.c_str());
		char *cstr = new char[subs.length() + 1];
		strcpy(cstr, subs.c_str());
		fargv.push_back(cstr);
	} while(iss);
	//fargv.pop_back();
	//printf("fargv.size(): %d\n", fargv.size());
	if(fargv.size() % 2 == 0){
		//printf("param_str: %s\n", param_str);
		mexErrMsgTxt("Parameters must be provided in pairs");
	}
	if(!readParams(fargv.size(), fargv.data())){
		mexErrMsgTxt("Parameters could not be parsed");
	}
}

mxArray *setCorners(const cv::Mat &corners){
	mxArray *mx_corners = mxCreateDoubleMatrix(2, 4, mxREAL);
	/* get a pointer to the real data in the output matrix */
	double *out_corners_ptr = mxGetPr(mx_corners);
	cv::Mat out_corners(4, 2, CV_64FC1, out_corners_ptr);
	cv::transpose(corners, out_corners);
	return mx_corners;
}

bool createInput(mxArray* &plhs) {
	if(input_buffer_size <= 1){
		input_buffer_size = 100;
	}
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
	mwSize dims[2] = { 1, 1 };
	plhs = mxCreateNumericArray(2, dims, mxUINT32_CLASS, mxREAL);
	//plhs[0] = mxCreateDoubleMatrix(1, 1, mxREAL);
	unsigned int *ret_val = (unsigned int*)mxGetPr(plhs);
	*ret_val = static_cast<unsigned int>(input->getNFrames());	
	return true;
}

bool getFrame(unsigned int input_id, mxArray* &plhs){
	std::map<int, InputStruct>::iterator it = input_pipelines.find(input_id);
	if(it == input_pipelines.end()){
		printf("Invalid input ID: %d\n", input_id);
		return false;
	}
	const cv::Mat frame = it->second.input->getFrame();
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
	copyMatrixToMatlab<unsigned char>(frame, (unsigned char*)mxGetPr(plhs), n_channels);
	delete(dims);
	return true;
}
bool createTracker(int input_id, mxArray* &plhs) {
	std::map<int, InputStruct>::iterator it = input_pipelines.find(input_id);
	if(it == input_pipelines.end()){
		printf("Invalid input ID: %d\n", input_id);
		return false;
	}
	Input input = it->second.input;
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
	mtf::utils::ObjUtils obj_utils;
	try{
		if(!obj_utils.selectObjects(input.get(), 1,
			patch_size, line_thickness, write_objs, sel_quad_obj,
			write_obj_fname.c_str())){
			mexErrMsgTxt("Object to be tracked could not be obtained.\n");
		}
	} catch(const mtf::utils::Exception &err){
		mexErrMsgTxt(cv::format("Exception of type %s encountered while obtaining the object to track: %s\n",
			err.type(), err.what()).c_str());
	}
	cv::Mat init_corners = obj_utils.getObj().corners.clone();
	if(!initializeTracker(tracker, pre_proc, input->getFrame(), init_corners)){
		mexErrMsgTxt("Tracker initialization failed");
	}
	++_tracker_id;
	trackers.insert(std::pair<int, TrackerStruct>(_tracker_id, TrackerStruct(tracker, pre_proc, input)));
	return true;
}

bool initializeTracker(Tracker &tracker, PreProc &pre_proc, 
	const cv::Mat &init_img, const cv::Mat &init_corners, mxArray* &plhs) {
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
		pre_proc->initialize(init_img);
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while initializing the pre processor: %s\n",
			err.type(), err.what());
		return false;
	}
	try{
		for(PreProc curr_obj = it->second.pre_proc; curr_obj; curr_obj = curr_obj->next){
			tracker->setImage(curr_obj->getFrame());
		}
		printf("Initializing tracker with object of size %f x %f\n", size_x, size_y);
		tracker->initialize(init_corners);
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while initializing the tracker: %s\n",
			err.type(), err.what());
		return false;
	}
	frame_id = 0;
	plhs = setCorners(tracker->getRegion());
	return true;
}

bool updateTracker(unsigned int tracker_id, const cv::Mat &curr_img, mxArray* &plhs) {
	std::map<int, TrackerStruct>::iterator it = trackers.find(tracker_id);
	if(it == trackers.end()){
		printf("Invalid tracker ID: %d\n", tracker_id);
		return false;
	}
	plhs = setCorners(it->second.tracker->getRegion());
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
	plhs = setCorners(it->second.tracker->getRegion());
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
		mexErrMsgTxt(cv::format("Invalid comand provided: %s.", cmd_str).c_str());
	}
	const int cmd_id = cmd_iter->second;

	switch(cmd_id) {
	case MEX_CREATE_INPUT:
	{
		if(nlhs != 2){
			mexErrMsgTxt("2 output arguments (input_id, n_frames) are needed to create input pipeline.");
		}
		if(nrhs > 1){
			if(!mxIsChar(prhs[1])){
				mexErrMsgTxt("Second input argument for creating input pipeline must be a string.");
			}
			parseParams(prhs[1]);
		}
		if(!createInput(plhs[1])){
			*ret_val = 0;
			return;
		}
		*ret_val = _input_id;
		return;
	}
	case MEX_REMOVE_INPUT:
	{
		if(nrhs < 2){
			mexErrMsgTxt("At least 2 input arguments are needed to remove input pipeline.");
		}
		if(nlhs != 1){
			mexErrMsgTxt("1 output argument is needed to remove input pipeline.");
		}
		unsigned int input_id = getID(prhs[1]);
		std::map<int, InputStruct>::iterator it = input_pipelines.find(input_id);
		if(it == input_pipelines.end()){
			printf("Invalid input ID: %d\n", input_id);
			*ret_val = 0;
			return;
		}
		it->second.t.interrupt();
		input_pipelines.erase(it);
		*ret_val = 1;
		return;
	}
	case MEX_CREATE_TRACKER:
	{
		if(nlhs != 2){
			mexErrMsgTxt("2 output arguments (tracker_id, init_region) are needed to create tracker.");
		}
		if(nrhs < 2){
			mexErrMsgTxt("At least 2 input arguments are needed to create tracker.");
		}
		if(nrhs > 2){
			if(!mxIsChar(prhs[2])){
				mexErrMsgTxt("The optional third input argument for creating tracker must be a string.");
			}
			parseParams(prhs[2]);
		}
		unsigned int input_id = getID(prhs[1]);
		if(!createTracker(input_id, plhs[1])){
			*ret_val = 0;
			return;
		}
		*ret_val = _tracker_id;
		return;
	}
	case MEX_SET_REGION:
	{
		if(nrhs < 2){
			mexErrMsgTxt("At least 2 input arguments are needed to reset tracker.");
		}
		if(nlhs != 2){
			mexErrMsgTxt("2 output arguments are needed to reset tracker.");
		}
		unsigned int tracker_id = getID(prhs[1]);
		cv::Mat curr_corners = getCorners(prhs[2]);
		if(!setRegion(tracker_id, curr_corners, plhs[1])){
			*ret_val = 0;
			return;
		}		
		*ret_val = 1;
		return;
	}
	case MEX_REMOVE_TRACKER:
	{
		if(nrhs < 2){
			mexErrMsgTxt("At least 2 input arguments are needed to reset tracker.");
		}
		if(nlhs != 1){
			mexErrMsgTxt("1 output argument is needed to reset tracker.");
		}
		unsigned int tracker_id = getID(prhs[1]);
		std::map<int, TrackerStruct>::iterator it = trackers.find(tracker_id);
		if(it == trackers.end()){
			printf("Invalid tracker ID: %d\n", tracker_id);
			*ret_val = 0;
			return;
		}
		it->second.t.interrupt();
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
