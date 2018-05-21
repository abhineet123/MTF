//! Matlab comes with its own horrible version of boost libraries that cause the mex module to segfault if boost::filesystem::absolute is used
//! It is also apparently impossible to force Matlab to load the correct version of boost libraries through LD_PRELOAD; attempting to do so causes Matlab itself to segfault at startup
// #define DISABLE_BOOST_ABSOLUTE

#include "mtf/TrackerStruct.h"
#include "mtf/Utilities/mexUtils.h"

#include <vector>
#include <map>
#include <memory>

#define MEX_INIT 1
#define MEX_GET_FRAME 2
#define MEX_TRACKER_CREATE 3
#define MEX_GET_REGION 4
#define MEX_SET_REGION 5
#define MEX_TRACKER_REMOVE 6
#define MEX_TRACKERS_REMOVE 7
#define MEX_QUIT 8
#define MEX_IS_INITIALIZED 9

using namespace std;
using namespace mtf::params;
typedef std::shared_ptr<mtf::utils::InputBase> Input;

static std::map<std::string, const int> cmd_list = {
	{ "init", MEX_INIT },
	{ "is_initialized", MEX_IS_INITIALIZED },
	{ "get_frame", MEX_GET_FRAME },
	{ "create_tracker", MEX_TRACKER_CREATE },
	{ "get_region", MEX_GET_REGION },
	{ "set_region", MEX_SET_REGION },
	{ "remove_tracker", MEX_TRACKER_REMOVE },
	{ "remove_trackers", MEX_TRACKERS_REMOVE },
	{ "quit", MEX_QUIT }
};

static Input input;
static std::map<int, TrackerStruct> trackers;

static unsigned int _tracker_id = 0;
bool createInput() {
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
	return true;
}

bool updateInput(){
	if(!input){
		printf("Input pipeline is not initialized");
		return false;
	}
	if(!input->update()){
		printf("Input pipeline could not be updated");
		return false;
	}
	return true;
}

bool getFrame(mxArray* &plhs) {
	if(!updateInput()){
		return false;
	}
	const cv::Mat frame = input->getFrame();
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

bool setRegion(unsigned int tracker_id, const cv::Mat &corners, mxArray* &plhs) {
	std::map<int, TrackerStruct>::iterator it = trackers.find(tracker_id);
	if(it == trackers.end()){
		printf("Invalid tracker ID: %d\n", tracker_id);
		return false;
	}
	try{
		it->second.setRegion(corners);
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while resetting the tracker: %s\n",
			err.type(), err.what());
		return false;
	}
	plhs = mtf::utils::setCorners(it->second.getRegion());
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

	auto cmd_iter = cmd_list.find(std::string(cmd_str));
	if(cmd_iter == cmd_list.end()){
		mexErrMsgTxt(cv::format("Invalid command provided: %s.", cmd_str).c_str());
	}
	const int cmd_id = cmd_iter->second;

	switch(cmd_id) {
	case MEX_INIT:
	{
		if(input){
			printf("Input pipeline has already been initialized");
			*ret_val = 1;
			return;
		}
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
			mwSize dims[2] = { 1, 1 };
			plhs[1] = mxCreateNumericArray(2, dims, mxUINT32_CLASS, mxREAL);
			unsigned int *n_frames = (unsigned int*)mxGetPr(plhs[1]);
			*n_frames = static_cast<unsigned int>(input->getNFrames());
		}
		return;
	}
	case MEX_IS_INITIALIZED:
	{
		if(input) {
			*ret_val = 1;
		} else {
			*ret_val = 0;
		}
		return;
	}
	case MEX_GET_FRAME:
	{
		if(nlhs != 2){
			mexErrMsgTxt("2 output arguments are needed to get frame from input pipeline.");
		}
		if(!getFrame(plhs[1])){
			*ret_val = 0;
			return;
		}
		*ret_val = 1;
		return;
	}
	case MEX_TRACKER_CREATE:
	{
		if(nrhs < 2){
			mexErrMsgTxt("At least 2 input arguments are needed to create tracker.");
		}
		if(!mxIsChar(prhs[1])){
			mexErrMsgTxt("Second input argument for creating tracker must be a string.");
		}
		if(!readParams(mtf::utils::toString(prhs[1]))) {
			mexErrMsgTxt("Parameters could not be parsed");
		}
		*ret_val = 0;

		if(!updateInput()){
			return;
		}
		cv::Mat init_corners;
		if(nrhs > 2){
			init_corners = mtf::utils::getCorners(prhs[2]);
		} else{
			mtf::utils::ObjUtils obj_utils;
			try{
				if(mex_live_init){
					if(!obj_utils.selectObjects(input.get(), 1,
						patch_size, line_thickness, write_objs, sel_quad_obj,
						write_obj_fname.c_str())){
						mexErrMsgTxt("Object(s) to be tracked could not be obtained.\n");
					}
				} else{
					if(!obj_utils.selectObjects(input->getFrame(), 1, patch_size, line_thickness,
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
		TrackerStruct tracker;
		if(!tracker.create(input->getFrame(), init_corners)){
			return;
		}
		++_tracker_id;
		trackers.insert(std::pair<int, TrackerStruct>(_tracker_id, tracker));
		*ret_val = _tracker_id;
		return;
	}
	case MEX_GET_REGION:
	{
		if(nrhs < 2){
			mexErrMsgTxt("At least 2 input arguments are needed to get tracker region.");
		}
		if(nlhs != 2){
			mexErrMsgTxt("2 output arguments are needed to get tracker region.");
		}
		if(!updateInput()){
			return;
		}
		*ret_val = 0;
		unsigned int tracker_id = mtf::utils::getID(prhs[1]);
		std::map<int, TrackerStruct>::iterator it = trackers.find(tracker_id);
		if(it == trackers.end()){
			printf("Invalid tracker ID: %d\n", tracker_id);
			return;
		}
		cv::Mat out_corners;
		if(!it->second.update(input->getFrame(), out_corners)){
			return;
		}
		plhs[1] = mtf::utils::setCorners(out_corners);
		*ret_val = 1;
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
	case MEX_TRACKERS_REMOVE:
	{
		printf("Removing all trackers...");
		trackers.clear();
		*ret_val = 1;
		printf("Done\n");
		return;
	}
	case MEX_QUIT:
	{
		printf("Clearing up...");
		input.reset();
		trackers.clear();
		*ret_val = 1;
		printf("Done\n");
		return;
	}
	default:
		mexErrMsgTxt(cv::format("Invalid command provided: %s.", cmd_str).c_str());
	}
}
