//! Matlab comes with its own horrible version of boost libraries that cause the mex module to segfault if boost::filesystem::absolute is used
//! It is also apparently impossible to force Matlab to load the correct version of boost libraries through LD_PRELOAD; attempting to do so causes Matlab itself to segfault at startup
//#define DISABLE_BOOST_ABSOLUTE

#include "mtf/TrackerStrct_mt.h"
#include "mtf/Utilities/mexUtils.h"

#include <map>
#include <memory>

#define MEX_INIT_INPUT 1
#define MEX_GET_FRAME 2
#define MEX_QUIT 3
#define MEX_CREATE_TRACKER 4
#define MEX_GET_REGION 5
#define MEX_SET_REGION 6
#define MEX_REMOVE_TRACKER 7
#define MEX_REMOVE_TRACKERS 8
#define MEX_IS_INITIALIZED 9

using namespace std;

static std::map<std::string, const int> cmd_list = {
	{ "init", MEX_INIT_INPUT },
	{ "is_initialized", MEX_IS_INITIALIZED },
	{ "quit", MEX_QUIT },
	{ "get_frame", MEX_GET_FRAME },
	{ "create_tracker", MEX_CREATE_TRACKER },
	{ "get_region", MEX_GET_REGION },
	{ "set_region", MEX_SET_REGION },
	{ "remove_tracker", MEX_REMOVE_TRACKER },
	{ "remove_trackers", MEX_REMOVE_TRACKERS }
};

static InputStructPtr input;
static std::map<int, TrackerStruct> trackers;

static unsigned int _tracker_id = 0;

bool createInput() {
	if(input_buffer_size <= 1){
		input_buffer_size = 100;
	}
	Input _input;
	try{
		_input.reset(mtf::getInput(pipeline));
		if(!_input->initialize()){
			printf("Pipeline could not be initialized successfully\n");
			return false;
		}
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while initializing the input pipeline: %s\n",
			err.type(), err.what());
		return false;
	}
	if(input) {
		input->reset(_input);
	} else {
		input.reset(new InputStruct(_input));
	}

	return true;
}
bool checkInput(bool verbose=true) {
	if(!input){
		if(verbose){
			printf("Input pipeline has not been initialized\n");
		}
		return false;
	}
	if(!input->isValid()) {
		if(verbose){
			printf("Input pipeline is not active\n");
		}
		return false;
	}
	return true;
}
bool getFrame(mxArray* &plhs){
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


bool initializeTracker(Tracker &tracker, PreProc &pre_proc,
	const cv::Mat &init_img, const cv::Mat &init_corners) {
	try{
		pre_proc->initialize(init_img);
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while initializing the pre processor: %s\n",
			err.type(), err.what());
		return false;
	}
	try{
		for(PreProc curr_obj = pre_proc; curr_obj; curr_obj = curr_obj->next){
			tracker->setImage(curr_obj->getFrame());
		}
		double min_x = init_corners.at<double>(0, 0);
		double min_y = init_corners.at<double>(1, 0);
		double max_x = init_corners.at<double>(0, 2);
		double max_y = init_corners.at<double>(1, 2);
		double size_x = max_x - min_x;
		double size_y = max_y - min_y;
		printf("Initializing tracker with object of size %f x %f...\n", size_x, size_y);

		tracker->initialize(init_corners);
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while initializing the tracker: %s\n",
			err.type(), err.what());
		return false;
	}
	printf("Initialization successful.\n");
	return true;
}
bool createTracker(const cv::Mat &init_corners) {
	if(!input->isValid()){
		printf("Input has not been initialized\n");
		return false;
	}
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
	if(!initializeTracker(tracker, pre_proc, input->getFrame(), init_corners)){
		printf("Tracker initialization failed");
		return false;
	}
	++_tracker_id;
	trackers.insert(std::pair<int, TrackerStruct>(_tracker_id,
		TrackerStruct(tracker, pre_proc, input, _tracker_id)));
	return true;
}

bool setRegion(unsigned int tracker_id, const cv::Mat &corners) {
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
	return true;
}
bool getRegion(unsigned int tracker_id, mxArray* &plhs) {
	std::map<int, TrackerStruct>::iterator it = trackers.find(tracker_id);
	if(it == trackers.end()){
		printf("Invalid tracker ID: %d\n", tracker_id);
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

	//printf("cmd_str: %s\n", cmd_str);
	//int k;
	//scanf("Press any key to continue: %d", &k);

	auto cmd_iter = cmd_list.find(std::string(cmd_str));
	if(cmd_iter == cmd_list.end()){
		mexErrMsgTxt(cv::format("Invalid command provided: %s.", cmd_str).c_str());
	}
	const int cmd_id = cmd_iter->second;

	switch(cmd_id) {
	case MEX_INIT_INPUT:
	{
		if(checkInput(false)) {
			printf("Input pipeline has already been initialized\n");
			*ret_val = 1;
			return;
		}
		if(nrhs < 2){
			mexErrMsgTxt("At least two input arguments are needed to initialize input pipeline.");
		}
		if(!mxIsChar(prhs[1])){
			mexErrMsgTxt("Second input argument for creating input pipeline must be a string.");
		}
		if(!readParams(mtf::utils::toString(prhs[1]))) {
			mexErrMsgTxt("Parameters could not be parsed");
		}
		if(!createInput()){
			*ret_val = 0;
			return;
		}
		if(nlhs >= 2){
			mwSize _dims[2] = { 1, 1 };
			plhs[1] = mxCreateNumericArray(2, _dims, mxUINT32_CLASS, mxREAL);
			unsigned int *n_frames = (unsigned int*)mxGetPr(plhs[1]);
			*n_frames = static_cast<unsigned int>(input->getNFrames());
		}
		*ret_val = 1;
		return;
	}
	case MEX_IS_INITIALIZED:
	{
		if(checkInput(false)) {
			*ret_val = 1;
		} else {
			*ret_val = 0;
		}
		return;
	}
	case MEX_GET_FRAME:
	{
		if(nlhs != 2){
			mexErrMsgTxt("2 output arguments (success, frame) are needed to get frame.");
		}
		*ret_val = 0;
		if(!checkInput()){ return; }
		if(!getFrame(plhs[1])){ return; }
		*ret_val = 1;
		return;
	}
	case MEX_CREATE_TRACKER:
	{	
		if(nrhs < 2){
			mexErrMsgTxt("At least 2 input arguments are needed to create tracker.");
		}
		if(!mxIsChar(prhs[1])){
			mexErrMsgTxt("The second input argument for creating tracker must be a string.");
		}
		*ret_val = 0;

		if(!checkInput()){ return; }

		if(!readParams(mtf::utils::toString(prhs[1]))) {
			mexErrMsgTxt("Parameters could not be parsed");
		}
		cv::Mat init_corners;
		if(nrhs > 2) {
			init_corners = mtf::utils::getCorners(prhs[2]);
		} else {
			init_corners.create(2, 4, CV_64FC1);
			ObjectSelectorThread obj_sel_thread(input, init_corners);
			boost::thread t = boost::thread{ boost::ref(obj_sel_thread) };
			try{
				t.join();
			} catch(boost::thread_interrupted) {
				printf("Caught exception from object selector thread\n");
			}
			if(!obj_sel_thread.success) {
				//cout << "init_corners:\n" << init_corners << "\n";
				//cout << "obj_sel_thread.corners:\n" << obj_sel_thread.getCorners() << "\n";
				mexErrMsgTxt("Initial corners could not be obtained\n");
			}
		}

		if(!createTracker(init_corners)) {
			mexErrMsgTxt("Tracker creation was unsuccessful corners\n");
		}
		if(nlhs > 1 && !getRegion(_tracker_id, plhs[1])) {
			mexErrMsgTxt("Tracker region could not be obtained\n");
		}

		*ret_val = _tracker_id;
		return;
	}
	case MEX_GET_REGION:
	{
		if(nlhs != 2){
			mexErrMsgTxt("2 output arguments [success, region] are needed to get tracker region.");
		}
		if(nrhs < 2){
			mexErrMsgTxt("At least 2 input arguments are needed to get tracker region.");
		}
		*ret_val = 0;

		unsigned int tracker_id = mtf::utils::getID(prhs[1]);
		if(!getRegion(tracker_id, plhs[1])){ return; }
		*ret_val = 1;
		return;
	}
	case MEX_SET_REGION:
	{
		if(nrhs < 2){
			mexErrMsgTxt("At least 2 input arguments are needed to reset tracker.");
		}
		*ret_val = 0;

		unsigned int tracker_id = mtf::utils::getID(prhs[1]);
		cv::Mat curr_corners = mtf::utils::getCorners(prhs[2]);
		if(!setRegion(tracker_id, curr_corners)){ return; }

		if(nlhs > 1 && !getRegion(_tracker_id, plhs[1])){ return; }

		*ret_val = 1;
		return;
	}
	case MEX_REMOVE_TRACKER:
	{
		if(nrhs < 2){
			mexErrMsgTxt("At least 2 input arguments are needed to remove tracker.");
		}
		unsigned int tracker_id = mtf::utils::getID(prhs[1]);
		std::map<int, TrackerStruct>::iterator it = trackers.find(tracker_id);
		if(it == trackers.end()){
			printf("Invalid tracker ID: %d\n", tracker_id);
			*ret_val = 0;
			return;
		}
		it->second.reset();
		trackers.erase(it);
		*ret_val = 1;
		return;
	}
	case MEX_REMOVE_TRACKERS:
	{
		printf("Removing all trackers...");
		for(auto it = trackers.begin(); it != trackers.end(); ++it){
			it->second.reset();
		}
		trackers.clear();
		*ret_val = 1;
		printf("Done\n");
		return;
	}
	case MEX_QUIT:
	{
		printf("Clearing up...");
		input->reset();
		for(auto it = trackers.begin(); it != trackers.end(); ++it){
			it->second.reset();
		}
		trackers.clear();
		*ret_val = 1;
		printf("Done\n");
		return;
	}
	default:
		mexErrMsgTxt(cv::format("Invalid command provided: %s.", cmd_str).c_str());
	}
}
