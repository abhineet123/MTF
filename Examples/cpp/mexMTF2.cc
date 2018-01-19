#include "mtf/mtf.h"
#include "mtf/pipeline.h"
#include "mtf/Utilities/mexUtils.h"

#include <vector>
#include <map>
#include <memory>
#include <boost/thread.hpp>

#define MEX_INIT_INPUT 1
#define MEX_GET_FRAME 2
#define MEX_QUIT 3
#define MEX_CREATE_TRACKER 4
#define MEX_GET_REGION 5
#define MEX_SET_REGION 6
#define MEX_REMOVE_TRACKER 7

using namespace std;
using namespace mtf::params;

struct InputStruct;
typedef std::shared_ptr<mtf::utils::InputBase> Input;
typedef std::shared_ptr<mtf::TrackerBase> Tracker;
typedef std::shared_ptr<InputStruct> InputConstPtr;
typedef PreProc_ PreProc;

static std::map<std::string, const int> cmd_list = {
	{ "init", MEX_INIT_INPUT },
	{ "quit", MEX_QUIT },
	{ "get_frame", MEX_GET_FRAME },
	{ "create_tracker", MEX_CREATE_TRACKER },
	{ "get_region", MEX_GET_REGION },
	{ "set_region", MEX_SET_REGION },
	{ "remove_tracker", MEX_REMOVE_TRACKER }
};

struct InputThread{
	InputThread(Input &_input) : input(_input){}
	void operator()(){
		while(input->update()){
			if(input->destroy){ return; }
			boost::this_thread::interruption_point();
		}
		input->destroy = true;
	}
private:
	Input input;
};
struct InputStruct : public mtf::utils::InputBase {
	InputStruct() : is_valid(false), thread_created(false){}
	InputStruct(Input &_input) : input(_input), thread_created(false){
		t = boost::thread{ InputThread(input) };
		thread_created = true;
	}
	~InputStruct() {}
	bool initialize() override{ return true; }
	bool update() override{ return true; }
	void remapBuffer(unsigned char** new_addr) override{}
	const cv::Mat& getFrame() const override{ return input->getFrame(); }
	cv::Mat& getFrame(mtf::utils::FrameType) override{ 
		throw mtf::utils::InvalidArgument("Mutable frame cannot be obtained");
	}	
	int getFrameID() const override{ return input->getFrameID(); }
	int getHeight() const override{ return input->getHeight(); }
	int getWidth() const override{ return input->getWidth(); }	

	void reset(Input &_input) {
		reset();
		input = _input;
		t = boost::thread{ InputThread(input) };
		is_valid = thread_created = true;
	}
	void reset() {
		if(thread_created) {
			t.interrupt();
			thread_created = false;
		}
		if(is_valid) {
			input.reset();
			is_valid = false;
		}		
	}
	bool isValid() { return is_valid; }
private:
	Input input;
	boost::thread t;
	bool is_valid, thread_created;
};
struct TrackerThread{
	TrackerThread(Tracker &_tracker, PreProc &_pre_proc, const InputConstPtr &_input) :
		tracker(_tracker), pre_proc(_pre_proc), input(_input){}
	void operator()(){
		int frame_id = 0;
		while(!input->destroy){
			if(frame_id == input->getFrameID()){
				//! do not process the same image multiple times
				continue;
			}
			frame_id = input->getFrameID();
			try{
				//! update pre-processor
				pre_proc->update(input->getFrame());
				//! update tracker
				tracker->update();
				if(reset_template){
					tracker->initialize(tracker->getRegion());
				}
			} catch(const mtf::utils::Exception &err){
				cout << cv::format("Exception of type %s encountered while updating the tracker: %s\n",
					err.type(), err.what());
				return;
			}
			boost::this_thread::interruption_point();
		}
	}

private:
	std::shared_ptr<const InputStruct> input;
	PreProc pre_proc;
	Tracker tracker;
};
struct TrackerConst{
	TrackerConst(Tracker &_tracker, PreProc &_pre_proc, const InputConstPtr &_input) :
		tracker(_tracker), pre_proc(_pre_proc){
		t = boost::thread{ TrackerThread(tracker, pre_proc, _input) };
	}
	void setRegion(const cv::Mat& corners){
		tracker->setRegion(corners);		
	}
	const cv::Mat& getRegion() {
		return tracker->getRegion();
	}
	void reset() {
		t.interrupt();
		tracker.reset();
	}
private:
	Tracker tracker;
	PreProc pre_proc;
	boost::thread t;
};

InputConstPtr input;
static std::map<int, TrackerConst> trackers;

static double min_x, min_y, max_x, max_y;
static double size_x, size_y;

static int img_height, img_width;
static vector<cv::Scalar> obj_cols;

static int frame_id;
static unsigned int _tracker_id = 0;

bool createInput() {
	if(input_buffer_size <= 1){
		input_buffer_size = 100;
	}
	Input _input;
	try{
		_input.reset(mtf::getInput(pipeline));
		if(!_input->initialize()){
			printf("Pipeline could not be initialized successfully. Exiting...\n");
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
		input.reset(_input);
	}
		
	return true;
}

bool getFrame(mxArray* &plhs){
	if(!input->isValid()){
		printf("Input has not been initialized\n");
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
		for(PreProc curr_obj = pre_proc; curr_obj; curr_obj = curr_obj->next){
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
	plhs = mtf::utils::setCorners(tracker->getRegion());
	return true;
}
bool createTracker(mxArray* &plhs) {
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
	mtf::utils::ObjUtils obj_utils;
	try{
		if(mex_live_init){
			if(!obj_utils.selectObjects(input.get(), 1,
				patch_size, line_thickness, write_objs, sel_quad_obj,
				write_obj_fname.c_str())){
				mexErrMsgTxt("Object to be tracked could not be obtained.\n");
			}
		} else {
			if(!obj_utils.selectObjects(input->getFrame(), 1,
				patch_size, line_thickness, write_objs, sel_quad_obj,
				write_obj_fname.c_str())){
				mexErrMsgTxt("Object to be tracked could not be obtained.\n");
			}
		}
	} catch(const mtf::utils::Exception &err){
		mexErrMsgTxt(cv::format("Exception of type %s encountered while obtaining the object to track: %s\n",
			err.type(), err.what()).c_str());
	}
	cv::Mat init_corners = obj_utils.getObj().corners.clone();
	if(!initializeTracker(tracker, pre_proc, input->getFrame(), init_corners, plhs)){
		mexErrMsgTxt("Tracker initialization failed");
	}
	++_tracker_id;
	trackers.insert(std::pair<int, TrackerConst>(_tracker_id, 
		TrackerConst(tracker, pre_proc, input)));
	return true;
}

bool setRegion(unsigned int tracker_id, const cv::Mat &corners, mxArray* &plhs) {
	std::map<int, TrackerConst>::iterator it = trackers.find(tracker_id);
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
bool getRegion(unsigned int tracker_id, mxArray* &plhs) {
	std::map<int, TrackerConst>::iterator it = trackers.find(tracker_id);
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
			mwSize dims[2] = { 1, 1 };
			plhs[1] = mxCreateNumericArray(2, dims, mxUINT32_CLASS, mxREAL);
			unsigned int *n_frames = (unsigned int*)mxGetPr(plhs[1]);
			*n_frames = static_cast<unsigned int>(input->getNFrames());
		}
		*ret_val = 1;
		return;
	}
	case MEX_GET_FRAME:
	{
		if(nlhs != 2){
			mexErrMsgTxt("2 output arguments (success, frame) are needed to get frame.");
		}
		if(!getFrame(plhs[1])){
			*ret_val = 0;
			return;
		}
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
		if(!mxIsChar(prhs[1])){
			mexErrMsgTxt("The optional third input argument for creating tracker must be a string.");
		}
		if(!readParams(mtf::utils::toString(prhs[1]))) {
			mexErrMsgTxt("Parameters could not be parsed");
		}
		if(!createTracker(plhs[1])){
			*ret_val = 0;
			return;
		}
		*ret_val = _tracker_id;
		return;
	}
	case MEX_GET_REGION:
	{
		if(nlhs != 2){
			mexErrMsgTxt("2 output arguments (tracker_id, init_region) are needed to create tracker.");
		}
		if(nrhs < 2){
			mexErrMsgTxt("At least 2 input arguments are needed to create tracker.");
		}
		unsigned int tracker_id = mtf::utils::getID(prhs[1]);
		if(!getRegion(tracker_id, plhs[1])){
			*ret_val = 0;
			return;
		}
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
	case MEX_REMOVE_TRACKER:
	{
		if(nrhs < 2){
			mexErrMsgTxt("At least 2 input arguments are needed to reset tracker.");
		}
		if(nlhs != 1){
			mexErrMsgTxt("1 output argument is needed to reset tracker.");
		}
		unsigned int tracker_id = mtf::utils::getID(prhs[1]);
		std::map<int, TrackerConst>::iterator it = trackers.find(tracker_id);
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
