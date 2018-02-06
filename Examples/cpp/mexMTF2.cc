//! Matlab comes with its own horrible version of boost libraries that cause the mex module to segfault if boost::filesystem::absolute is used
//! It is also apparently impossible to force Matlab to load the correct version of boost libraries through LD_PRELOAD; attempting to do so causes Matlab itself to segfault at startup
#define DISABLE_BOOST_ABSOLUTE

#include "mtf/mtf.h"
#include "mtf/pipeline.h"
#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/mexUtils.h"
#ifndef DISABLE_VISP
#include <visp3/core/vpImagePoint.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayD3D.h>
#endif

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
typedef std::shared_ptr<InputStruct> InputStructPtr;
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
	InputStruct(Input &_input) : input(_input),
		is_valid(false), thread_created(false){
		t = boost::thread{ InputThread(input) };
		is_valid = thread_created = true;
	}
	~InputStruct() {}
	bool initialize() override{ return true; }
	bool update() override{ return true; }
	void remapBuffer(unsigned char** new_addr) override{}
	const cv::Mat& getFrame() const override{ return input->getFrame(); }
	cv::Mat& getFrame(mtf::utils::FrameType) override{
		throw mtf::utils::InvalidArgument("Mutable frame cannot be obtained");
	}
#ifndef DISABLE_VISP
	void getFrame(vpImage<vpRGBa> &vp_img) const override{
		input->getFrame(vp_img);
	}
#endif
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
	bool isValid() const {
		return is_valid && thread_created && !input->destroy;
	}
private:
	Input input;
	boost::thread t;
	bool is_valid, thread_created;
};

struct ObjectSelectorThread{
	ObjectSelectorThread(InputStructPtr &_input, cv::Mat &_corners) : success(false), 
		input(_input), corners(_corners) {}
	void operator()(){
		mtf::utils::ObjUtils obj_utils;
#ifndef DISABLE_VISP
		try{
			if(mex_live_init){
				if(!obj_utils.selectObjectsVP(input.get(), 1,
					patch_size, line_thickness, write_objs, sel_quad_obj,
					write_obj_fname.c_str())){
					cout << "Object to be tracked could not be obtained.\n";
					return;
				}
			} else {
				if(!obj_utils.selectObjectsVP(input->getFrame(), 1,
					patch_size, line_thickness, write_objs, sel_quad_obj,
					write_obj_fname.c_str())){
					cout << "Object to be tracked could not be obtained.\n";
					return;
				}
			}
			//if(!obj_utils.addRectObjectVP(input.get(), "Select an object",
			//	line_thickness, patch_size)){
			//	cout << "Object to be tracked could not be obtained.\n";
			//}
		} catch(const mtf::utils::Exception &err){
			cout << cv::format("Exception of type %s encountered while obtaining the object to track: %s\n",
				err.type(), err.what());
			return;
		}
#else
		try{
			if(mex_live_init){
				if(!obj_utils.selectObjects(input.get(), 1,
					patch_size, line_thickness, write_objs, sel_quad_obj,
					write_obj_fname.c_str())){
					cout << "Object to be tracked could not be obtained.\n";
				}
			} else {
				if(!obj_utils.selectObjects(input->getFrame(), 1,
					patch_size, line_thickness, write_objs, sel_quad_obj,
					write_obj_fname.c_str())){
					cout << "Object to be tracked could not be obtained.\n";
				}
			}
		} catch(const mtf::utils::Exception &err){
			cout << cv::format("Exception of type %s encountered while obtaining the object to track: %s\n",
				err.type(), err.what());
		}
#endif
		obj_utils.getObj().corners.copyTo(corners);
		success = true;
		//cout << "ObjectSelectorThread :: corners: " << corners << "\n";
	}
	const cv::Mat& getCorners() const{ return corners; }
	bool success;
private:
	InputStructPtr input;
	cv::Mat corners;
};

struct TrackerThread{
	TrackerThread(Tracker &_tracker, PreProc &_pre_proc, const InputStructPtr &_input,
		unsigned int id = 1, int _visualize = 0) : input(_input), pre_proc(_pre_proc),
		tracker(_tracker), visualize(_visualize){
		win_name = cv::format("mexMTF:: %d", id);
		proc_win_name = cv::format("%s (Pre-processed)", win_name.c_str());
		//if(visualize) {
		//	cout << "Visualization is enabled for tracker " << id << "\n";
		//}
		
	}
	void operator()(){
		int frame_id = 0;

		if(visualize) {
#ifndef DISABLE_VISP			
			// Select one of the available video-devices
#if defined VISP_HAVE_X11
			display.reset(new vpDisplayX);
#elif defined VISP_HAVE_GTK
			display.reset(new vpDisplayGTK);
#elif defined VISP_HAVE_GDI
			display.reset(new vpDisplayGDI);
#elif defined VISP_HAVE_D3D9
			display.reset(new vpDisplayD3D);
#elif defined VISP_HAVE_OPENCV
			display.reset(new vpDisplayOpenCV);
#else
			throw InvalidArgument("None of the window backends supported by ViSP are available");
#endif 
			disp_frame.init(static_cast<int>(input->getHeight()),
				static_cast<int>(input->getWidth()));
			display->init(disp_frame, -1, -1, win_name);
#else
			cv::namedWindow(win_name, cv::WINDOW_AUTOSIZE);
			if(show_proc_img) {
				cv::namedWindow(proc_win_name, cv::WINDOW_AUTOSIZE);
			}
#endif
		}
		while(input->isValid()){
			if(frame_id == input->getFrameID()){
				//! do not process the same image multiple times
				//cout << "skipping frame " << frame_id << "\n";
				continue;
			}
			frame_id = input->getFrameID();
			//cout << "processing frame " << frame_id << "\n";
			try{
				//! update pre-processor
				pre_proc->update(input->getFrame());
				//! update tracker
				tracker->update();

				if(visualize) {
#ifndef DISABLE_VISP
					input->getFrame(disp_frame);
					mtf::utils::drawRegion(disp_frame, tracker->getRegion(), vpColor::red,
						line_thickness, tracker->name.c_str(), 0.50, show_corner_ids, 1 - show_corner_ids);
					vpDisplay::display(disp_frame);
					char pressed_key;
					if(vpDisplay::getKeyboardEvent(disp_frame, &pressed_key, false)) {
						if(pressed_key % 256 == 27){
							break;
						}
					}
#else
					cv::Mat disp_frame = input->getFrame().clone();
					mtf::utils::drawRegion(disp_frame, tracker->getRegion(), cv::Scalar(0, 0, 255),
						line_thickness, tracker->name.c_str(), 0.50, show_corner_ids, 1 - show_corner_ids);
					imshow(win_name, disp_frame);
					if(show_proc_img){
						pre_proc->showFrame(proc_win_name);
					}
					int pressed_key = cv::waitKey(1 - pause_after_frame);
					if(pressed_key % 256 == 27){
						break;
					}
#endif
				}
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
#ifndef DISABLE_VISP
		if(visualize) {
			vpDisplay::close(disp_frame);
		}
#endif
	}

private:
	std::shared_ptr<const InputStruct> input;
	PreProc pre_proc;
	Tracker tracker;
	string win_name, proc_win_name;
#ifndef DISABLE_VISP
	std::unique_ptr<vpDisplay> display;
	vpImage<vpRGBa> disp_frame;
#endif
	int visualize;
};
struct TrackerStruct{
	TrackerStruct(Tracker &_tracker, PreProc &_pre_proc, const InputStructPtr &_input,
		unsigned int id) :
		tracker(_tracker), pre_proc(_pre_proc){
		t = boost::thread{ TrackerThread(tracker, pre_proc, _input, id, mex_visualize) };
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
		input.reset(new InputStruct(_input));
	}

	return true;
}
bool checkInput() {
	if(!input){
		printf("Input pipeline has not been initialized\n");
		return false;
	}
	if(!input->isValid()) {
		printf("Input pipeline is not active\n");
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
				printf("Caught exception from object selector thread");
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
