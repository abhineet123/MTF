#include <boost/thread.hpp>

#include "mtf/mtf.h"
#include "mtf/pipeline.h"
#include "mtf/Utilities/miscUtils.h"
#ifndef DISABLE_VISP
#include <visp3/core/vpImagePoint.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayD3D.h>
#endif


using namespace mtf::params;

struct InputStruct;
typedef std::shared_ptr<mtf::utils::InputBase> Input;
typedef std::shared_ptr<mtf::TrackerBase> Tracker;
typedef std::shared_ptr<InputStruct> InputStructPtr;
typedef PreProc_ PreProc;


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
	ObjectSelectorThread(InputStructPtr &_input, cv::Mat &_corners, bool _live_init) : success(false),
		input(_input), corners(_corners), live_init(_live_init){}
	void operator()(){
		mtf::utils::ObjUtils obj_utils;
#ifndef DISABLE_VISP
		try{
			if(live_init){
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
			if(live_init){
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
	bool live_init;
};

struct TrackerThread{
	TrackerThread(Tracker &_tracker, PreProc &_pre_proc, const InputStructPtr &_input,
		unsigned int id, int _visualize, const char* module_name) : input(_input), pre_proc(_pre_proc),
		tracker(_tracker), visualize(_visualize){
		win_name = cv::format("%s :: %d", module_name, id);
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
		unsigned int id, bool _visualize, const char* module_name) :
		tracker(_tracker), pre_proc(_pre_proc), visualize(_visualize){
		t = boost::thread{ TrackerThread(tracker, pre_proc, _input, id, 
			visualize, module_name) };
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
	bool visualize;
};

