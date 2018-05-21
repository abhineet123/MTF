#include "mtf/mtf.h"
#include "mtf/pipeline.h"
#include "mtf/Config/parameters.h"

typedef std::shared_ptr<mtf::TrackerBase> Tracker;
typedef PreProc_ PreProc;

using namespace mtf::params;

struct TrackerStruct{
	TrackerStruct() : tracker(nullptr), pre_proc(nullptr){}
	TrackerStruct(Tracker &_tracker, PreProc &_pre_proc) :
		tracker(_tracker), pre_proc(_pre_proc){
	}
	bool create(const cv::Mat init_frame, const cv::Mat init_corners) {
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
		try{
			pre_proc = mtf::getPreProc(tracker->inputType(), pre_proc_type);
		} catch(const mtf::utils::Exception &err){
			printf("Exception of type %s encountered while creating the pre processor: %s\n",
				err.type(), err.what());
			return false;
		}
		try{
			pre_proc->initialize(init_frame);
		} catch(const mtf::utils::Exception &err){
			printf("Exception of type %s encountered while initializing the pre processor: %s\n",
				err.type(), err.what());
			return false;
		}
		double min_x = init_corners.at<double>(0, 0);
		double min_y = init_corners.at<double>(1, 0);
		double max_x = init_corners.at<double>(0, 2);
		double max_y = init_corners.at<double>(1, 2);
		double size_x = max_x - min_x;
		double size_y = max_y - min_y;
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
		return true;
	}
	bool update(const cv::Mat &curr_img, cv::Mat &out_corners) {
		if(!pre_proc || !tracker){
			printf("Tracker has not been created");
			return false;
		}
		double fps = 0, fps_win = 0;
		double tracking_time, tracking_time_with_input;
		mtf_clock_get(start_time_with_input);
		try{
			//! update pre-processor
			pre_proc->update(curr_img);
			mtf_clock_get(start_time);
			//! update tracker
			tracker->update();
			if(print_fps){
				mtf_clock_get(end_time);
				mtf_clock_measure(start_time, end_time, tracking_time);
				mtf_clock_measure(start_time_with_input, end_time, tracking_time_with_input);
				fps = 1.0 / tracking_time;
				fps_win = 1.0 / tracking_time_with_input;
				printf("fps: %f\t fps_win=%f\n", fps, fps_win);
			}
			if(reset_template){
				tracker->initialize(tracker->getRegion());
			}
		} catch(const mtf::utils::Exception &err){
			printf("Exception of type %s encountered while updating the tracker: %s\n",
				err.type(), err.what());
			return false;
		}	
		out_corners = tracker->getRegion();
		return true;
	}

	void setRegion(const cv::Mat& corners){
		if(!pre_proc || !tracker){
			printf("Tracker has not been created");
			return;
		}
		tracker->setRegion(corners);
	}
	const cv::Mat& getRegion() {
		return tracker->getRegion();
	}
	void reset() {
		tracker.reset();
	}
	std::string name() {
		return tracker->name;
	}
private:
	Tracker tracker;
	PreProc pre_proc;
};