#include <ros/ros.h>
#include <ros/package.h>
#include "mtf_bridge/SharedImageReader.h"
#include "mtf_bridge/PatchTrackers.h"

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// C++
#include <vector>
#include <math.h>
#include <assert.h>
#include <Eigen/Core>

// Modular Tracking Framework
#include "mtf/mtf.h"
#include "mtf/pipeline.h"
#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/objUtils.h"

using namespace mtf;
using namespace mtf::params;
using namespace mtf::utils;

typedef std::shared_ptr<mtf::TrackerBase> Tracker_;
std::vector<PreProc_> pre_procs;

int const rate = 30;

using namespace mtf::params;

struct TrackerStruct{
	TrackerStruct(Tracker_ &_tracker, PreProc_ &_pre_proc, int _id) :
		tracker(_tracker), pre_proc(_pre_proc), id(_id){
	}
	bool update(const cv::Mat &frame, int frame_id = -1) {
		if(!pre_proc || !tracker){
			printf("Tracker has not been created");
			return false;
		}
		try{
			//! update pre-processor
			pre_proc->update(frame, frame_id);
			//! update tracker
			tracker->update();
		} catch(const mtf::utils::Exception &err){
			printf("Exception of type %s encountered while updating the tracker: %s\n",
				err.type(), err.what());
			return false;
		}
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
	int getID() const{ return id; }
private:
	Tracker_ tracker;
	PreProc_ pre_proc;
	int id;
};


cv::Mat display_frame;
std::vector<cv::Point> new_tracker_points;
std::vector<TrackerStruct> trackers;
int tracker_id;
std::vector<cv::Scalar> colors;
int n_colors;

ros::Publisher tracker_pub;
SharedImageReader *image_reader;

struct Patch {
    Patch(std::vector<cv::Point> points) {
        assert(points.size() == 4);
        for(unsigned long i = 0; i < points.size(); ++i) {
            corners[i] = points[i];
        }
    }
    cv::Point2d operator[](int i) { return corners[i];}
    cv::Point2d corners[4];
};

cv::Point get_patch_center(const cv::Point2d (&cv_corners)[4]) {
	;
    Eigen::Vector3d tl(cv_corners[0].x, cv_corners[0].y, 1);
    Eigen::Vector3d tr(cv_corners[1].x, cv_corners[1].y, 1);
    Eigen::Vector3d br(cv_corners[2].x, cv_corners[2].y, 1);
    Eigen::Vector3d bl(cv_corners[3].x, cv_corners[3].y, 1);

    Eigen::Vector3d center_vec = center_vec = tl.cross(br).cross(tr.cross(bl));

    cv::Point center;
    center.x = center_vec(0) / center_vec(2);
    center.y = center_vec(1) / center_vec(2);
    return center;
}

mtf_bridge::Patch get_tracker_patch(TrackerStruct &tracker) {
	cv::Point2d cv_corners[4];
	mtf::utils::Corners(tracker.getRegion()).points(cv_corners);
    mtf_bridge::Point top_left;
    top_left.x = cv_corners[0].x;
    top_left.y = cv_corners[0].y;

    mtf_bridge::Point top_right;
    top_right.x = cv_corners[1].x;
    top_right.y = cv_corners[1].y;

    mtf_bridge::Point bot_right;
    bot_right.x = cv_corners[2].x;
    bot_right.y = cv_corners[2].y;

    mtf_bridge::Point bot_left;
    bot_left.x = cv_corners[3].x;
    bot_left.y = cv_corners[3].y;

    mtf_bridge::Patch patch;
    patch.corners[0] = top_left;
    patch.corners[1] = top_right;
    patch.corners[2] = bot_right;
    patch.corners[3] = bot_left;

	cv::Point center_point = get_patch_center(cv_corners);

    mtf_bridge::Point center;
    center.x = center_point.x;
    center.y = center_point.y;
    patch.center = center;

    return patch;
}

void update_trackers() {
    if (trackers.empty()) {
        return;
    }
	//! opdate trackers
    mtf_bridge::PatchTrackers tracker_msg;
    for(std::vector<TrackerStruct>::iterator tracker = trackers.begin(); 
		tracker != trackers.end(); ++tracker) {
		(*tracker).update(*(image_reader->getFrame()), image_reader->getFrameID());
        mtf_bridge::Patch tracker_patch = get_tracker_patch(*tracker);
        tracker_msg.trackers.push_back(tracker_patch);
    }
    tracker_pub.publish(tracker_msg);
}

void draw_patch(const cv::Point2d* corners, cv::Scalar color) {
    line(display_frame, corners[0], corners[1], color);
    line(display_frame, corners[1], corners[2], color);
    line(display_frame, corners[2], corners[3], color);
    line(display_frame, corners[3], corners[0], color);
}

void draw_frame(std::string cv_window_title) {
    cv::cvtColor(*(image_reader->getFrame()), display_frame, cv::COLOR_RGB2BGR);

    // Draw trackers
    if (!trackers.empty()) {
		for(std::vector<TrackerStruct>::iterator tracker = trackers.begin();
			tracker != trackers.end(); ++tracker) {
			cv::Point2d cv_corners[4];
			mtf::utils::Corners((*tracker).getRegion()).points(cv_corners);
			cv::Scalar obj_col = colors[(*tracker).getID() % n_colors];
			draw_patch(cv_corners, obj_col);
			cv::Point center = get_patch_center(cv_corners);
			cv::circle(display_frame, center, 5, obj_col, -1);
            // Black outline
            cv::circle(display_frame, center, 5, cv::Scalar(0, 0, 0), 2);
        }
    }

    // Show construction of tracker
    for (std::vector<cv::Point>::const_iterator point = new_tracker_points.begin(); point != new_tracker_points.end(); ++point) {
        // White filled circle
        cv::circle(display_frame, *point, 5, cv::Scalar(255, 255, 255), -1);
        // Black outline
        cv::circle(display_frame, *point, 5, cv::Scalar(0, 0, 0), 2);
    }
    imshow(cv_window_title, display_frame);
    char key = cv::waitKey(10);

    if (key == 'd') {
        if (trackers.size() > 0) {
            trackers.erase(trackers.begin());
            pre_procs.erase(pre_procs.begin());
        }
    }
}

bool create(const cv::Mat init_frame, const cv::Mat init_corners, int frame_id) {
	Tracker_ tracker;
	PreProc_ pre_proc;
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
		pre_proc = mtf::getPreProc(pre_procs, tracker->inputType(), pre_proc_type);
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while creating the pre processor: %s\n",
			err.type(), err.what());
		return false;
	}
	try{
		pre_proc->initialize(init_frame, frame_id);
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while initializing the pre processor: %s\n",
			err.type(), err.what());
		return false;
	}
	try{
		for(PreProc_ curr_obj = pre_proc; curr_obj; curr_obj = curr_obj->next){
			tracker->setImage(curr_obj->getFrame());
		}
		tracker->initialize(init_corners);
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while initializing the tracker: %s\n",
			err.type(), err.what());
		return false;
	}

	trackers.push_back(TrackerStruct(tracker, pre_proc, tracker_id));
	pre_procs.push_back(pre_proc);
	++tracker_id;
	return true;
}

// Assume points are ordered: 1 ---- 2
//                            |      |
//                            4 ---- 3
void initialize_tracker() {
	if(!readParams(0, nullptr)){ return; }

    cv::Mat cv_corners(2, 4, CV_64FC1);
	for(unsigned int i = 0; i < 4; ++i) {
		cv_corners.at<double>(0, i) = new_tracker_points[i].x;
		cv_corners.at<double>(1, i) = new_tracker_points[i].y;
	}	
	if(!create(*(image_reader->getFrame()), cv_corners, image_reader->getFrameID())) {
		ROS_INFO_STREAM("Tracker could not be created");
	}
    ROS_INFO_STREAM("Tracker initialized");
    draw_frame("TrackingNode");
}

void mouse_cb(int mouse_event, int x, int y, int flags, void* param) {
    // Right mouse click restarts setting of tracker points
    if (mouse_event == CV_EVENT_RBUTTONUP) {
        new_tracker_points.clear();
        return;
    }

    if (mouse_event == CV_EVENT_LBUTTONUP) {
        ROS_DEBUG_STREAM("Click at x: " << x << ", " << y);
        new_tracker_points.push_back(cv::Point(x,y));
        if (new_tracker_points.size() == 4) {
            initialize_tracker();
            new_tracker_points.clear();
        }
        return;
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "trackingNode");
    ros::NodeHandle nh_("~");

    // Initialize input_obj
	config_dir = ros::package::getPath("mtf_bridge") + "/cfg";
	if(!readParams(argc, argv)){ return EXIT_FAILURE; }

	ObjUtils(obj_cols).getCols(colors);
	n_colors = colors.size();

    image_reader = new SharedImageReader();
	tracker_id = 0;

    // Initialize OpenCV window and mouse callback
    std::string cv_window_title = "TrackingNode";
	cv::namedWindow(cv_window_title, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(cv_window_title, mouse_cb);

    tracker_pub = nh_.advertise<mtf_bridge::PatchTrackers>("patch_tracker", 1);

	ros::Rate loop_rate(rate);

    while(!image_reader->isInitialized()) {
        ROS_INFO_STREAM("Waiting while system initializes");
        ros::spinOnce();
        ros::Duration(0.7).sleep();
    }

    ROS_INFO_STREAM("Left click to select tracker corners. Right click to reset corners, press 'd' to delete tracker");

	while(ros::ok()){
		ros::spinOnce();
        update_trackers();
        draw_frame(cv_window_title);
		loop_rate.sleep();
    }
    return 0;
}
