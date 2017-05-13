#include "mtf/mtf.h"

// tools for reading in images from various sources like image sequences, 
// videos and cameras as well as for pre processing them
#include "mtf/Tools/pipeline.h"
// general OpenCV tools for selecting objects, reading ground truth, etc.
#include "mtf/Tools/cvUtils.h"

#include "mtf/Config/parameters.h"
#include "mtf/Config/datasets.h"

#include "mtf/Utilities/miscUtils.h"

#include <string.h>
#include <vector>

#include <fstream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"

#define MAX_FPS 1e6

using namespace Eigen;

using namespace std;
using namespace mtf::params;
namespace fs = boost::filesystem;

typedef unique_ptr<InputBase> Input_;
typedef unique_ptr<mtf::nt::SearchMethod> SM;

int main(int argc, char * argv[]) {

	if(!readParams(argc, argv)){ return EXIT_FAILURE; }

#ifdef ENABLE_PARALLEL
	Eigen::initParallel();
#endif

	actor = "Misc";
	seq_name = "uav_sim";
	seq_path = db_root_path + "/" + actor;

	printf("*******************************\n");
	printf("Using parameters:\n");
	printf("n_trackers: %d\n", n_trackers);
	printf("actor_id: %d\n", actor_id);
	printf("source_id: %d\n", seq_id);
	printf("source_name: %s\n", seq_name.c_str());
	printf("actor: %s\n", actor.c_str());
	printf("pipeline: %c\n", pipeline);
	printf("img_source: %c\n", img_source);
	printf("show_cv_window: %d\n", show_cv_window);
	printf("mtf_sm: %s\n", mtf_sm);
	printf("mtf_am: %s\n", mtf_am);
	printf("mtf_ssm: %s\n", mtf_ssm);
	printf("********************************\n");

	/* initialize pipeline*/
	Input_ input(getInput(pipeline));
	if(!input->initialize()){
		printf("Pipeline could not be initialized successfully\n");
		return EXIT_FAILURE;
	}
	cv::Mat satellite_img = cv::imread(cv::format("%s/%s/YJ_map_right.bmp", 
		seq_path.c_str(), seq_name.c_str()));
	/**
	leave one pixel border arund the bounding box
	so numerical image derivatives can be computed more easily
	*/
	cv::Mat uav_img_corners = mtf::utils::getFrameCorners(input->getFrame(), 1);


	/*********************************** initialize tracker ***********************************/
	cv::Mat init_location = mtf::utils::readTrackerLocation(std::string(cv::format(
		"%s/%s/initial_location.txt", seq_path.c_str(), seq_name.c_str()).c_str())
		);

	cout << "init_location:\n" << init_location << "\n";

	//+++++++++++++++++++initial the center point of init_location+++++++++++++++++++
	Matrix2Xd init_pt(2, 1);
	init_pt << 1444, 612;

	hom_normalized_init = 0;
	//+++++++++++++++++++read ground truth key points++++++++++++++++++++++++++++++++

	printf("+++++++++++++++read ground truth trajectory key points+++++++++++++\n");

	string gt_kpt_flnm = string(seq_path) + "/" + string(seq_name) + "/uav_trajectory_groundtruth.txt";
	ifstream gt_kpt(gt_kpt_flnm);
	vector<cv::Point2d> gt_kpt_corners;
	cv::Point2d tmp;
	unsigned int i = 0;
	while(gt_kpt >> tmp.x >> tmp.y) {
		gt_kpt_corners.push_back(tmp);
		cout << gt_kpt_corners.at(i).x << " " << gt_kpt_corners.at(i).y << endl;
		++i;
	}
	//printf("The number of ground truth trajectory key points: %s", gt_kpt_corners.size());
	printf("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	SM uav_tracker(mtf::getSM(mtf_sm, mtf_am, mtf_ssm, mtf_ilm));
	if(!uav_tracker){
		printf("UAV tracker could not be initialized successfully\n");
		return EXIT_FAILURE;
	}

	/* initialize frame pre processors */
	GaussianSmoothing uav_pre_proc(uav_tracker->inputType()), satellite_pre_proc(uav_tracker->inputType());
	satellite_pre_proc.initialize(satellite_img);
	uav_tracker->initialize(satellite_pre_proc.getFrame(), init_location);

	uav_pre_proc.initialize(input->getFrame());

	string satellite_win_name = "Satellite Image";
	if(show_cv_window) {
		cv::namedWindow(satellite_win_name, cv::WINDOW_AUTOSIZE);
	}

	double fps, fps_win;
	double tracking_time, tracking_time_with_input;
	double avg_fps = 0, avg_fps_win = 0;
	int fps_count = 0;
	double avg_err = 0;

	int valid_frame_count = 0;
	cv::Point2d corners[4];
	double fps_font_size = 1.00;
	cv::Scalar gt_color(255, 0, 0), uav_color(0, 255, 0);

	CVUtils cv_utils;
	cv::Mat satellite_img_copy, satellite_img_copy1, satellite_img_small;
	satellite_img_small.create(1000, 900, satellite_img.type());
	satellite_img.copyTo(satellite_img_copy1);
	// ++++++++draw ground truth trajectory on the image++++++++++++
	if(show_ground_truth){
		for(i = 0; i < gt_kpt_corners.size() - 2; i++){
			line(satellite_img_copy1, gt_kpt_corners.at(i), gt_kpt_corners.at(i + 1), gt_color, 2);
		}
		line(satellite_img_copy1, gt_kpt_corners.at(gt_kpt_corners.size() - 2), gt_kpt_corners.at(gt_kpt_corners.size() - 1), gt_color, 2);
	}

	int templ_img_type = uav_tracker->inputType() == CV_32FC1 ? CV_64FC1 : CV_64FC3;
	int templ_uchar_img_type = uav_tracker->inputType() == CV_32FC1 ? CV_8UC1 : CV_8UC3;


	/*********************************** update tracker ***********************************/
	vector<cv::Point2d> ctr_pts;
	while(true) {

		satellite_img_copy1.copyTo(satellite_img_copy);

		if(record_frames || show_cv_window){
			/* draw tracker positions to satellite image */
			cv_utils.cornersToPoint2D(corners, uav_tracker->getRegion());
			line(satellite_img_copy, corners[0], corners[1], cv_utils.getObjCol(0), line_thickness);
			line(satellite_img_copy, corners[1], corners[2], cv_utils.getObjCol(0), line_thickness);
			line(satellite_img_copy, corners[2], corners[3], cv_utils.getObjCol(0), line_thickness);
			line(satellite_img_copy, corners[3], corners[0], cv_utils.getObjCol(0), line_thickness);
			putText(satellite_img_copy, uav_tracker->name, corners[0],
				cv::FONT_HERSHEY_SIMPLEX, fps_font_size, cv_utils.getObjCol(0));

			Matrix2Xd warped_pts(2, 1);
			VectorXd curr_ssm_state(uav_tracker->getSSM()->getStateSize());
			uav_tracker->getSSM()->estimateWarpFromCorners(curr_ssm_state, init_location, uav_tracker->getRegion());
			uav_tracker->getSSM()->applyWarpToPts(warped_pts, init_pt, curr_ssm_state);

			cv::Point2d tmpPt(warped_pts(0, 0), warped_pts(1, 0));

			printf("x: %f, y: %f\n", tmpPt.x, tmpPt.y);
			//line(satellite_img_copy1, gt_kpt_corners.at(i), gt_kpt_corners.at(i+1), CV_RGB(0,255,0), 3);
			if(ctr_pts.size() > 1){
				line(satellite_img_copy1, tmpPt, ctr_pts.back(), uav_color, 2);
			}
			ctr_pts.push_back(tmpPt);
			//printf("The size of ctr_pts: %d\n", ctr_pts.size());
			//char fps_text[100];
			//snprintf(fps_text, 100, "frame: %d c: %9.3f a: %9.3f cw: %9.3f aw: %9.3f fps",
			//	input->getFrameID() + 1, fps, avg_fps, fps_win, avg_fps_win);
			//putText(satellite_img_copy, fps_text, fps_origin, cv::FONT_HERSHEY_SIMPLEX, fps_font_size, fps_color);

			//if(show_tracking_error){
			//	char err_text[100];
			//	snprintf(err_text, 100, "ce: %12.8f ae: %12.8f", tracking_err, avg_err);
			//	putText(satellite_img_copy, err_text, err_origin, cv::FONT_HERSHEY_SIMPLEX, err_font_size, err_color);
			//}

			if(show_cv_window){
				cv::resize(satellite_img_copy, satellite_img_small, cv::Size(satellite_img_small.cols, satellite_img_small.rows));
				imshow(satellite_win_name, satellite_img_small);
				imshow("UAV Image", input->getFrame());
				int pressed_key = cv::waitKey(1 - pause_after_frame);
				if(pressed_key == 27){
					break;
				}
				if(pressed_key == 32){
					pause_after_frame = 1 - pause_after_frame;
				}
			}
		}
		if(!show_cv_window && (input->getFrameID() + 1) % 50 == 0){
			printf("frame_id: %5d avg_fps: %15.9f avg_fps_win: %15.9f avg_err: %15.9f\n",
				input->getFrameID() + 1, avg_fps, avg_fps_win, avg_err);
		}
		if(input->n_frames > 0 && input->getFrameID() >= input->n_frames - 1){
			printf("==========End of input stream reached==========\n");
			break;
		}

		mtf_clock_get(start_time_with_input);
		//! update frame
		if(!input->update()){
			printf("Frame %d could not be read from the input pipeline", input->getFrameID() + 1);
			break;
		}
		uav_pre_proc.update(input->getFrame(), input->getFrameID());

		// extract template
		cv::Mat prev_uav_location = uav_tracker->getRegion().clone();
		uav_tracker->initialize(uav_pre_proc.getFrame(), uav_img_corners);
		uav_tracker->setRegion(prev_uav_location);		
		mtf_clock_get(start_time);

		//! update tracker
		uav_tracker->update(satellite_pre_proc.getFrame());

		mtf_clock_get(end_time);
		mtf_clock_measure(start_time, end_time, tracking_time);
		mtf_clock_measure(start_time_with_input, end_time, tracking_time_with_input);

		VectorXd curr_patch(uav_tracker->getAM()->getPatchSize());
		uav_tracker->getAM()->extractPatch(curr_patch, uav_tracker->getSSM()->getPts());
		cv::Mat  curr_img_cv_uchar(uav_tracker->getAM()->getResY(), uav_tracker->getAM()->getResX(),
			templ_uchar_img_type);
		cv::Mat(uav_tracker->getAM()->getResY(), uav_tracker->getAM()->getResX(),
			templ_img_type, curr_patch.data()).convertTo(curr_img_cv_uchar, curr_img_cv_uchar.type());

		imshow("Current Tracker Patch in Satellite Image", curr_img_cv_uchar);

		fps = 1.0 / tracking_time;
		fps_win = 1.0 / tracking_time_with_input;

		if(!std::isinf(fps) && fps < MAX_FPS){
			++fps_count;
			avg_fps += (fps - avg_fps) / fps_count;
			// if fps is not inf then fps_win too must be non inf
			avg_fps_win += (fps_win - avg_fps_win) / fps_count;
		}
	}//end while(true)

	printf("Average FPS: %15.10f\n", avg_fps);
	printf("Average FPS with Input: %15.10f\n", avg_fps_win);
	if(show_tracking_error){
		printf("Average Tracking Error: %15.10f\n", avg_err);
		printf("Frames used for computing the average: %d\n", valid_frame_count);
	}
	string outfl = string(seq_path) + "/" + string(seq_name) + "/uav_trajectory_traj_results.txt";
	ofstream outtrajectory(outfl);
	//if(!outtrajectory)return;
	for(i = 0; i < ctr_pts.size(); i++){
		outtrajectory << ctr_pts.at(i).x << " " << ctr_pts.at(i).y << endl;
	}
	outtrajectory.close();
	printf("Number of points: %lu\n", ctr_pts.size());

	cv::waitKey(0);
	cv::destroyAllWindows();
	return 0;
}
