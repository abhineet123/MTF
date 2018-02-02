//! main header that provides functions for creating trackers
#include "mtf/mtf.h"
//! tools for reading in images from various sources like image sequences, 
//! videos and cameras, pre processing these images and getting
//! objects to track either from ground truth or interactively from the user
#include "mtf/pipeline.h"
//! parameters for different modules
#include "mtf/Config/parameters.h"
//! general utilities for image drawing, etc.
#include "mtf/Utilities/miscUtils.h"
//! MTF specific exceptions
#include "mtf/Utilities/excpUtils.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"

#include <vector>
#include <memory>
/**
any FPS larger than this is considered meaningless and
not used for computing the average
*/
#define MAX_FPS 1e6

using namespace std;
using namespace mtf::params;
namespace fs = boost::filesystem;

typedef mtf::utils::TrackErrT TrackErrT;
typedef unique_ptr<mtf::TrackerBase> Tracker_;

int main(int argc, char * argv[]) {
	printf("\nStarting MTF...\n");

	// *************************************************************************************************** //
	// ********************************** read configuration parameters ********************************** //
	// *************************************************************************************************** //

	if(!readParams(argc, argv)){ return EXIT_FAILURE; }

#ifdef ENABLE_PARALLEL
	Eigen::initParallel();
#endif
	printf("*******************************\n");
	printf("Using parameters:\n");
	printf("n_trackers: %u\n", n_trackers);
	printf("actor_id: %d\n", actor_id);
	printf("source_id: %d\n", seq_id);
	printf("source_name: %s\n", seq_name.c_str());
	printf("actor: %s\n", actor.c_str());
	printf("pipeline: %c\n", pipeline);
	printf("img_source: %c\n", img_source);
	printf("mtf_visualize: %d\n", mtf_visualize);
	printf("read_obj_from_gt: %d\n", read_obj_from_gt);
	printf("write_tracking_data: %d\n", write_tracking_data);
	printf("mtf_sm: %s\n", mtf_sm);
	printf("mtf_am: %s\n", mtf_am);
	printf("mtf_ssm: %s\n", mtf_ssm);
	printf("*******************************\n");

	// *********************************************************************************************** //
	// ********************************** initialize input pipeline ********************************** //
	// *********************************************************************************************** //

	Input_ input;
	try{
		input.reset(mtf::getInput(pipeline));
		if(!input->initialize()){
			printf("Pipeline could not be initialized successfully. Exiting...\n");
			return EXIT_FAILURE;
		}
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while initializing the input pipeline: %s\n",
			err.type(), err.what());
		return EXIT_FAILURE;
	}
	//printf("done getting no. of frames\n");
	printf("n_frames=%d\n", input->getNFrames());

	if(invalid_state_check){
		/**
		tracker is considered to have invalid state or suffered an unrecoverable
		loss of tracking if the MCD/CLE error exceeds the distance between
		the farthest corners of the input images
		*/
		if(invalid_state_err_thresh <= 0){
			invalid_state_err_thresh = sqrt(input->getHeight()*input->getHeight() +
				input->getWidth()*input->getWidth());
		}
		printf("invalid_state_err_thresh=%f\n", invalid_state_err_thresh);
	}

	if(init_frame_id > 0){
		if(input->getNFrames() > 0 && init_frame_id >= input->getNFrames()){
			printf("init_frame_id: %d is larger than the maximum frame ID in the sequence: %d\n",
				init_frame_id, input->getNFrames() - 1);
			return EXIT_FAILURE;
		}
		printf("Skipping to frame %d before initializing trackers...\n", init_frame_id + 1);
		for(int frame_id = 0; frame_id < init_frame_id; ++frame_id){
			if(!input->update()){
				printf("Frame %d could not be read from the input pipeline\n", input->getFrameID() + 1);
				return EXIT_FAILURE;
			}
		}
	}
	if(start_frame_id < init_frame_id){
		start_frame_id = init_frame_id;
	}

	// **************************************************************************************************** //
	// ******************* get objects to be tracked and read ground truth if available ******************* //
	// **************************************************************************************************** //

	mtf::utils::ObjUtils obj_utils(img_resize_factor);
	try{
		if(!mtf::getObjectsToTrack(obj_utils, input.get())){
			printf("Object(s) to be tracked could not be obtained.\n");
			return EXIT_FAILURE;
		}
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while obtaining the objects to track: %s\n",
			err.type(), err.what());
		return EXIT_FAILURE;
	}
	//! no. of frames for which ground truth is available
	int valid_gt_frames = obj_utils.getGTSize();

	// ************************************************************************************************** //
	// ***************************** initialize trackers and pre processors ***************************** //
	// ************************************************************************************************** //

	if(n_trackers > 1){
		printf("Multi tracker setup enabled\n");
		write_tracking_data = reinit_on_failure = show_ground_truth = 0;
	}
	if(res_from_size){
		printf("Getting sampling resolution from object size...\n");
	}

	FILE *multi_fid = nullptr;
	vector<Tracker_> trackers(n_trackers);
	vector<PreProc_> pre_procs(n_trackers);
	for(unsigned int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
		if(n_trackers > 1){ multi_fid = readTrackerParams(multi_fid); }
		try{
			if(res_from_size){
				resx = static_cast<unsigned int>(obj_utils.getObj(tracker_id).size_x / res_from_size);
				resy = static_cast<unsigned int>(obj_utils.getObj(tracker_id).size_y / res_from_size);
			}
			trackers[tracker_id].reset(mtf::getTracker(mtf_sm, mtf_am, mtf_ssm, mtf_ilm));
			if(!trackers[tracker_id]){
				printf("Tracker could not be created successfully\n");
				return EXIT_FAILURE;
			}
		} catch(const mtf::utils::Exception &err){
			printf("Exception of type %s encountered while creating the tracker: %s\n", err.type(), err.what());
			return EXIT_FAILURE;
		}
		try{
			pre_procs[tracker_id] = mtf::getPreProc(pre_procs, trackers[tracker_id]->inputType(), pre_proc_type);
			pre_procs[tracker_id]->initialize(input->getFrame(), input->getFrameID());
		} catch(const mtf::utils::Exception &err){
			printf("Exception of type %s encountered while initializing the pre processor: %s\n",
				err.type(), err.what());
			return EXIT_FAILURE;
		}
		try{
			for(PreProc_ curr_obj = pre_procs[tracker_id]; curr_obj; curr_obj = curr_obj->next){
				trackers[tracker_id]->setImage(curr_obj->getFrame());
			}
			printf("Initializing tracker %d with object of size %f x %f\n", tracker_id,
				obj_utils.getObj(tracker_id).size_x, obj_utils.getObj(tracker_id).size_y);
			trackers[tracker_id]->initialize(obj_utils.getObj(tracker_id).corners);
		} catch(const mtf::utils::Exception &err){
			printf("Exception of type %s encountered while initializing the tracker: %s\n",
				err.type(), err.what());
			return EXIT_FAILURE;
		}
	}

	if(input->getNFrames() == 0){
		start_frame_id = init_frame_id;
	} else if(read_obj_from_gt && start_frame_id >= valid_gt_frames){
		printf("Ground truth is only available for %d frames so starting tracking there instead\n",
			valid_gt_frames);
		start_frame_id = valid_gt_frames - 1;
	}
	if(start_frame_id > init_frame_id){
		if(input->getNFrames() > 0 && start_frame_id >= input->getNFrames()){
			printf("start_frame_id: %d is larger than the maximum frame ID in the sequence: %d\n",
				start_frame_id, input->getNFrames() - 1);
			return EXIT_FAILURE;
		}
		printf("Skipping to frame %d before starting tracking...\n", start_frame_id + 1);
		while(input->getFrameID() < start_frame_id){
			if(!input->update()){
				printf("Frame %d could not be read from the input pipeline\n", input->getFrameID() + 1);
				return EXIT_FAILURE;
			}
		}
		try{
			for(unsigned int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
				pre_procs[tracker_id]->update(input->getFrame(), input->getFrameID());
				trackers[tracker_id]->setRegion(obj_utils.getGT(input->getFrameID()));
			}
		} catch(const mtf::utils::Exception &err){
			printf("Exception of type %s encountered while resetting the tracker location: %s\n",
				err.type(), err.what());
			return EXIT_FAILURE;
		}
	}

	// ******************************************************************************************** //
	// *************************************** setup output *************************************** //
	// ******************************************************************************************** //

	string cv_win_name = "MTF :: " + seq_name;
	string proc_win_name = "Processed Image :: " + seq_name;
	try{
		if(mtf_visualize) {
			cv::namedWindow(cv_win_name, cv::WINDOW_AUTOSIZE);
			if(show_proc_img) {
				cv::namedWindow(proc_win_name, cv::WINDOW_AUTOSIZE);
			}
		}
	} catch(...){
		printf("OpenCV window could not be created to show tracking output. Turning this option off...");
		mtf_visualize = 0;
	}

	/**
	if reading object from ground truth did not succeed then the
	ground truth is invalid and computing tracking error is not possible
	nor is it possible to reinitialize the object from ground truth
	*/
	if(!read_obj_from_gt){
		reset_at_each_frame = reinit_at_each_frame = reinit_on_failure =
			show_tracking_error = write_tracking_error = show_ground_truth = 0;
	}
	/**
	original GT will be overwritten by tracking data only if it is not read from reinit gt -
	existence of the latter pre supposes that the original GT has been generated completely
	*/
	overwrite_gt = overwrite_gt && !use_reinit_gt;
	/**
	Tracker will be reinitialized at each frame from ground truth so that tracking will be frame to frame;
	normally used for testing trackers with synthetic sequences
	*/
	if(reinit_at_each_frame){
		if(reinit_at_each_frame == 1){
			printf("Reinitializing tracker at each frame...\n");
		} else{
			printf("Reinitializing tracker every %d frames...\n", reinit_at_each_frame);
		}		
		reinit_on_failure = false;
	} else if(reset_at_each_frame){
		if(reset_at_each_frame == 1){
			printf("Resetting tracker at each frame...\n");
		} else{
			printf("Resetting tracker every %d frames...\n", reinit_at_each_frame);
		}
		reinit_on_failure = false;
	}
	//! functionality that needs the full ground truth
	bool full_gt_needed = show_tracking_error || write_tracking_error || reinit_on_failure || 
		reset_at_each_frame || reinit_at_each_frame || show_ground_truth;
	if(full_gt_needed){
		if(input->getNFrames() <= 0 || valid_gt_frames < input->getNFrames()){
			//! full ground truth is not available
			if(input->getNFrames() <= 0){
				printf("Disabling tracking error computation\n");
			} else if(valid_gt_frames > 0){
				printf("Disabling tracking error computation since ground truth is only available for %d out of %d frames\n",
					valid_gt_frames, input->getNFrames());
			} else{
				printf("Disabling tracking error computation since ground truth is not available\n");
			}
			reset_at_each_frame = reinit_at_each_frame = reinit_on_failure =
				show_tracking_error = write_tracking_error = show_ground_truth = 0;
		} else{
			printf("Using %s error to measure tracking accuracy\n",
				mtf::utils::toString(static_cast<TrackErrT>(tracking_err_type)));
		}
	}
	FILE *tracking_data_fid = nullptr, *tracking_error_fid = nullptr;
	if(write_tracking_data){
		string tracking_data_dir, tracking_data_path;
		if(tracking_data_fname.empty()){
			tracking_data_fname = cv::format("%s_%s_%s_%s", mtf_sm, mtf_am, mtf_ssm,
				mtf::utils::getDateTime().c_str());
		} else{
			//! externally specified tracking_data_fname is assumed to indicate batch mode 
			//! where if overwrite_gt is enabled, it is so by accident
			overwrite_gt = false;
		}
		if(overwrite_gt){
			tracking_data_dir = seq_path;
			tracking_data_fname = seq_name;
		} else{
			if(reinit_at_each_frame){
				std::string reinit_dir = "reinit";
				if(reinit_at_each_frame>1){
					reinit_dir = cv::format("%s_%d", reinit_dir.c_str(), reinit_at_each_frame);
				}
				tracking_data_dir = cv::format("log/tracking_data/%s/%s/%s", reinit_dir.c_str(), actor.c_str(), seq_name.c_str());
			} else if(reset_at_each_frame){
				std::string reset_dir = reset_to_init ? "reset_to_init" : "reset";
				if(reset_at_each_frame>1){
					reset_dir = cv::format("%s_%d", reset_dir.c_str(), reset_at_each_frame);
				}
				tracking_data_dir = cv::format("log/tracking_data/%s/%s/%s",
					reset_dir.c_str(), actor.c_str(), seq_name.c_str());
			} else if(reinit_on_failure){
				std::string reinit_data_dir = std::floor(reinit_err_thresh) == reinit_err_thresh ?
					//! reinit_err_thresh is an integer
					cv::format("reinit_%d_%d", static_cast<int>(reinit_err_thresh), reinit_frame_skip) :
					//! reinit_err_thresh is not an integer
					cv::format("reinit_%4.2f_%d", reinit_err_thresh, reinit_frame_skip);
				tracking_data_dir = cv::format("log/tracking_data/%s/%s/%s",
					reinit_data_dir.c_str(), actor.c_str(), seq_name.c_str());
			} else{
				tracking_data_dir = cv::format("log/tracking_data/%s/%s", actor.c_str(), seq_name.c_str());;
			}
			if(invert_seq){
				tracking_data_fname = cv::format("%s_inv", tracking_data_fname.c_str());
			}
			if(init_frame_id > 0){
				tracking_data_fname = cv::format("%s_init_%d", tracking_data_fname.c_str(), init_frame_id);
			}
		}
		if(!fs::exists(tracking_data_dir)){
			printf("Tracking data directory: %s does not exist. Creating it...\n", tracking_data_dir.c_str());
			fs::create_directories(tracking_data_dir);
		}
		tracking_data_path = cv::format("%s/%s.txt", tracking_data_dir.c_str(), tracking_data_fname.c_str());
		if(overwrite_gt){
			printf("Overwriting existing GT at %s with the tracking data\n", tracking_data_path.c_str());
			//! create backup of existing GT if any
			if(fs::exists(tracking_data_path)){
				string backup_gt_path = cv::format("%s/%s.back_mtf", tracking_data_dir.c_str(), tracking_data_fname.c_str());
				printf("Backing up existing GT to: %s\n", backup_gt_path.c_str());
				fs::rename(tracking_data_path, backup_gt_path);
			}
		} else{
			printf("Writing tracking data to: %s\n", tracking_data_path.c_str());
		}
		tracking_data_fid = fopen(tracking_data_path.c_str(), "w");
		fprintf(tracking_data_fid, "frame ulx uly urx ury lrx lry llx lly\n");
		if(overwrite_gt){
			//! write the original GT for frames before the one where the tracker is initialized, if any
			for(int frame_id = 0; frame_id < init_frame_id; ++frame_id){
				mtf::utils::writeCorners(tracking_data_fid, obj_utils.getGT(frame_id), frame_id);
			}
		}
		if(write_tracking_error){
			std::string tracking_err_path = cv::format("%s/%s.err", tracking_data_dir.c_str(), tracking_data_fname.c_str());
			printf("Writing tracking errors to: %s\n", tracking_err_path.c_str());
			tracking_error_fid = fopen(tracking_err_path.c_str(), "w");
			fprintf(tracking_error_fid, "frame\t MCD\t CLE\t Jaccard\n");
		}
	}
	cv::VideoWriter output;
	if(record_frames){
		if(record_frames_dir.empty()){
			record_frames_dir = "log";
		}
		if(!fs::exists(record_frames_dir)){
			printf("Recording directory: %s does not exist. Creating it...\n", record_frames_dir.c_str());
			fs::create_directories(record_frames_dir);
		}
		if(record_frames_fname.empty()){
			record_frames_fname = write_tracking_data ? tracking_data_fname :
				cv_format("%s_%s_%s_%d", mtf_sm, mtf_am, mtf_ssm, 1 - hom_normalized_init);
		}
		std::string record_frames_path = cv::format("%s/%s.avi", record_frames_dir.c_str(), tracking_data_fname.c_str());
		printf("Recording tracking video to: %s\n", record_frames_path.c_str());
		output.open(record_frames_path, CV_FOURCC('M', 'J', 'P', 'G'), 24, input->getFrame().size());
	}

	if(mtf_visualize){
		for(unsigned int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
			if(static_cast<int>(tracker_labels.size()) < tracker_id + 1){
				tracker_labels.push_back(trackers[tracker_id]->name);
			}
		}
	}
	double fps = 0, fps_win = 0;
	double tracking_time, tracking_time_with_input;
	double avg_fps = 0, avg_fps_win = 0;
	int fps_count = 0;
	double avg_err = 0;

	cv::Point fps_origin(10, 20);
	double fps_font_size = 0.50;
	cv::Scalar fps_color(0, 255, 0);
	cv::Point err_origin(10, 40);
	double err_font_size = 0.50;
	cv::Scalar err_color(0, 255, 0);
	cv::Scalar gt_color(0, 255, 0);

	if(reset_template){ printf("Template resetting is enabled\n"); }

	double tracking_err = 0;
	int failure_count = 0;
	bool is_initialized = true;
	int valid_frame_count = 0;
	int reinit_frame_id = init_frame_id;
	//! store the IDs of the frames where tracking failure is detected if reinit_from_gt is enabled;
	std::vector<int> failure_frame_ids;
	//! set to true if a call to the tracker update function results in a runtime error gettimng thrown
	bool invalid_tracker_state = false, tracker_failed = false;

	if(reinit_on_failure){
		printf("Tracker reinitialization on failure is enabled with error threshold: %f and skipped frames: %d\n",
			reinit_err_thresh, reinit_frame_skip);
	}
	if(frame_gap < 1){
		frame_gap = 1;
	} else if(frame_gap > 1){
		printf("Using a gap of %d between consecutive tracked frames\n", frame_gap);
	}
	bool resized_images = img_resize_factor != 1;

	// ********************************************************************************************** //
	// *************************************** start tracking ! ************************************* //
	// ********************************************************************************************** //
	while(true) {
		// *********************** compute tracking error and reinitialize if needed *********************** //
		cv::Mat gt_corners;
		if(show_tracking_error || write_tracking_error || reinit_on_failure || show_ground_truth){
			gt_corners = resized_images ? obj_utils.getGT(input->getFrameID(), reinit_frame_id) / img_resize_factor :
				obj_utils.getGT(input->getFrameID(), reinit_frame_id);
		}
		cv::Mat tracker_corners = trackers[0]->getRegion().clone();
		if(resized_images){ tracker_corners /= img_resize_factor; }

		//! non finite entries in the tracker region indicate invalid tracker state
		if(invalid_state_check){
			if(mtf::utils::hasNaN<double>(tracker_corners) || mtf::utils::hasInf<double>(tracker_corners)){
				printf("Tracker corners in frame %d have non finite values\n", input->getFrameID() + 1);
				invalid_tracker_state = true;
			}
		}
		if(print_corners){
			cout << tracker_corners << "\n";
		}
		if(show_tracking_error || write_tracking_error || reinit_on_failure){
			/**
			both tracking result and GT are in the coordinate frame of the resized image
			so must be scaled before computing the error
			*/
			tracking_err = mtf::utils::getTrackingError(static_cast<TrackErrT>(tracking_err_type),
				gt_corners, tracker_corners, tracking_error_fid, input->getFrameID());
			if(invalid_state_check){
				if(tracking_err > invalid_state_err_thresh){
					printf("Tracking error in frame %d exceeds the invalid state threshold: %f\n",
						input->getFrameID() + 1, tracking_err);
					invalid_tracker_state = true;
				}
			}
			//printf("tracking_err: %f\n", tracking_err);
			//printf("done computing error\n");
			if(invalid_tracker_state || std::isnan(tracking_err) || std::isinf(tracking_err) || tracking_err > reinit_err_thresh){
				tracker_failed = true;
			}
			if(reinit_on_failure && tracker_failed){
				++failure_count;
				failure_frame_ids.push_back(input->getFrameID() + 1);
				printf("Tracking failure %4d detected in frame %5d with error: %10.6f. ",
					failure_count, input->getFrameID() + 1, tracking_err);
				if(write_tracking_data){
					fprintf(tracking_data_fid, "frame%05d.jpg tracker_failed\n", input->getFrameID() + 1);
				}
				if(input->getFrameID() + reinit_frame_skip >= input->getNFrames()){
					printf("Reinitialization is not possible as insufficient frames (%d) are left to skip. Exiting...\n",
						input->getNFrames() - input->getFrameID() - 1);
					break;
				}
				bool skip_success = true;
				for(int skip_id = 0; skip_id < reinit_frame_skip; ++skip_id) {
					if(!input->update()){
						printf("Frame %d could not be read from the input pipeline\n", input->getFrameID() + 1);
						skip_success = false;
						break;
					}
				}
				if(!skip_success){ break; }
				printf("Reinitializing in frame %5d...\n", input->getFrameID() + 1);
				reinit_frame_id = input->getFrameID();
				try{
					if(reinit_with_new_obj){
						//! delete the old tracker instance and create a new one before reinitializing
						trackers[0].reset(mtf::getTracker(mtf_sm, mtf_am, mtf_ssm, mtf_ilm));
						if(!trackers[0]){
							printf("Tracker could not be created successfully\n");
							return EXIT_FAILURE;
						}
						pre_procs[0]->update(input->getFrame(), input->getFrameID());
						for(PreProc_ curr_obj = pre_procs[0]; curr_obj; curr_obj = curr_obj->next){
							trackers[0]->setImage(curr_obj->getFrame());
						}
					}
					trackers[0]->initialize(obj_utils.getGT(reinit_frame_id));
					tracker_corners = trackers[0]->getRegion().clone();
				} catch(const mtf::utils::Exception &err){
					printf("Exception of type %s encountered while reinitializing the tracker: %s\n",
						err.type(), err.what());
					return EXIT_FAILURE;
				}
				if(resized_images){ tracker_corners /= img_resize_factor; }

				is_initialized = true;
				invalid_tracker_state = tracker_failed = false;
			}
			if(is_initialized){
				is_initialized = false;
			} else{
				//! exclude initialization frames from those used for computing the average error                
				++valid_frame_count;
				avg_err += (tracking_err - avg_err) / valid_frame_count;
			}
		}

		if(reinit_at_each_frame){
			if((input->getFrameID() - init_frame_id) % reinit_at_each_frame == 0){
				try{
					if(reinit_with_new_obj){
						//! delete the old tracker instance and create a new one before reinitializing
						trackers[0].reset(mtf::getTracker(mtf_sm, mtf_am, mtf_ssm, mtf_ilm));
						if(!trackers[0]){
							printf("Tracker could not be created successfully\n");
							return EXIT_FAILURE;
						}
						for(PreProc_ pre_proc = pre_procs[0]; pre_proc; pre_proc = pre_proc->next){
							trackers[0]->setImage(pre_proc->getFrame());
						}
					}
					trackers[0]->initialize(obj_utils.getGT(input->getFrameID()));
					tracker_corners = trackers[0]->getRegion().clone();
					if(resized_images){ tracker_corners /= img_resize_factor; }
				} catch(const mtf::utils::Exception &err){
					printf("Exception of type %s encountered while reinitializing the tracker: %s\n",
						err.type(), err.what());
					return EXIT_FAILURE;
				}
				invalid_tracker_state = tracker_failed = false;
			}
		} else if(reset_at_each_frame){
			if((input->getFrameID() - init_frame_id) % reset_at_each_frame == 0){
				try{
					cv::Mat reset_location = reset_to_init ? obj_utils.getGT(init_frame_id) :
						obj_utils.getGT(input->getFrameID());
					trackers[0]->setRegion(reset_location);
					tracker_corners = trackers[0]->getRegion().clone();
					if(resized_images){ tracker_corners /= img_resize_factor; }
				} catch(const mtf::utils::Exception &err){
					printf("Exception of type %s encountered while resetting the tracker: %s\n",
						err.type(), err.what());
					return EXIT_FAILURE;
				}
				invalid_tracker_state = tracker_failed = false;
			}
		}
		if(invalid_tracker_state){
			printf("Unrecoverable tracking loss detected. Exiting...\n");
			break;
		}

		// *************************** display/save tracking output *************************** //

		if(record_frames || mtf_visualize) {
			if(record_frames && write_tracking_data){
				output.write(input->getFrame());
			}
			/**
			draw tracker positions on OpenCV window
			*/
			for(unsigned int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
				cv::Mat drawn_corners = tracker_id == 0 ? tracker_corners :
					resized_images ? trackers[tracker_id]->getRegion() / img_resize_factor :
					trackers[tracker_id]->getRegion();
				mtf::utils::drawRegion(input->getFrame(mtf::utils::MUTABLE), drawn_corners,
					obj_utils.getObjCol(tracker_id), line_thickness, tracker_labels[tracker_id].c_str(),
					fps_font_size, show_corner_ids, 1 - show_corner_ids);
			}
			/**
			draw ground truth corners on OpenCV window
			*/
			if(show_ground_truth){
				mtf::utils::drawRegion(input->getFrame(mtf::utils::MUTABLE), gt_corners,
					gt_color, line_thickness, "ground_truth", fps_font_size,
					show_corner_ids, 1 - show_corner_ids);
			}
			/**
			write tracker speed in FPS - both current and average as well as
			with and without considering input/pre processing pipeline delay
			*/
			std::string fps_text = cv::format("frame: %d c: %9.3f a: %9.3f cw: %9.3f aw: %9.3f fps",
				input->getFrameID() + 1, fps, avg_fps, fps_win, avg_fps_win);
			putText(input->getFrame(mtf::utils::MUTABLE), fps_text, fps_origin, cv::FONT_HERSHEY_SIMPLEX, fps_font_size, fps_color);
			/**
			write tracking error for the first tracker - both current and average
			*/
			if(show_tracking_error){
				std::string err_text = cv::format("ce: %12.8f ae: %12.8f", tracking_err, avg_err);
				if(show_jaccard_error){
					double jaccard_error = static_cast<TrackErrT>(tracking_err_type) == TrackErrT::Jaccard ? tracking_err :
						mtf::utils::getJaccardError(gt_corners, tracker_corners,
						input->getFrame().cols, input->getFrame().rows);
					err_text = err_text + cv::format(" je: %12.8f", jaccard_error);
				}
				putText(input->getFrame(mtf::utils::MUTABLE), err_text, err_origin, cv::FONT_HERSHEY_SIMPLEX, err_font_size, err_color);
			}

			if(record_frames && !write_tracking_data){
				output.write(input->getFrame());
			}
			if(mtf_visualize){
				imshow(cv_win_name, input->getFrame());
				if(show_proc_img){
					pre_procs[0]->showFrame(proc_win_name);
				}
				int pressed_key = cv::waitKey(1 - pause_after_frame);
				if(pressed_key % 256 == 27){
					break;
				}
				if(pressed_key % 256 == 32){
					pause_after_frame = 1 - pause_after_frame;
				}
			}
		}
		if(!mtf_visualize && (input->getFrameID() + 1) % 50 == 0){
			printf("frame_id: %5d avg_fps: %15.9f avg_fps_win: %15.9f avg_err: %15.9f\n",
				input->getFrameID() + 1, avg_fps, avg_fps_win, avg_err);
		}
		if(write_tracking_data){
			mtf::utils::writeCorners(tracking_data_fid, tracker_corners, input->getFrameID());
		}

		if(input->getNFrames() > 0 && input->getFrameID() >= input->getNFrames() - 1){
			printf("==========End of input stream reached==========\n");
			break;
		}

		// ******************************* update pipeline and trackers ******************************* //

		mtf_clock_get(start_time_with_input);
		//! update pipeline
		for(int skip_id = 0; skip_id < frame_gap; ++skip_id) {
			if(!input->update()){
				printf("Frame %d could not be read from the input pipeline\n", input->getFrameID() + 1);
				break;
			}
		}
		//! update trackers       
		for(unsigned int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
			//! update pre processor
			pre_procs[tracker_id]->update(input->getFrame(), input->getFrameID());
			try{
				mtf_clock_get(start_time);
				/**
				update tracker;
				this call is equivalent to : trackers[tracker_id]->update(pre_proc->getFrame());
				as the image has been passed at the time of initialization through setImage()
				and does not need to be passed again as long as the new
				image is read into the same locatioon
				*/
				trackers[tracker_id]->update();
				mtf_clock_get(end_time);
				mtf_clock_measure(start_time, end_time, tracking_time);
				mtf_clock_measure(start_time_with_input, end_time, tracking_time_with_input);
			} catch(const mtf::utils::InvalidTrackerState &err){
				//! exception thrown by MTF modsules when the tracker ends up in an invalid state 
				//! due to NaNs or Infs in the result of some numerical computation
				printf("Invalid tracker state encountered in frame %d: %s\n", input->getFrameID() + 1, err.what());
				invalid_tracker_state = true;
				//! allow the tracker to be reinitialized if this option is enabled otherwise exit
				continue;
			} catch(const mtf::utils::Exception &err){
				printf("Exception of type %s encountered while updating the tracker: %s\n", 
					err.type(), err.what());
				return EXIT_FAILURE;
			}
			fps = 1.0 / tracking_time;
			fps_win = 1.0 / tracking_time_with_input;
			if(reset_template && (input->getFrameID() - init_frame_id) % reset_template == 0){
				trackers[tracker_id]->initialize(trackers[tracker_id]->getRegion());
			}
		}
		if(!std::isinf(fps) && fps < MAX_FPS){
			++fps_count;
			avg_fps += (fps - avg_fps) / fps_count;
			// if fps is not inf then fps_win too must be non inf
			avg_fps_win += (fps_win - avg_fps_win) / fps_count;
		}
	}
	cv::destroyAllWindows();

	//double avg_fps = accumulate( fps_vector.begin(), fps_vector.end(), 0.0 )/ fps_vector.size();
	//double avg_fps_win = accumulate( fps_win_vector.begin(), fps_win_vector.end(), 0.0 )/ fps_win_vector.size();
	printf("Average FPS: %15.10f\n", avg_fps);
	printf("Average FPS with Input: %15.10f\n", avg_fps_win);
	if(show_tracking_error){
		printf("Average Tracking Error: %15.10f\n", avg_err);
		printf("Frames used for computing the average: %d", valid_frame_count);
		if(input->getNFrames() > 0){
			printf(" / %d", input->getNFrames() - start_frame_id - 1);
		}
		printf("\n");
	}
	if(reinit_on_failure){
		printf("Number of failures: %d\n", failure_count);
		if(failure_count > 0){
			printf("Failures detected in frame(s): %d", failure_frame_ids[0]);
			for(unsigned int i = 1; i < failure_frame_ids.size(); ++i){
				printf(", %d", failure_frame_ids[i]);
			}
			printf("\n");
		}
	}
	if(write_tracking_data){
		if(!reinit_on_failure && invalid_tracker_state && input->getNFrames() > 0){
			for(int frame_id = input->getFrameID(); frame_id < input->getNFrames(); ++frame_id){
				fprintf(tracking_data_fid, "frame%05d.jpg invalid_tracker_state\n", frame_id + 1);
			}
		}
		fclose(tracking_data_fid);
		FILE *tracking_stats_fid = fopen("log/tracking_stats.txt", "a");
		fprintf(tracking_stats_fid, "%s\t %s\t %s\t %s\t %d\t %s\t %15.9f\t %15.9f",
			seq_name.c_str(), mtf_sm, mtf_am, mtf_ssm, hom_normalized_init,
			tracking_data_fname.c_str(), avg_fps, avg_fps_win);
		if(show_tracking_error){
			fprintf(tracking_stats_fid, "\t %15.9f", avg_err);
		}
		if(reinit_on_failure){
			fprintf(tracking_stats_fid, "\t %d", failure_count);
		}
		fprintf(tracking_stats_fid, "\n");
		fclose(tracking_stats_fid);
	}
	printf("Cleaning up...\n");
	if(record_frames){
		output.release();
	}
	if(write_tracking_error){
		fclose(tracking_error_fid);
	}
	pre_procs.clear();
	trackers.clear();
	return EXIT_SUCCESS;
}

