/**
application to generate synthetic sequences by warping an image using
random perturbations produced by different SSMs
*/

#include "mtf/mtf.h"
// tools for reading in images from various sources like image sequences, 
// videos and cameras as well as for pre processing them
#include "mtf/pipeline.h"
#include "mtf/Config/parameters.h"
#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/imgUtils.h"

#include <vector>
#include <memory>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"

#define MAX_FPS 1e6

using namespace Eigen;

using namespace std;
using namespace mtf::params;
namespace fs = boost::filesystem;

typedef unique_ptr<mtf::TrackerBase> Tracker_;

int main(int argc, char * argv[]) {

	printf("Starting MTF sequence registration tool...\n");

	if(!readParams(argc, argv)){ return EXIT_FAILURE; }

#ifdef ENABLE_PARALLEL
	Eigen::initParallel();
#endif

	printf("*******************************\n");
	printf("Using parameters:\n");
	printf("n_trackers: %d\n", n_trackers);
	printf("actor_id: %d\n", actor_id);
	printf("source_id: %d\n", seq_id);
	printf("source_name: %s\n", seq_name.c_str());
	printf("actor: %s\n", actor.c_str());
	printf("pipeline: %c\n", pipeline);
	printf("img_source: %c\n", img_source);
	printf("reg_show_output: %d\n", reg_show_output);
	printf("reg_ssm: %s\n", reg_ssm.c_str());
	printf("********************************\n");

	/* initialize pipeline*/
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

	cv::Mat init_corners = mtf::utils::getFrameCorners(input->getFrame(), reg_track_border);	
	resx = static_cast<unsigned int>(input->getWidth());
	resy = static_cast<unsigned int>(input->getHeight());
	// ************************************************************************************************** //
	// ***************************** initialize trackers and pre processors ***************************** //
	// ************************************************************************************************** //

	Tracker_ tracker;
	PreProc_ pre_proc;
	try{
		tracker.reset(mtf::getTracker(mtf_sm, mtf_am, mtf_ssm, mtf_ilm));
		if(!tracker){
			printf("Tracker could not be created successfully\n");
			return EXIT_FAILURE;
		}
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while creating the tracker: %s\n", err.type(), err.what());
		return EXIT_FAILURE;
	}
	try{
		pre_proc = mtf::getPreProc(tracker->inputType(), pre_proc_type);
		pre_proc->initialize(input->getFrame(), input->getFrameID());
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while initializing the pre processor: %s\n",
			err.type(), err.what());
		return EXIT_FAILURE;
	}
	try{
		for(PreProc_ curr_obj = pre_proc; curr_obj; curr_obj = curr_obj->next){
			tracker->setImage(curr_obj->getFrame());
		}
		tracker->initialize(init_corners);
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while initializing the tracker: %s\n",
			err.type(), err.what());
		return EXIT_FAILURE;
	}

	mtf::SSM ssm(mtf::getSSM(reg_ssm.c_str()));
	if(!ssm){
		printf("State space model could not be initialized");
		return EXIT_FAILURE;
	}
	ssm->initialize(init_corners);

	cv::Mat out_img;
	int out_img_type, n_channels;
	if(reg_grayscale_img){
		printf("Generating grayscale images\n");
		out_img_type = CV_8UC1;
		n_channels = 1;
		//! first frame in the synthetic sequence is the grayscale version of the original image 
		out_img.create(input->getHeight(), input->getWidth(), CV_8UC1);
		cv::cvtColor(input->getFrame(), out_img, CV_BGR2GRAY);
	} else{
		printf("Generating RGB images\n");
		out_img_type = CV_8UC3;
		n_channels = 3;
		//! original image is the first frame in the synthetic sequence
		out_img = input->getFrame().clone();
	}
	cv::Mat warped_img(input->getHeight(), input->getWidth(), out_img_type);
	
	std::string out_dir = cv::format("%s/%s_registered", db_root_path.c_str(), seq_name.c_str());
	cv::VideoWriter output;
	if(reg_save_as_video){
		std::string out_file = cv::format("%s.avi", out_dir.c_str());
		printf("Saving output to MJPG video file: %s with %d fps\n", out_file.c_str(), reg_video_fps);
		output.open(out_file, CV_FOURCC('M', 'J', 'P', 'G'), reg_video_fps, input->getFrame().size());
		output.write(out_img);
	} else{
		//! for OpenCV imwrite function
		printf("Saving output as JPEG images at  %s\n", out_dir.c_str());
		if(!fs::exists(out_dir)){
			printf("Output directory does not exist. Creating it...\n");
			fs::create_directories(out_dir);
		}
		cv::imwrite(cv::format("%s/frame%05d.jpg", out_dir.c_str(), 1), out_img);
	}
	const char* warped_img_win_name = "warped_img";
	cv::Mat write_mask;
	
	printf("Sequence generation started...\n");
	while(input->update()){
		pre_proc->update(input->getFrame(), input->getFrameID());
		try{
			/**
			update tracker;
			this call is equivalent to : trackers[tracker_id]->update(pre_proc->getFrame());
			as the image has been passed at the time of initialization through setImage()
			and does not need to be passed again as long as the new
			image is read into the same locatioon
			*/
			tracker->update();
		} catch(const mtf::utils::InvalidTrackerState &err){
			//! exception thrown by MTF modsules when the tracker ends up in an invalid state 
			//! due to NaNs or Infs in the result of some numerical computation
			printf("Invalid tracker state encountered in frame %d: %s\n", input->getFrameID() + 1, err.what());
			return EXIT_FAILURE;
			continue;
		} catch(const mtf::utils::Exception &err){
			printf("Exception of type %s encountered while updating the tracker: %s\n", 
				err.type(), err.what());
			return EXIT_FAILURE;
		}
		VectorXd inv_warp = ssm->estimateWarpFromCorners(init_corners, tracker->getRegion());
		VectorXd curr_warp(ssm->getStateSize());
		ssm->invertState(curr_warp, inv_warp);
		cv::Mat warped_corners = ssm->applyWarpToCorners(init_corners, curr_warp);
		ssm->setCorners(warped_corners);
		if(reg_grayscale_img){
			warped_img.setTo(cv::Scalar(0));
		} else{
			warped_img.setTo(cv::Vec3b(0, 0, 0));
		}		
		mtf::utils::writePixelsToImage(warped_img, mtf::utils::reshapePatch(input->getFrame()),
		ssm->getPts(), n_channels, write_mask);
		if(reg_show_output){
			cv::imshow(warped_img_win_name, warped_img);
			if(cv::waitKey(500) == 27){ break; }
		}
		if(reg_save_as_video){
			output.write(warped_img);
		} else{
			cv::imwrite(cv::format("%s/frame%05d.jpg", out_dir.c_str(), input->getFrameID() + 1), warped_img);
		}
		if(input->getFrameID() + 1 % 50 == 0){
			printf("Done %d frames\t", input->getFrameID());
		}
	}
	if(reg_save_as_video){
		output.release();
	}
	return EXIT_SUCCESS;
}

