#include "mtf/mtf.h"
// tools for reading in images from various sources like image sequences, 
// videos and cameras as well as for pre processing them
#include "mtf/Tools/pipeline.h"
// general OpenCV utilities for selecting objects, reading ground truth, etc.
#include "mtf/Tools/cvUtils.h"
// parameters for different modules
#include "mtf/Config/parameters.h"
// general utilities for image drawing, etc.
#include "mtf/Utilities/miscUtils.h"

#include <vector>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"

#define MAX_FPS 1e6

using namespace std;
using namespace mtf::params;
namespace fs = boost::filesystem;

int main(int argc, char * argv[]) {
	// *************************************************************************************************** //
	// ********************************** read configuration parameters ********************************** //
	// *************************************************************************************************** //

	if(!readParams(argc, argv)){ return EXIT_FAILURE; }

	printf("*******************************\n");
	printf("Using parameters:\n");
	printf("actor_id: %d\n", actor_id);
	printf("source_id: %d\n", source_id);
	printf("source_name: %s\n", source_name.c_str());
	printf("actor: %s\n", actor.c_str());
	printf("pipeline: %c\n", pipeline);
	printf("img_source: %c\n", img_source);
	printf("*******************************\n");

	// *********************************************************************************************** //
	// ********************************** initialize input pipeline ********************************** //
	// *********************************************************************************************** //

	Input_ input(getInput(pipeline));
	if(!input->initialize()){
		printf("Pipeline could not be initialized successfully. Exiting...\n");
		return EXIT_FAILURE;
	}
	//printf("done getting no. of frames\n");
	printf("n_frames=%d\n", input->n_frames);

	if(init_frame_id > 0){
		printf("Skipping %d frames...\n", init_frame_id);
	}
	for(int frame_id = 0; frame_id < init_frame_id; frame_id++){
		if(!input->update()){
			printf("Frame %d could not be read from the input pipeline", input->getFrameID() + 1);
			return EXIT_FAILURE;
		}
	}
	// ************************************************************************************* //
	// ********************************* read ground truth ********************************* //
	// ************************************************************************************* //

	CVUtils cv_utils;
	if(use_reinit_gt){
		if(!cv_utils.readReinitGT(source_name, source_path, init_frame_id, input->n_frames, use_opt_gt, opt_gt_ssm)){
			printf("Reinitialization ground truth could not be read for frame %d\n", init_frame_id + 1);
			return false;
		}
	} else if(!cv_utils.readGT(source_name, source_path, input->n_frames, init_frame_id, debug_mode, use_opt_gt, opt_gt_ssm)){
		printf("Ground truth could not be read for frame %d\n", init_frame_id + 1);
		return false;
	}

	// ******************************************************************************************** //
	// *************************************** setup output *************************************** //
	// ******************************************************************************************** //

	string cv_win_name = "showGroundTruth :: " + source_name;
	if(show_cv_window) {
		cv::namedWindow(cv_win_name, cv::WINDOW_AUTOSIZE);
	}
	int valid_gt_frames = cv_utils.getGTSize();
	if(input->n_frames <= 0 || valid_gt_frames < input->n_frames){
		if(input->n_frames <= 0){
			printf("Ground truth is unavailable\n");
		} else if(valid_gt_frames > 0){
			printf("Ground truth is only available for %d out of %d frames\n",
				valid_gt_frames, input->n_frames);			
		} 
		return EXIT_FAILURE;
	}
	cv::VideoWriter output;
	if(record_frames){
		output.open("ground_truth.avi", CV_FOURCC('M', 'J', 'P', 'G'), 24, input->getFrame().size());
	}
	cv::Scalar gt_color(0, 255, 0), reinit_gt_color(0, 0, 255);
	cv::Point2d corners[4];

	// *********************************************************************************************** //
	// ************************************** start visualization ************************************ //
	// *********************************************************************************************** //
	cv::Point text_origin(10, 20);
	double text_font_size = 0.50;
	cv::Scalar text_color(0, 255, 0);
	mtf::SSM ssm;
	std::string out_fname;
	VectorXd ssm_params, curr_state, next_state, curr_state_inv;
	if(gt_write_ssm_params){
		std::string out_dir = db_root_path + "/" + actor + "/" + "SSM";
		if(!fs::exists(out_dir)){
			printf("SSM param directory: %s does not exist. Creating it...\n", out_dir.c_str());
			fs::create_directories(out_dir);
		}
		ssm.reset(mtf::getSSM(mtf_ssm));
		out_fname = out_dir + "/" + source_name + "." + ssm->name;
		printf("Writing GT SSM parameters to: %s\n", out_fname.c_str());
		fclose(fopen(out_fname.c_str(), "w"));
		ssm_params.resize(ssm->getStateSize());
		curr_state.resize(ssm->getStateSize());
		next_state.resize(ssm->getStateSize());
		curr_state_inv.resize(ssm->getStateSize());
	}


	while(true) {
		// *************************** display/save ground truth annotated frames *************************** //
		const cv::Mat &curr_corners = cv_utils.getGT(input->getFrameID());
		if(gt_write_ssm_params){
			ssm->setCorners(curr_corners);
			curr_state = ssm->getState();
			ssm->invertState(curr_state_inv, curr_state);
		}
		mtf::utils::drawRegion(input->getFrame(MUTABLE), curr_corners,
			gt_color, line_thickness);
		if(reinit_on_failure){
			mtf::utils::drawRegion(input->getFrame(MUTABLE), cv_utils.getReinitGT(input->getFrameID()),
				reinit_gt_color, line_thickness);
			//printf("frame_id: %d : \n", input->getFrameID());
			//cout << reinit_gt_corners << "\n";
		}
		putText(input->getFrame(MUTABLE), cv::format("frame: %d", input->getFrameID() + 1),
			text_origin, cv::FONT_HERSHEY_SIMPLEX, text_font_size, text_color);
		if(show_cv_window){
			imshow(cv_win_name, input->getFrame());
			int pressed_key = cv::waitKey(1 - pause_after_frame);

			if(pressed_key == 27){
				break;
			}
			if(pressed_key == 32){
				pause_after_frame = 1 - pause_after_frame;
			}
		}

		if(input->n_frames > 0 && input->getFrameID() >= input->n_frames - 1){
			printf("==========End of input stream reached==========\n");
			break;
		}
		if(record_frames){
			output.write(input->getFrame());
		}
		// ******************************* update pipeline ******************************* //

		// update pipeline
		if(!input->update()){
			printf("Frame %d could not be read from the input pipeline", input->getFrameID() + 1);
			break;
		}
		if(gt_write_ssm_params){
			ssm->setCorners(cv_utils.getGT(input->getFrameID()));
			next_state = ssm->getState();			
			ssm->composeWarps(ssm_params, next_state, curr_state_inv);
			mtf::utils::printScalarToFile(input->getFrameID(), nullptr, out_fname.c_str(),
				"%d", "a", "\t", "\t");
			mtf::utils::printMatrixToFile(ssm_params.transpose(), nullptr, out_fname.c_str());
		}
	}
	cv::destroyAllWindows();
	if(record_frames){
		output.release();
	}
	return EXIT_SUCCESS;
}
