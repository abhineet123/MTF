// tools for reading in images from various sources like image sequences, 
// videos and cameras as well as for pre processing them
#include "mtf/pipeline.h"
// parameters for different modules
#include "mtf/Config/parameters.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
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


	// *********************************************************************************************** //
	// ********************************** initialize input pipeline ********************************** //
	// *********************************************************************************************** //
	img_source = rec_source;
	Input_ input(mtf::getInput(pipeline));
	if(!input->initialize()){
		printf("Pipeline could not be initialized successfully. Exiting...\n");
		return EXIT_FAILURE;
	}

	if(init_frame_id > 0){
		printf("Skipping %d frames...\n", init_frame_id);
	}
	for(int frame_id = 0; frame_id < init_frame_id; frame_id++){
		if(!input->update()){
			printf("Frame %d could not be read from the input pipeline", input->getFrameID() + 1);
			return EXIT_FAILURE;
		}
	}

	string cv_win_name = "Recording Sequence...";
	if(show_cv_window) {
		cv::namedWindow(cv_win_name, cv::WINDOW_AUTOSIZE);
	}
	cv::VideoWriter output;
	output.open(cv::format("rec_seq_%s.avi", rec_seq_suffix.c_str()), CV_FOURCC('M', 'J', 'P', 'G'), rec_fps, input->getFrame().size());

	while(true) {
		if(show_cv_window){
			imshow(cv_win_name, input->getFrame());
			int pressed_key = cv::waitKey(1);
			if(pressed_key == 27){
				break;
			}
		}

		if(input->getNFrames() > 0 && input->getFrameID() >= input->getNFrames() - 1){
			printf("==========End of input stream reached==========\n");
			break;
		}
		output.write(input->getFrame());
		// ******************************* update pipeline ******************************* //

		// update pipeline
		if(!input->update()){
			printf("Frame %d could not be read from the input pipeline", input->getFrameID() + 1);
			break;
		}
	}
	cv::destroyAllWindows();
	output.release();
	return EXIT_SUCCESS;
}
