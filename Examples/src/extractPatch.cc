#include "mtf/mtf.h"

// tools for reading in images from various sources like image sequences, 
// videos and cameras as well as for pre processing them
#include "mtf/Tools/pipeline.h"
// general OpenCV tools for selecting objects, reading ground truth, etc.
#include "mtf/Tools/cvUtils.h"
#include "mtf/Config/parameters.h"
#include "mtf/Config/datasets.h"

#include <string.h>
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "boost/filesystem/path.hpp"

using namespace std;
using namespace mtf::params;
namespace fs = boost::filesystem;

int main(int argc, char * argv[]) {

	if(!readParams(argc, argv)){ return EXIT_FAILURE; }

#ifdef ENABLE_PARALLEL
	Eigen::initParallel();
#endif
	printf("*******************************\n");
	printf("Using parameters:\n");
	printf("n_trackers: %d\n", n_trackers);
	printf("actor_id: %d\n", actor_id);
	printf("source_id: %d\n", source_id);
	printf("source_name: %s\n", source_name.c_str());	
	printf("actor: %s\n", actor.c_str());
	printf("pipeline: %c\n", pipeline);
	printf("img_source: %c\n", img_source);
	printf("show_cv_window: %d\n", show_cv_window);
	printf("*******************************\n");

	/* initialize pipeline*/
	Input_ input(getInput(pipeline));
	if(!input->initialize()){
		printf("Pipeline could not be initialized successfully\n");
		return EXIT_FAILURE;
	}
	printf("n_frames=%d\n", input->n_frames);
	printf("source_path: %s\n", source_path.c_str());

	CVUtils cv_utils;
	if(!cv_utils.readObjectFromGT(source_name, source_path, input->n_frames,
		init_frame_id, use_opt_gt, opt_gt_ssm, use_reinit_gt, debug_mode)){
		printf("Failed to read initial object from ground truth; using manual selection...\n");
		read_obj_from_gt = 0;
	}

	//cv::Mat img_rgb, img_gs;
	//char img_fname[500];
	//snprintf(img_fname, 500, "%s/%s/frame00001.jpg", source_path, source_name);
	//cv::Mat img_raw = cv::imread(img_fname);
	//mtf::initializeFrame(img_rgb, img_gs, img_raw);	


	mtf::AMParams am_params(resx, resy);
	mtf::SSDParams ssd_params(&am_params, ssd_show_template, ssd_forgetting_factor);
	mtf::SSD patch_extractor_am(&ssd_params);
	mtf::SSMParams ssm_params(resx, resy);
	mtf::TranslationParams trans_params(&ssm_params, debug_mode);
	mtf::Translation patch_extractor_ssm(&trans_params);

	patch_extractor_ssm.initialize(cv_utils.getGT(0));
	patch_extractor_am.initializePixVals(patch_extractor_ssm.getPts());

	int patch_size = patch_extractor_am.getNPix();
	int n_extracted_patches = extracted_frame_ids.size();
	if (extracted_frame_ids[0] == -1) {
		n_extracted_patches = input->n_frames;
	}

	// Define matrix for extracted patches
	MatrixXd extracted_patches(patch_size, n_extracted_patches);
	NoProcessing no_proc;

	for(int patch_id = 0; patch_id < n_extracted_patches; patch_id++){
		int frame_id;
		if (extracted_frame_ids[0] == -1) {
			frame_id = patch_id + 1;
		}
		else frame_id = extracted_frame_ids[patch_id];

		char img_fname[500];
		snprintf(img_fname, 500, "%s/%s/frame%05d.jpg", source_path.c_str(), source_name.c_str(), frame_id);
		printf("Reading image: %s\n", img_fname);
		cv::Mat img_raw = cv::imread(img_fname);
		// Allocate memory for rgb and gray scale of type CV_32F 
		// and convert the given RGB image to grayscale
		no_proc.initialize(img_raw);
		// Share memory with eigen image
		patch_extractor_am.setCurrImg(no_proc.getFrame());
		patch_extractor_ssm.setCorners(cv_utils.getGT(frame_id-1));
		patch_extractor_am.updatePixVals(patch_extractor_ssm.getPts());
		// **********DEBUG ******************
		//cout << "Using corners:\n";
		//cout << cv_utils.getGT(frame_id - 1]<<"\n";
		//cout << "SSM corners:\n";
		//cout << patch_extractor_ssm.getCorners() << "\n";
		//cout << "SSM points:\n";
		//cout << patch_extractor_ssm.getPts() << "\n";
		//cout << "Pixel Values:\n";
		//cout << patch_extractor_am.getCurrPixVals() << "\n";

		// Save the patch into a column of the output matrix
		extracted_patches.col(patch_id) = patch_extractor_am.getCurrPixVals();
		// Debug, only show one image at every 50
		if (patch_id % 50 == 0) {
			// Construct Mat object for current patch. It points to the memory without copying data
			cv::Mat curr_img_cv = cv::Mat(patch_extractor_am.getResY(), patch_extractor_am.getResX(), CV_64FC1,
				const_cast<double*>(patch_extractor_am.getCurrPixVals().data()));
			// Construct Mat object for the uint8 gray scale patch
			cv::Mat  curr_img_cv_uchar(patch_extractor_am.getResY(), patch_extractor_am.getResX(), CV_8UC1);
			// Convert the RGB patch to grayscale
			curr_img_cv.convertTo(curr_img_cv_uchar, curr_img_cv_uchar.type());
			cv::Mat  img_gs_uchar(no_proc.getFrame().rows, no_proc.getFrame().cols, CV_8UC1);
			no_proc.getFrame().convertTo(img_gs_uchar, img_gs_uchar.type());

			imshow("Image", img_raw);
			imshow("Image GS", img_gs_uchar);
			imshow("Patch", curr_img_cv_uchar);
			if(cv::waitKey(0)%256 == 27)
				break;
		}
	}

	string out_fname = cv::format("%s/%s/%s_%dx%d_%d.bin",
		db_root_path.c_str(), actor.c_str(), source_name.c_str(), 
		resx, resy, extraction_id);
	cout << "Saving to file: " << out_fname << endl;
	ofstream out_file;
	out_file.open(out_fname, ios::out | ios::binary);
	out_file.write((char*)(&patch_size),sizeof(int));// rows
	out_file.write((char*)(&n_extracted_patches),sizeof(int));// cols
	out_file.write((char*)(extracted_patches.data()),
		sizeof(double)*extracted_patches.size());
	out_file.close();
	return 0;
}
