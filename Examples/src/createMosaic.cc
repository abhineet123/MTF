//! main header that provides functions for creating trackers
#include "mtf/mtf.h"
#include <mtf/Tools/pipeline.h>
#include<mtf/Utilities/miscUtils.h>
#include<mtf/Utilities/imgUtils.h>
#include<mtf/Utilities/warpUtils.h>

typedef std::unique_ptr<mtf::TrackerBase> Tracker_;

int main(int argc, char * argv[]){

	if(!readParams(argc, argv)){ return EXIT_FAILURE; }

	Input_ input(getInput(pipeline));
	if(!input->initialize()){ return EXIT_FAILURE; };

	if(init_frame_id > 0){
		printf("Skipping %d frames...\n", init_frame_id);
	}
	for(int frame_id = 0; frame_id < init_frame_id; ++frame_id){
		input->update();
	}
	if(res_from_size){
		resx = input->getFrame().cols / res_from_size;
		resy = input->getFrame().rows / res_from_size;
	}
	Tracker_ tracker(mtf::getTracker(mtf_sm, mtf_am, mtf_ssm, mtf_ilm));
	if(!tracker){ return EXIT_FAILURE; }


	resx = input->getFrame().cols;
	resy = input->getFrame().rows;
	//! dummy SSM to convert the tracker output to a location in the mosaic image
	sim_normalized_init = aff_normalized_init = hom_normalized_init = 0;
	mtf::SSM mos_ssm(mtf::getSSM(mtf_ssm));

	cv::Mat init_corners = mtf::utils::getFrameCorners(input->getFrame(), mos_track_border);
	mos_ssm->initialize(init_corners);


	//! dummy AM - 3 channel SSD - to extract pixel values from the current image
	mtf::AM mos_am(mtf::getAM("ssd3", "0"));
	NoProcessing mos_pre_proc(mos_am->inputType());
	mos_pre_proc.initialize(input->getFrame());
	mos_am->setCurrImg(mos_pre_proc.getFrame());

	cv::Mat curr_patch(mos_am->getResY(), mos_am->getResX(),
		mos_am->getNChannels() == 1 ? CV_8UC1 : CV_8UC3);

	GaussianSmoothing pre_proc(tracker->inputType());
	pre_proc.initialize(input->getFrame());
	tracker->setImage(pre_proc.getFrame());

	//mtf::PtsT init_pts = mtf::utils::getFramePts(input->getFrame(), 0);

	mtf::PtsT init_pts = mtf::utils::getPtsFromCorners(
		mtf::utils::getFrameCorners(input->getFrame(), 0),
		input->getFrame().cols, input->getFrame().rows);

	tracker->initialize(init_corners);

	int mosaic_width = input->getFrame().cols + 2 * mos_border;
	int mosaic_height = input->getFrame().rows + 2 * mos_border;

	cv::Mat mosaic_img(mosaic_height, mosaic_width,
		mos_am->getNChannels() == 1 ? CV_8UC1 : CV_8UC3, CV_RGB(0, 0, 0));
	double resize_factor_x = static_cast<double>(mos_disp_width) / static_cast<double>(mosaic_img.cols);
	double resize_factor_y = static_cast<double>(mos_disp_height) / static_cast<double>(mosaic_img.rows);
	double mos_resize_factor = resize_factor_x > resize_factor_y ? resize_factor_x : resize_factor_y;
	cv::Mat mosaic_disp_img(mosaic_img.rows*mos_resize_factor, mosaic_img.cols*mos_resize_factor,
		mos_am->getNChannels() == 1 ? CV_8UC1 : CV_8UC3, CV_RGB(0, 0, 0));
	cv::Mat mosaic_disp_corners(2, 4, CV_64FC1);
	cv::Mat write_mask, write_mask_disp;
	if(mos_use_write_mask){
		write_mask.create(mosaic_height, mosaic_width, CV_8UC1);
		write_mask_disp.create(write_mask.rows*mos_resize_factor, write_mask.cols*mos_resize_factor, CV_8UC1);
		write_mask.setTo(cv::Scalar(0));
		write_mask_disp.setTo(cv::Scalar(0));
	}

	printf("Using displayed image of size: %d x %d\n", mosaic_disp_img.cols, mosaic_disp_img.rows);


	mtf::CornersT prev_location_norm = mtf::utils::Corners(
		mtf::utils::getFrameCorners(input->getFrame(), 0)).eig();
	mtf::CornersT curr_location_norm;

	mos_ssm->setCorners(mtf::CornersT(prev_location_norm.array() + mos_border));

	mtf::utils::writePixelsToImage(mosaic_img, mos_am->getPatch(init_pts),
		mos_ssm->getPts(), mos_am->getNChannels(), write_mask);

	while(input->update()){

		pre_proc.update(input->getFrame());
		tracker->update();

		VectorXd curr_warp(mos_ssm->getStateSize());
		VectorXd inv_warp(mos_ssm->getStateSize());

		mos_ssm->estimateWarpFromCorners(curr_warp, init_corners, tracker->getRegion());
		mos_ssm->invertState(inv_warp, curr_warp);
		mos_ssm->applyWarpToCorners(curr_location_norm, prev_location_norm, inv_warp);
		mos_ssm->setCorners(mtf::CornersT(curr_location_norm.array() + mos_border));

		//mos_ssm->compositionalUpdate(inv_warp);

		tracker->initialize(init_corners);

		//! extract current patch
		mos_pre_proc.update(input->getFrame());
		mtf::PixValT eig_patch = mos_am->getPatch(init_pts);
		//! write the patch to the image at the warped locations given by the tracker
		mtf::utils::writePixelsToImage(mosaic_img, eig_patch,
			mos_ssm->getPts(), mos_am->getNChannels(), write_mask);

		//! resize the mosaic image to display on screen
		cv::resize(mosaic_img, mosaic_disp_img, mosaic_disp_img.size());
		if(mos_use_write_mask){
			cv::resize(write_mask, write_mask_disp, write_mask_disp.size());
		}
		//! draw the location of the current warped bounding box given by the tracker
		mos_ssm->getCorners(mosaic_disp_corners);
		mosaic_disp_corners *= mos_resize_factor;
		mtf::utils::drawRegion(mosaic_disp_img, mosaic_disp_corners, CV_RGB(0, 255, 0), 2);
		if(mos_show_grid){
			mtf::PtsT mosaic_disp_grid = mos_ssm->getPts()*mos_resize_factor;
			mtf::utils::drawGrid(mosaic_disp_img, mosaic_disp_grid, mos_ssm->getResX(), mos_ssm->getResY());
		}
		mtf::utils::drawRegion(input->getFrame(MUTABLE), tracker->getRegion(), CV_RGB(255, 0, 0), 2);
		cv::imshow("Mosaic", mosaic_disp_img);
		if(mos_use_write_mask){
			cv::imshow("Mask", write_mask_disp);
		}
		cv::imshow("Current Image", input->getFrame());
		cv::imshow("Current Patch", mtf::utils::reshapePatch(eig_patch,
			mos_am->getResY(), mos_am->getResX(), mos_am->getNChannels()));

		prev_location_norm = curr_location_norm;

		if(cv::waitKey(1) == 27){ break; }
	}
	cv::destroyAllWindows();
	return 0;
}

