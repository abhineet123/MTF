//! main header that provides functions for creating trackers
#include "mtf/mtf.h"
#include <mtf/Tools/pipeline.h>
#include<mtf/Utilities/miscUtils.h>
#include<mtf/Utilities/imgUtils.h>
#include<mtf/Utilities/warpUtils.h>

typedef std::unique_ptr<mtf::TrackerBase> Tracker_;

int main(int argc, char * argv[]){

	if(!readParams(argc, argv)){ return EXIT_FAILURE; }

	printf("Starting online mosaicing with:\n");
	printf("mos_inv_tracking: %d\n", mos_inv_tracking);
	printf("mos_use_norm_corners: %d\n", mos_use_norm_corners);
	printf("mos_track_border: %d\n", mos_track_border);
	printf("mos_border_size: %d x %d\n", mos_border_width, mos_border_height);
	printf("mos_init_offset: %d, %d\n", mos_init_offset_x, mos_init_offset_y);
	printf("mos_disp_size: %d x %d\n", mos_disp_width, mos_disp_height);
	printf("mos_show_grid: %d\n", mos_show_grid);
	printf("mos_use_write_mask: %d\n", mos_use_write_mask);
	printf("mos_save_img: %d\n", mos_save_img);

	Input_ input(getInput(pipeline));
	if(!input->initialize()){ return EXIT_FAILURE; };

	if(init_frame_id > 0){
		printf("Skipping %d frames...\n", init_frame_id);
	}
	for(int frame_id = 0; frame_id < init_frame_id; ++frame_id){
		input->update();
	}
	if(res_from_size){
		resx = (input->getFrame().cols - 2 * mos_track_border) / res_from_size;
		resy = (input->getFrame().rows - 2 * mos_track_border) / res_from_size;
	}
	Tracker_ tracker(mtf::getTracker(mtf_sm, mtf_am, mtf_ssm, mtf_ilm));
	if(!tracker){ return EXIT_FAILURE; }

	resx = input->getFrame().cols;
	resy = input->getFrame().rows;
	//! dummy SSM to convert the tracker output to a location in the mosaic image
	sim_normalized_init = aff_normalized_init = hom_normalized_init = 0;
	mtf::SSM mos_ssm(mtf::getSSM("8"));

	cv::Mat init_corners = mtf::utils::getFrameCorners(input->getFrame(), mos_track_border);
	mos_ssm->initialize(init_corners);

	//! dummy AM - 3 channel SSD - to extract pixel values from the current image
	mtf::AM mos_am(mtf::getAM("ssd3", "0"));
	NoProcessing mos_pre_proc(mos_am->inputType());
	mos_pre_proc.initialize(input->getFrame());
	mos_am->setCurrImg(mos_pre_proc.getFrame());

	int n_channels = mos_am->getNChannels();
	int mos_img_type = n_channels == 1 ? CV_8UC1 : CV_8UC3;
	int img_size = input->getFrame().cols * input->getFrame().rows * n_channels;

	GaussianSmoothing pre_proc(tracker->inputType());
	pre_proc.initialize(input->getFrame());
	tracker->setImage(pre_proc.getFrame());

	//mtf::PtsT init_pts = mtf::utils::getFramePts(input->getFrame(), 0);

	mtf::PtsT init_pts = mtf::utils::getPtsFromCorners(
		mtf::utils::getFrameCorners(input->getFrame(), 0),
		input->getFrame().cols, input->getFrame().rows);

	tracker->initialize(init_corners);

	int mos_width = input->getFrame().cols + 2 * mos_border_width;
	int mos_height = input->getFrame().rows + 2 * mos_border_height;

	cv::Mat mos_img(mos_height, mos_width, mos_img_type, CV_RGB(0, 0, 0));
	double resize_factor_x = static_cast<double>(mos_disp_width) / static_cast<double>(mos_img.cols);
	double resize_factor_y = static_cast<double>(mos_disp_height) / static_cast<double>(mos_img.rows);
	double mos_resize_factor = resize_factor_x < resize_factor_y ? resize_factor_x : resize_factor_y;
	cv::Mat mos_disp_img(mos_img.rows*mos_resize_factor, mos_img.cols*mos_resize_factor, mos_img_type, CV_RGB(0, 0, 0));
	cv::Mat mos_mask, mos_mask_disp;
	if(mos_use_write_mask){
		mos_mask.create(mos_height, mos_width, CV_8UC1);
		mos_mask_disp.create(mos_mask.rows*mos_resize_factor, mos_mask.cols*mos_resize_factor, CV_8UC1);
		mos_mask.setTo(cv::Scalar(0));
		mos_mask_disp.setTo(cv::Scalar(0));
	}

	std::string mos_disp_win = cv_format("Mosaic :: %s", source_name.c_str()), mos_mask_win = "Mask",
		mos_patch_win = "Current Patch", img_win = "Current Image";
	if(show_cv_window){
		if(mos_use_write_mask){ cv::namedWindow(mos_mask_win); }
		cv::namedWindow(mos_patch_win);
		cv::namedWindow(img_win);
		cv::namedWindow(mos_disp_win);
		printf("Using displayed image of size: %d x %d\n", mos_disp_img.cols, mos_disp_img.rows);
	}

	mtf::CornersT init_mos_location_norm(mtf::utils::Corners(init_corners).eig());
	// mtf::CornersT prev_mos_location_norm(mtf::utils::Corners(
	// mtf::utils::getFrameCorners(input->getFrame(), 0)).eig());
	mtf::CornersT init_mos_location(mtf::utils::Corners(
		mtf::utils::getFrameCorners(input->getFrame(), 0)).eig());

	init_mos_location.row(0).array() += mos_border_width + mos_init_offset_x;
	init_mos_location.row(1).array() += mos_border_height + mos_init_offset_y;

	if(mos_use_norm_corners == 1){ mos_ssm->setCorners(init_mos_location); }

	VectorXd norm_warp(mos_ssm->getStateSize());
	mos_ssm->estimateWarpFromCorners(norm_warp, init_mos_location_norm, init_mos_location);
	utils::printMatrix(norm_warp.transpose(), "norm_warp");

	mtf::PtsT mos_pts = mos_ssm->getPts();
	if(mos_use_norm_corners == 2){
		mos_ssm->applyWarpToPts(mos_pts, mos_ssm->getPts(), norm_warp);
	}
	mtf::PixValT mos_patch = Map<VectorXc>(input->getFrame().data, img_size).cast<double>();
	mtf::utils::writePixelsToImage(mos_img, mos_patch, mos_pts, n_channels, mos_mask);

	mtf::CornersT prev_mos_location(init_mos_location), prev_mos_location_norm(init_mos_location_norm);
	cv::Mat prev_img = pre_proc.getFrame().clone();

	while(input->update()){
		VectorXd curr_warp(mos_ssm->getStateSize());
		mtf::CornersT curr_mos_location_norm, curr_mos_location;
		if(mos_inv_tracking){
			pre_proc.update(input->getFrame());
			tracker->initialize(pre_proc.getFrame(), init_corners);
			tracker->update(prev_img);
			mos_ssm->estimateWarpFromCorners(curr_warp, init_corners, tracker->getRegion());
			prev_img = pre_proc.getFrame().clone();
		} else{
			pre_proc.update(input->getFrame());
			tracker->update();
			VectorXd inv_warp(mos_ssm->getStateSize());
			mos_ssm->estimateWarpFromCorners(inv_warp, init_corners, tracker->getRegion());
			mos_ssm->invertState(curr_warp, inv_warp);
			//mos_ssm->compositionalUpdate(inv_warp);
			tracker->initialize(init_corners);
		}
		if(mos_use_norm_corners){
			if(mos_use_norm_corners == 1){
				mos_ssm->applyWarpToCorners(curr_mos_location_norm, prev_mos_location_norm, curr_warp);
				mos_ssm->applyWarpToCorners(curr_mos_location, curr_mos_location_norm, norm_warp);
				mos_ssm->setCorners(curr_mos_location);
			} else{
				mos_ssm->compositionalUpdate(curr_warp);
				mos_ssm->applyWarpToCorners(curr_mos_location, mos_ssm->getCorners(), norm_warp);
			}
		} else{
			mos_ssm->applyWarpToCorners(curr_mos_location, prev_mos_location, curr_warp);
			mos_ssm->setCorners(curr_mos_location);
		}
		//mos_ssm->setCorners(curr_mos_location);

		//! extract current patch
		//mos_pre_proc.update(input->getFrame());
		//mtf::PixValT eig_patch = mos_am->getPatch(init_pts);
		mos_patch = Map<VectorXc>(input->getFrame().data, img_size).cast<double>();
		if(mos_use_norm_corners == 2){
			mos_ssm->applyWarpToPts(mos_pts, mos_ssm->getPts(), norm_warp);
		} else{
			mos_pts = mos_ssm->getPts();
		}
		//! write the patch to the image at the warped locations given by the tracker
		mtf::utils::writePixelsToImage(mos_img, mos_patch, mos_pts, n_channels, mos_mask);

		prev_mos_location = curr_mos_location;
		prev_mos_location_norm = curr_mos_location_norm;

		if(show_cv_window){
			//! resize the mosaic image to display on screen
			cv::resize(mos_img, mos_disp_img, mos_disp_img.size());
			//! draw the location of the current warped bounding box given by the tracker
			mtf::utils::drawRegion(mos_disp_img, mtf::utils::Corners(curr_mos_location).mat()*mos_resize_factor,
				CV_RGB(0, 255, 0), 2);
			if(mos_show_grid){
				mtf::utils::drawGrid(mos_disp_img, mtf::PtsT(mos_pts*mos_resize_factor),
					mos_ssm->getResX(), mos_ssm->getResY());
			}
			mtf::utils::drawRegion(input->getFrame(MUTABLE), tracker->getRegion(), CV_RGB(255, 0, 0), 2);
			cv::imshow(mos_disp_win, mos_disp_img);

			if(mos_use_write_mask){
				cv::resize(mos_mask, mos_mask_disp, mos_mask_disp.size());
				cv::imshow(mos_mask_win, mos_mask_disp);
			}
			cv::imshow(img_win, input->getFrame());
			cv::imshow(mos_patch_win, mtf::utils::reshapePatch(mos_patch, mos_am->getResY(), mos_am->getResX(),
				n_channels));

			if(cv::waitKey(1) == 27){ break; }
		} else if(input->getFrameID() % 100 == 0){
			printf("Done %d frames\n", input->getFrameID());
		}
	}
	if(show_cv_window){ cv::destroyAllWindows(); }
	if(mos_save_img){
		if(mos_out_fname.empty()){
			mos_out_fname = cv_format("mosaic_%s_%s_%s_%s_inv%d_norm%d_mask%d_%ld.bmp",
				source_name.c_str(), mtf_sm, mtf_am, mtf_ssm, mos_inv_tracking,
				mos_use_norm_corners, mos_use_write_mask, time(0));
		}
		printf("Writing mosaic image to %s\n", mos_out_fname.c_str());
		cv::imwrite(mos_out_fname, mos_img);
	}
	return 0;
}

