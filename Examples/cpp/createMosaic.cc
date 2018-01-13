//! main header that provides functions for creating trackers
#include <mtf/mtf.h>
#include <mtf/pipeline.h>
#include <mtf/Utilities/miscUtils.h>
#include <mtf/Utilities/imgUtils.h>
#include <mtf/Utilities/warpUtils.h>

typedef std::unique_ptr<mtf::TrackerBase> Tracker_;
using namespace mtf::params;

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

	Input_ input(mtf::getInput(pipeline));
	if(!input->initialize()){ return EXIT_FAILURE; };

	if(init_frame_id > 0){
		printf("Skipping %d frames...\n", init_frame_id);
	}
	for(int frame_id = 0; frame_id < init_frame_id; ++frame_id){
		input->update();
	}
	if(res_from_size){
		resx = static_cast<unsigned int>((input->getFrame().cols - 2 * mos_track_border) / res_from_size);
		resy = static_cast<unsigned int>((input->getFrame().rows - 2 * mos_track_border) / res_from_size);
	}
	Tracker_ tracker(mtf::getTracker(mtf_sm, mtf_am, mtf_ssm, mtf_ilm));
	if(!tracker){ return EXIT_FAILURE; }

	resx = input->getFrame().cols;
	resy = input->getFrame().rows;
	//! dummy Homography SSM to convert the tracker output to a location in the mosaic image
	sim_normalized_init = aff_normalized_init = hom_normalized_init = 0;
	mtf::SSM mos_ssm(mtf::getSSM("8"));
	//! dummy AM - 3 channel SSD - to extract pixel values from the current image
	mtf::AM mos_am(mtf::getAM("ssd3", "0"));
	mtf::utils::NoFiltering mos_pre_proc(mos_am->inputType());
	mos_pre_proc.initialize(input->getFrame());
	mos_am->setCurrImg(mos_pre_proc.getFrame());

	int n_channels = mos_am->getNChannels();
	int mos_img_type = n_channels == 1 ? CV_8UC1 : CV_8UC3;
	int img_size = input->getFrame().cols * input->getFrame().rows * n_channels;

	mtf::utils::GaussianSmoothing pre_proc(tracker->inputType());

	//mtf::PtsT init_pts = mtf::utils::getFramePts(input->getFrame(), 0);
	//mtf::PtsT init_pts = mtf::utils::getPtsFromCorners(
	//	mtf::utils::getFrameCorners(input->getFrame(), 0),
	//	input->getFrame().cols, input->getFrame().rows);	

	int mos_width = input->getFrame().cols + 2 * mos_border_width;
	int mos_height = input->getFrame().rows + 2 * mos_border_height;

	cv::Mat mos_img(mos_height, mos_width, mos_img_type, CV_RGB(0, 0, 0));
	cv::Mat init_corners = mtf::utils::getFrameCorners(input->getFrame(), mos_track_border);
	mtf::CornersT init_mos_location_norm(mtf::utils::Corners(init_corners).eig());
	mtf::CornersT init_mos_location(mtf::utils::Corners(
		mtf::utils::getFrameCorners(input->getFrame(), 0)).eig());
	init_mos_location.row(0).array() += mos_border_width + mos_init_offset_x;
	init_mos_location.row(1).array() += mos_border_height + mos_init_offset_y;

	cv::Mat mos_curr_tracked_img, mos_prev_tracked_img;
	if(mos_use_norm_corners == 3){
		mos_curr_tracked_img.create(mos_height, mos_width, mos_img_type);
		mos_curr_tracked_img.setTo(cv::Scalar(0));
		mos_curr_tracked_img.copyTo(mos_prev_tracked_img);
		pre_proc.initialize(mos_curr_tracked_img);
		mos_ssm->initialize(init_mos_location);
	} else{
		pre_proc.initialize(input->getFrame());
		mos_ssm->initialize(init_corners);
	}
	tracker->setImage(pre_proc.getFrame());

	VectorXd norm_warp(mos_ssm->getStateSize());
	mos_ssm->estimateWarpFromCorners(norm_warp, init_mos_location_norm, init_mos_location);
	//utils::printMatrix(norm_warp.transpose(), "norm_warp");

	mtf::CornersT prev_mos_location(init_mos_location), prev_mos_location_norm(init_mos_location_norm);
	mtf::CornersT curr_mos_location(init_mos_location), curr_mos_location_norm(init_mos_location_norm);

	cv::Mat prev_proc_frame = pre_proc.getFrame().clone();

	cv::Mat mos_disp_img, mos_mask, mos_mask_disp;
	if(mos_use_write_mask){
		mos_mask.create(mos_height, mos_width, CV_8UC1);
		mos_mask.setTo(cv::Scalar(0));
	}
	cv::Mat init_corners_norm_3 = init_corners.clone();
	init_corners_norm_3.row(0) += mos_border_width + mos_init_offset_x;
	init_corners_norm_3.row(1) += mos_border_height + mos_init_offset_y;
	cv::Mat curr_corners_norm_3 = init_corners_norm_3.clone();
	if(mos_use_norm_corners == 3){
		tracker->initialize(init_corners_norm_3);
		mos_img.copyTo(mos_curr_tracked_img);
	} else{
		tracker->initialize(init_corners);
		if(mos_use_norm_corners == 1){
			mos_ssm->setCorners(init_mos_location);
		}
	}
	mtf::PtsT mos_pts = mos_ssm->getPts();
	if(mos_use_norm_corners == 2){
		mos_ssm->applyWarpToPts(mos_pts, mos_ssm->getPts(), norm_warp);
	}
	mtf::PixValT mos_patch = Map<mtf::VectorXc>(input->getFrame().data, img_size).cast<double>();
	mtf::PixValT prev_patch = mos_patch;
	mtf::utils::writePixelsToImage(mos_img, mos_patch, mos_pts, n_channels, mos_mask);
	std::string mos_disp_win = cv::format("Mosaic :: %s", seq_name.c_str()), mos_mask_win = "Mask",
		mos_patch_win = "Current Patch", img_win = "Current Image";
	double mos_resize_factor;
	if(enable_visualization){
		double resize_factor_x = static_cast<double>(mos_disp_width) / static_cast<double>(mos_img.cols);
		double resize_factor_y = static_cast<double>(mos_disp_height) / static_cast<double>(mos_img.rows);
		mos_resize_factor = resize_factor_x < resize_factor_y ? resize_factor_x : resize_factor_y;
		mos_disp_width = static_cast<int>(mos_width * mos_resize_factor);
		mos_disp_height = static_cast<int>(mos_height * mos_resize_factor);
		mos_disp_img.create(mos_disp_height, mos_disp_width, mos_img_type);
		mos_disp_img.setTo(cv::Scalar(0));
		if(mos_use_write_mask && mos_show_mask){
			mos_mask_disp.create(mos_disp_height, mos_disp_width, CV_8UC1);
			mos_mask_disp.setTo(cv::Scalar(0));
			cv::namedWindow(mos_mask_win);
		}
		if(mos_show_patch){ cv::namedWindow(mos_patch_win); }
		cv::namedWindow(img_win);
		cv::namedWindow(mos_disp_win);
		printf("Using displayed image of size: %d x %d\n", mos_disp_img.cols, mos_disp_img.rows);
	}

	VectorXd curr_warp(mos_ssm->getStateSize());

	while(input->update()){

		mos_patch = Map<mtf::VectorXc>(input->getFrame().data, img_size).cast<double>();
		if(mos_use_norm_corners == 3){
			cv::Mat temp;
			mtf::utils::writePixelsToImage(mos_prev_tracked_img, prev_patch, mos_ssm->getPts(), n_channels, temp);
			mtf::utils::writePixelsToImage(mos_curr_tracked_img, mos_patch, mos_ssm->getPts(), n_channels, temp);
			if(enable_visualization && mos_show_tracked_img){
				//! resize the mosaic image to display on screen
				cv::resize(mos_prev_tracked_img, mos_disp_img, mos_disp_img.size());
				mtf::utils::drawRegion(mos_disp_img, curr_corners_norm_3*mos_resize_factor, CV_RGB(0, 255, 0), 2);
				cv::imshow("mos_prev_tracked_img", mos_disp_img);
				cv::resize(mos_curr_tracked_img, mos_disp_img, mos_disp_img.size());
				mtf::utils::drawRegion(mos_disp_img, curr_corners_norm_3*mos_resize_factor, CV_RGB(0, 255, 0), 2);
				cv::imshow("mos_curr_tracked_img", mos_disp_img);
			}
			if(mos_inv_tracking){
				pre_proc.update(mos_curr_tracked_img);
				tracker->initialize(curr_corners_norm_3);
				pre_proc.update(mos_prev_tracked_img);
				tracker->update();
				mos_ssm->estimateWarpFromCorners(curr_warp, curr_corners_norm_3, tracker->getRegion());
				curr_corners_norm_3 = tracker->getRegion().clone();
			} else{
				pre_proc.update(mos_prev_tracked_img);
				tracker->initialize(curr_corners_norm_3);
				pre_proc.update(mos_curr_tracked_img);
				tracker->update();
				VectorXd inv_warp = mos_ssm->estimateWarpFromCorners(curr_corners_norm_3, tracker->getRegion());
				mos_ssm->invertState(curr_warp, inv_warp);
				curr_corners_norm_3 = mos_ssm->applyWarpToCorners(curr_corners_norm_3, curr_warp);
			}
			curr_mos_location = mos_ssm->applyWarpToCorners(curr_mos_location, curr_warp);
			mos_ssm->setCorners(curr_mos_location);
			prev_patch = mos_patch;
		} else{
			pre_proc.update(input->getFrame());
			if(mos_inv_tracking){
				tracker->initialize(pre_proc.getFrame(), init_corners);
				tracker->update(prev_proc_frame);
				mos_ssm->estimateWarpFromCorners(curr_warp, init_corners, tracker->getRegion());
				prev_proc_frame = pre_proc.getFrame().clone();
			} else{
				tracker->update();
				VectorXd inv_warp(mos_ssm->getStateSize());
				mos_ssm->estimateWarpFromCorners(inv_warp, init_corners, tracker->getRegion());
				mos_ssm->invertState(curr_warp, inv_warp);
				tracker->initialize(init_corners);
			}
			if(mos_use_norm_corners == 1){
				mos_ssm->applyWarpToCorners(curr_mos_location_norm, prev_mos_location_norm, curr_warp);
				mos_ssm->applyWarpToCorners(curr_mos_location, curr_mos_location_norm, norm_warp);
				mos_ssm->setCorners(curr_mos_location);
			} else if(mos_use_norm_corners == 2){
				mos_ssm->compositionalUpdate(curr_warp);
				mos_ssm->applyWarpToCorners(curr_mos_location, mos_ssm->getCorners(), norm_warp);
			} else{
				mos_ssm->applyWarpToCorners(curr_mos_location, prev_mos_location, curr_warp);
				mos_ssm->setCorners(curr_mos_location);
			}
		}

		//mos_ssm->setCorners(curr_mos_location);

		//! extract current patch
		//mos_pre_proc.update(input->getFrame());
		//mtf::PixValT eig_patch = mos_am->getPatch(init_pts);		
		if(mos_use_norm_corners == 2){
			mos_ssm->applyWarpToPts(mos_pts, mos_ssm->getPts(), norm_warp);
		} else{
			mos_pts = mos_ssm->getPts();
		}
		//! write the patch to the image at the warped locations given by the tracker
		mtf::utils::writePixelsToImage(mos_img, mos_patch, mos_pts, n_channels, mos_mask);

		prev_mos_location = curr_mos_location;
		prev_mos_location_norm = curr_mos_location_norm;

		if(enable_visualization){
			//! resize the mosaic image to display on screen
			cv::resize(mos_img, mos_disp_img, mos_disp_img.size());
			//! draw the location of the current warped bounding box given by the tracker
			mtf::utils::drawRegion(mos_disp_img, mtf::utils::Corners(curr_mos_location).mat()*mos_resize_factor,
				CV_RGB(0, 255, 0), 2);			

			if(mos_show_grid){
				mtf::utils::drawGrid(mos_disp_img, mtf::PtsT(mos_pts*mos_resize_factor),
					mos_ssm->getResX(), mos_ssm->getResY());
			}
			if(mos_use_norm_corners != 3){
				mtf::utils::drawRegion(input->getFrame(mtf::utils::MUTABLE), tracker->getRegion(), CV_RGB(255, 0, 0), 2);
			}

			putText(input->getFrame(mtf::utils::MUTABLE), cv_format("Frame %d of %d", input->getFrameID() + 1, input->getNFrames()),
				cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.50, CV_RGB(0, 255, 0));
			if(mos_use_write_mask && mos_show_mask){
				cv::resize(mos_mask, mos_mask_disp, mos_mask_disp.size());
				cv::imshow(mos_mask_win, mos_mask_disp);
			}
			cv::imshow(mos_disp_win, mos_disp_img);
			cv::imshow(img_win, input->getFrame());
			if(mos_show_patch){
				cv::imshow(mos_patch_win, mtf::utils::reshapePatch(mos_patch, mos_am->getResY(), mos_am->getResX(),
					n_channels));
			}
			if(cv::waitKey(1) == 27){ break; }
		} else if(input->getFrameID() % 100 == 0){
			printf("Done %d frames\n", input->getFrameID());
		}
	}
	if(enable_visualization){ cv::destroyAllWindows(); }
	if(mos_save_img){
		if(mos_out_fname.empty()){
			mos_out_fname = cv::format("mosaic_%s_%s_%s_%s_inv%d_norm%d_mask%d_%ld.%s",
				seq_name.c_str(), mtf_sm, mtf_am, mtf_ssm, mos_inv_tracking,
				mos_use_norm_corners, mos_use_write_mask, time(0), mos_out_fmt.c_str());
		}
		printf("Writing mosaic image to %s\n", mos_out_fname.c_str());
		cv::imwrite(mos_out_fname, mos_img);
	}
	return 0;
}

