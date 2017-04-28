/**
application to generate synthetic sequences by warping an image using
random perturbations produced by different SSMs
*/

#include "mtf/mtf.h"

// tools for reading in images from various sources like image sequences, 
// videos and cameras as well as for pre processing them
#include "mtf/Tools/pipeline.h"
// general OpenCV tools for selecting objects, reading ground truth, etc.
#include "mtf/Tools/cvUtils.h"
#include "mtf/Config/parameters.h"
#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/warpUtils.h"
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

int main(int argc, char * argv[]) {

	printf("Starting MTF synthetic sequence generator...\n");

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
	printf("syn_show_output: %d\n", syn_show_output);
	printf("syn_ssm: %s\n", syn_ssm.c_str());
	printf("syn_ilm: %s\n", syn_ilm.c_str());
	printf("********************************\n");

	/* initialize pipeline*/
	Input_ input(getInput(pipeline));
	if(!input->initialize()){
		printf("Pipeline could not be initialized successfully\n");
		return EXIT_FAILURE;
	}
	if(syn_frame_id > 0){
		if(input->n_frames > 0 && syn_frame_id >= input->n_frames){
			printf("syn_frame_id: %d is larger than the maximum frame ID in the sequence: %d\n",
				syn_frame_id, input->n_frames - 1);
			return EXIT_FAILURE;
		}
		printf("Skipping to frame %d before initializing trackers...\n", syn_frame_id + 1);
		for(int frame_id = 0; frame_id < syn_frame_id; ++frame_id){
			if(!input->update()){
				printf("Frame %d could not be read from the input pipeline\n", input->getFrameID() + 1);
				return EXIT_FAILURE;
			}
		}
	}
	CVUtils cv_utils;
	//! need to read only one object
	n_trackers = 1;
	if(!getObjectsToTrack(cv_utils, input.get())){
		printf("Object(s) to be tracked could not be read\n");
		return EXIT_FAILURE;
	}

	cv::Mat init_corners;
	double size_x, size_y;
	if(syn_warp_entire_image){
		size_x = static_cast<double>(input->getWidth());
		size_y = static_cast<double>(input->getHeight());
		init_corners.create(2, 4, CV_64FC1);
		//! corners of the image itself
		init_corners.at<double>(0, 0) = 0;
		init_corners.at<double>(0, 1) = size_x - 1;
		init_corners.at<double>(0, 2) = size_x - 1;
		init_corners.at<double>(0, 3) = 0;
		init_corners.at<double>(1, 0) = 0;
		init_corners.at<double>(1, 1) = 0;
		init_corners.at<double>(1, 2) = size_y - 1;
		init_corners.at<double>(1, 3) = size_y - 1;
	} else{
		init_corners = cv_utils.getObj().corners.clone();
		size_x = 2*cv_utils.getObj().size_x;
		size_y = 2*cv_utils.getObj().size_y;
	}

	//mtf::utils::printMatrix<double>(init_corners, "init_corners");

	double font_size = 1.00;

	resx = size_x;
	resy = size_y;

	mtf::SSM ssm(mtf::getSSM(syn_ssm.c_str()));
	if(!ssm){
		printf("State space model could not be initialized");
		return EXIT_FAILURE;
	}
	ssm->initialize(init_corners);
	cv::Mat original_corners(2, 4, CV_64FC1);
	ssm->getCorners(original_corners);
	mtf::PtsT original_pts = ssm->getPts();

	if(syn_grayscale_img){
		printf("Generating grayscale images\n");
	} else{
		printf("Generating RGB images\n");
	}
	const char* am_type = syn_grayscale_img ? "ssd" : "ssd3";
	mtf::AM am(mtf::getAM(am_type, syn_ilm.c_str()));
	if(!am){
		printf("Appearance model could not be initialized");
		return EXIT_FAILURE;
	}
	int out_img_type = syn_grayscale_img ? CV_8UC1 : CV_8UC3;
	pre_proc_type = "none";
	PreProc_ pre_proc(getPreProc(am->inputType(), pre_proc_type));
	pre_proc->initialize(input->getFrame());
	am->setCurrImg(pre_proc->getFrame());
	am->initializePixVals(ssm->getPts());
	const mtf::PixValT &original_patch = am->getInitPixVals();

	//mtf::utils::printMatrix(original_patch, "original_patch");
	//mtf::utils::printMatrix<double>(original_corners, "original_corners");

	//! generate random warp
	mtf::vectorvd syn_ssm_sigma, syn_ssm_mean;
	VectorXd state_sigma;
	if(syn_pix_sigma > 0){
		state_sigma.resize(ssm->getStateSize());
		ssm->estimateStateSigma(state_sigma, syn_pix_sigma);
	} else{
		getSamplerParams(syn_ssm_sigma, syn_ssm_mean, syn_ssm_sigma_ids, syn_ssm_mean_ids, "Synthetic");
		state_sigma = Map<const VectorXd>(syn_ssm_sigma[0].data(), syn_ssm_sigma[0].size());
	}
	VectorXd state_mean = VectorXd::Zero(ssm->getStateSize());
	VectorXd ssm_perturbation(ssm->getStateSize()), am_perturbation;
	VectorXd inv_ssm_perturbation(ssm->getStateSize()), inv_am_perturbation;
	ssm->initializeSampler(state_sigma, state_mean);

	const bool using_ilm = am->getStateSize() ? true : false;

	if(using_ilm){
		mtf::vectorvd syn_am_sigma, syn_am_mean;
		getAMSamplerParams(syn_am_sigma, syn_am_mean, syn_am_sigma_ids, syn_am_mean_ids, "Synthetic");
		VectorXd am_state_sigma = Map<const VectorXd>(syn_am_sigma[0].data(), syn_am_sigma[0].size());
		VectorXd am_state_mean = VectorXd::Zero(am->getStateSize());
		mtf::utils::printMatrix(am_state_sigma.transpose(), "am_state_sigma");
		mtf::utils::printMatrix(am_state_mean.transpose(), "am_state_mean");
		am_perturbation.resize(am->getStateSize());
		inv_am_perturbation.resize(am->getStateSize());
		am->initializeSimilarity();
		am->initializeSampler(am_state_sigma, am_state_mean);
	}
	std::string out_seq_name = getSyntheticSeqName();
	std::string syn_gt_path = cv::format("%s/Synthetic/%s.txt", db_root_path.c_str(), out_seq_name.c_str());
	printf("Writing synthetic sequence GT to: %s\n", syn_gt_path.c_str());

	printf("n_pts: %d\n", ssm->getNPts());
	printf("n_pix: %ld\n", original_patch.size());

	if(syn_continuous_warping){
		printf("Using continuous warping\n");
	}
	if(syn_use_inv_warp){
		printf("Using inverse warping method\n");
	}

	cv::Mat warped_bounding_box(2, 4, CV_64FC1), original_bounding_box(2, 4, CV_64FC1);
	int nearest_pt_ids[4];
	if(syn_warp_entire_image){
		printf("Warping entire image\n");
		for(int corner_id = 0; corner_id < 4; ++corner_id){
			nearest_pt_ids[corner_id] = mtf::utils::getNearestPt(cv_utils.getObj().corners.at<double>(0, corner_id),
				cv_utils.getObj().corners.at<double>(1, corner_id), original_pts, ssm->getNPts());

			original_bounding_box.at<double>(0, corner_id) = original_pts(0, nearest_pt_ids[corner_id]);
			original_bounding_box.at<double>(1, corner_id) = original_pts(1, nearest_pt_ids[corner_id]);
		}
	} else{
		printf("Warping only object region\n");
		original_bounding_box = cv_utils.getObj().corners;
	}
	cv::Mat out_img;
	if(syn_grayscale_img){
		//! first frame in the synthetic sequence is the grayscale version of the original image 
		out_img.create(input->getHeight(), input->getWidth(), CV_8UC1);
		cv::cvtColor(input->getFrame(), out_img, CV_BGR2GRAY);
	} else{
		//! original image is the first frame in the synthetic sequence
		out_img = input->getFrame().clone();
	}
	cv::Mat warped_img(input->getHeight(), input->getWidth(), out_img_type);
	PreProc_ pre_proc_warped;
	if(using_ilm && syn_warp_entire_image && syn_am_on_obj){
		printf("Applying AM tansformations only to the object\n");
		pre_proc_warped = getPreProc(am->inputType(), pre_proc_type);
		pre_proc_warped->initialize(warped_img);
	}
	if(syn_add_noise){
		printf("Adding Gaussian distributed noise with mean %5.2f and sigma %5.2f\n",
			syn_noise_mean, syn_noise_sigma);
		//! add Gaussian distributed random noise to the first image
		mtf::utils::addGaussianNoise(out_img, out_img, am->getNChannels(),
			syn_noise_mean, syn_noise_sigma);
	}
	std::string out_dir = cv::format("%s/Synthetic/%s", db_root_path.c_str(), out_seq_name.c_str());
	cv::VideoWriter output;
	vector<int> compression_params;
	if(syn_save_as_video){
		std::string out_file = cv::format("%s.avi", out_dir.c_str());
		printf("Saving output to MJPG video file: %s with %d fps\n", out_file.c_str(), syn_video_fps);
		output.open(out_file, CV_FOURCC('M', 'J', 'P', 'G'), syn_video_fps, input->getFrame().size());
		output.write(out_img);
	} else{
		//! for OpenCV imwrite function
		printf("Saving output as JPEG images at  %s with quality %d\n", out_dir.c_str(), syn_jpg_quality);
		if(!fs::exists(out_dir)){
			printf("Output directory does not exist. Creating it...\n");
			fs::create_directories(out_dir);
		}
		compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
		compression_params.push_back(syn_jpg_quality);
		cv::imwrite(cv::format("%s/frame%05d.jpg", out_dir.c_str(), 1), out_img, compression_params);
	}
	FILE *syn_gt_fid = fopen(syn_gt_path.c_str(), "w");
	mtf::utils::writeCorners(syn_gt_fid, original_bounding_box, 0, true);
	const char* warped_img_win_name = "warped_img";
	cv::Mat write_mask;

	double corner_change_norm_avg = 0, corner_change_norm_std=0;
	printf("Sequence generation started...\n");
	for(int frame_id = 1; frame_id <= syn_n_frames; ++frame_id){
		ssm->generatePerturbation(ssm_perturbation);
		/**
		the SSM will NOT resize any of its output arguments or even check
		if they have the correct size (except in debugging version) so passing
		an argument with iincorrect size will lead to a segmentation fault
		*/		
		ssm->invertState(inv_ssm_perturbation, ssm_perturbation);
		//! apply warp perturbation to the SSM
		ssm->compositionalUpdate(ssm_perturbation);
		mtf::PtsT warped_pts = ssm->getPts();

		if(using_ilm){
			am->generatePerturbation(am_perturbation);
			am->invertState(inv_am_perturbation, am_perturbation);
			//cout << "am_perturbation: " << am_perturbation.transpose() << "\n";
			am->updateState(am_perturbation);
		}

		//mtf::utils::printMatrixToFile(warped_pts, "warped_pts", "log/syn_log.txt");
		if(syn_warp_entire_image){
			/**
			if the entire image has been warped, we need to find the points in the image-wide grid
			that are nearest to the original bounding box corners and use the warped points corresponding to
			these as the warped bounding box corners
			*/
			for(int corner_id = 0; corner_id < 4; ++corner_id){
				warped_bounding_box.at<double>(0, corner_id) = warped_pts(0, nearest_pt_ids[corner_id]);
				warped_bounding_box.at<double>(1, corner_id) = warped_pts(1, nearest_pt_ids[corner_id]);
			}
		} else{
			/**
			if only the bounding box has been warped, SSM corners are identical to the warped bounding box corners
			*/
			ssm->getCorners(warped_bounding_box);
		}

		if(!syn_continuous_warping){
			//! reset the SSM to its previous state
			ssm->compositionalUpdate(inv_ssm_perturbation);
			if(using_ilm){
				//! reset the AM to its previous state
				am->updateState(inv_am_perturbation);
			}
			//utils::printMatrix(ssm->getCorners(), "ssm_corners");
		}		
		if(syn_grayscale_img){
			warped_img.setTo(cv::Scalar(0));
		} else{
			warped_img.setTo(cv::Vec3b(0, 0, 0));
		}
		
		if(syn_use_inv_warp){
			if(syn_continuous_warping){
				//! reset the SSM to its previous state
				ssm->compositionalUpdate(inv_ssm_perturbation);
				if(using_ilm){
					//! reset the AM to its previous state
					am->updateState(inv_am_perturbation);
				}
			}

			//! apply inverse of the warp perturbation to the SSM
			ssm->compositionalUpdate(inv_ssm_perturbation);
			if(using_ilm){
				//! reset the AM to its previous state
				am->updateState(inv_am_perturbation);
			}

			am->updatePixVals(ssm->getPts());	
			if(!syn_am_on_obj && using_ilm){
				am->updateSimilarity();
			}
			if(syn_warp_entire_image){
				mtf::utils::generateInverseWarpedImg(warped_img, am->getCurrPixVals(),
					am->getImgWidth(), am->getImgHeight(), ssm->getNPts(),
					syn_show_output, warped_img_win_name);
				if(syn_am_on_obj && using_ilm){
					mtf::PtsT warped_box_pts = mtf::utils::getPtsFromCorners(
						warped_bounding_box, am->getResX(), am->getResY());
					pre_proc_warped->update(warped_img);
					am->setCurrImg(pre_proc_warped->getFrame());
					am->updatePixVals(warped_box_pts);
					am->updateSimilarity();
					mtf::utils::writePixelsToImage(warped_img, am->getCurrPixVals(),
						warped_box_pts, am->getNChannels(), write_mask);
					am->setCurrImg(pre_proc->getFrame());
				}
			} else {
				am->getCurrImg().convertTo(warped_img, warped_img.type());
				cv::imshow("current patch", mtf::utils::reshapePatch(am->getCurrPixVals(), 
					am->getResY(), am->getResX(), am->getNChannels()));
				if(cv::waitKey(1) == 27){ break; }
				mtf::utils::writePixelsToImage(warped_img, am->getCurrPixVals(), 
					original_pts, am->getNChannels(), write_mask);
			}
			//mtf::utils::generateInverseWarpedImg(warped_img, ssm->getPts(),
			//	am->getCurrImg(), original_pts, am->getImgWidth(), am->getImgHeight(),
			//	ssm->getNPts(), syn_show_output, warped_img_win_name);

			//! reset the SSM to the previous state
			ssm->compositionalUpdate(ssm_perturbation);
			if(using_ilm){
				//! reset the AM to its previous state
				am->updateState(am_perturbation);
			}
		} else{
			if(syn_warp_entire_image){
				cv::Mat warped_corners(2, 4, CV_64FC1);
				ssm->getCorners(warped_corners);
				mtf::utils::generateWarpedImg(warped_img, warped_corners, warped_pts,
					original_patch, am->getCurrImg(), am->getImgWidth(), am->getImgHeight(),
					ssm->getNPts(), syn_background_type, syn_show_output, warped_img_win_name);
			} else {
				am->getCurrImg().convertTo(warped_img, warped_img.type());
				mtf::utils::writePixelsToImage(warped_img, am->getCurrPixVals(), warped_pts,
					am->getNChannels(), write_mask);
			}
		}
		if(syn_add_noise){
			//! add Gaussian distributed random noise to the warped image
			mtf::utils::addGaussianNoise(warped_img, warped_img, am->getNChannels(),
				syn_noise_mean, syn_noise_sigma);
		}
		if(syn_show_output){
			cv::Mat original_img = input->getFrame().clone();
			mtf::utils::drawRegion(original_img, original_bounding_box,
				cv::Scalar(0, 255, 0), line_thickness, nullptr, font_size,
				show_corner_ids, 1 - show_corner_ids);
			mtf::utils::drawRegion(original_img, warped_bounding_box,
				cv::Scalar(0, 0, 255), line_thickness, nullptr, font_size,
				show_corner_ids, 1 - show_corner_ids);
			cv::Mat warped_img_annotated = warped_img.clone();
			mtf::utils::drawRegion(warped_img_annotated, warped_bounding_box,
				cv::Scalar(0, 0, 255), line_thickness, nullptr, font_size,
				show_corner_ids, 1 - show_corner_ids);
			cv::imshow("original_img", original_img);
			cv::imshow(warped_img_win_name, warped_img_annotated);
			if(cv::waitKey(500) == 27){ break; }
		}
		if(syn_save_as_video){
			output.write(warped_img);
		} else{
			cv::imwrite(cv::format("%s/frame%05d.jpg", out_dir.c_str(), frame_id + 1),
				warped_img, compression_params);
		}	

		mtf::utils::writeCorners(syn_gt_fid, warped_bounding_box, frame_id);
		double corner_change_norm = mtf::utils::getTrackingError(mtf::utils::TrackErrT::MCD, 
			original_bounding_box, warped_bounding_box);
		double delta = corner_change_norm - corner_change_norm_avg;
		corner_change_norm_avg += delta / static_cast<double>(frame_id);
		corner_change_norm_std += delta*(corner_change_norm - corner_change_norm_avg);
		if(frame_id % 50 == 0){
			printf("Done %d frames\t", frame_id);
			printf("corner_change_norm statistics: avg: %f std: %f\n",
				corner_change_norm_avg, sqrt(corner_change_norm_std / frame_id));
		}
	}
	corner_change_norm_std = sqrt(corner_change_norm_std/syn_n_frames);
	printf("corner_change_norm statistics: avg: %f std: %f\n",
		corner_change_norm_avg, corner_change_norm_std);
	if(syn_save_as_video){
		output.release();
	}
	fclose(syn_gt_fid);
	return EXIT_SUCCESS;
}

