//! main header that provides functions for creating trackers
#include "mtf/mtf.h"
//! tools for reading in images from various sources like image sequences, 
//! videos and cameras, pre processing these images and getting
//! objects to track either from ground truth or interactively from the user
#include "mtf/Tools/pipeline.h"
//! parameters for different modules
#include "mtf/Config/parameters.h"
//! general utilities for image drawing, etc.
#include "mtf/Utilities/miscUtils.h"
//! MTF specific exceptions
#include "mtf/Utilities/excpUtils.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "boost/filesystem/path.hpp"

#include <vector>
#include <memory>

using namespace std;
using namespace mtf::params;
namespace fs = boost::filesystem;

typedef mtf::utils::TrackErrT TrackErrT;
typedef std::shared_ptr<mtf::TrackerBase> Tracker_;
typedef std::unique_ptr<mtf::FeatureBase> Detector_;

bool isDuplicate(const cv::Mat &qr_location, const std::vector<Tracker_> &trackers){
	for(int tracker_id = 0; tracker_id < trackers.size(); ++tracker_id) {
		// center location error between the QR and tracker locations
		double dist = utils::getTrackingError(TrackErrT::CL, qr_location, trackers[tracker_id]->getRegion());
		if(dist < qr_duplicate_min_dist){ return true; }
	}
	return false;
}

int main(int argc, char * argv[]) {
	printf("\nStarting marker tracker...\n");

	// ********************************** read configuration parameters ********************************** //
	if(!readParams(argc, argv)){ return EXIT_FAILURE; }

#ifdef ENABLE_PARALLEL
	Eigen::initParallel();
#endif
	printf("*******************************\n");
	printf("Using parameters:\n");
	printf("qr_detector_ssm: %s\n", qr_detector_ssm.c_str());
	printf("qr_duplicate_min_dist: %f\n", qr_duplicate_min_dist);
	printf("qr_min_size: %f\n", qr_min_size);
	printf("mtf_sm: %s\n", mtf_sm);
	printf("mtf_am: %s\n", mtf_am);
	printf("mtf_ssm: %s\n", mtf_ssm);
	printf("*******************************\n");

	// ********************************** initialize input pipeline ********************************** //
	Input_ input(getInput(pipeline));
	if(!input->initialize()){
		printf("Pipeline could not be initialized successfully. Exiting...\n");
		return EXIT_FAILURE;
	}

	// ********************************** initialize QR detector ********************************** //
	Detector_ marker_detector(dynamic_cast<Detector_::element_type*>(
		mtf::getTracker("feat", "ncc", qr_detector_ssm.c_str(), "0")));
	if(!marker_detector){
		throw std::invalid_argument("Marker detector could not be created successfully\n");
	}
	PreProc_ marker_pre_proc = getPreProc(marker_detector->inputType(), pre_proc_type);
	if(qr_input.empty()){
		throw std::invalid_argument("No input files provided");
	}
	// ********************************** read QR images ********************************** //
	if(qr_n_markers < 0){
		qr_n_markers = qr_input.size();
	}
	printf("Reading %d marker images from %s\n", qr_n_markers, qr_root_dir.c_str());
	std::vector<cv::Mat> marker_images_orig, marker_images_proc;
	for(int i = 0; i < qr_n_markers; ++i){
		marker_images_orig.push_back(cv::imread(qr_root_dir + "/" + qr_input[i]));
		if(marker_images_orig.back().empty()){
			throw std::invalid_argument(cv_format("Input image %s could not be read",
				qr_input[i].c_str()));
		}
		//cv::imshow(input_fname, qr_images.back());
	}
	//cv::waitKey(0);
	//cv::destroyAllWindows();

	for(int img_id = 0; img_id < marker_images_orig.size(); ++img_id){
		marker_pre_proc->initialize(marker_images_orig[img_id]);
		marker_images_proc.push_back(marker_pre_proc->getFrame().clone());
	}
	

	marker_pre_proc->update(input->getFrame(), input->getFrameID());
	int img_width = marker_pre_proc->getWidth(), img_height = marker_pre_proc->getHeight();
	std::string marker_window = "Looking for marker", detector_window = "New Marker Detected",
		main_window = "MTF :: trackMarkers";
	vector<Tracker_> trackers;
	vector<PreProc_> pre_procs;
	bool reinit_detector = true, rearrange_corners = true, destroy_marker_window = true;

	// *************************************** start tracking ************************************* //
	while(true) {
		if(!input->update()){
			printf("Frame %d could not be read from the input pipeline\n", input->getFrameID() + 1);
			break;
		}
		cv::Mat detector_mask(img_height, img_width, CV_8U);
		detector_mask.setTo(255);
		for(int tracker_id = 0; tracker_id < trackers.size(); ++tracker_id) {
			pre_procs[tracker_id]->update(input->getFrame(), input->getFrameID());
			trackers[tracker_id]->update();
			cv::Rect curr_location_rect = utils::getBestFitRectangle<int>(
				trackers[tracker_id]->getRegion(), img_width, img_height);
			if(curr_location_rect.height > 0 && curr_location_rect.width > 0){
				detector_mask(curr_location_rect).setTo(0);
			}
		}
		if(!marker_images_proc.empty()){
			marker_pre_proc->update(input->getFrame(), input->getFrameID());
			if(reinit_detector){
				cv::Mat init_marker_corners(utils::Corners(
					cv::Rect_<int>(0, 0, marker_images_proc.back().cols, marker_images_proc.back().rows)).mat());
				marker_detector->initialize(marker_images_proc.back(), init_marker_corners);
				marker_detector->setImage(marker_pre_proc->getFrame());
				reinit_detector = false;
			}
			cv::Mat curr_marker_corners;
			bool marker_found = marker_detector->detect(detector_mask, curr_marker_corners);
			if(marker_found){
				cv::Rect_<double> curr_qr_rect = utils::getBestFitRectangle<double>(curr_marker_corners);
				if(curr_qr_rect.width > qr_min_size && curr_qr_rect.height > qr_min_size &&
					!isDuplicate(utils::Corners(curr_qr_rect).mat(), trackers)){
					//printf("\n\n Found a match....\n\n");
					mtf::TrackerBase *tracker = mtf::getTracker(mtf_sm, mtf_am, mtf_ssm, mtf_ilm);
					if(!tracker){
						printf("Tracker could not be created successfully\n");
						return EXIT_FAILURE;
					}
					PreProc_ pre_proc = getPreProc(pre_procs, tracker->inputType(), pre_proc_type);
					pre_proc->initialize(input->getFrame(), input->getFrameID());
					for(PreProc_ curr_obj = pre_proc; curr_obj; curr_obj = curr_obj->next){
						tracker->setImage(curr_obj->getFrame());
					}
					if(qr_init_with_rect){
						tracker->initialize(utils::Corners(curr_qr_rect).mat());
					} else{
						tracker->initialize(curr_marker_corners);
					}					
					utils::printMatrix<double>(tracker->getRegion(), "Marker detected at");
					utils::drawRegion(input->getFrame(MUTABLE), tracker->getRegion(),
						cv::Scalar(0, 0, 255));
					cv::imshow(detector_window, input->getFrame(MUTABLE));
					cv::waitKey(500);
					cv::destroyWindow(detector_window);
					pre_procs.push_back(pre_proc);
					trackers.push_back(Tracker_(tracker));
					marker_images_proc.pop_back();
					marker_images_orig.pop_back();
					reinit_detector = true;
				}
			} else{
				cv::imshow(marker_window, marker_images_orig.back());
			}
		} else if(destroy_marker_window){
			cv::destroyWindow(marker_window);
			destroy_marker_window = false;
		}
		for(int tracker_id = 0; tracker_id < trackers.size(); ++tracker_id) {
			utils::drawRegion(input->getFrame(MUTABLE), trackers[tracker_id]->getRegion());
		}
		if(qr_n_markers >= 3 && trackers.size() == qr_n_markers){
			cv::Mat qr_region(2, qr_n_markers, CV_64FC1);
			for(int qr_id = 0; qr_id < qr_n_markers; ++qr_id) {
				cv::Point2d centroid = utils::getCentroid(trackers[qr_id]->getRegion());
				qr_region.at<double>(0, qr_id) = centroid.x;
				qr_region.at<double>(1, qr_id) = centroid.y;
			}
			if(rearrange_corners){
				std::vector<int> rearrange_idx = utils::rearrangeIntoRegion(qr_region);
				utils::rearrange(trackers, rearrange_idx);
				utils::rearrange(pre_procs, rearrange_idx);
				//utils::printMatrix(Map<RowVectorXi>(rearrange_idx.data(), rearrange_idx.size()), "rearrange_idx", "%d");
				//std::cout << "qr_region before:\n" << qr_region<<"\n";
				utils::rearrangeCols<double>(qr_region, rearrange_idx);
				//std::cout << "qr_region after:\n" << qr_region << "\n";
				rearrange_corners = false;				
			}
			utils::drawRegion(input->getFrame(MUTABLE), qr_region, cv::Scalar(255, 0, 0));
		}
		//	printf("n_trackers: %d\n", trackers.size());
		cv::imshow(main_window, input->getFrame(MUTABLE));
		if(cv::waitKey(1) == 27){ break; }
	}
	cv::destroyAllWindows();
	pre_procs.clear();
	trackers.clear();
	return EXIT_SUCCESS;
}

