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

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"

#include <vector>
#include <memory>
#include <limits>

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

cv::Mat  rearrangeIntoRegion(const cv::Mat &region_corners){
	cv::Mat region_rect = utils::Corners(utils::getBestFitRectangle<double>(region_corners)).mat();
	cv::Mat rearrange_idx(region_corners.cols, 1, CV_32S);

	for(int corner_id = 0; corner_id < region_corners.cols; ++corner_id){
		double min_dist = std::numeric_limits<double>::infinity();
		int min_idx = 0;
		double x1 = region_corners.at<double>(0, corner_id);
		double y1 = region_corners.at<double>(1, corner_id);
		for(int rect_id = 0; rect_id < region_rect.cols; ++rect_id){
			double x2 = region_rect.at<double>(0, rect_id);
			double y2 = region_rect.at<double>(1, rect_id);
			double dx = x1 - x2, dy = y1 - y2;
			double dist = dx*dx + dy*dy;
			if(dist < min_dist){
				min_dist = dist;
				min_idx = rect_id;
			}
		}
		rearrange_idx.at<int>(corner_id) = min_idx;
	}
	return rearrange_idx;
}

int main(int argc, char * argv[]) {
	printf("\nStarting QR tracker...\n");

	// *************************************************************************************************** //
	// ********************************** read configuration parameters ********************************** //
	// *************************************************************************************************** //

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

	// *********************************************************************************************** //
	// ********************************** initialize input pipeline ********************************** //
	// *********************************************************************************************** //

	Input_ input(getInput(pipeline));
	if(!input->initialize()){
		printf("Pipeline could not be initialized successfully. Exiting...\n");
		return EXIT_FAILURE;
	}

	Detector_ qr_detector(dynamic_cast<Detector_::element_type*>(mtf::getTracker("feat", "ncc", qr_detector_ssm.c_str(), "0")));
	if(!qr_detector){
		throw std::invalid_argument("QR detector could not be created successfully\n");
	}
	PreProc_ qr_pre_proc = getPreProc(qr_detector->inputType(), pre_proc_type);
	if(qr_input.empty()){
		throw std::invalid_argument("No input files provided");
	}
	std::vector<cv::Mat> qr_images;
	for(auto input_fname : qr_input){
		qr_images.push_back(cv::imread(qr_root_dir + "/" + input_fname));
	}
	//cv::imshow("c1", qr_images[0]);
	//cv::imshow("c2", qr_images[1]);
	//cv::imshow("c3", qr_images[2]);
	//cv::imshow("c4", qr_images[3]);
	//cv::imshow("pen", qr_images[4]);
	//cv::waitKey(0);
	//cv::destroyAllWindows();

	for(int img_id = 0; img_id < qr_images.size(); ++img_id){
		qr_pre_proc->initialize(qr_images[img_id]);
		qr_images[img_id] = qr_pre_proc->getFrame().clone();
	}

	int n_qr_needed = qr_images.size();
	cv::Mat qr_region(2, n_qr_needed, CV_64FC1);

	vector<Tracker_> trackers;
	vector<PreProc_> pre_procs;
	cv::Mat mask(input->getHeight(), input->getWidth(), CV_8U);
	mask.setTo(255);
	// ********************************************************************************************** //
	// *************************************** start tracking ! ************************************* //
	// ********************************************************************************************** //
	bool detector_needs_initializing = true;
	bool corners_need_rearranging = true;
	while(true) {
		if(!input->update()){
			printf("Frame %d could not be read from the input pipeline\n", input->getFrameID() + 1);
			break;
		}
		qr_pre_proc->update(input->getFrame(), input->getFrameID());
		if(!qr_images.empty()){
			if(detector_needs_initializing){
				cv::Mat init_qr_corners(utils::Corners(
					cv::Rect_<int>(0, 0, qr_images[0].cols, qr_images[0].rows)).mat());
				qr_detector->initialize(qr_images.back(), init_qr_corners);
				qr_detector->setImage(qr_pre_proc->getFrame());
				detector_needs_initializing = false;
			}
			cv::Mat curr_qr_corners;
			bool qr_found = qr_detector->detect(mask, curr_qr_corners);
			if(qr_found){
				cv::Rect_<double> curr_qr_rect = utils::getBestFitRectangle<double>(curr_qr_corners);
				if(curr_qr_rect.width > qr_min_size && curr_qr_rect.height > qr_min_size &&
					!isDuplicate(utils::Corners(curr_qr_rect).mat(), trackers)){
					printf("\n\n Found a match....\n\n");
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

					tracker->initialize(utils::Corners(curr_qr_rect).mat());
					utils::printMatrix<double>(tracker->getRegion(), "QR code detected at");
					utils::drawRegion(input->getFrame(MUTABLE), tracker->getRegion(),
						cv::Scalar(0, 0, 255));
					cv::imshow("New QR Code Detected", input->getFrame(MUTABLE));
					cv::waitKey(500);
					cv::destroyWindow("New QR Code Detected");
					pre_procs.push_back(pre_proc);
					trackers.push_back(Tracker_(tracker));
					qr_images.pop_back();
					detector_needs_initializing = true;
				}
			}
		}
		mask.setTo(255);
		for(int tracker_id = 0; tracker_id < trackers.size(); ++tracker_id) {
			pre_procs[tracker_id]->update(input->getFrame(), input->getFrameID());
			trackers[tracker_id]->update();
			cv::Rect curr_location_rect = utils::getBestFitRectangle<int>(trackers[tracker_id]->getRegion(),
				input->getWidth(), input->getHeight());
			if(curr_location_rect.height>0 && curr_location_rect.width > 0){
				mask(curr_location_rect).setTo(0);
			}
			utils::drawRegion(input->getFrame(MUTABLE), trackers[tracker_id]->getRegion());
		}
		if(trackers.size() > 0){
			if(n_qr_needed == trackers.size()){
				for(int qr_id = 0; qr_id < n_qr_needed; ++qr_id) {
					cv::Point2d centroid = utils::getCentroid(trackers[qr_id]->getRegion());
					qr_region.at<double>(0, qr_id) = centroid.x;
					qr_region.at<double>(1, qr_id) = centroid.y;
				}
				if(corners_need_rearranging){
					cv::Mat rearrange_idx = rearrangeIntoRegion(qr_region);
					corners_need_rearranging = false;
					auto trackers_copy(trackers);
					auto qr_region_copy = qr_region.clone();
					for(int qr_id = 0; qr_id < n_qr_needed; ++qr_id) {
						int idx = rearrange_idx.at<int>(qr_id);
						trackers[idx] = trackers_copy[qr_id];
						qr_region.at<double>(0, idx) = qr_region_copy.at<double>(0, qr_id);
						qr_region.at<double>(1, idx) = qr_region_copy.at<double>(1, qr_id);
					}
				}
				utils::drawRegion(input->getFrame(MUTABLE), qr_region, cv::Scalar(255, 0, 0));
			}
			//else{
			//	printf("n_trackers: %d\n", trackers.size());
			//}
		}
		cv::imshow("QRTracker", input->getFrame(MUTABLE));
		if(cv::waitKey(1) == 27){ break; }
	}
	cv::destroyAllWindows();
	pre_procs.clear();
	trackers.clear();
	return EXIT_SUCCESS;
}

