#include "mtf/ThirdParty/CV3/CV3.h"
#include "mtf/Utilities/excpUtils.h"
#include "mtf/Utilities/miscUtils.h"

#define CV3_TRACKER_TYPE CV3Params::TrackerType::MIL

CV3Params::CV3Params(
	TrackerType _tracker_type) :
	tracker_type(_tracker_type){}

CV3Params::CV3Params(const CV3Params *params) :
tracker_type(CV3_TRACKER_TYPE){
	if(params){
		tracker_type = params->tracker_type;		
	}
}

std::string CV3Params::toString(TrackerType _tracker_type){
	switch(_tracker_type){
	case TrackerType::MIL:
		return "MIL";
	case TrackerType::BOOSTING:
		return "BOOSTING";
	case TrackerType::MEDIANFLOW:
		return "MEDIANFLOW";
	case TrackerType::TLD:
		return "TLD";
	case TrackerType::KCF:
		return "KCF";
	case TrackerType::GOTURN:
		return "GOTURN";
	default:
		throw mtf::utils::InvalidArgument(cv::format(
			"Invalid tracker type provided %d", static_cast<int>(_tracker_type)));
	}
}

CV3::CV3(const ParamType *cv3_params) : TrackerBase(),
params(cv3_params){
	std::string tracker_type_str = params.toString(params.tracker_type);

	printf("Using CV3 Tracker with:\n");
	printf("tracker_type: %s\n", tracker_type_str.c_str());

#if (CV_MINOR_VERSION < 3)
	tracker = cv::Tracker::create(tracker_type_str);
#else
	switch(_tracker_type){
	case TrackerType::MIL:
		tracker = TrackerMIL::create();
	case TrackerType::BOOSTING:
		tracker = TrackerBoosting::create();
	case TrackerType::MEDIANFLOW:
		tracker = TrackerMedianFlow::create();
	case TrackerType::TLD:
		tracker = TrackerTLD::create();
	case TrackerType::KCF:
		tracker = TrackerKCF::create();
	case TrackerType::GOTURN:
		tracker = TrackerGOTURN::create();
	default:
		throw mtf::utils::InvalidArgument(cv::format(
			"Invalid tracker type provided %d", static_cast<int>(_tracker_type)));
	}
#endif
	cv_corners_mat.create(2, 4, CV_64FC1);

}
void CV3::setImage(const cv::Mat &img){
	curr_img_cv = img;
}

// overloaded variants that avoid passing the image repeatedly by sharing
// memory with the buffer where the images are read in by the input pipeline
void CV3::initialize(const cv::Mat &corners){
	std::cout << "CV3::initialize::corners:\n" << corners << "\n";
	cv::Rect2d bbox = mtf::utils::getBestFitRectangle<double>(corners);
	try{
		tracker->init(curr_img_cv, bbox);
	} catch(const std::exception &err){
		throw mtf::utils::InvalidTrackerState(err.what());
	}
}

void CV3::update(){
	try{
		cv::Rect2d bbox;
		bool ok = tracker->update(curr_img_cv, bbox);
		if(!ok){
			throw mtf::utils::InvalidTrackerState("Tracker update failed");
		}
		cv_corners_mat.at<double>(0, 0) = cv_corners_mat.at<double>(0, 3) = bbox.x;
		cv_corners_mat.at<double>(1, 0) = cv_corners_mat.at<double>(1, 1) = bbox.y;
		cv_corners_mat.at<double>(0, 1) = cv_corners_mat.at<double>(0, 2) = bbox.x + bbox.width;
		cv_corners_mat.at<double>(1, 2) = cv_corners_mat.at<double>(1, 3) = bbox.y + bbox.height;

	} catch(const std::exception &err){
		throw mtf::utils::InvalidTrackerState(err.what());
	}

}


