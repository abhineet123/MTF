#include "mtf/ThirdParty/CV3/CV3.h"

#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/excpUtils.h"

#define CV3_SM 0
#define CV3_AM 0
#define CV3_SSM 0
#define CV3_MAX_ITERS 30
#define CV3_SAMPLING_RES 50
#define CV3_LAMBDA 0.001
#define CV3_THRESH_GRAD 60
#define CV3_PYR_N_LEVELS 0
#define CV3_PYR_LEVEL_TO_STOP 1


CV3Params::CV3Params(SMType _sm_type, AMType _am_type,
	SSMType _ssm_type,
	int _max_iters,
	int _resx,
	int _resy,
	double _lambda,
	double _thresh_grad,
	int _pyr_n_levels,
	int _pyr_level_to_stop) :
	sm_type(_sm_type),
	am_type(_am_type),
	ssm_type(_ssm_type),
	max_iters(_max_iters),
	resx(_resx),
	resy(_resy),
	lambda(_lambda),
	thresh_grad(_thresh_grad),
	pyr_n_levels(_pyr_n_levels),
	pyr_level_to_stop(_pyr_level_to_stop){}

CV3Params::CV3Params(const CV3Params *params) :
sm_type(static_cast<SMType>(VISP_SM)),
am_type(static_cast<AMType>(VISP_AM)),
ssm_type(static_cast<SSMType>(VISP_SSM)),
max_iters(VISP_MAX_ITERS),
resx(VISP_SAMPLING_RES),
resy(VISP_SAMPLING_RES),
lambda(VISP_LAMBDA),
thresh_grad(VISP_THRESH_GRAD),
pyr_n_levels(VISP_PYR_N_LEVELS),
pyr_level_to_stop(VISP_PYR_LEVEL_TO_STOP){
	if(params){
		sm_type = params->sm_type;
		am_type = params->am_type;
		ssm_type = params->ssm_type;

		max_iters = params->max_iters;
		resx = params->resx;
		resy = params->resy;
		lambda = params->lambda;
		thresh_grad = params->thresh_grad;
		pyr_n_levels = params->pyr_n_levels;
		pyr_level_to_stop = params->pyr_level_to_stop;
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
	default:
		throw utils::InvalidArgument(cv::format(
			"Invalid tracker type provided %d", static_cast<int>(_tracker_type)));
	}
}

CV3::CV3(const ParamType *cv3_params) : TrackerBase(),
params(cv3_params),
warp(nullptr), tracker(nullptr){
	std::string tracker_type_str = params.toString(params.tracker_type);

	printf("Using CV3 Tracker with:\n");
	printf("tracker_type: %s\n", tracker_type_str.c_str());
	printf("max_iters: %d\n", params.max_iters);
	printf("resx: %d\n", params.resx);
	printf("resy: %d\n", params.resy);
	printf("lambda: %f\n", params.lambda);
	printf("thresh_grad: %f\n", params.thresh_grad);
	printf("pyr_n_levels: %d\n", params.pyr_n_levels);
	printf("pyr_level_to_stop: %d\n", params.pyr_level_to_stop);

	tracker = cv::Tracker::create(tracker_type_str);
	cv_corners_mat.create(2, 4, CV_64FC1);

}
void CV3::setImage(const cv::Mat &img){
	curr_img_cv = img;
}

// overloaded variants that avoid passing the image repeatedly by sharing
// memory with the buffer where the images are read in by the input pipeline
void CV3::initialize(const cv::Mat &corners){
	cout << "CV3::initialize::corners:\n" << corners << "\n";

	setOptimalSamplingRatio(corners);
	std::vector<vpImagePoint> init_corners;
	init_corners.push_back(vpImagePoint(corners.at<double>(1, 0), corners.at<double>(0, 0)));
	init_corners.push_back(vpImagePoint(corners.at<double>(1, 1), corners.at<double>(0, 1)));
	init_corners.push_back(vpImagePoint(corners.at<double>(1, 2), corners.at<double>(0, 2)));// ends the first triangle
	init_corners.push_back(vpImagePoint(corners.at<double>(1, 2), corners.at<double>(0, 2)));// start the second triangle
	init_corners.push_back(vpImagePoint(corners.at<double>(1, 3), corners.at<double>(0, 3)));
	init_corners.push_back(vpImagePoint(corners.at<double>(1, 0), corners.at<double>(0, 0)));

	//vpImagePoint ip;
	//ip.set_ij();  init_corners.push_back(ip);
	//ip.set_ij(corners.at<double>(1, 1), corners.at<double>(0, 1));  init_corners.push_back(ip);
	//ip.set_ij(corners.at<double>(1, 2), corners.at<double>(0, 2));  init_corners.push_back(ip);// ends the first triangle
	//ip.set_ij(corners.at<double>(1, 2), corners.at<double>(0, 2));  init_corners.push_back(ip);// start the second triangle
	//ip.set_ij(corners.at<double>(1, 3), corners.at<double>(0, 3));  init_corners.push_back(ip);
	//ip.set_ij(corners.at<double>(1, 0), corners.at<double>(0, 0));  init_corners.push_back(ip);
	try{
		vpImage<unsigned char> curr_img_vp((unsigned char* const)curr_img_cv.data, curr_img_cv.rows, curr_img_cv.cols, true);
		tracker->initFromPoints(curr_img_vp, init_corners, false);
		updateCorners();
	} catch(const vpTrackingException &err){
		throw mtf::utils::InvalidTrackerState(err.what());
	}
}

void CV3::update(){
	try{
		vpImage<unsigned char> curr_img_vp((unsigned char* const)curr_img_cv.data, curr_img_cv.rows, curr_img_cv.cols, true);
		tracker->track(curr_img_vp);
		updateCorners();
	} catch(const vpTrackingException &err){
		throw mtf::utils::InvalidTrackerState(err.what());
	}
}

// return true if the tracker requires the raw RGB image
void CV3::updateCorners(){
	// Get the estimated parameters
	vpColVector p = tracker->getp();
	// Instantiate and get the reference zone
	vpTemplateTrackerZone zone_ref = tracker->getZoneRef();
	// Instantiate a warped zone
	vpTemplateTrackerZone zone_warped;
	// Update the warped zone given the tracker estimated parameters
	warp->warpZone(zone_ref, p, zone_warped);

	//cv_corners[0].x = zone_warped.getMinx();
	//cv_corners[0].y = zone_warped.getMiny();

	//cv_corners[1].x = zone_warped.getMaxx();
	//cv_corners[1].y = zone_warped.getMiny();

	//cv_corners[2].x = zone_warped.getMaxx();
	//cv_corners[2].y = zone_warped.getMaxy();

	//cv_corners[3].x = zone_warped.getMinx();
	//cv_corners[3].y = zone_warped.getMaxy();

	vpTemplateTrackerTriangle triangle;
	zone_warped.getTriangle(0, triangle);
	std::vector<vpImagePoint> corners;
	// Get the 3 triangle corners
	triangle.getCorners(corners);
	cv_corners_mat.at<double>(0, 0) = corners[0].get_j();
	cv_corners_mat.at<double>(1, 0) = corners[0].get_i();

	cv_corners_mat.at<double>(0, 1) = corners[1].get_j();
	cv_corners_mat.at<double>(1, 1) = corners[1].get_i();

	cv_corners_mat.at<double>(0, 2) = corners[2].get_j();
	cv_corners_mat.at<double>(1, 2) = corners[2].get_i();

	zone_warped.getTriangle(1, triangle);
	triangle.getCorners(corners);

	cv_corners_mat.at<double>(0, 3) = corners[1].get_j();
	cv_corners_mat.at<double>(1, 3) = corners[1].get_i();
}

void CV3::setRegion(const cv::Mat& corners){
	try{
		tracker->resetTracker();
		initialize(corners);
		updateCorners();
	} catch(const vpTrackingException &err){
		throw mtf::utils::InvalidTrackerState(err.what());
	}
}

