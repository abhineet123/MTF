#include "mtf/ThirdParty/ViSP/ViSP.h"
#include <visp3/core/vpImage.h>
#include <visp3/tt/vpTemplateTrackerSSD.h>
#include <visp3/tt/vpTemplateTrackerSSDForwardAdditional.h>
#include <visp3/tt/vpTemplateTrackerSSDForwardCompositional.h>
#include <visp3/tt/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp3/tt/vpTemplateTrackerSSDESM.h>
#include <visp3/tt/vpTemplateTrackerZNCCForwardAdditional.h>
#include <visp3/tt/vpTemplateTrackerZNCCInverseCompositional.h>

#include <visp3/tt/vpTemplateTrackerWarpAffine.h>
#include <visp3/tt/vpTemplateTrackerWarpHomography.h>
#include <visp3/tt/vpTemplateTrackerWarpHomographySL3.h>
#include <visp3/tt/vpTemplateTrackerWarpSRT.h>
#include <visp3/tt/vpTemplateTrackerWarpTranslation.h>
#include <visp3/tt/vpTemplateTrackerWarpRT.h>

#ifdef VISP_HAVE_MODULE_TT_MI
#  include <visp3/tt_mi/vpTemplateTrackerMIESM.h>
#  include <visp3/tt_mi/vpTemplateTrackerMIForwardAdditional.h>
#  include <visp3/tt_mi/vpTemplateTrackerMIForwardCompositional.h>
#  include <visp3/tt_mi/vpTemplateTrackerMIInverseCompositional.h>
#endif

#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/excpUtils.h"

#define VISP_SM 0
#define VISP_AM 0
#define VISP_SSM 0
#define  VISP_MAX_ITERS 30
#define  VISP_SAMPLING_RES 50
#define  VISP_LAMBDA 0.001
#define  VISP_THRESH_GRAD 60
#define  VISP_PYR_N_LEVELS 0
#define  VISP_PYR_LEVEL_TO_STOP 1


ViSPParams::ViSPParams(SMType _sm_type, AMType _am_type,
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

ViSPParams::ViSPParams(const ViSPParams *params) :
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

ViSP::ViSP(const ParamType *visp_params) : TrackerBase(),
params(visp_params),
warp(nullptr), tracker(nullptr){
	printf("Using ViSP Template Tracker with:\n");
	printf("max_iters: %d\n", params.max_iters);
	printf("resx: %d\n", params.resx);
	printf("resy: %d\n", params.resy);
	printf("lambda: %f\n", params.lambda);
	printf("thresh_grad: %f\n", params.thresh_grad);
	printf("pyr_n_levels: %d\n", params.pyr_n_levels);
	printf("pyr_level_to_stop: %d\n", params.pyr_level_to_stop);

	switch(params.sm_type) {
	case SMType::FCLK:
		printf("Using FCLK SM\n");
		break;
	case SMType::ICLK:
		printf("Using ICLK SM\n");
		break;
	case SMType::FALK:
		printf("Using FALK SM\n");
		break;
	case SMType::ESM:
		printf("Using ESM SM\n");
		break;
	default:
		throw std::invalid_argument("Invalid search method provided");
	}

	switch(params.ssm_type) {
	case SSMType::Homography:
		printf("Using Homography SSM\n");
		warp.reset(new vpTemplateTrackerWarpHomography);
		break;
	case SSMType::SL3:
		printf("Using SL3 SSM\n");
		warp.reset(new vpTemplateTrackerWarpHomographySL3);
		break;
	case SSMType::Affine:
		printf("Using Affine SSM\n");
		warp.reset(new vpTemplateTrackerWarpAffine);
		break;
	case SSMType::Similarity:
		printf("Using Similarity SSM\n");
		warp.reset(new vpTemplateTrackerWarpSRT);
		break;
	case SSMType::Translation:
		printf("Using Translation SSM\n");
		warp.reset(new vpTemplateTrackerWarpTranslation);
		break;
#ifdef VISP_HAVE_MODULE_TT_MI
	case SSMType::Isometry:
		printf("Using Isometry SSM\n");
		warp.reset(new vpTemplateTrackerWarpRT);
		break;
#endif
	default:
		throw std::invalid_argument("Invalid state space model provided");
	}

	switch(params.am_type) {
	case AMType::SSD:
		switch(params.sm_type) {
		case SMType::FCLK:
			tracker.reset(new vpTemplateTrackerSSDForwardCompositional(warp.get()));
			break;
		case SMType::ICLK:
			tracker.reset(new vpTemplateTrackerSSDInverseCompositional(warp.get()));
			break;
		case SMType::FALK:
			tracker.reset(new vpTemplateTrackerSSDForwardAdditional(warp.get()));
			break;
		case SMType::ESM:
			tracker.reset(new vpTemplateTrackerSSDESM(warp.get()));
			break;
		default:
			throw std::invalid_argument("Invalid search method provided for SSD");
		}
		printf("Using SSD AM\n");
		break;
	case AMType::ZNCC:
		switch(params.sm_type) {
		case SMType::FCLK:
			throw std::invalid_argument("Invalid search method provided for ZNCC");
		case SMType::ICLK:
			tracker.reset(new vpTemplateTrackerZNCCInverseCompositional(warp.get()));
			break;
		case SMType::FALK:
			tracker.reset(new vpTemplateTrackerZNCCForwardAdditional(warp.get()));
			break;
		case SMType::ESM:
			throw std::invalid_argument("Invalid search method provided for ZNCC");
		default:
			throw std::invalid_argument("Invalid search method provided for ZNCC");
		}
		printf("Using ZNCC AM\n");
		break;
	case AMType::MI:
		switch(params.sm_type) {
		case SMType::FCLK:
			tracker.reset(new vpTemplateTrackerMIForwardCompositional(warp.get()));
			break;
		case SMType::ICLK:
			tracker.reset(new vpTemplateTrackerMIInverseCompositional(warp.get()));
			break;
		case SMType::FALK:
			tracker.reset(new vpTemplateTrackerMIForwardAdditional(warp.get()));
			break;
		case SMType::ESM:
			tracker.reset(new vpTemplateTrackerMIESM(warp.get()));
			break;
		default:
			throw std::invalid_argument("Invalid search method provided for MI");
		}
		printf("Using MI AM\n");
		break;
	default:
		throw std::invalid_argument("Invalid appearance model provided");
	}


	tracker->setLambda(params.lambda);
	if(params.thresh_grad > 0) {
		tracker->setThresholdGradient(params.thresh_grad);
	}
	tracker->setIterationMax(params.max_iters);
	if(params.pyr_n_levels > 0) {
		tracker->setPyramidal(params.pyr_n_levels, params.pyr_level_to_stop);
	}

	cv_corners_mat.create(2, 4, CV_64FC1);

}
void ViSP::setImage(const cv::Mat &img){
	curr_img_cv = img;
}
void ViSP::setOptimalSamplingRatio(const cv::Mat &corners){

	cv::Rect_<double> best_fit_rect = mtf::utils::getBestFitRectangle<double>(corners);
	int size_x = static_cast<int>(best_fit_rect.width);
	int size_y = static_cast<int>(best_fit_rect.height);

	int n_pix_desired = params.resx*params.resy;

	int sampling_ratio_x = static_cast<int>(ceil(static_cast<double>(size_x) /
		static_cast<double>(params.resx)));
	int sampling_ratio_y = static_cast<int>(ceil(static_cast<double>(size_y) /
		static_cast<double>(params.resy)));

	int sampling_res_x = size_x / sampling_ratio_x;
	int sampling_res_y = size_y / sampling_ratio_y;
	int n_pix = sampling_res_x*sampling_res_y;
	int n_pix_diff = abs(n_pix_desired - n_pix);

	int n_pix_diff_inc_y = n_pix_diff + 1, n_pix_diff_inc_x = n_pix_diff + 1;
	printf("Candidates: \n");
	printf("Ratio: %d x %d :: Res: %d x %d n_pix: %d \n", sampling_ratio_x, sampling_ratio_y,
		sampling_res_x, sampling_res_y, n_pix);

	if(sampling_ratio_x > 1){
		int sampling_res_x_inc = size_x / (sampling_ratio_x - 1);
		int n_pix_inc_x = sampling_res_x_inc*sampling_res_y;
		n_pix_diff_inc_x = abs(n_pix_desired - n_pix_inc_x);
		printf("Ratio: %d x %d :: Res: %d x %d n_pix: %d \n", sampling_ratio_x - 1, sampling_ratio_y,
			sampling_res_x_inc, sampling_res_y, n_pix_inc_x);
	}
	if(sampling_ratio_y > 1){
		int sampling_res_y_inc = size_y / (sampling_ratio_y - 1);
		int n_pix_inc_y = sampling_res_x*sampling_res_y_inc;
		n_pix_diff_inc_y = abs(n_pix_desired - n_pix_inc_y);
		printf("Ratio: %d x %d :: Res: %d x %d n_pix: %d \n", sampling_ratio_x, sampling_ratio_y - 1,
			sampling_res_x, sampling_res_y_inc, n_pix_inc_y);
	}

	if(n_pix_diff < n_pix_diff_inc_x){
		if(n_pix_diff > n_pix_diff_inc_y){
			--sampling_ratio_y;
		}
	} else{
		if(n_pix_diff_inc_x > n_pix_diff_inc_y){
			--sampling_ratio_y;
		} else{
			--sampling_ratio_x;
		}
	}
	sampling_res_x = size_x / sampling_ratio_x;
	sampling_res_y = size_y / sampling_ratio_y;
	n_pix = sampling_res_x*sampling_res_y;

	printf("Chosen: \n");
	printf("Ratio: %d x %d :: Res: %d x %d n_pix: %d \n", sampling_ratio_x, sampling_ratio_y,
		sampling_res_x, sampling_res_y, n_pix);

	tracker->setSampling(sampling_ratio_y, sampling_ratio_x);
}
// overloaded variants that avoid passing the image repeatedly by sharing
// memory with the buffer where the images are read in by the input pipeline
void ViSP::initialize(const cv::Mat &corners){
	cout << "ViSP::initialize::corners:\n" << corners << "\n";

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

void ViSP::update(){
	try{
		vpImage<unsigned char> curr_img_vp((unsigned char* const)curr_img_cv.data, curr_img_cv.rows, curr_img_cv.cols, true);
		tracker->track(curr_img_vp);
		updateCorners();
	} catch(const vpTrackingException &err){
		throw mtf::utils::InvalidTrackerState(err.what());
	}
}

// return true if the tracker requires the raw RGB image
void ViSP::updateCorners(){
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

void ViSP::setRegion(const cv::Mat& corners){
	try{
		tracker->resetTracker();
		initialize(corners);
		updateCorners();
	} catch(const vpTrackingException &err){
		throw mtf::utils::InvalidTrackerState(err.what());
	}
}

