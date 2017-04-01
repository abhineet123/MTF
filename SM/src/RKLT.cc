#include "mtf/SM/RKLT.h"

_MTF_BEGIN_NAMESPACE

template<class AM, class SSM>
RKLT<AM, SSM>::RKLT(const ParamType *rklt_params,
	GridBase *_grid_tracker, TemplTrackerType *_templ_tracker) :
	CompositeBase(), params(rklt_params),
	templ_tracker(_templ_tracker), grid_tracker(_grid_tracker){
	printf("\n");
	printf("Using RKL SM with:\n");
	printf("enable_spi: %d\n", params.enable_spi);
	printf("enable_feedback: %d\n", params.enable_feedback);
	printf("failure_detection: %d\n", params.failure_detection);
	printf("failure_thresh: %f\n", params.failure_thresh);
	printf("debug_mode: %d\n", params.debug_mode);
	printf("templ_tracker: %s with:\n", templ_tracker->name.c_str());
	printf("\t appearance model: %s\n", templ_tracker->getAM().name.c_str());
	printf("\t state space model: %s\n", templ_tracker->getSSM().name.c_str());
	printf("\n");

	name = "rklt";

	if(params.enable_spi){
		if(!templ_tracker->supportsSPI()){
			throw std::invalid_argument("Template tracker does not support SPI\n");
		} else if(grid_tracker->getResX() != templ_tracker->getAM().getResX() ||
			grid_tracker->getResY() != templ_tracker->getAM().getResY()){
			throw std::invalid_argument(
				cv::format("Sampling resolution of the template tracker: %d x %d is not same as the grid size: %d x %d\n",
				templ_tracker->getAM().getResX(), templ_tracker->getAM().getResY(),
				grid_tracker->getResX(), grid_tracker->getResY()));
		} else{
			printf("SPI is enabled\n");
		}
	}
	input_type = templ_tracker->inputType();
	if(input_type != grid_tracker->inputType()){
		input_type = HETEROGENEOUS_INPUT;
	}
	if(params.failure_detection){
		printf("Template tracker failure detection is enabled with a threshold of %f\n",
			params.failure_thresh);
		grid_corners_mat.create(2, 4, CV_64FC1);
	}
}
template<class AM, class SSM>
RKLT<AM, SSM>::~RKLT(){
	if(templ_tracker){ delete(templ_tracker); }
	if(grid_tracker){ delete(grid_tracker); }
}
template<class AM, class SSM>
void  RKLT<AM, SSM>::initialize(const cv::Mat &corners){
	grid_tracker->initialize(corners);
	if(params.enable_spi){
		grid_tracker->initPixMask();
	}
	templ_tracker->initialize(corners);
	cv_corners_mat = templ_tracker->getRegion();
}

template<class AM, class SSM>
void  RKLT<AM, SSM>::setImage(const cv::Mat &cv_img){
	if(inputType() == HETEROGENEOUS_INPUT){
		if(cv_img.type() == grid_tracker->inputType()){
			grid_tracker->setImage(cv_img);
		}
		if(cv_img.type() == templ_tracker->inputType()){
			templ_tracker->setImage(cv_img);
		}
	} else {
		templ_tracker->setImage(cv_img);
		grid_tracker->setImage(cv_img);
	}
}

template<class AM, class SSM>
void  RKLT<AM, SSM>::update(){
	grid_tracker->update();
	templ_tracker->setRegion(grid_tracker->getRegion());
	if(params.enable_spi){
		templ_tracker->setSPIMask((const bool*)grid_tracker->getPixMask());
	}
	templ_tracker->update();
	cv_corners_mat = templ_tracker->getRegion();
	if(params.failure_detection){
		double corner_diff = cv::norm(cv_corners_mat, grid_tracker->getRegion());
		if(corner_diff > params.failure_thresh){
			cv_corners_mat = grid_tracker->getRegion();
			return;
		}
	}
	if(params.enable_feedback){
		grid_tracker->setRegion(cv_corners_mat);
	}
}

template<class AM, class SSM>
void  RKLT<AM, SSM>::setRegion(const cv::Mat &corners){
	grid_tracker->setRegion(corners);
	templ_tracker->setRegion(corners);
}
_MTF_END_NAMESPACE

#ifndef HEADER_ONLY_MODE
#include "mtf/Macros/register.h"
_REGISTER_TRACKERS(RKLT);
#endif