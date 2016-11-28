#include "mtf/SM/NT/RKLT.h"
#include "mtf/Utilities/spiUtils.h"

_MTF_BEGIN_NAMESPACE

namespace nt{
	
	RKLT::RKLT(const ParamType *rklt_params,
		GridBase *_grid_tracker, TemplTrackerType *_templ_tracker) :
		CompositeBase(), params(rklt_params),
		templ_tracker(_templ_tracker), grid_tracker(_grid_tracker),
		using_expanded_mask(false){
		printf("\n");
		printf("Using RKL (NT) SM with:\n");
		printf("enable_spi: %d\n", params.enable_spi);
		printf("enable_feedback: %d\n", params.enable_feedback);
		printf("failure_detection: %d\n", params.failure_detection);
		printf("failure_thresh: %f\n", params.failure_thresh);
		printf("debug_mode: %d\n", params.debug_mode);
		printf("templ_tracker: %s with:\n", templ_tracker->name.c_str());
		printf("\t appearance model: %s\n", templ_tracker->getAM()->name.c_str());
		printf("\t state space model: %s\n", templ_tracker->getSSM()->name.c_str());
		printf("\n");

		name = "rklt_nt";

		templ_resx = templ_tracker->getAM()->getResX();
		templ_resy = templ_tracker->getAM()->getResY();
		grid_resx = grid_tracker->getResX();
		grid_resy = grid_tracker->getResY();

		res_ratio_x = templ_resx / grid_resx;
		res_ratio_y = templ_resy / grid_resy;

		if(params.enable_spi){
			if(!templ_tracker->supportsSPI()){
				throw std::invalid_argument("Template tracker does not support SPI\n");
			} else if(templ_resx % grid_resx != 0 || templ_resy % grid_resy != 0){
				throw std::invalid_argument(
					cv::format("Sampling resolution of the template tracker: %d x %d is not compatible with the grid size: %d x %d\n",
					templ_resx, templ_resy, grid_resx, grid_resy));
			} else{
				printf("SPI is enabled\n");
				if(res_ratio_x != 1 || res_ratio_y != 1){
					using_expanded_mask = true;
					expanded_mask.resize(templ_resx*templ_resy);
					printf("Using expanded SPI mask with ratio %d x %d\n", res_ratio_x, res_ratio_y);
				}
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

	RKLT::~RKLT(){
		if(templ_tracker){ delete(templ_tracker); }
		if(grid_tracker){ delete(grid_tracker); }
	}

	void  RKLT::initialize(const cv::Mat &corners){
		grid_tracker->initialize(corners);
		templ_tracker->initialize(corners);
		cv_corners_mat = templ_tracker->getRegion();
	}
	
	void  RKLT::setImage(const cv::Mat &cv_img){
		if(cv_img.type() == grid_tracker->inputType()){
			grid_tracker->setImage(cv_img);
		}
		if(cv_img.type() == templ_tracker->inputType()){
			templ_tracker->setImage(cv_img);
		}
	}
	
	void  RKLT::update(){
		grid_tracker->update();
		templ_tracker->setRegion(grid_tracker->getRegion());
		if(params.enable_spi){
			if(using_expanded_mask){
				utils::expandMask((bool*)expanded_mask.data(), (const bool*)grid_tracker->getPixMask(),
					res_ratio_x, res_ratio_y, grid_resx, grid_resy,
					templ_resx, templ_resy);
				templ_tracker->setSPIMask((const bool*)expanded_mask.data());
			} else{
				templ_tracker->setSPIMask((const bool*)grid_tracker->getPixMask());
			}			
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
	
	void  RKLT::setRegion(const cv::Mat &corners){
		grid_tracker->setRegion(corners);
		templ_tracker->setRegion(corners);
	}
}
_MTF_END_NAMESPACE



