#include "mtf/AM/SSD.h"
#include "mtf/Utilities/imgUtils.h"
#include "mtf/Utilities/miscUtils.h"
#include "opencv2/highgui/highgui.hpp"

#define SSD_SHOW_TEMPLATE false

_MTF_BEGIN_NAMESPACE

//! value constructor
SSDParams::SSDParams(const AMParams *am_params,
bool _show_template) :
AMParams(am_params),
show_template(_show_template){}

//! default/copy constructor
SSDParams::SSDParams(const SSDParams *params) :
AMParams(params),
show_template(SSD_SHOW_TEMPLATE){
	if(params){
		show_template = params->show_template;
	}
}

SSD::SSD(const ParamType *ssd_params, const int _n_channels) :
SSDBase(ssd_params, _n_channels),
params(ssd_params), use_running_avg(false),
old_pix_wt(0){
	printf("\n");
	printf("Using Sum of Squared Differences AM with...\n");
	printf("grad_eps: %e\n", grad_eps);
	printf("hess_eps: %e\n", hess_eps);
	printf("likelihood_alpha: %f\n", params.likelihood_alpha);
	printf("dist_from_likelihood: %d\n", params.dist_from_likelihood);
	printf("learning_rate: %f\n", params.learning_rate);
	printf("show_template: %d\n", params.show_template);
	name = "ssd";

	use_running_avg = params.learning_rate < 0 || params.learning_rate > 1;
	old_pix_wt = 1 - params.learning_rate;	
}

void SSD::updateModel(const Matrix2Xd& curr_pts){
	assert(curr_pts.cols() == n_pix);
	++frame_count;
	//! update the template, aka init_pix_vals with the running or weighted average of
	//! the patch corresponding to the provided points
	if(uchar_input){
		switch(n_channels){
		case 1:
			utils::sc::getWeightedPixVals<uchar>(I0, curr_img_cv, curr_pts, frame_count, params.learning_rate,
				use_running_avg, n_pix, img_height, img_width, pix_norm_mult, pix_norm_add);
			break;
		case 3:
			utils::mc::getWeightedPixVals<uchar>(I0, curr_img_cv, curr_pts, frame_count, params.learning_rate,
				use_running_avg, n_pix, img_height, img_width, pix_norm_mult, pix_norm_add);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	} else{
		switch(n_channels){
		case 1:
			utils::getWeightedPixVals(I0, curr_img, curr_pts, frame_count, params.learning_rate,
				use_running_avg, n_pix, img_height, img_width, pix_norm_mult, pix_norm_add);
			break;
		case 3:
			utils::mc::getWeightedPixVals<float>(I0, curr_img_cv, curr_pts, frame_count, params.learning_rate,
				use_running_avg, n_pix, img_height, img_width, pix_norm_mult, pix_norm_add);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	}
	//! re initialize any quantities that depend on the template
	reinitialize();
	if(params.show_template){
		cv::imshow("Template", utils::reshapePatch(I0, resy, resx, n_channels));
	}
}

_MTF_END_NAMESPACE

