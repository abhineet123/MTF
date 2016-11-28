#include "mtf/AM/SSD.h"
#include "mtf/Utilities/imgUtils.h"
#include "mtf/Utilities/miscUtils.h"
#include "opencv2/highgui/highgui.hpp"

#define SSD_SHOW_TEMPLATE false
#define SSD_FORGETTING_FACTOR 0.5

_MTF_BEGIN_NAMESPACE

//! value constructor
SSDParams::SSDParams(const AMParams *am_params,
bool _show_template,
double _forgetting_factor) :
AMParams(am_params),
show_template(_show_template),
forgetting_factor(_forgetting_factor){}

//! default/copy constructor
SSDParams::SSDParams(const SSDParams *params) :
AMParams(params),
show_template(SSD_SHOW_TEMPLATE),
forgetting_factor(SSD_FORGETTING_FACTOR){
	if(params){
		show_template = params->show_template;
		forgetting_factor = params->forgetting_factor;
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
	printf("show_template: %d\n", params.show_template);
	printf("forgetting_factor: %f\n", params.forgetting_factor);
	name = "ssd";

	use_running_avg = params.forgetting_factor < 0 || params.forgetting_factor > 1;
	old_pix_wt = 1 - params.forgetting_factor;	
}

void SSD::updateModel(const Matrix2Xd& curr_pts){
	assert(curr_pts.cols() == n_pix);
	++frame_count;
	//! update the template, aka init_pix_vals with the running or weighted average of
	//! the patch corresponding to the provided points
	if(n_channels == 1){
		if(use_running_avg){
			for(int pix_id = 0; pix_id < n_pix; pix_id++){
				double pix_val = pix_norm_mult * utils::getPixVal<PIX_INTERP_TYPE, PIX_BORDER_TYPE>(
					curr_img, curr_pts(0, pix_id), curr_pts(1, pix_id),
					img_height, img_width) + pix_norm_add;
				I0(pix_id) += (pix_val - I0(pix_id)) / frame_count;
			}
		} else{
			for(int pix_id = 0; pix_id < n_pix; pix_id++){
				double pix_val = utils::getPixVal<PIX_INTERP_TYPE, PIX_BORDER_TYPE>(
					curr_img, curr_pts(0, pix_id), curr_pts(1, pix_id), img_height, img_width);
				I0(pix_id) = old_pix_wt*I0(pix_id) + params.forgetting_factor*pix_val;
			}
		}

	} else{
		double *I0_data = I0.data();
		if(use_running_avg){
			for(int pix_id = 0; pix_id < n_pix; pix_id++){
				double pix_val[3];
				utils::getPixVal<PIX_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val, curr_img_cv,
					curr_pts(0, pix_id), curr_pts(1, pix_id), img_height, img_width);
				I0_data[0] += (pix_val[0] - I0_data[0]) / frame_count;
				I0_data[1] += (pix_val[1] - I0_data[1]) / frame_count;
				I0_data[2] += (pix_val[2] - I0_data[2]) / frame_count;
				I0_data += 3;
			}
		} else{
			for(int pix_id = 0; pix_id < n_pix; pix_id++){
				double pix_val[3];
				utils::getPixVal<PIX_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val, curr_img_cv,
					curr_pts(0, pix_id), curr_pts(1, pix_id), img_height, img_width);
				I0_data[0] = params.forgetting_factor*pix_val[0] + old_pix_wt*I0_data[0];
				I0_data[1] = params.forgetting_factor*pix_val[1] + old_pix_wt*I0_data[1];
				I0_data[2] = params.forgetting_factor*pix_val[2] + old_pix_wt*I0_data[2];
				I0_data += 3;
			}
		}
	}
	//! re initialize any quantities that depend on the template
	if(is_initialized.similarity){
		initializeSimilarity();
	}
	if(is_initialized.grad){
		initializeGrad();
	}
	if(is_initialized.hess){
		initializeHess();
	}
	if(params.show_template){
		cv::imshow("Template", utils::reshapePatch(I0, resy, resx, n_channels));
	}
}

_MTF_END_NAMESPACE

