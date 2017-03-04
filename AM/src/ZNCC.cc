#include "mtf/AM/ZNCC.h"
#include "mtf/Utilities/imgUtils.h"

#define ZNCC_DEBUG false

_MTF_BEGIN_NAMESPACE

//! value constructor
ZNCCParams::ZNCCParams(const AMParams *am_params,
 bool _debug_mode) :
AMParams(am_params){	
	debug_mode = _debug_mode;
}
//! default/copy constructor
ZNCCParams::ZNCCParams(const ZNCCParams *params) :
AMParams(params),
debug_mode(ZNCC_DEBUG){
	if(params){		
		debug_mode = params->debug_mode;
	}
}

ZNCC::ZNCC(const ParamType *zncc_params, const int _n_channels) :
params(zncc_params),
SSDBase(zncc_params, _n_channels) {
	name = "zncc";
	printf("\n");
	printf("Using Zero mean Normalized Cross Correlation AM with:\n");
	printf("likelihood_alpha: %f\n", params.likelihood_alpha);
	printf("debug_mode: %d\n", params.debug_mode);

	I0_mean = I0_var = I0_std = 0;
	It_mean = It_var = It_std = 0;
}

void ZNCC::initializePixVals(const Matrix2Xd& init_pts) {
	assert(init_pts.cols() == n_pix);
	if(!isInitialized()->pix_vals){
		I0.resize(patch_size);
		It.resize(patch_size);
	}
	++frame_count;

	if(uchar_input){
		switch(n_channels){
		case 1:
			utils::sc::getPixVals<uchar>(I0, curr_img_cv, init_pts, n_pix,
				img_height, img_width);
			break;
		case 3:
			utils::mc::getPixVals<uchar>(I0, curr_img_cv, init_pts, n_pix,
				img_height, img_width);
			break;
		default:
			mc_not_implemeted(ZNCC::initializePixVals, n_channels);
		}
	} else{
		switch(n_channels){
		case 1:
			utils::getPixVals(I0, curr_img, init_pts, n_pix,
				img_height, img_width);
			break;
		case 3:
			utils::mc::getPixVals<float>(I0, curr_img_cv, init_pts, n_pix,
				img_height, img_width);
			break;
		default:
			mc_not_implemeted(ZNCC::initializePixVals, n_channels);
		}
	}


	I0_mean = I0.mean();
	I0 = (I0.array() - I0_mean);

	I0_var = I0.squaredNorm() / patch_size;
	I0_std = sqrt(I0_var);
	I0 /= I0_std;

	pix_norm_mult = 1.0 / I0_std;
	pix_norm_add = -I0_mean;

	if(!is_initialized.pix_vals){
		It = I0;
		It_mean = I0_mean;
		It_var = I0_var;
		It_std = I0_std;
		is_initialized.pix_vals = true;
	}
}

void ZNCC::updatePixVals(const Matrix2Xd& curr_pts) {
	assert(curr_pts.cols() == n_pix);

	if(uchar_input){
		switch(n_channels){
		case 1:
			utils::sc::getPixVals<uchar>(It, curr_img_cv, curr_pts, n_pix, img_height, img_width);
			break;
		case 3:
			utils::mc::getPixVals<uchar>(It, curr_img_cv, curr_pts, n_pix, img_height, img_width);
			break;
		default:
			mc_not_implemeted(ZNCC::updatePixVals, n_channels);
		}
	} else{
		switch(n_channels){
		case 1:
			utils::getPixVals(It, curr_img, curr_pts, n_pix, img_height, img_width);
			break;
		case 3:
			utils::mc::getPixVals<float>(It, curr_img_cv, curr_pts, n_pix, img_height, img_width);
			break;
		default:
			mc_not_implemeted(ZNCC::updatePixVals, n_channels);
		}
	}

	It_mean = It.mean();
	It = (It.array() - It_mean);

	It_var = It.squaredNorm() / patch_size;
	It_std = sqrt(It_var);
	It /= It_std;

	pix_norm_mult = 1.0 / It_std;
	pix_norm_add = -It_mean;
}

double ZNCC::getLikelihood() const{
	return exp(-params.likelihood_alpha*((I0 / I0.norm() - It / It.norm()).squaredNorm()));
}

_MTF_END_NAMESPACE

