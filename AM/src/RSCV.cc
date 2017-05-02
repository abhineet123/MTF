#include "mtf/AM/RSCV.h"
#include "mtf/Utilities/imgUtils.h"
#include "mtf/Utilities/histUtils.h"
#include "mtf/Utilities/miscUtils.h"

#define RSCV_N_BINS 256
#define RSCV_USE_BSPL 0
#define RSCV_POU 0
#define RSCV_PRE_SEED 0
#define RSCV_WEIGHTED_MAPPING false
#define RSCV_MAPPED_GRADIENT false
#define RSCV_APPROX_DIST_FEAT true
#define RSCV_DEBUG_MODE 0

_MTF_BEGIN_NAMESPACE

//! value constructor
RSCVParams::RSCVParams(const AMParams *am_params,
bool _use_bspl, int _n_bins, double _pre_seed,
bool _partition_of_unity, bool _weighted_mapping,
bool _mapped_gradient, bool _approx_dist_feat,
bool _debug_mode) :
AMParams(am_params),
use_bspl(_use_bspl),
n_bins(_n_bins),
pre_seed(_pre_seed),
partition_of_unity(_partition_of_unity),
weighted_mapping(_weighted_mapping),
mapped_gradient(_mapped_gradient),
approx_dist_feat(_approx_dist_feat),
debug_mode(_debug_mode){
	if(n_bins <= 0)
		n_bins = RSCV_N_BINS;
}
//! default/copy constructor
RSCVParams::RSCVParams(const RSCVParams *params) :
AMParams(params),
use_bspl(RSCV_USE_BSPL),
n_bins(RSCV_N_BINS),
partition_of_unity(RSCV_POU),
pre_seed(RSCV_PRE_SEED),
weighted_mapping(RSCV_WEIGHTED_MAPPING),
mapped_gradient(RSCV_MAPPED_GRADIENT),
approx_dist_feat(RSCV_APPROX_DIST_FEAT),
debug_mode(RSCV_DEBUG_MODE){
	if(params){
		use_bspl = params->use_bspl;
		if(params->n_bins > 0){
			n_bins = params->n_bins;
		}
		pre_seed = params->pre_seed;
		partition_of_unity = params->partition_of_unity;
		weighted_mapping = params->weighted_mapping;
		mapped_gradient = params->mapped_gradient;
		approx_dist_feat = params->approx_dist_feat;
		debug_mode = params->debug_mode;
	}
}

RSCVDist::RSCVDist(const string &_name, const unsigned int _patch_size,
	const int _n_bins, const bool _approx_dist_feat) : SSDBaseDist(_name),
	patch_size(_patch_size), n_bins(_n_bins),
	approx_dist_feat(_approx_dist_feat){}

RSCV::RSCV(const ParamType *rscv_params, const int _n_channels) :
SSDBase(rscv_params, _n_channels), params(rscv_params){
	name = "rscv";

	printf("\n");
	printf("Using Reversed Sum of Conditional Variance AM with:\n");
	printf("use_bspl: %d\n", params.use_bspl);
	printf("n_bins: %d\n", params.n_bins);
	printf("pre_seed: %f\n", params.pre_seed);
	printf("partition_of_unity: %d\n", params.partition_of_unity);
	printf("weighted_mapping: %d\n", params.weighted_mapping);
	printf("mapped_gradient: %d\n", params.mapped_gradient);
	printf("approx_dist_feat: %d\n", params.approx_dist_feat);
	printf("debug_mode: %d\n", params.debug_mode);

	/**
	preseeding the joint histogram by 's' is equivalent to 
	preseeding marginal histograms by s * n_bins
	*/
	hist_pre_seed = params.n_bins * params.pre_seed;

	double norm_pix_min = 0, norm_pix_max = params.n_bins - 1;

	if(params.use_bspl && params.partition_of_unity){
		if(params.n_bins < 4){
			throw utils::InvalidArgument(
				cv::format("RSCV::Too few bins %d specified to enforce partition of unity constraint", params.n_bins));
		}
		norm_pix_min = 1;
		norm_pix_max = params.n_bins - 2;
	}
	printf("norm_pix_min: %f\n", norm_pix_min);
	printf("norm_pix_max: %f\n", norm_pix_max);

	pix_norm_mult = (norm_pix_max - norm_pix_min) / (PIX_MAX - PIX_MIN);
	pix_norm_add = norm_pix_min;

	if((pix_norm_mult != 1.0) || (pix_norm_add != 0.0)){
		printf("Image normalization is enabled\n");
	}
	intensity_map.resize(params.n_bins);
	init_hist.resize(params.n_bins);
	curr_hist.resize(params.n_bins);
	curr_joint_hist.resize(params.n_bins, params.n_bins);

	if(params.use_bspl){
		init_hist_mat.resize(params.n_bins, patch_size);
		curr_hist_mat.resize(params.n_bins, patch_size);
		_init_bspl_ids.resize(patch_size, Eigen::NoChange);
		_curr_bspl_ids.resize(patch_size, Eigen::NoChange);
		_std_bspl_ids.resize(params.n_bins, Eigen::NoChange);
		for(int bin_id = 0; bin_id < params.n_bins; bin_id++) {
			_std_bspl_ids(bin_id, 0) = max(0, bin_id - 1);
			_std_bspl_ids(bin_id, 1) = min(params.n_bins - 1, bin_id + 2);
		}
	}

}

void RSCV::initializePixVals(const Matrix2Xd& init_pts){
	if(!is_initialized.pix_vals){
		I0.resize(patch_size);
		It.resize(patch_size);
		It_orig.resize(patch_size);
	}
	switch(input_type){
	case InputType::MTF_8UC1:
		utils::sc::getPixVals<uchar>(I0, curr_img_cv, init_pts, n_pix,
			img_height, img_width, pix_norm_mult, pix_norm_add);
		break;
	case InputType::MTF_8UC3:
		utils::mc::getPixVals<uchar>(I0, curr_img_cv, init_pts, n_pix,
			img_height, img_width, pix_norm_mult, pix_norm_add);
		break;
	case InputType::MTF_32FC1:
		utils::getPixVals(I0, curr_img, init_pts, n_pix,
			img_height, img_width, pix_norm_mult, pix_norm_add);
		break;
	case InputType::MTF_32FC3:
		utils::mc::getPixVals<float>(I0, curr_img_cv, init_pts, n_pix,
			img_height, img_width, pix_norm_mult, pix_norm_add);
		break;
	default:
		throw utils::InvalidArgument("RSCV::initializePixVals::Invalid input type found");
	}
	//printf("here we are in RSCV::initializePixVals\n");
	//utils::printMatrixToFile(init_pix_vals, "initializePixVals::init_pix_vals", "log/mtf_log.txt");

	//if (pix_norm_mult != 1.0){ init_pix_vals /= pix_norm_mult; }
	//if (pix_norm_add != 0.0){ init_pix_vals = init_pix_vals.array() + pix_norm_add; }
	if(params.use_bspl){
		utils::getBSplHist(init_hist, init_hist_mat, _init_bspl_ids,
			I0, _std_bspl_ids, params.pre_seed, patch_size);
	}
	if(!is_initialized.pix_vals){
		It = I0;
		It_orig = It;
		is_initialized.pix_vals = true;
	}
	//if (params.debug_mode){
	//	utils::printMatrixToFile(init_pix_vals, "init_pix_vals", "log/mtf_log.txt", "%15.9f", "w");
	//}

}

void RSCV::updatePixVals(const Matrix2Xd& curr_pts){
	switch(input_type){
	case InputType::MTF_8UC1:
		utils::sc::getPixVals<uchar>(It_orig, curr_img_cv, curr_pts, n_pix, img_height, img_width,
			pix_norm_mult, pix_norm_add);
		break;
	case InputType::MTF_8UC3:
		utils::mc::getPixVals<uchar>(It_orig, curr_img_cv, curr_pts, n_pix, img_height, img_width,
			pix_norm_mult, pix_norm_add);
		break;
	case InputType::MTF_32FC1:
		utils::getPixVals(It_orig, curr_img, curr_pts, n_pix, img_height, img_width,
			pix_norm_mult, pix_norm_add);
		break;
	case InputType::MTF_32FC3:
		utils::mc::getPixVals<float>(It_orig, curr_img_cv, curr_pts, n_pix, img_height, img_width,
			pix_norm_mult, pix_norm_add);
		break;
	default:
		throw utils::InvalidArgument("RSCV::updatePixVals::Invalid input type found");
	}
	//if (params.debug_mode){
	//	utils::printMatrixToFile(curr_pix_vals, "orig curr_pix_vals", "log/mtf_log.txt");
	//}

	//if (pix_norm_mult != 1.0){ curr_pix_vals /= pix_norm_mult;}
	//if (pix_norm_add != 0.0){ curr_pix_vals = curr_pix_vals.array() + pix_norm_add; }

	if(params.use_bspl){
		utils::getBSplJointHist(curr_joint_hist, curr_hist, curr_hist_mat, _curr_bspl_ids,
			It_orig, _init_bspl_ids, init_hist_mat, _std_bspl_ids,
			hist_pre_seed, params.pre_seed, patch_size);
		//utils::getBSplJointHist(curr_joint_hist, curr_hist, init_hist,
		//	curr_pix_vals, init_pix_vals, hist_pre_seed, params.pre_seed, patch_size);
	} else{
		//utils::printMatrixToFile(init_pix_vals, "init_pix_vals", "log/mtf_log.txt");
		utils::getDiracJointHist(curr_joint_hist, curr_hist, init_hist,
			It_orig, I0, 0, 0, patch_size, params.n_bins);
	}
	//if (params.debug_mode){
	//	utils::printMatrixToFile(curr_joint_hist, "curr_joint_hist", "log/mtf_log.txt");
	//	utils::printMatrixToFile(curr_hist, "curr_hist", "log/mtf_log.txt");
	//	utils::printMatrixToFile(init_hist, "init_hist", "log/mtf_log.txt");
	//}

	for(int bin_id = 0; bin_id < params.n_bins; ++bin_id){
		if(curr_hist(bin_id) == 0){
			/**
			since the sum of all entries in a row of the joint histogram is zero
			each individual entry must be zero too
			*/
			intensity_map(bin_id) = bin_id;
		} else{
			double wt_sum = 0;
			for(int j = 0; j < params.n_bins; ++j){
				wt_sum += j * curr_joint_hist(bin_id, j);
			}
			intensity_map(bin_id) = wt_sum / curr_hist(bin_id);
		}
	}
	if(params.weighted_mapping){
		utils::mapPixVals<utils::InterpType::Linear>(It, It_orig, intensity_map, patch_size);
	} else{
		utils::mapPixVals<utils::InterpType::Nearest>(It, It_orig, intensity_map, patch_size);
	}
	//if (params.debug_mode){
	//	utils::printMatrixToFile(intensity_map, "intensity_map", "log/mtf_log.txt");
	//	utils::printMatrixToFile(curr_pix_vals, "curr_pix_vals", "log/mtf_log.txt");
	//}
}

void RSCV::updatePixGrad(const Matrix2Xd &curr_pts){
	if(params.mapped_gradient){
		switch(input_type){
		case InputType::MTF_8UC1:
			utils::sc::getImgGrad<uchar>(dIt_dx, curr_img_cv, params.weighted_mapping,
				intensity_map, curr_pts, grad_eps, n_pix, img_height, img_width);
			break;
		case InputType::MTF_8UC3:
			utils::mc::getImgGrad<uchar>(dIt_dx, curr_img_cv, params.weighted_mapping,
				intensity_map, curr_pts, grad_eps, n_pix, img_height, img_width);
			break;
		case InputType::MTF_32FC1:
			utils::getImgGrad(dIt_dx, curr_img, params.weighted_mapping,
				intensity_map, curr_pts, grad_eps, n_pix, img_height, img_width);
			break;
		case InputType::MTF_32FC3:
			utils::mc::getImgGrad<float>(dIt_dx, curr_img_cv, params.weighted_mapping,
				intensity_map, curr_pts, grad_eps, n_pix, img_height, img_width);
			break;
		default:
			throw utils::InvalidArgument("RSCV::updatePixGrad::Invalid input type found");
		}
	} else{
		ImageBase::updatePixGrad(curr_pts);
	}
}

void RSCV::updatePixHess(const Matrix2Xd &curr_pts){
	if(params.mapped_gradient){
		switch(input_type){
		case InputType::MTF_8UC1:
			utils::sc::getImgHess<uchar>(d2It_dx2, curr_img_cv, params.weighted_mapping, intensity_map,
				curr_pts, hess_eps, n_pix, img_height, img_width);
			break;
		case InputType::MTF_8UC3:
			utils::mc::getImgHess<uchar>(d2It_dx2, curr_img_cv, params.weighted_mapping, intensity_map,
				curr_pts, hess_eps, n_pix, img_height, img_width);
			break;
		case InputType::MTF_32FC1:
			utils::getImgHess(d2It_dx2, curr_img, params.weighted_mapping, intensity_map,
				curr_pts, hess_eps, n_pix, img_height, img_width);
			break;
		case InputType::MTF_32FC3:
			utils::mc::getImgHess<float>(d2It_dx2, curr_img_cv, params.weighted_mapping, intensity_map,
				curr_pts, hess_eps, n_pix, img_height, img_width);
			break;
		default:
			throw utils::InvalidArgument("RSCV::updatePixHess::Invalid input type found");
		}
	} else{
		ImageBase::updatePixHess(curr_pts);
	}
}
void RSCV::updatePixHess(const Matrix2Xd& curr_pts, const Matrix16Xd &warped_offset_pts){
	if(params.mapped_gradient){
		switch(input_type){
		case InputType::MTF_8UC1:
			utils::sc::getWarpedImgHess<uchar>(d2It_dx2, curr_img_cv, params.weighted_mapping, intensity_map,
				curr_pts, warped_offset_pts, hess_eps, n_pix, img_height, img_width);
			break;
		case InputType::MTF_8UC3:
			utils::mc::getWarpedImgHess<uchar>(d2It_dx2, curr_img_cv, params.weighted_mapping, intensity_map,
				curr_pts, warped_offset_pts, hess_eps, n_pix, img_height, img_width);
			break;
		case InputType::MTF_32FC1:
			utils::getWarpedImgHess(d2It_dx2, curr_img, params.weighted_mapping, intensity_map,
				curr_pts, warped_offset_pts, hess_eps, n_pix, img_height, img_width);
			break;
		case InputType::MTF_32FC3:
			utils::mc::getWarpedImgHess<float>(d2It_dx2, curr_img_cv, params.weighted_mapping, intensity_map,
				curr_pts, warped_offset_pts, hess_eps, n_pix, img_height, img_width);
			break;
		default:
			throw utils::InvalidArgument("RSCV::updatePixHess::Invalid input type found");
		}
	} else{
		ImageBase::updatePixHess(curr_pts, warped_offset_pts);
	}
}

void RSCV::updatePixGrad(const Matrix8Xd &warped_offset_pts){
	if(params.mapped_gradient){
		switch(input_type){
		case InputType::MTF_8UC1:
			utils::sc::getWarpedImgGrad<uchar>(dIt_dx, curr_img_cv, params.weighted_mapping,
				intensity_map, warped_offset_pts,
				grad_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		case InputType::MTF_8UC3:
			utils::mc::getWarpedImgGrad<uchar>(dIt_dx, curr_img_cv, params.weighted_mapping,
				intensity_map, warped_offset_pts, grad_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		case InputType::MTF_32FC1:
			utils::getWarpedImgGrad(dIt_dx, curr_img, params.weighted_mapping,
				intensity_map, warped_offset_pts,
				grad_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		case InputType::MTF_32FC3:
			utils::mc::getWarpedImgGrad<float>(dIt_dx, curr_img_cv, params.weighted_mapping,
				intensity_map, warped_offset_pts, grad_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		default:
			throw utils::InvalidArgument("RSCV::updatePixGrad::Invalid input type found");
		}
	} else{
		ImageBase::updatePixGrad(warped_offset_pts);
	}
}


void RSCV::updateDistFeat(double* feat_addr){
	if(params.approx_dist_feat){
		SSDBase::updateDistFeat(feat_addr);
	} else{
		for(size_t pix = 0; pix < patch_size; pix++) {
			*feat_addr++ = It_orig(pix);
		}
	}
}
double RSCVDist::operator()(const double* a, const double* b,
	size_t size, double worst_dist) const{
	assert(size == patch_size);

	if(approx_dist_feat){
		return SSDBaseDist::operator()(a, b, size, worst_dist);
	}
	MatrixXi joint_hist = MatrixXi::Zero(n_bins, n_bins);
	VectorXi hist = VectorXi::Zero(n_bins);
	VectorXd intensity_map(n_bins);
	for(size_t pix_id = 0; pix_id < size; ++pix_id) {
		int a_int = static_cast<int>(a[pix_id]);
		int b_int = static_cast<int>(b[pix_id]);
		hist(b_int) += 1;
		joint_hist(b_int, a_int) += 1;
	}
	for(int bin_id = 0; bin_id < n_bins; ++bin_id){
		if(hist(bin_id) == 0){
			/**
			since the sum of all entries in a row of the joint histogram is zero,
			each individual entry must be zero too
			*/
			intensity_map(bin_id) = bin_id;
		} else{
			double wt_sum = 0;
			for(int j = 0; j < n_bins; j++){
				wt_sum += j * joint_hist(bin_id, j);
			}
			intensity_map(bin_id) = wt_sum / hist(bin_id);
		}
	}
	double result = 0;
	const double* last = a + size;
	const double* lastgroup = last - 3;
	/* Process 4 items with each loop for efficiency. */
	while(a < lastgroup) {
		double diff0 = a[0] - intensity_map(static_cast<int>(rint(b[0])));
		double diff1 = a[1] - intensity_map(static_cast<int>(rint(b[1])));
		double diff2 = a[2] - intensity_map(static_cast<int>(rint(b[2])));
		double diff3 = a[3] - intensity_map(static_cast<int>(rint(b[3])));
		result += diff0 * diff0 + diff1 * diff1 + diff2 * diff2 + diff3 * diff3;
		a += 4;
		b += 4;
		if((worst_dist > 0) && (result > worst_dist)) {
			return result;
		}
	}
	/* Process last 0-3 pixels.  Not needed for standard vector lengths. */
	while(a < last) {
		double diff0 = *a++ - intensity_map(static_cast<int>(rint(*b++))); 
		result += diff0 * diff0;
	}
	return result;
}

_MTF_END_NAMESPACE

