#include "mtf/AM/SCV.h"
#include "mtf/Utilities/imgUtils.h"
#include "mtf/Utilities/histUtils.h"
#include "mtf/Utilities/miscUtils.h"

_MTF_BEGIN_NAMESPACE

//! value constructor
SCVParams::SCVParams(const AMParams *am_params,
HistType _hist_type, int _n_bins, double _pre_seed,
bool _partition_of_unity, bool _weighted_mapping,
bool _mapped_gradient, bool _approx_dist_feat,
bool _debug_mode) :
AMParams(am_params),
hist_type(_hist_type),
n_bins(_n_bins),
pre_seed(_pre_seed),
partition_of_unity(_partition_of_unity),
weighted_mapping(_weighted_mapping),
mapped_gradient(_mapped_gradient),
approx_dist_feat(_approx_dist_feat),
debug_mode(_debug_mode){
	if(n_bins <= 0){ n_bins = SCV_N_BINS; }
}
//! default/copy constructor
SCVParams::SCVParams(const SCVParams *params) :
AMParams(params),
hist_type(SCV_HIST_TYPE),
n_bins(SCV_N_BINS),
pre_seed(SCV_PRE_SEED),
partition_of_unity(SCV_POU),
weighted_mapping(SCV_WEIGHTED_MAPPING),
mapped_gradient(SCV_MAPPED_GRADIENT),
approx_dist_feat(SCV_APPROX_DIST_FEAT),
debug_mode(SCV_DEBUG_MODE){
	if(params){
		hist_type = params->hist_type;
		n_bins = params->n_bins;
		pre_seed = params->pre_seed;
		partition_of_unity = params->partition_of_unity;
		weighted_mapping = params->weighted_mapping;
		mapped_gradient = params->mapped_gradient;
		approx_dist_feat = params->approx_dist_feat;
		debug_mode = params->debug_mode;
		if(n_bins <= 0)
			n_bins = SCV_N_BINS;
	}
}

const char* SCVParams::toString(HistType _hist_type){
	switch(_hist_type){
	case HistType::Dirac:
		return "Dirac";
	case HistType::Bilinear:
		return "Bilinear";
	case HistType::BSpline:
		return "BSpline";
	default:
		throw std::invalid_argument("Invalid histogram type provided");
	}
}

SCV::SCV(const ParamType *scv_params, const int _n_channels) :
SSDBase(scv_params, _n_channels), params(scv_params), init_img(0, 0, 0){
	name = "scv";
	printf("\n");
	printf("Using Sum of Conditional Variance AM with:\n");
	printf("hist_type: %s\n", SCVParams::toString(params.hist_type));
	printf("n_bins: %d\n", params.n_bins);
	printf("pre_seed: %f\n", params.pre_seed);
	printf("partition_of_unity: %d\n", params.partition_of_unity);
	printf("weighted_mapping: %d\n", params.weighted_mapping);
	printf("mapped_gradient: %d\n", params.mapped_gradient);
	printf("approx_dist_feat: %d\n", params.approx_dist_feat);
	printf("debug_mode: %d\n", params.debug_mode);

	// preseeding the joint histogram by 's' is equivalent to 
	// preseeding individual histograms by s * n_bins
	hist_pre_seed = params.n_bins * params.pre_seed;

	double norm_pix_min = 0, norm_pix_max = params.n_bins - 1;

	if(params.hist_type == HistType::BSpline && params.partition_of_unity){
		if(params.n_bins < 4){
			throw std::invalid_argument(
				cv::format("SCV::Too few bins %d specified to enforce partition of unity constraint", params.n_bins));
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

	if(params.hist_type == HistType::BSpline){
		init_hist_mat.resize(params.n_bins, patch_size);
		curr_hist_mat.resize(params.n_bins, patch_size);
		_init_bspl_ids.resize(patch_size, Eigen::NoChange);
		_curr_bspl_ids.resize(patch_size, Eigen::NoChange);
		_std_bspl_ids.resize(params.n_bins, Eigen::NoChange);
		for(int i = 0; i < params.n_bins; i++) {
			_std_bspl_ids(i, 0) = max(0, i - 1);
			_std_bspl_ids(i, 1) = min(params.n_bins - 1, i + 2);
		}
	}
}

void SCV::initializePixVals(const Matrix2Xd& init_pts){
	ImageBase::initializePixVals(init_pts);
	//if (pix_norm_mult != 1.0){ init_pix_vals /= pix_norm_mult; }
	//if (pix_norm_add != 0.0){ init_pix_vals = init_pix_vals.array() + pix_norm_add; }
	if(params.hist_type == HistType::BSpline){
		utils::getBSplHist(init_hist, init_hist_mat, _init_bspl_ids,
			I0, _std_bspl_ids, params.pre_seed, patch_size);
	}

	if(params.debug_mode){
		utils::printMatrixToFile(I0, "init_pix_vals", "log/scv.txt", "%15.9f", "w");
	}
	// create a copy of the initial pixel values which will hold the original untampered template 
	// that will be used for remapping the initial pixel values when the intensity map is updated; 
	// it will also be used for updating the intensity map itself since using the remapped
	// initial pixel values to update the joint probability distribution and thus the intensity map will cause bias
	I0_orig = I0;

	if(params.mapped_gradient){
		// save a copy of the initial image to recompute the mapped image gradient when the intensity
		// map is updated with a new image

		if(!uchar_input && n_channels == 1){
			_init_img = curr_img;
			new (&init_img) EigImgT(_init_img.data(), _init_img.rows(), _init_img.cols());
		} else{
			init_img_cv = curr_img_cv.clone();
		}
	}
}

void SCV::initializePixGrad(const Matrix2Xd &_init_pts){
	ImageBase::initializePixGrad(_init_pts);
	if(params.mapped_gradient){
		// save a copy of the initial points to recompute the mapped gradient of the initial image
		// when the intensity map is updated with a new image
		init_pts = _init_pts;
	}
}

void SCV::initializePixGrad(const Matrix8Xd &warped_offset_pts){
	ImageBase::initializePixGrad(warped_offset_pts);
	if(params.mapped_gradient){
		// save a copy of the initial warped offset pts to recompute the mapped gradient of the initial image
		// when the intensity map is updated with a new image
		init_warped_offset_pts = warped_offset_pts;
	}
}

void SCV::updateSimilarity(bool prereq_only){
	switch(params.hist_type){
	case HistType::Dirac:
		utils::getDiracJointHist(curr_joint_hist, curr_hist, init_hist,
			It, I0_orig, 0, 0, patch_size, params.n_bins);
		break;
	case HistType::Bilinear:
		utils::getBilinearJointHist(curr_joint_hist, curr_hist, init_hist,
			It, I0_orig, 0, 0, patch_size, params.n_bins);
		break;
	case HistType::BSpline:
		utils::getBSplJointHist(curr_joint_hist, curr_hist, curr_hist_mat, _curr_bspl_ids,
			It, _init_bspl_ids, init_hist_mat, _std_bspl_ids,
			hist_pre_seed, params.pre_seed, patch_size);
		break;
	default:
		throw std::invalid_argument("Invalid histogram type provided");
	}
	for(int bin_id = 0; bin_id < params.n_bins; ++bin_id){
		if(init_hist(bin_id) == 0){
			/**
			since the sum of all entries in a column of the joint histogram is zero
			each individual entry must be zero too
			*/
			intensity_map(bin_id) = bin_id;
		} else{
			double wt_sum = 0;
			for(int i = 0; i < params.n_bins; i++){
				wt_sum += i * curr_joint_hist(i, bin_id);
			}
			intensity_map(bin_id) = wt_sum / init_hist(bin_id);
		}
	}
	if(params.weighted_mapping){
		utils::mapPixVals<utils::InterpType::Linear>(I0, I0_orig, intensity_map, patch_size);
	} else{
		utils::mapPixVals<utils::InterpType::Nearest>(I0, I0_orig, intensity_map, patch_size);
	}
	SSDBase::updateSimilarity(prereq_only);
}

void SCV::updatePixGrad(const Matrix2Xd &curr_pts){
	ImageBase::updatePixGrad(curr_pts);
	if(params.mapped_gradient){
		if(uchar_input){
			switch(n_channels){
			case 1:
				utils::sc::getImgGrad<uchar>(dI0_dx, init_img_cv, params.weighted_mapping,
					intensity_map, init_pts, grad_eps, n_pix, img_height, img_width);
				break;
			case 3:
				utils::mc::getImgGrad<uchar>(dI0_dx, init_img_cv, params.weighted_mapping,
					intensity_map, init_pts, grad_eps, n_pix, img_height, img_width);
				break;
			default:
				mc_not_implemeted(SCV::updatePixGrad, n_channels);
			}
		} else{
			switch(n_channels){
			case 1:
				utils::getImgGrad(dI0_dx, init_img, params.weighted_mapping,
					intensity_map, init_pts, grad_eps, n_pix, img_height, img_width);
				break;
			case 3:
				utils::mc::getImgGrad<float>(dI0_dx, init_img_cv, params.weighted_mapping,
					intensity_map, init_pts, grad_eps, n_pix, img_height, img_width);
				break;
			default:
				mc_not_implemeted(SCV::updatePixGrad, n_channels);
			}
		}
	}
}

void SCV::updatePixGrad(const Matrix8Xd &warped_offset_pts){
	ImageBase::updatePixGrad(warped_offset_pts);
	if(params.mapped_gradient){
		if(uchar_input){
			switch(n_channels){
			case 1:
				utils::sc::getWarpedImgGrad<uchar>(dI0_dx, init_img_cv, params.weighted_mapping,
					intensity_map, init_warped_offset_pts,
					grad_eps, n_pix, img_height, img_width, pix_norm_mult);
				break;
			case 3:
				utils::mc::getWarpedImgGrad<uchar>(dI0_dx, init_img_cv, params.weighted_mapping,
					intensity_map, init_warped_offset_pts,
					grad_eps, n_pix, img_height, img_width, pix_norm_mult);
				break;
			default:
				mc_not_implemeted(SCV::updatePixGrad, n_channels);
			}
		} else{
			switch(n_channels){
			case 1:
				utils::getWarpedImgGrad(dI0_dx, init_img, params.weighted_mapping,
					intensity_map, init_warped_offset_pts,
					grad_eps, n_pix, img_height, img_width, pix_norm_mult);
				break;
			case 3:
				utils::mc::getWarpedImgGrad<float>(dI0_dx, init_img_cv, params.weighted_mapping,
					intensity_map, init_warped_offset_pts,
					grad_eps, n_pix, img_height, img_width, pix_norm_mult);
				break;
			default:
				mc_not_implemeted(SCV::updatePixGrad, n_channels);
			}
		}
	}
}

void SCV::updatePixHess(const Matrix2Xd &curr_pts){
	ImageBase::updatePixHess(curr_pts);
	if(params.mapped_gradient){
		if(uchar_input){
			switch(n_channels){
			case 1:
				utils::sc::getImgHess<uchar>(d2I0_dx2, init_img_cv, params.weighted_mapping,
					intensity_map, init_pts, hess_eps, n_pix, img_height, img_width);
				break;
			case 3:
				utils::mc::getImgHess<uchar>(d2I0_dx2, init_img_cv, params.weighted_mapping,
					intensity_map, init_pts, hess_eps, n_pix, img_height, img_width);
				break;
			default:
				mc_not_implemeted(SCV::updatePixHess, n_channels);
			}
		} else{
			switch(n_channels){
			case 1:
				utils::getImgHess(d2I0_dx2, init_img, params.weighted_mapping,
					intensity_map, init_pts, hess_eps, n_pix, img_height, img_width);
				break;
			case 3:
				utils::mc::getImgHess<float>(d2I0_dx2, init_img_cv, params.weighted_mapping,
					intensity_map, init_pts, hess_eps, n_pix, img_height, img_width);
				break;
			default:
				mc_not_implemeted(SCV::updatePixHess, n_channels);
			}
		}

	}
}
double SCVDist::operator()(const double* a, const double* b,
	size_t size, double worst_dist) const{
	assert(size == patch_size);

	if(params.approx_dist_feat){
		return SSDDist::operator()(a, b, size, worst_dist);
	}
	/**
	this has to be as fast as possible so BSpline histogram option is disabled here
	and the histograms can thus be of integeral type
	*/
	MatrixXi joint_hist = MatrixXi::Zero(params.n_bins, params.n_bins);
	VectorXi hist = VectorXi::Zero(params.n_bins);
	VectorXd intensity_map(params.n_bins);
	for(int pix_id = 0; pix_id < size; ++pix_id) {
		int a_int = static_cast<int>(a[pix_id]);
		int b_int = static_cast<int>(b[pix_id]);
		hist(a_int) += 1;
		joint_hist(b_int, a_int) += 1;
	}
	for(int bin_id = 0; bin_id < params.n_bins; ++bin_id){
		if(hist(bin_id) == 0){
			/**
			since the sum of all entries in a column of the joint histogram is zero
			each individual entry must be zero too
			*/
			intensity_map(bin_id) = bin_id;
		} else{
			double wt_sum = 0;
			for(int i = 0; i < params.n_bins; ++i){
				wt_sum += i * joint_hist(i, bin_id);
			}
			intensity_map(bin_id) = wt_sum / static_cast<double>(hist(bin_id));
		}
	}
	double result = 0;
	const double* last = a + size;
	const double* lastgroup = last - 3;
	/**
	Process 4 items with each loop for efficiency.
	*/
	while(a < lastgroup) {
		double diff0 = b[0] - intensity_map(static_cast<int>(rint(a[0])));
		double diff1 = b[1] - intensity_map(static_cast<int>(rint(a[1])));
		double diff2 = b[2] - intensity_map(static_cast<int>(rint(a[2])));
		double diff3 = b[3] - intensity_map(static_cast<int>(rint(a[3])));
		result += diff0 * diff0 + diff1 * diff1 + diff2 * diff2 + diff3 * diff3;
		a += 4;
		b += 4;
		if((worst_dist > 0) && (result > worst_dist)) {
			return result;
		}
	}
	/**
	Process last 0-3 pixels.  Not needed for standard vector lengths.
	*/
	while(a < last) {
		double diff0 = intensity_map(static_cast<int>(rint(*a++))) - *b++;
		result += diff0 * diff0;
	}
	return result;
}
_MTF_END_NAMESPACE

