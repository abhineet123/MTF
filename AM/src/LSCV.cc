#include "mtf/AM/LSCV.h"
#include "mtf/Utilities/imgUtils.h"
#include "mtf/Utilities/histUtils.h"
#include "mtf/Utilities/miscUtils.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#define LSCV_N_BINS 256
#define LSCV_PRE_SEED 0
#define LSCV_WEIGHTED_MAPPING 0
#define LSCV_SUB_REGIONS 3
#define LSCV_SPACING 10
#define LSCV_AFFINE_MAPPING 0
#define LSCV_ONCE_PER_FRAME 0
#define LSCV_SHOW_SUBREGIONS 0
#define LSCV_APPROX_DIST_FEAT 0

_MTF_BEGIN_NAMESPACE

//! value constructor
LSCVParams::LSCVParams(const AMParams *am_params,
int _sub_regions_x, int _sub_regions_y,
int _spacing_x, int _spacing_y,
bool _affine_mapping, bool _once_per_frame,
int _n_bins, double _pre_seed, bool _weighted_mapping,
bool _show_subregions, bool _approx_dist_feat) :
AMParams(am_params),
n_sub_regions_x(_sub_regions_x),
n_sub_regions_y(_sub_regions_y),
spacing_x(_spacing_x),
spacing_y(_spacing_y),
affine_mapping(_affine_mapping),
once_per_frame(_once_per_frame),
n_bins(_n_bins),
pre_seed(_pre_seed),
weighted_mapping(_weighted_mapping),
show_subregions(_show_subregions),
approx_dist_feat(_approx_dist_feat){
	if(n_bins <= 0) // use default
		n_bins = LSCV_N_BINS;
}
//! copy constructor
LSCVParams::LSCVParams(const LSCVParams *params) :
AMParams(params),
n_sub_regions_x(LSCV_SUB_REGIONS),
n_sub_regions_y(LSCV_SUB_REGIONS),
spacing_x(LSCV_SPACING),
spacing_y(LSCV_SPACING),
affine_mapping(LSCV_AFFINE_MAPPING),
once_per_frame(LSCV_ONCE_PER_FRAME),
n_bins(LSCV_N_BINS),
pre_seed(LSCV_PRE_SEED),
weighted_mapping(LSCV_WEIGHTED_MAPPING),
show_subregions(LSCV_SHOW_SUBREGIONS),
approx_dist_feat(LSCV_APPROX_DIST_FEAT){

	if(params){
		n_sub_regions_x = params->n_sub_regions_x;
		n_sub_regions_y = params->n_sub_regions_y;
		spacing_x = params->spacing_x;
		spacing_y = params->spacing_y;

		affine_mapping = params->affine_mapping;
		once_per_frame = params->once_per_frame;

		n_bins = params->n_bins;

		pre_seed = params->pre_seed;
		weighted_mapping = params->weighted_mapping;

		show_subregions = params->show_subregions;
		approx_dist_feat = params->approx_dist_feat;

		if(n_bins <= 0)
			n_bins = LSCV_N_BINS;
	}
}

LSCVDist::LSCVDist(const string &_name, const bool _approx_dist_feat,
	const int _n_bins, const int _n_sub_regions_x, const int _n_sub_regions_y,
	const int _n_sub_regions, const unsigned int _n_pix,
	const unsigned int _resx, const unsigned int _resy,
	const MatrixX2i *_sub_region_x, const MatrixX2i *_sub_region_y,
	const MatrixXd *_sub_region_wts, const MatrixXi *_subregion_idx,
	const ColPivHouseholderQR<MatrixX2d> *_intensity_vals_dec) :
	SSDBaseDist(_name), approx_dist_feat(_approx_dist_feat),
	n_pix(_n_pix), resx(_resx), resy(_resy), n_bins(_n_bins),
	n_sub_regions_x(_n_sub_regions_x), n_sub_regions_y(_n_sub_regions_y),
	n_sub_regions(_n_sub_regions), 
	sub_region_x(_sub_region_x), sub_region_y(_sub_region_y),
	sub_region_wts(_sub_region_wts), subregion_idx(_subregion_idx),
	intensity_vals_dec(_intensity_vals_dec){}

LSCV::LSCV(const ParamType *lscv_params, int _n_channels) :
SSDBase(lscv_params, _n_channels), params(lscv_params),
init_patch(0, 0, 0), curr_patch(0, 0, 0){
	name = "lscv";

	printf("\n");
	printf("Using Localized Sum of Conditional Variance AM with:\n");

	printf("sub_regions: %d x %d\n", params.n_sub_regions_x, params.n_sub_regions_y);
	printf("sub region spacing: %d x %d\n", params.spacing_x, params.spacing_y);
	printf("affine_mapping: %d\n", params.affine_mapping);
	printf("once_per_frame: %d\n", params.once_per_frame);

	printf("n_bins: %d\n", params.n_bins);
	printf("pre_seed: %f\n", params.pre_seed);
	printf("weighted_mapping: %d\n", params.weighted_mapping);

	printf("show_subregions: %d\n", params.show_subregions);
	printf("approx_dist_feat: %d\n", params.approx_dist_feat);

	n_sub_regions = params.n_sub_regions_x*params.n_sub_regions_y;

	// preseeding the joint histogram by 's' is equivalent to 
	// preseeding individual histograms by s * n_bins
	hist_pre_seed = params.n_bins * params.pre_seed;

	double norm_pix_min = 0, norm_pix_max = params.n_bins - 1;

	printf("norm_pix_min: %f\n", norm_pix_min);
	printf("norm_pix_max: %f\n", norm_pix_max);

	intensity_range = static_cast<int>(norm_pix_max - norm_pix_min + 1);
	intensity_vals.resize(intensity_range, Eigen::NoChange);
	intensity_vals.col(0) = VectorXd::LinSpaced(intensity_range, norm_pix_min, norm_pix_max);
	intensity_vals.col(1).fill(1);

	intensity_vals_dec.compute(intensity_vals);

	pix_norm_mult = (norm_pix_max - norm_pix_min) / (PIX_MAX - PIX_MIN);
	pix_norm_add = norm_pix_min;

	if((pix_norm_mult != 1.0) || (pix_norm_add != 0.0)){
		printf("Image normalization is enabled\n");
	}
	intensity_map.resize(params.n_bins);
	init_hist.resize(params.n_bins);
	curr_hist.resize(params.n_bins);
	curr_joint_hist.resize(params.n_bins, params.n_bins);

	sub_region_x.resize(params.n_sub_regions_x, Eigen::NoChange);
	sub_region_y.resize(params.n_sub_regions_y, Eigen::NoChange);

	sub_region_size_x = resx - (params.n_sub_regions_x - 1)*params.spacing_x;
	sub_region_size_y = resy - (params.n_sub_regions_y - 1)*params.spacing_y;

	if(sub_region_size_x <= 0 || sub_region_size_y <= 0){
		throw std::invalid_argument(
			cv::format("LSCV :: Patch size : %dx%d is not enough to use the specified region spacing and / or count",
			resx, resy));
	}

	printf("Using sub regions of size: %d x %d\n", sub_region_size_x, sub_region_size_y);
	if(params.once_per_frame){
		printf("Updating the template only once per frame\n");
	}
	for(int idx = 0; idx < params.n_sub_regions_x; idx++){
		sub_region_x(idx, 0) = idx*params.spacing_x;
		sub_region_x(idx, 1) = sub_region_x(idx, 0) + sub_region_size_x - 1;
	}
	for(int idy = 0; idy < params.n_sub_regions_y; idy++){
		sub_region_y(idy, 0) = idy*params.spacing_y;
		sub_region_y(idy, 1) = sub_region_y(idy, 0) + sub_region_size_y - 1;
	}

	_subregion_idx.resize(params.n_sub_regions_y, params.n_sub_regions_x);
	sub_region_centers.resize(n_sub_regions, Eigen::NoChange);
	for(int idy = 0; idy < params.n_sub_regions_y; idy++){
		double mean_y = static_cast<double>(sub_region_y(idy, 0) + sub_region_y(idy, 1)) / 2.0;
		for(int idx = 0; idx < params.n_sub_regions_x; idx++){

			_subregion_idx(idy, idx) = idy * params.n_sub_regions_x + idx;

			double mean_x = static_cast<double>(sub_region_x(idx, 0) + sub_region_x(idx, 1)) / 2.0;
			sub_region_centers(_subregion_idx(idy, idx), 0) = mean_x;
			sub_region_centers(_subregion_idx(idy, idx), 1) = mean_y;
		}
	}

	sub_region_wts.resize(n_pix, n_sub_regions);
	for(unsigned int pix_id = 0; pix_id < n_pix; pix_id++){
		unsigned int pix_x = pix_id % resx;
		unsigned int pix_y = pix_id / resx;
		double pix_wt_sum = 0;
		for(int idy = 0; idy < params.n_sub_regions_y; idy++){
			for(int idx = 0; idx < params.n_sub_regions_x; idx++){
				int diff_x = static_cast<int>(pix_x - sub_region_centers(_subregion_idx(idy, idx), 0));
				int diff_y = static_cast<int>(pix_y - sub_region_centers(_subregion_idx(idy, idx), 1));
				double pix_wt = 1.0 / static_cast<double>(1.0 + diff_x*diff_x + diff_y*diff_y);
				pix_wt_sum += sub_region_wts(pix_id, _subregion_idx(idy, idx)) = pix_wt;
			}
		}
		// normalize all weights for this pixel so they sum to unity
		sub_region_wts.row(pix_id).array() /= pix_wt_sum;
	}

	I0_mapped.resize(n_pix);

	if(params.show_subregions){
		_pts_idx.resize(resy, resx);
		for(unsigned int idy = 0; idy < resy; ++idy){
			for(unsigned int idx = 0; idx < resx; ++idx){
				_pts_idx(idy, idx) = idy * resx + idx;
			}
		}
		patch_win_name = "LSCV Subregtions";
		cv::namedWindow(patch_win_name);
		sub_region_id = 0;
	}
}

void LSCV::initializePixVals(const Matrix2Xd& init_pts){
	if(!is_initialized.pix_vals){
		I0.resize(n_pix);
		It.resize(n_pix);
	}
	if(params.uchar_input){
		utils::sc::getPixVals<uchar>(I0, curr_img_cv, init_pts, n_pix,
			img_height, img_width, pix_norm_mult, pix_norm_add);
	} else{
		utils::getPixVals(I0, curr_img, init_pts, n_pix,
			img_height, img_width, pix_norm_mult, pix_norm_add);
	}
	/**
	create a copy of the initial pixel values which will hold the original untampered template 
	that will be used for remapping the initial pixel values when the intensity map is updated; 
	it will also be used for updating the intensity map itself since using the remapped
	initial pixel values to update the joint probability distribution and thus the intensity map will cause bias
	*/
	I0_orig = I0;

	if(!is_initialized.pix_vals){
		It = I0;

		new (&init_patch) MatrixXdMr(I0_orig.data(), resy, resx);
		new (&curr_patch) MatrixXdMr(It.data(), resy, resx);

		is_initialized.pix_vals = true;
	}
	if(params.show_subregions){
		showSubRegions(curr_img, init_pts);
	}
}

void LSCV::updatePixVals(const Matrix2Xd& curr_pts){
	if(params.uchar_input){
		utils::sc::getPixVals<uchar>(It, curr_img_cv, curr_pts, n_pix,
			img_height, img_width, pix_norm_mult, pix_norm_add);
	} else{
		utils::getPixVals(It, curr_img, curr_pts, n_pix,
			img_height, img_width, pix_norm_mult, pix_norm_add);
	}
	if(params.show_subregions){
		if(first_iter){
			sub_region_id = (sub_region_id + 1) % n_sub_regions;
		}
		showSubRegions(curr_img, curr_pts);
	}
}

void LSCV::updateSimilarity(bool prereq_only){
	if(params.once_per_frame && !first_iter)
		return;

	I0.setZero();
	for(int idx = 0; idx < params.n_sub_regions_x; idx++){
		int start_x = sub_region_x(idx, 0);
		int end_x = sub_region_x(idx, 1);
		for(int idy = 0; idy < params.n_sub_regions_y; idy++){
			utils::getDiracJointHist(curr_joint_hist, curr_hist, init_hist,
				curr_patch, init_patch, start_x, end_x,
				sub_region_y(idy, 0), sub_region_y(idy, 1),
				0, 0, n_pix, params.n_bins);
			for(int bin_id = 0; bin_id < params.n_bins; bin_id++){
				if(init_hist(bin_id) == 0){
					intensity_map(bin_id) = bin_id;
				} else{
					double wt_sum = 0;
					for(int i = 0; i < params.n_bins; i++){
						wt_sum += i * curr_joint_hist(i, bin_id);
					}
					intensity_map(bin_id) = wt_sum / init_hist(bin_id);
				}
			}
			if(params.affine_mapping){
				Vector2d affine_params = intensity_vals_dec.solve(intensity_map);
				I0_mapped = (affine_params(0)*I0_orig).array() + affine_params(1);
			} else{
				if(params.weighted_mapping){
					utils::mapPixVals<utils::InterpType::Linear>(I0_mapped, I0_orig, intensity_map, n_pix);
				} else{
					utils::mapPixVals<utils::InterpType::Nearest>(I0_mapped, I0_orig, intensity_map, n_pix);
				}
			}
			for(unsigned int pix_id = 0; pix_id < n_pix; ++pix_id){
				I0(pix_id) += I0_mapped(pix_id)*sub_region_wts(pix_id, _subregion_idx(idy, idx));
			}
		}
	}

	SSDBase::updateSimilarity(prereq_only);
}

void  LSCV::showSubRegions(const EigImgT& img, const Matrix2Xd& pts){
	patch_img = cv::Mat(img.rows(), img.cols(), CV_32FC1, const_cast<float*>(img.data()));
	patch_img_uchar.create(img.rows(), img.cols(), CV_8UC3);
	patch_img.convertTo(patch_img_uchar, patch_img_uchar.type());
	cv::cvtColor(patch_img_uchar, patch_img_uchar, CV_GRAY2BGR);
	cv::Scalar region_cols[8] = {
		cv::Scalar(0, 0, 255),
		cv::Scalar(0, 255, 0),
		cv::Scalar(255, 0, 0),
		cv::Scalar(255, 255, 0),
		cv::Scalar(0, 255, 255),
		cv::Scalar(255, 0, 255),
		cv::Scalar(0, 0, 0),
		cv::Scalar(255, 255, 255)
	};
	int n_cols = sizeof(region_cols) / sizeof(cv::Scalar);
	int col_id = sub_region_id % n_cols;
	int idx = sub_region_id % params.n_sub_regions_x;
	int idy = sub_region_id / params.n_sub_regions_x;
	int x1 = sub_region_x(idx, 0);
	int x2 = sub_region_x(idx, 1);
	int y1 = sub_region_y(idy, 0);
	int y2 = sub_region_y(idy, 1);

	cv::Point2d ul(pts(0, _pts_idx(y1, x1)), pts(1, _pts_idx(y1, x1)));
	cv::Point2d ur(pts(0, _pts_idx(y1, x2)), pts(1, _pts_idx(y1, x2)));
	cv::Point2d lr(pts(0, _pts_idx(y2, x2)), pts(1, _pts_idx(y2, x2)));
	cv::Point2d ll(pts(0, _pts_idx(y2, x1)), pts(1, _pts_idx(y2, x1)));
	cv::Point2d centroid = (ul + ur + lr + ll)*0.25;

	cv::line(patch_img_uchar, ul, ur, region_cols[col_id], 2);
	cv::line(patch_img_uchar, ur, lr, region_cols[col_id], 2);
	cv::line(patch_img_uchar, lr, ll, region_cols[col_id], 2);
	cv::line(patch_img_uchar, ll, ul, region_cols[col_id], 2);

	putText(patch_img_uchar, cv_format("%d", _subregion_idx(idy, idx)), centroid,
		cv::FONT_HERSHEY_SIMPLEX, 0.50, region_cols[col_id]);

	imshow(patch_win_name, patch_img_uchar);
	char key = cv::waitKey(1);
	if(key == 'n' || key == 'N'){
		sub_region_id = (sub_region_id + 1) % n_sub_regions;
		printf("sub_region_id: %d\n", sub_region_id);
	}
}


double LSCVDist::operator()(const double* a, const double* b,
	size_t size, double worst_dist) const{
	assert(size == n_pix);

	if(approx_dist_feat){
		return SSDBaseDist::operator()(a, b, size, worst_dist);
	}
	Map<const MatrixXdr> _init_patch(a, resy, resx);
	Map<const MatrixXdr> _curr_patch(b, resy, resx);

	VectorXd _mapped_a = VectorXd::Zero(size);
	for(int idx = 0; idx < n_sub_regions_x; ++idx){
		int start_x = (*sub_region_x)(idx, 0);
		int end_x = (*sub_region_x)(idx, 1);
		for(int idy = 0; idy < n_sub_regions_y; ++idy){
			MatrixXi _joint_hist = MatrixXi::Zero(n_bins, n_bins);
			VectorXi _hist = VectorXi::Zero(n_bins);
			for(int y = (*sub_region_y)(idy, 0); y <= (*sub_region_y)(idy, 1); ++y) {
				for(int x = start_x; x <= end_x; ++x) {
					int pix1_int = static_cast<int>(_init_patch(y, x));
					int pix2_int = static_cast<int>(_curr_patch(y, x));
					_hist(pix1_int) += 1;
					_joint_hist(pix2_int, pix1_int) += 1;
				}
			}
			VectorXd _intensity_map(n_bins);
			for(int bin_id = 0; bin_id < n_bins; ++bin_id){
				if(_hist(bin_id) == 0){
					_intensity_map(bin_id) = bin_id;
				} else{
					double wt_sum = 0;
					for(int i = 0; i < n_bins; ++i){
						wt_sum += i * _joint_hist(i, bin_id);
					}
					_intensity_map(bin_id) = wt_sum / _hist(bin_id);
				}
			}
			Vector2d _affine_params = intensity_vals_dec->solve(_intensity_map);
			for(unsigned int pix_id = 0; pix_id < n_pix; ++pix_id){
				_mapped_a(pix_id) += (_affine_params(0)*a[pix_id] + _affine_params(1))*
					(*sub_region_wts)(pix_id, (*subregion_idx)(idy, idx));
			}
		}
	}

	double result = 0;
	const double* last = b + size;
	const double* lastgroup = last - 3;
	/**
	Process 4 items with each loop for efficiency.
	*/
	const double* _a = _mapped_a.data();
	while(b < lastgroup) {
		double diff0 = b[0] - _a[0];
		double diff1 = b[1] - _a[1];
		double diff2 = b[2] - _a[2];
		double diff3 = b[3] - _a[3];
		result += diff0 * diff0 + diff1 * diff1 + diff2 * diff2 + diff3 * diff3;
		_a += 4;
		b += 4;
		if((worst_dist > 0) && (result > worst_dist)) {
			return result;
		}
	}
	/**
	Process last 0-3 pixels.  Not needed for standard vector lengths.
	*/
	while(b < last) {
		double diff0 = *_a++ - *b++;
		result += diff0 * diff0;
	}
	return result;
}


_MTF_END_NAMESPACE

