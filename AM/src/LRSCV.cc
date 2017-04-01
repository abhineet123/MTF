#include "mtf/AM/LRSCV.h"
#include "mtf/Utilities/imgUtils.h"
#include "mtf/Utilities/histUtils.h"
#include "mtf/Utilities/miscUtils.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

_MTF_BEGIN_NAMESPACE

//! value constructor
LRSCVParams::LRSCVParams(const AMParams *am_params,
int _sub_regions_x, int _sub_regions_y,
int _spacing_x, int _spacing_y,
bool _affine_mapping, bool _once_per_frame,
int _n_bins, double _pre_seed, bool _weighted_mapping,
bool _show_subregions, bool _debug_mode) :
AMParams(am_params){

	sub_regions_x = _sub_regions_x;
	sub_regions_y = _sub_regions_y;
	spacing_x = _spacing_x;
	spacing_y = _spacing_y;
	affine_mapping = _affine_mapping;
	once_per_frame = _once_per_frame;

	n_bins = _n_bins;
	pre_seed = _pre_seed;
	weighted_mapping = _weighted_mapping;

	show_subregions = _show_subregions;
	debug_mode = _debug_mode;

	if(n_bins <= 0) // use default
		n_bins = LRSCV_N_BINS;
}
//! default and copy constructor
LRSCVParams::LRSCVParams(const LRSCVParams *params) :
AMParams(params),
sub_regions_x(LRSCV_SUB_REGIONS),
sub_regions_y(LRSCV_SUB_REGIONS),
spacing_x(LRSCV_SPACING),
spacing_y(LRSCV_SPACING),
affine_mapping(LRSCV_AFFINE_MAPPING),
once_per_frame(LRSCV_ONCE_PER_FRAME),
n_bins(LRSCV_N_BINS),
pre_seed(LRSCV_PRE_SEED),
weighted_mapping(LRSCV_WEIGHTED_MAPPING),
show_subregions(LRSCV_SHOW_SUBREGIONS),
debug_mode(LRSCV_DEBUG_MODE){
	if(params){
		sub_regions_x = params->sub_regions_x;
		sub_regions_y = params->sub_regions_y;
		spacing_x = params->spacing_x;
		spacing_y = params->spacing_y;

		affine_mapping = params->affine_mapping;
		once_per_frame = params->once_per_frame;

		n_bins = params->n_bins;

		pre_seed = params->pre_seed;
		weighted_mapping = params->weighted_mapping;

		show_subregions = params->show_subregions;
		debug_mode = params->debug_mode;

		if(n_bins <= 0)
			n_bins = LRSCV_N_BINS;
	}
}

LRSCV::LRSCV(const ParamType *lrscv_params) : SSDBase(lrscv_params),
params(lrscv_params), init_patch(0, 0, 0), curr_patch(0, 0, 0){
	name = "lrscv";

	printf("\n");
	printf("Using Localized Reversed Sum of Conditional Variance AM with:\n");

	printf("sub_regions: %d x %d\n", params.sub_regions_x, params.sub_regions_y);
	printf("spacing: %d x %d\n", params.spacing_x, params.spacing_y);
	printf("affine_mapping: %d\n", params.affine_mapping);
	printf("once_per_frame: %d\n", params.once_per_frame);

	printf("n_bins: %d\n", params.n_bins);
	printf("pre_seed: %f\n", params.pre_seed);
	printf("weighted_mapping: %d\n", params.weighted_mapping);

	printf("show_subregions: %d\n", params.show_subregions);
	printf("debug_mode: %d\n", params.debug_mode);

	patch_size_x = resx;
	patch_size_y = resy;
	n_sub_regions = params.sub_regions_x*params.sub_regions_y;

	// preseeding the joint histogram by 's' is equivalent to 
	// preseeding individual histograms by s * n_bins
	hist_pre_seed = params.n_bins * params.pre_seed;

	double norm_pix_min = 0, norm_pix_max = params.n_bins - 1;

	printf("norm_pix_min: %f\n", norm_pix_min);
	printf("norm_pix_max: %f\n", norm_pix_max);

	int intensity_range = norm_pix_max - norm_pix_min + 1;
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

	sub_region_x.resize(params.sub_regions_x, Eigen::NoChange);
	sub_region_y.resize(params.sub_regions_y, Eigen::NoChange);

	sub_region_size_x = patch_size_x - (params.sub_regions_x - 1)*params.spacing_x;
	sub_region_size_y = patch_size_y - (params.sub_regions_y - 1)*params.spacing_y;

	if(sub_region_size_x <= 0 || sub_region_size_y <= 0){
		throw std::invalid_argument(
			cv::format("LRSCV :: Patch size : %dx%d is not enough to use the specified region spacing and / or count",
			patch_size_x, patch_size_y)
			);
	}

	printf("Using sub regions of size: %d x %d\n", sub_region_size_x, sub_region_size_y);
	if(params.once_per_frame){
		printf("Updating the template only once per frame\n");
	}
	for(int idx = 0; idx < params.sub_regions_x; idx++){
		sub_region_x(idx, 0) = idx*params.spacing_x;
		sub_region_x(idx, 1) = sub_region_x(idx, 0) + sub_region_size_x - 1;
	}
	for(int idy = 0; idy < params.sub_regions_y; idy++){
		sub_region_y(idy, 0) = idy*params.spacing_y;
		sub_region_y(idy, 1) = sub_region_y(idy, 0) + sub_region_size_y - 1;
	}

	_subregion_idx.resize(params.sub_regions_y, params.sub_regions_x);
	sub_region_centers.resize(n_sub_regions, Eigen::NoChange);
	for(int idy = 0; idy < params.sub_regions_y; idy++){
		double mean_y = static_cast<double>(sub_region_y(idy, 0) + sub_region_y(idy, 1)) / 2.0;
		for(int idx = 0; idx < params.sub_regions_x; idx++){

			_subregion_idx(idy, idx) = idy * params.sub_regions_x + idx;

			double mean_x = static_cast<double>(sub_region_x(idx, 0) + sub_region_x(idx, 1)) / 2.0;
			sub_region_centers(_subregion_idx(idy, idx), 0) = mean_x;
			sub_region_centers(_subregion_idx(idy, idx), 1) = mean_y;
		}
	}

	sub_region_wts.resize(n_pix, n_sub_regions);
	for(int pix_id = 0; pix_id < n_pix; pix_id++){
		int pix_x = pix_id % patch_size_x;
		int pix_y = pix_id / patch_size_x;
		double pix_wt_sum = 0;
		for(int idy = 0; idy < params.sub_regions_y; idy++){
			for(int idx = 0; idx < params.sub_regions_x; idx++){
				int diff_x = pix_x - sub_region_centers(_subregion_idx(idy, idx), 0);
				int diff_y = pix_y - sub_region_centers(_subregion_idx(idy, idx), 1);
				double pix_wt = 1.0 / static_cast<double>(1.0 + diff_x*diff_x + diff_y*diff_y);
				pix_wt_sum += sub_region_wts(pix_id, _subregion_idx(idy, idx)) = pix_wt;
			}
		}
		// normalize all weights for this pixel so they sum to unity
		sub_region_wts.row(pix_id).array() /= pix_wt_sum;
	}

	mapped_pix_vals.resize(n_sub_regions);
	for(int i = 0; i < n_sub_regions; i++){
		mapped_pix_vals[i].resize(n_pix);
	}

	if(params.show_subregions){
		_pts_idx.resize(resy, resx);
		for(int idy = 0; idy < resy; idy++){
			for(int idx = 0; idx < resx; idx++){
				_pts_idx(idy, idx) = idy * resx + idx;
			}
		}
		patch_win_name = "LRSCV Subregions";
		cv::namedWindow(patch_win_name);
		sub_region_id = 0;
	}
}

void LRSCV::initializePixVals(const Matrix2Xd& init_pts){
	if(!is_initialized.pix_vals){
		I0.resize(n_pix);
		It.resize(n_pix);
	}

	if(uchar_input){
		utils::sc::getPixVals<uchar>(I0, curr_img_cv, init_pts, n_pix,
			img_height, img_width, pix_norm_mult, pix_norm_add);
	} else{
		utils::getPixVals(I0, curr_img, init_pts, n_pix,
			img_height, img_width, pix_norm_mult, pix_norm_add);
	}

	if(!is_initialized.pix_vals){
		It = I0;

		new (&init_patch) MatrixXdMr(I0.data(), patch_size_y, patch_size_x);
		new (&curr_patch) MatrixXdMr(It.data(), patch_size_y, patch_size_x);

		is_initialized.pix_vals = true;
	}
	if(params.show_subregions){
		showSubRegions(curr_img, init_pts);
	}
}


void LRSCV::updatePixVals(const Matrix2Xd& curr_pts){

	if(uchar_input){
		utils::sc::getPixVals<uchar>(It, curr_img_cv, curr_pts, n_pix,
			img_height, img_width, pix_norm_mult, pix_norm_add);
	} else{
		utils::getPixVals(It, curr_img, curr_pts, n_pix,
			img_height, img_width, pix_norm_mult, pix_norm_add);
	}

	if(params.once_per_frame && !first_iter)
		return;

	for(int idx = 0; idx < params.sub_regions_x; idx++){
		int start_x = sub_region_x(idx, 0);
		int end_x = sub_region_x(idx, 1);
		for(int idy = 0; idy < params.sub_regions_y; idy++){
			utils::getDiracJointHist(curr_joint_hist, curr_hist, init_hist,
				curr_patch, init_patch, start_x, end_x,
				sub_region_y(idy, 0), sub_region_y(idy, 1),
				0, 0, n_pix, params.n_bins);
			updateIntensityMap();
			updateMappedPixVals(_subregion_idx(idy, idx));
		}
	}
	for(int pix_id = 0; pix_id < n_pix; pix_id++){
		It(pix_id) = 0;
		for(int region_id = 0; region_id < n_sub_regions; region_id++){
			It(pix_id) += mapped_pix_vals[region_id](pix_id)*sub_region_wts(pix_id, region_id);
		}
	}
	if(params.show_subregions){
		if(first_iter){
			sub_region_id = (sub_region_id + 1) % n_sub_regions;
		}
		showSubRegions(curr_img, curr_pts);
	}
}

void LRSCV::updateIntensityMap(){
	for(int i = 0; i < params.n_bins; i++){
		if(curr_hist(i) == 0){
			// since the sum of all entries in a column of the joint histogram is zero
			// each individual entry must be zero too
			intensity_map(i) = i;
		} else{
			double wt_sum = 0;
			for(int j = 0; j < params.n_bins; j++){
				wt_sum += j * curr_joint_hist(i, j);
			}
			intensity_map(i) = wt_sum / curr_hist(i);
		}
	}
}

void LRSCV::updateMappedPixVals(int index){
	if(params.affine_mapping){
		affine_params = intensity_vals_dec.solve(intensity_map);
		mapped_pix_vals[index] = (affine_params(0)*It).array() + affine_params(1);
	} else{
		if(params.weighted_mapping){
			utils::mapPixVals<utils::InterpType::Linear>(mapped_pix_vals[index], It, intensity_map, n_pix);
		} else{
			utils::mapPixVals<utils::InterpType::Nearest>(mapped_pix_vals[index], It, intensity_map, n_pix);
		}
	}
}

#ifdef _WIN32
#define snprintf  _snprintf
#endif
void  LRSCV::showSubRegions(const ImageT &img, const Matrix2Xd& pts){
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
	int idx = sub_region_id % params.sub_regions_x;
	int idy = sub_region_id / params.sub_regions_x;
	int x1 = sub_region_x(idx, 0);
	int x2 = sub_region_x(idx, 1);
	int y1 = sub_region_y(idy, 0);
	int y2 = sub_region_y(idy, 1);

	cv::Point ul(pts(0, _pts_idx(y1, x1)), pts(1, _pts_idx(y1, x1)));
	cv::Point ur(pts(0, _pts_idx(y1, x2)), pts(1, _pts_idx(y1, x2)));
	cv::Point lr(pts(0, _pts_idx(y2, x2)), pts(1, _pts_idx(y2, x2)));
	cv::Point ll(pts(0, _pts_idx(y2, x1)), pts(1, _pts_idx(y2, x1)));
	cv::Point centroid = (ul + ur + lr + ll)*0.25;

	cv::line(patch_img_uchar, ul, ur, region_cols[col_id], 2);
	cv::line(patch_img_uchar, ur, lr, region_cols[col_id], 2);
	cv::line(patch_img_uchar, lr, ll, region_cols[col_id], 2);
	cv::line(patch_img_uchar, ll, ul, region_cols[col_id], 2);

	char buffer[20];
	snprintf(buffer, 20, "%d", _subregion_idx(idy, idx));
	putText(patch_img_uchar, buffer, centroid,
		cv::FONT_HERSHEY_SIMPLEX, 0.50, region_cols[col_id]);

	imshow(patch_win_name, patch_img_uchar);
	char key = cv::waitKey(1);
	if(key == 'n' || key == 'N'){
		sub_region_id = (sub_region_id + 1) % n_sub_regions;
		printf("sub_region_id: %d\n", sub_region_id);
	}
}

_MTF_END_NAMESPACE

