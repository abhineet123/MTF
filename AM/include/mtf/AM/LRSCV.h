#ifndef MTF_LRSCV_H
#define MTF_LRSCV_H

#include "SSDBase.h"

#define LRSCV_N_BINS 256
#define LRSCV_PRE_SEED 0
#define LRSCV_WEIGHTED_MAPPING 0
#define LRSCV_DEBUG_MODE 0
#define LRSCV_SUB_REGIONS 3
#define LRSCV_SPACING 10
#define LRSCV_AFFINE_MAPPING 0
#define LRSCV_ONCE_PER_FRAME 0
#define LRSCV_SHOW_SUBREGIONS 0

_MTF_BEGIN_NAMESPACE

struct LRSCVParams : AMParams{

	//! no. of sub regions in horizontal and vertical directions
	int sub_regions_x, sub_regions_y;
	//! spacing in pixels between adjacent sub regions
	int spacing_x, spacing_y;
	// use affine or linear mapping instead of the standard one
	bool affine_mapping;
	// update the template only once per frame, i.e. when new_frame is set to true
	bool once_per_frame;

	//! no. of bins in the histograms
	int n_bins;
	//! initial value with which each bin of the joint histogram is pre-seeded
	//! to avoid numerical instabilities due to empty or near empty bins
	double pre_seed;

	//! enable this to map each intensity to the weighted average of the two entries of the intensity map corresponding
	//! to the floor and ceil of that intensity; if disabled it will be mapped to the entry corresponding to its floor 
	//! leading to some information loss due to the fractional part that was discarded
	bool weighted_mapping;

	// show the locations of the subregions
	bool show_subregions;

	//! decides whether logging data will be printed for debugging purposes; 
	//! only matters if logging option is enabled at compile time
	bool debug_mode;

	//! value constructor
	LRSCVParams(const AMParams *am_params,
		int _sub_regions_x, int _sub_regions_y,
		int _spacing_x, int _spacing_y,
		bool _affine_mapping, bool _once_per_frame,
		int _n_bins, double _pre_seed, bool _weighted_mapping,
		bool _show_subregions, bool _debug_mode);
	//! default and copy constructor
	LRSCVParams(const LRSCVParams *params = nullptr);
};

//! Locally adaptive Reversed Sum of Conditional Variance
class LRSCV : public SSDBase{

public:

	typedef LRSCVParams ParamType;

	LRSCV(const ParamType *lrscv_params = nullptr);
	void initializePixVals(const Matrix2Xd& init_pts) override;
	void updatePixVals(const Matrix2Xd& curr_pts) override;

private:

	ParamType params;

	int mapping_type;
	double hist_pre_seed;
	int patch_size_x, patch_size_y;
	int sub_region_size_x, sub_region_size_y;
	int n_sub_regions;

	VectorXd intensity_map;

	//! let A = err_vec_size = n_bins*n_bins and N = n_pix = no. of pixels
	//! n_bins x n_bins joint histograms; 
	MatrixXd curr_joint_hist;
	VectorXd init_hist, curr_hist;

	ColPivHouseholderQR<MatrixX2d> intensity_vals_dec;
	MatrixXdMr init_patch, curr_patch;
	MatrixX2i sub_region_x, sub_region_y;
	MatrixX2d sub_region_centers;
	vector<VectorXd> mapped_pix_vals;
	MatrixX2d intensity_vals;
	Vector2d affine_params;
	MatrixXd sub_region_wts;

	//! only used internally to increase speed by offlining as many computations as possible;
	MatrixX2i _std_bspl_ids, _init_bspl_ids, _curr_bspl_ids;
	EigImgMat _init_img;
	//! used for indexing the sub region locations
	MatrixXi _subregion_idx;
	//! used for indexing the pixel locations in the flattened patch
	MatrixXi _pts_idx;

	cv::Mat patch_img, patch_img_uchar;

	char *patch_win_name;
	int sub_region_id;
	char *log_fname;

	void showSubRegions(const EigImgT& img, const Matrix2Xd& pts);
	void updateIntensityMap();
	void updateMappedPixVals(int index);
};

_MTF_END_NAMESPACE

#endif