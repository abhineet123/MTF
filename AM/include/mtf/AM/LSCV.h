#ifndef MTF_LSCV_H
#define MTF_LSCV_H

#include "SSDBase.h"

_MTF_BEGIN_NAMESPACE

struct LSCVParams : AMParams{

	//! no. of sub regions in horizontal and vertical directions
	int n_sub_regions_x, n_sub_regions_y;
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
	bool approx_dist_feat;

	//! value constructor
	LSCVParams(const AMParams *am_params,
		int _sub_regions_x, int _sub_regions_y,
		int _spacing_x, int _spacing_y,
		bool _affine_mapping, bool _once_per_frame,
		int _n_bins, double _pre_seed, bool _weighted_mapping,
		bool _show_subregions, bool _approx_dist_feat);
	//! copy constructor
	LSCVParams(const LSCVParams *params = nullptr);
};

struct LSCVDist : SSDBaseDist{
	typedef double ElementType;
	typedef double ResultType;
	LSCVDist(const string &_name, bool _approx_dist_feat,
		int _n_bins, int _n_sub_regions_x, int _n_sub_regions_y,
		int _n_sub_regions, int _intensity_range,
		unsigned int _n_pix, unsigned int _resx, unsigned int _resy,
		const int *__sub_region_x, const int *__sub_region_y,
		const double *__sub_region_wts, const int *__subregion_idx,
		double *_intensity_vals);
	double operator()(const double* a, const double* b,
		size_t size, double worst_dist = -1) const override;
private:
	bool approx_dist_feat;
	unsigned int n_pix, resx, resy;
	int n_bins, n_sub_regions_x, n_sub_regions_y, n_sub_regions;
	int intensity_range;
	const int *_sub_region_x, *_sub_region_y;
	const double *_sub_region_wts;
	const int *_subregion_idx;
	double *_intensity_vals;
};

//! Locally adaptive Sum of Conditional Variance
class LSCV : public SSDBase{

public:

	typedef LSCVParams ParamType;
	typedef LSCVDist DistType;

	LSCV(const ParamType *lscv_params = nullptr, int _n_channels=1);

	void initializePixVals(const Matrix2Xd& init_pts) override;
	void updatePixVals(const Matrix2Xd& curr_pts) override;

	void updateSimilarity(bool prereq_only = true) override;

	const DistType* getDistPtr() override{
		return new DistType(name, params.approx_dist_feat, n_pix, resx, resy,
			params.n_bins, params.n_sub_regions_x, params.n_sub_regions_y, 
			n_sub_regions, intensity_range, sub_region_x.data(), sub_region_y.data(), 
			sub_region_wts.data(), _subregion_idx.data(), intensity_vals.data());
	}

protected:

	ParamType params;
	VectorXd I0_orig;

	double hist_pre_seed;

	VectorXd intensity_map;

	int n_sub_regions;
	int sub_region_size_x, sub_region_size_y;
	int intensity_range;

	// let A = err_vec_size = n_bins*n_bins and N = n_pix = no. of pixels
	//! n_bins x n_bins joint histograms; 
	MatrixXd curr_joint_hist;
	VectorXd init_hist, curr_hist;

	MatrixXdMr init_patch, curr_patch;
	MatrixX2i sub_region_x, sub_region_y;
	MatrixX2d sub_region_centers;
	VectorXd I0_mapped;
	MatrixX2d intensity_vals;
	ColPivHouseholderQR<MatrixX2d> intensity_vals_dec;
	MatrixXd sub_region_wts;

	// only used internally to increase speed by offlining as many computations as possible;
	MatrixX2i _std_bspl_ids;
	MatrixX2i _init_bspl_ids;
	MatrixX2i _curr_bspl_ids;
	EigImgMat _init_img;
	MatrixXi _subregion_idx;//used for indexing the sub region locations
	MatrixXi _pts_idx;//used for indexing the pixel locations in the flattened patch

	cv::Mat patch_img;
	cv::Mat patch_img_uchar;
	char *patch_win_name;
	int sub_region_id;
	char *log_fname;

	void  showSubRegions(const EigImgT& img, const Matrix2Xd& pts);
};

_MTF_END_NAMESPACE

#endif