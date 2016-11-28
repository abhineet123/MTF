#ifndef MTF_RSCV_H
#define MTF_RSCV_H

#include "SSDBase.h"

#define RSCV_N_BINS 256
#define RSCV_USE_BSPL 0
#define RSCV_POU 0
#define RSCV_PRE_SEED 0
#define RSCV_WEIGHTED_MAPPING false
#define RSCV_MAPPED_GRADIENT false
#define RSCV_APPROX_DIST_FEAT true
#define RSCV_DEBUG_MODE 0

_MTF_BEGIN_NAMESPACE

struct RSCVParams : AMParams{
	// use BSpline function of order 3 rather than the Dirac Delta function
	//! as the kernel function for Parzen density estimation
	bool use_bspl;
	//! no. of bins in the histograms
	//! if use_bspl and partition_of_unity are enabled, this should be 2 more than the desired no. of bins (w.r.t normalized pixel range)
	//! since the actual range within which the pixel values are normalized is 2 less than this value to avoid
	//!	boundary conditions while computing the contribution of each pixel to different bins by ensuring that pixels with the maximum and
	//! minimum values contribute to all 4 bins required by the bspline function of order 3 used here;
	int n_bins;
	//! decides whether the partition of unity constraint has to be strictly observed for border bins;
	//! if enabled, the pixel values will be normalized in the range [1, n_bins-2] so each pixel contributes to all 4 bins
	bool partition_of_unity;
	//! initial value with which each bin of the joint histogram is pre-seeded
	//! to avoid numerical instabilities due to empty or near empty bins
	double pre_seed;
	// enable this to map each intensity to the weighted average of the two entries of the intensity map corresponding
	// to the floor and ceil of that intensity; if disabled it will be mapped to the entry corresponding to its floor 
	// leading to some information loss due to the fractional part that was discarded
	bool weighted_mapping;
	// enable this to use intensity mapping while numerically computing the image gradient (using finite difference); 
	// if disabled, the original intensities will be used instead;
	bool mapped_gradient;
	// decides if true RSCV will be computed between the distance features or 
	// an approximate one based on mapping the distance feature itself with 
	// the original template; if it is turned off then the joint distribution will be 
	// computed from the two arguments to the distance functor itself after which the 
	// second argument will be mapped according to the expectation formula; note that 
	// using a KD tree index with this turned off will not produce the desired results because 
	// in that case this AM is no longer KD Tree compatible
	bool approx_dist_feat;
	//! decides whether logging data will be printed for debugging purposes; 
	//! only matters if logging option is enabled at compile time
	bool debug_mode;

	//! value constructor
	RSCVParams(const AMParams *am_params,
		bool _use_bspl, int _n_bins, double _pre_seed,
		bool _partition_of_unity, bool _weighted_mapping,
		bool _mapped_gradient, bool _approx_dist_feat, 
		bool _debug_mode);
	//! default/copy constructor
	RSCVParams(const RSCVParams *params = nullptr);
};

//! Reversed Sum of Conditional Variance
class RSCV : public SSDBase{
public:

	typedef RSCVParams ParamType;

	RSCV(const ParamType *rscv_params = nullptr, const int _n_channels = 1);

	void initializePixVals(const Matrix2Xd& init_pts) override;
	void updatePixVals(const Matrix2Xd& curr_pts) override;

	void updatePixGrad(const Matrix2Xd &curr_pts) override;
	void updatePixHess(const Matrix2Xd &curr_pts) override;

	void updatePixGrad(const Matrix8Xd &warped_offset_pts) override;
	void updatePixHess(const Matrix2Xd& curr_pts, 
		const Matrix16Xd &warped_offset_pts) override;

	double operator()(const double* a, const double* b,
		size_t size, double worst_dist = -1) const override;
	void updateDistFeat(double* feat_addr) override;
	void updateDistFeat() override{}
	const double* getDistFeat() override{ 
		return params.approx_dist_feat ? It.data() : It_orig.data();
	}

protected:

	ParamType params;

	double hist_pre_seed;
	VectorXd intensity_map;
	// let A = n_bins*n_bins and N = n_pix = no. of pixels
	PixValT It_orig;
	//! n_bins x n_bins joint histograms; 
	MatrixXd curr_joint_hist;
	VectorXd init_hist, curr_hist;
	MatrixXd init_hist_mat, curr_hist_mat;

private:

	//! used to increase speed by offlining as many computations as possible;
	MatrixX2i _std_bspl_ids, _init_bspl_ids, _curr_bspl_ids;
};

_MTF_END_NAMESPACE

#endif