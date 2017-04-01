#ifndef MTF_SCV_H
#define MTF_SCV_H

#include "SSDBase.h"

#define SCV_N_BINS 256
#define SCV_HIST_TYPE HistType::Dirac
#define SCV_POU false
#define SCV_PRE_SEED 0
#define SCV_WEIGHTED_MAPPING false
#define SCV_MAPPED_GRADIENT false
#define SCV_APPROX_DIST_FEAT false
#define SCV_LIKELIHOOD_ALPHA 50
#define SCV_DEBUG_MODE false

_MTF_BEGIN_NAMESPACE

struct SCVParams : AMParams{

	enum class HistType{ Dirac, Bilinear, BSpline };
	static const char* toString(HistType _hist_type);
	//! method used for computing the joint histogram:
	//! Dirac: Dirac delta function that uses nearest neighbor interpolation
	//! Bilinear: Bilinearr interpolation
	//! BSpline: use BSpline function of order 3 as the kernel function for Parzen density estimation
	HistType hist_type;
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
	// enable this to automatically update the initial pixel gradient and hessian using the latest intensity map 
	// whenever the current pixel gradient is updated assuming that the current pixel values and thus the intensity map must
	// have changed since the last time the initial pixel gradient was computed;
	bool mapped_gradient;
	// decides if true SCV will be computed between the distance features; if it is turned off 
	// then the joint distribution will be 
	// computed from the two arguments to the distance functor itself after which the 
	// first argument will be mapped according to the expectation formula; note that 
	// using a KD tree index with this turned off will not produce the desired results because 
	// in that case this AM is no longer KD Tree compatible
	bool approx_dist_feat;
	//! decides whether logging data will be printed for debugging purposes; 
	//! only matters if logging option is enabled at compile time
	bool debug_mode;

	//! value constructor
	SCVParams(const AMParams *am_params,
		HistType _hist_type, int _n_bins, double _pre_seed,
		bool _partition_of_unity, bool _weighted_mapping,
		bool _mapped_gradient, bool _approx_dist_feat,
		bool _debug_mode);
	//! default/copy constructor
	SCVParams(const SCVParams *params = nullptr);
};

//! Sum of Conditional Variance
class SCV : public SSDBase{
public:
	typedef SCVParams ParamType;
	typedef SCVParams::HistType HistType;


	SCV(const ParamType *scv_params = nullptr, const int _n_channels = 1);
	void initializePixVals(const Matrix2Xd& init_pts) override;
	void initializePixGrad(const Matrix2Xd &init_pts) override;
	void initializePixGrad(const Matrix8Xd &warped_offset_pts) override;

	void updatePixGrad(const Matrix2Xd &curr_pts) override;
	void updatePixGrad(const Matrix8Xd &warped_offset_pts) override;

	void updateSimilarity(bool prereq_only = true) override;


	void updatePixHess(const Matrix2Xd &curr_pts) override;
	using AppearanceModel::updatePixHess;
	double operator()(const double* a, const double* b,
		size_t size, double worst_dist = -1) const override;

protected:

	ParamType params;

	cv::Mat init_img_cv;
	EigImgT init_img;
	VectorXd I0_orig;
	Matrix2Xd init_pts;
	Matrix8Xd init_warped_offset_pts;

	double hist_pre_seed;

	VectorXd intensity_map;

	// let A = err_vec_size = n_bins*n_bins and N = n_pix = no. of pixels
	//! n_bins x n_bins joint histograms; 
	MatrixXd curr_joint_hist;
	VectorXd init_hist, curr_hist;
	MatrixXd init_hist_mat, curr_hist_mat;

private:
	// only used internally to increase speed by offlining as many computations as possible;
	MatrixX2i _std_bspl_ids;
	MatrixX2i _init_bspl_ids;
	MatrixX2i _curr_bspl_ids;
	EigImgMat _init_img;
};

_MTF_END_NAMESPACE

#endif