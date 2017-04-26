#ifndef MTF_LKLD_H
#define MTF_LKLD_H

#include "AppearanceModel.h"

_MTF_BEGIN_NAMESPACE

struct LKLDParams : AMParams{

	//! no. of sub regions in horizontal and vertical directions
	int n_sub_regions_x, n_sub_regions_y;
	//! spacing in pixels between adjacent sub regions
	int spacing_x, spacing_y;
	//! no. of bins in the histograms used internally - dimensionality of the LKLD error vector will be n_bins * n_bins; 
	//! if partition_of_unity is enabled, this should be 2 more than the desired no. of bins (w.r.t normalized pixel range)
	//! since the actual range within which the pixel values are normalized is 2 less than this value to avoid
	//!	boundary conditions while computing the contribution of each pixel to different bins by ensuring that pixels with the maximum and
	//! minimum values contribute to all 4 bins required by the bspl function of degree 3 used here;
	int n_bins;
	//! initial value with which each bin of the joint histogram is pre-seeded
	//! to avoid numerical instabilities due to empty or near empty bins (caused e.g. by having to assume log(0) = 0 for empty bins) 
	double pre_seed;
	//! decides whether the partition of unity constraint has to be strictly observed for border bins;
	//! if enabled, the pixel values will be normalized in the range [1, n_bins-2] so each pixel contributes to all 4 bins
	bool partition_of_unity;
	//! decides whether logging data will be printed for debugging purposes; 
	//! only matters if logging is enabled at compile time
	bool debug_mode;

	//! value constructor
	LKLDParams(const AMParams *am_params,
		int _n_sub_regions_x, int _n_sub_regions_y,
		int _spacing_x, int _spacing_y,
		int _n_bins, double _pre_seed,
		bool _partition_of_unity,
		bool _debug_mode);
	//! default/copy constructor
	LKLDParams(const  LKLDParams *params = nullptr);
};

struct LKLDDist : AMDist{
	typedef double ElementType;
	typedef double ResultType;
	LKLDDist(const string &_name, unsigned int _feat_size) :
		AMDist(_name), feat_size(_feat_size){}
	double operator()(const double* a, const double* b,
		size_t size, double worst_dist = -1) const override;
private:
	unsigned int feat_size;
};


// Localized Kullback–Leibler Divergence
class LKLD : public AppearanceModel{
public:
	typedef LKLDParams ParamType;
	typedef LKLDDist DistType;

	LKLD(const ParamType *kld_params = nullptr);

	double getLikelihood() const override{
		return exp(f);
	}

	void initializeSimilarity() override;
	void initializeGrad() override;
	void initializeHess() override;

	void updateSimilarity(bool prereq_only = true) override;
	void updateInitGrad() override;
	void updateCurrGrad() override;

	void cmptInitHessian(MatrixXd &hessian, const MatrixXd &curr_pix_jacobian) override;
	void cmptCurrHessian(MatrixXd &hessian, const MatrixXd &curr_pix_jacobian) override;

	void cmptInitHessian(MatrixXd &init_hessian, const MatrixXd &init_pix_jacobian,
		const MatrixXd &init_pix_hessian) override;
	void cmptCurrHessian(MatrixXd &curr_hessian, const MatrixXd &curr_pix_jacobian,
		const MatrixXd &curr_pix_hessian) override;

	void cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian) override;
	void cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian,
		const MatrixXd &curr_pix_hessian) override;

	/**
	Support for FLANN library
	*/
	const DistType* getDistPtr() override{
		return new DistType(name, feat_size);
	}
	void initializeDistFeat() override;
	unsigned int getDistFeatSize() override{ return feat_size; }
	void updateDistFeat() override{
		updateDistFeat(feat_vec.data());
	}
	const double* getDistFeat() override{ return feat_vec.data(); }
	void updateDistFeat(double* feat_addr) override;

private:

	ParamType params;

	char *log_fname;
	char *time_fname;

	//! multiplicative factor for normalizing histograms
	double hist_norm_mult;

	// let A = err_vec_size = n_bins*n_bins and N = n_pix = no. of pixels
	MatrixXd init_hists, curr_hists;
	MatrixXd init_hists_log, curr_hists_log;
	MatrixXd init_grad_factors, curr_grad_factors;

	MatrixXd init_hist_mat, curr_hist_mat;
	//! n_bins X N gradients of the individual histograms w.r.t. pixel values
	MatrixXd init_hist_grad, curr_hist_grad;
	MatrixXd init_hist_hess, curr_hist_hess;

	int n_sub_regions;
	int patch_size_x, patch_size_y;
	int sub_region_size_x, sub_region_size_y;
	int sub_region_n_pix;
	MatrixX2i sub_region_x, sub_region_y;
	MatrixXi sub_region_pix_id;

	int feat_size;
	VectorXd feat_vec;

	// only used internally to increase speed by offlining as many computations as possible;
	MatrixX2i _std_bspl_ids;
	MatrixX2i _init_bspl_ids;
	MatrixX2i _curr_bspl_ids;
};

_MTF_END_NAMESPACE

#endif