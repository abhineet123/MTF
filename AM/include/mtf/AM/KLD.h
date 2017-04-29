#ifndef MTF_KLD_H
#define MTF_KLD_H

#include "AppearanceModel.h"

_MTF_BEGIN_NAMESPACE

struct KLDParams : AMParams{
	int n_bins; //! no. of bins in the histograms used internally - dimensionality of the KLD error vector will be n_bins * n_bins; 
	//! if partition_of_unity is enabled, this should be 2 more than the desired no. of bins (w.r.t normalized pixel range)
	//! since the actual range within which the pixel values are normalized is 2 less than this value to avoid
	//!	boundary conditions while computing the contribution of each pixel to different bins by ensuring that pixels with the maximum and
	//! minimum values contribute to all 4 bins required by the bspl function of degree 3 used here;
	bool partition_of_unity; //! decides whether the partition of unity constraint has to be strictly observed for border bins;
	//! if enabled, the pixel values will be normalized in the range [1, n_bins-2] so each pixel contributes to all 4 bins
	double pre_seed; //! initial value with which each bin of the joint histogram is pre-seeded
	//! to avoid numerical instabilities due to empty or near empty bins (caused e.g. by having to assume log(0) = 0 for empty bins) 
	bool debug_mode; //! decides whether logging data will be printed for debugging purposes; 
	//! only matters if logging is enabled at compile time

	//! value constructor
	KLDParams(const AMParams *am_params,
		int _n_bins, double _pre_seed,
		bool _partition_of_unity, bool _debug_mode);
	//! default/copy constructor
	KLDParams(const KLDParams *params = nullptr);
};

struct KLDDist : AMDist{
	typedef bool is_kdtree_distance;
	typedef double ElementType;
	typedef double ResultType;
	KLDDist(const string &_name, const unsigned int _feat_size) :
		AMDist(_name), feat_size(_feat_size){}
	double operator()(const double* a, const double* b,
		size_t size, double worst_dist = -1) const override;
	inline double accum_dist(const double& a, const double& b, int) const{
		double ratio = a / b;
		return ratio > 0 ? a * log(ratio) : 0;
	}
private:
	const unsigned int feat_size;
};


// (nagative) Kullback–Leibler Divergence
class KLD : public AppearanceModel{
public:
	typedef KLDParams ParamType;
	typedef KLDDist DistType;

	KLD(const ParamType *kld_params = nullptr);

	void initializeSimilarity() override;
	void updateSimilarity(bool prereq_only = 1) override;
	void initializeGrad() override;
	void initializeHess() override;

	void updateInitGrad() override;
	void updateCurrGrad() override;

	void cmptInitHessian(MatrixXd &hessian, const MatrixXd &curr_pix_jacobian) override;
	void cmptCurrHessian(MatrixXd &hessian, const MatrixXd &curr_pix_jacobian) override;

	void cmptInitHessian(MatrixXd &init_hessian, const MatrixXd &init_pix_jacobian,
		const MatrixXd &init_pix_hessian) override;
	void cmptCurrHessian(MatrixXd &curr_hessian, const MatrixXd &curr_pix_jacobian,
		const MatrixXd &curr_pix_hessian) override;

	/**
	Support for FLANN library
	*/
	const DistType* getDistFunc() override{
		return new DistType(name, feat_size);
	}
	void initializeDistFeat() override;
	unsigned int getDistFeatSize() override { return feat_size; }
	void updateDistFeat() override {
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
	VectorXd init_hist, curr_hist;
	MatrixXd init_hist_mat, curr_hist_mat;

	//! n_bins X N gradients of the individual histograms w.r.t. pixel values
	MatrixXd init_hist_grad, curr_hist_grad;
	MatrixXd init_hist_hess, curr_hist_hess;
	VectorXd init_grad_factor, curr_grad_factor;
	VectorXd init_hist_log, curr_hist_log;

	int feat_size;
	VectorXd feat_vec;

	// only used internally to increase speed by offlining as many computations as possible;
	MatrixX2i _std_bspl_ids;
	MatrixX2i _init_bspl_ids;
	MatrixX2i _curr_bspl_ids;
};

_MTF_END_NAMESPACE

#endif