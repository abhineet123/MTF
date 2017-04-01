#ifndef MTF_MI_H
#define MTF_MI_H

#define MI_N_BINS 8
#define MI_PRE_SEED 10
#define MI_POU false
#define MI_PIX_MAPPER nullptr
#define MI_DEBUG_MODE false

#include "AppearanceModel.h"

_MTF_BEGIN_NAMESPACE

struct MIParams : AMParams{
	//! no. of bins in the histograms used internally - dimensionality of the MI error vector will be n_bins * n_bins; 
	//! if partition_of_unity is enabled, this should be 2 more than the desired no. of bins (w.r.t normalized pixel range)
	//! since the actual range within which the pixel values are normalized is 2 less than this value to avoid
	//!	boundary conditions while computing the contribution of each pixel to different bins by ensuring that pixels with the maximum and
	//! minimum values contribute to all 4 bins required by the bspl function of degree 3 used here;
	int n_bins; 
	 //! initial value with which each bin of the joint histogram is pre-seeded
	//! to avoid numerical instabilities due to empty or near empty bins
	//! (caused e.g. by having to assume log(0) = 0 for empty bins) 
	double pre_seed;
	//! decides whether the partition of unity constraint has to be strictly observed for border bins;
	//! if enabled, the pixel values will be normalized in the range [1, n_bins-2] so each pixel contributes to all 4 bins
	bool partition_of_unity;
	ImageBase *pix_mapper;

	bool debug_mode; //! decides whether logging data will be printed for debugging purposes; 
	//! only matters if logging is enabled at compile time

	//! value constructor
	MIParams(const AMParams *am_params,
		int _n_bins, double _pre_seed,
		bool _partition_of_unity,		
		ImageBase *_pix_mapper,
		bool _debug_mode);
	//! default/copy constructor
	MIParams(const MIParams *params = nullptr);
};

class MI : public AppearanceModel{
public:

	typedef MIParams ParamType;

	ParamType params;

	MI(const ParamType *mi_params = nullptr, const int _n_channels = 1);

	double max_similarity;
	double getLikelihood() const override;

	void initializePixVals(const Matrix2Xd& init_pts) override;
	void initializeSimilarity() override;
	void initializeGrad() override;
	void initializeHess() override;

	void updatePixVals(const Matrix2Xd& curr_pts) override;
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

	//static const utils::BSpline3WithDiffPtr bSpline3_arr[4];

	//-----------------------------------functor support-----------------------------------//
	int feat_size;
	VectorXd feat_vec;

	typedef double ElementType;
	typedef double ResultType;

	double log_hist_norm_mult;

	void initializeDistFeat() override{
		feat_vec.resize(feat_size);
	}
	void updateDistFeat() override{
		updateDistFeat(feat_vec.data());
	}
	const double* getDistFeat() override{ return feat_vec.data(); }
	void updateDistFeat(double* feat_addr) override;
	double operator()(const double* hist1_mat_addr, const double* hist2_mat_addr,
		size_t hist_mat_size, double worst_dist = -1) const override;
	int getDistFeatSize() override{ return feat_size; }

private:

	char *log_fname;
	char *time_fname;

	//! value with which to preseed the individual histograms
	double hist_pre_seed;
	//! multiplicative factor for normalizing histograms
	double hist_norm_mult;
	int joint_hist_size;

	// let A = joint_hist_size = n_bins*n_bins and N = n_pix = no. of pixels
	//! n_bins x n_bins joint histograms; 
	MatrixXd joint_hist;
	VectorXd init_hist, curr_hist;
	MatrixXd init_hist_mat, curr_hist_mat;

	//! (n_bins*n_bins) X N gradients of the (flattened) current joint histogram w.r.t. initial and current pixel values
	MatrixXd init_joint_hist_grad, curr_joint_hist_grad;
	//! n_bins X N gradients of the individual histograms w.r.t. pixel values
	MatrixXd init_hist_grad, curr_hist_grad;
	MatrixXd init_hist_hess, curr_hist_hess;
	MatrixXd init_grad_factor, curr_grad_factor;

	VectorXd init_hist_log, curr_hist_log;
	MatrixXd joint_hist_log;

	MatrixXd self_joint_hist, self_joint_hist_log;
	MatrixXd self_grad_factor;

	// only used internally to increase speed by offlining as many computations as possible;
	MatrixX2i _std_bspl_ids;
	MatrixX2i _init_bspl_ids;
	MatrixX2i _curr_bspl_ids;
	MatrixXi _linear_idx;

	void cmptSelfHist();
};

_MTF_END_NAMESPACE

#endif