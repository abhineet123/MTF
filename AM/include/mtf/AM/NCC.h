#ifndef MTF_NCC_H
#define MTF_NCC_H

#include "AppearanceModel.h"

_MTF_BEGIN_NAMESPACE

struct NCCParams : AMParams{
	bool fast_hess; 

	//! value constructor
	NCCParams(const AMParams *am_params,
		bool _fast_hess);
	//! default/copy constructor
	NCCParams(const NCCParams *params = nullptr);
};

struct NCCDist : AMDist{
	typedef bool is_kdtree_distance;
	typedef double ElementType;
	typedef double ResultType;
	NCCDist(const string &_name, unsigned int _patch_size) : 
		AMDist(_name), patch_size(_patch_size){}
	double operator()(const double* a, const double* b,
		size_t size, double worst_dist = -1) const override;
	double accum_dist(const double& a, const double& b, int) const{ return -a*b; }
private:
	unsigned int patch_size;
};

//! Normalized Cross Correlation
class NCC : public AppearanceModel{
public:
	typedef NCCParams ParamType;
	typedef NCCDist DistType;

	NCC(const ParamType *ncc_params = nullptr, const int _n_channels = 1);

	double getLikelihood() const override;
	//-------------------------------initialize functions------------------------------------//
	void initializeSimilarity() override;
	void initializeGrad() override;
	void initializeHess() override {}

	//-------------------------------update functions------------------------------------//
	void updateSimilarity(bool prereq_only = true) override;
	void updateInitGrad() override;
	// nothing is done here since curr_grad is same as and shares memory with  curr_pix_diff
	void updateCurrGrad() override;

	void cmptInitJacobian(RowVectorXd &df_dp,const MatrixXd &dI0_dp) override;
	void cmptCurrJacobian(RowVectorXd &df_dp, const MatrixXd &dIt_dp) override;
	void cmptDifferenceOfJacobians(RowVectorXd &df_dp_diff,
		const MatrixXd &dI0_dpssm, const MatrixXd &dIt_dpssm)  override;

	void cmptInitHessian(MatrixXd &init_hessian, const MatrixXd &init_pix_jacobian) override;
	void cmptInitHessian(MatrixXd &init_hessian, const MatrixXd &init_pix_jacobian,
		const MatrixXd &init_pix_hessian) override;

	void cmptCurrHessian(MatrixXd &curr_hessian, const MatrixXd &curr_pix_jacobian) override;
	void cmptCurrHessian(MatrixXd &curr_hessian, const MatrixXd &curr_pix_jacobian,
		const MatrixXd &curr_pix_hessian) override;

	void cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian) override;
	void cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian,
		const MatrixXd &curr_pix_hessian) override{
		cmptSelfHessian(self_hessian, curr_pix_jacobian);
	}

	void estimateOpticalFlow(std::vector<cv::Point2f> &out_pts, const cv::Mat &prev_img,
		const std::vector<cv::Point2f> &prev_pts, cv::Size search_window, int n_pts,
		int max_iters, double term_eps, bool const_grad=true) const override;
	void updateModel(const Matrix2Xd& curr_pts) override;

	/*Support for FLANN library*/
	VectorXd curr_feat_vec;
	const DistType* getDistPtr() override{
		return new DistType(name, patch_size);
	}
	void updateDistFeat(double* feat_addr) override;
	void updateDistFeat() override{
		updateDistFeat(curr_feat_vec.data());
	}
	unsigned int  getDistFeatSize() override{ return patch_size; }
	void initializeDistFeat() override{
		curr_feat_vec.resize(getDistFeatSize());
	}
	const double* getDistFeat() override{ return curr_feat_vec.data(); }

#ifndef DISABLE_SPI
	bool supportsSPI() const override{ return n_channels == 1; }
#endif

private:
	ParamType params;

	//! mean of the initial and current pixel values
	double I0_mean, It_mean;
	double a, b, c;
	double bc, b2c;

	VectorXd I0_cntr, It_cntr;
	VectorXd I0_cntr_c, It_cntr_b;
	VectorXd df_dI0_ncntr, df_dIt_ncntr;
	double df_dI0_ncntr_mean, df_dIt_ncntr_mean;
	bool use_running_avg;
};

_MTF_END_NAMESPACE

#endif