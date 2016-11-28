#ifndef MTF_SPSS_H
#define MTF_SPSS_H

#include "AppearanceModel.h"

#define SPSS_K 0.01
#define SPSS_PIX_MAPPER nullptr

_MTF_BEGIN_NAMESPACE

struct SPSSParams : AMParams{
	double k;
	ImageBase *pix_mapper;

	SPSSParams(const AMParams *am_params,
		double _k, 
		ImageBase *_pix_mapper);
	SPSSParams(const SPSSParams *params = nullptr);
};

//! Sum of Pixelwise Structural Similarity
class SPSS : public AppearanceModel{
public:
	typedef SPSSParams ParamType;

	SPSS(const ParamType *spss_params = nullptr, const int _n_channels = 1);

	double getLikelihood() const override;

	//-------------------------------initialize functions------------------------------------//
	void initializePixVals(const Matrix2Xd& init_pts) override;
	void initializeSimilarity() override;
	void initializeGrad() override;
	void initializeHess() override{}

	//-------------------------------update functions------------------------------------//
	void updatePixVals(const Matrix2Xd& curr_pts) override;

	void updateSimilarity(bool prereq_only = true) override;
	void updateInitGrad() override;
	// nothing is done here since curr_grad is same as and shares memory with  curr_pix_diff
	void updateCurrGrad() override;

	void cmptInitHessian(MatrixXd &init_hessian, const MatrixXd &init_pix_jacobian) override;
	void cmptInitHessian(MatrixXd &init_hessian, const MatrixXd &init_pix_jacobian,
		const MatrixXd &init_pix_hessian) override;

	void cmptCurrHessian(MatrixXd &curr_hessian, const MatrixXd &curr_pix_jacobian) override;
	void cmptCurrHessian(MatrixXd &curr_hessian, const MatrixXd &curr_pix_jacobian,
		const MatrixXd &curr_pix_hessian) override;

	void cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian) override;
	void cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian,
		const MatrixXd &curr_pix_hessian) override;

	/*Support for FLANN library*/
	typedef bool is_kdtree_distance;
	typedef double ElementType;
	typedef double ResultType;
	int getDistFeatSize() override{ return patch_size; }
	void initializeDistFeat() override{}
	void updateDistFeat(double* feat_addr) override;
	const double* getDistFeat() override{ return getCurrPixVals().data(); }
	void updateDistFeat() override{}
	double operator()(const double* a, const double* b, size_t size, double worst_dist = -1) const override;
	ResultType accum_dist(const ElementType& a, const ElementType& b, int) const;

protected:
	ParamType params;

	double c;

	VectorXd curr_err_vec;
	VectorXd curr_err_vec_num, curr_err_vec_den;

	VectorXd pix_vals_prod;
	VectorXd init_pix_vals_sqr;
	VectorXd curr_pix_vals_sqr;
};

_MTF_END_NAMESPACE

#endif