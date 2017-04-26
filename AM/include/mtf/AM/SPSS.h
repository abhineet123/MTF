#ifndef MTF_SPSS_H
#define MTF_SPSS_H

#include "AppearanceModel.h"

_MTF_BEGIN_NAMESPACE

struct SPSSParams : AMParams{
	double k;
	ImageBase *pix_mapper;

	SPSSParams(const AMParams *am_params,
		double _k, 
		ImageBase *_pix_mapper);
	SPSSParams(const SPSSParams *params = nullptr);
};

struct SPSSDist : AMDist{
	typedef bool is_kdtree_distance;
	typedef double ElementType;
	typedef double ResultType;
	SPSSDist(const string &_name, double _c) : AMDist(_name), c(_c){}
	double operator()(const double* a, const double* b,
		size_t size, double worst_dist = -1) const override;
	double accum_dist(const ElementType& a,
		const ElementType& b, int) const{
		return -(2 * a*b + c) / (a*a + b*b + c);
	}
private:
	double c;
};

//! Sum of Pixelwise Structural Similarity
class SPSS : public AppearanceModel{
public:
	typedef SPSSParams ParamType;
	typedef SPSSDist DistType;

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

	void cmptInitHessian(MatrixXd &d2f_dp2, const MatrixXd &dI0_dp) override;
	void cmptInitHessian(MatrixXd &d2f_dp2, const MatrixXd &dI0_dp,
		const MatrixXd &d2I0_dp2) override;

	void cmptCurrHessian(MatrixXd &d2f_dp2, const MatrixXd &dIt_dp) override;
	void cmptCurrHessian(MatrixXd &d2f_dp2, const MatrixXd &dIt_dp,
		const MatrixXd &d2It_dp2) override;

	void cmptSelfHessian(MatrixXd &d2f_dp2, const MatrixXd &dIt_dp) override;
	void cmptSelfHessian(MatrixXd &d2f_dp2, const MatrixXd &dIt_dp,
		const MatrixXd &d2It_dp2) override;

	/*Support for FLANN library*/
	const DistType* getDistPtr() override{
		return new DistType(name, c);
	}
	unsigned int getDistFeatSize() override{ return patch_size; }
	void initializeDistFeat() override{}
	void updateDistFeat(double* feat_addr) override;
	const double* getDistFeat() override{ return getCurrPixVals().data(); }
	void updateDistFeat() override{}

protected:
	ParamType params;

	double c;
	VectorXd f_vec, f_vec_den;
	VectorXd I0_sqr, It_sqr;
};

_MTF_END_NAMESPACE

#endif