#ifndef MTF_NGF_H
#define MTF_NGF_H

#include "AppearanceModel.h"
#include <Eigen/Sparse>

_MTF_BEGIN_NAMESPACE

struct NGFParams : AMParams{
	//! estimate of noise level in the image
	double eta;
	//! use SSD formulation of NGF (not implemented completely yet); 
	bool use_ssd;
	//! value constructor
	NGFParams(const AMParams *am_params,
		double _eta, bool _use_ssd);
	//! default/copy constructor
	NGFParams(const NGFParams *params = nullptr);
};

struct NGFDist : AMDist{
	typedef double ElementType;
	typedef double ResultType;
	NGFDist(const string &_name, bool _use_ssd,
		unsigned int _patch_size);
	double operator()(const double* a, const double* b,
		size_t size, double worst_dist = -1) const override;
private:
	bool use_ssd;
	unsigned int patch_size;
};

//! Normalized Gradient Fields
class NGF : public AppearanceModel{
public:
	typedef NGFParams ParamType;
	typedef NGFDist DistType;
	typedef SparseMatrix<double> SpMat;
	typedef Triplet<double> SpTr;

	NGF(const ParamType *ngf_params = nullptr, const int _n_channels = 1);

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
	void updateCurrGrad() override;

	//-------------------------------interfacing functions------------------------------------//
	void cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian) override;
	void cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian,
		const MatrixXd &curr_pix_hessian) override{
		cmptSelfHessian(self_hessian, curr_pix_jacobian);
	};

	// -------------------- distance feature functions -------------------- //
	int feat_size;
	VectorXd curr_feat_vec;
	const DistType* getDistPtr() override{
		return new DistType(name, params.use_ssd, patch_size);
	}
	unsigned int getDistFeatSize() override{ return feat_size; }
	void initializeDistFeat() override;
	void updateDistFeat(double* feat_addr) override;
	const double* getDistFeat() override{ return curr_feat_vec.data(); }
	void updateDistFeat() override{ updateDistFeat(curr_feat_vec.data()); }

protected:
	ParamType params;
	double epsilon;

	PixGradT norm_dI0_dx, norm_dIt_dx;
	Matrix2Xd _init_pts, _curr_pts;
	VectorXd fac_t, fac_0, rc;
	SpMat dr_dIt, dr_dI0;
	VectorXd grad_I0_x, grad_I0_y, grad_It_x, grad_It_y;
	VectorXd grad_I0_norm, grad_It_norm;
	VectorXd grad_I0_squared_norm, grad_It_squared_norm;

};

_MTF_END_NAMESPACE

#endif