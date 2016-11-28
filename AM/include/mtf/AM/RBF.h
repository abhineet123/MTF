#ifndef MTF_RBF_H
#define MTF_RBF_H

#define RBF_ADDITIVE_UPDATE 0
#define RBF_N_CTRL_PTS 3

#include "IlluminationModel.h"

_MTF_BEGIN_NAMESPACE
struct RBFParams : ILMParams{
	bool additive_update;
	//! no. of sub regions in horizontal and vertical directions
	int n_ctrl_pts_x, n_ctrl_pts_y;
	//! value constructor
	RBFParams(ILMParams *ilm_params, bool _additive_update,
		int _n_ctrl_pts_x, int _n_ctrl_pts_y);
	//! default/copy constructor
	RBFParams(const RBFParams *params = nullptr);
};
//! Radial Basis Function illumination model
class RBF : public IlluminationModel{
public:
	typedef RBFParams ParamType;
	RBF(const ParamType *_pgb_params = nullptr);
	virtual ~RBF(){}
	void initialize(double *p) override{
		Map<VectorXd>(p, state_size).fill(0);
	}
	int getStateSize() const override{ return state_size; };
	void apply(double *g, const double *I, const double *p) override;
	void invert(double *inv_p, const double *p) override;
	void update(double *new_p, const double *old_p, const double *dp) override;

	void cmptParamJacobian(double *df_dp, const double *df_dg,
		const double *I, const double *p) override;
	void cmptPixJacobian(double *df_dI, const double *df_dg,
		const double *I, const double *p) override;

	void cmptParamHessian(double *d2f_dp2, const double *d2f_dg2,
		const double *I, const double *p) override;
	void cmptPixHessian(double *d2f_dI2, const double *d2f_dg2,
		const double *I, const double *p) override;
	void cmptCrossHessian(double *d2f_dp_dI, const double *d2f_dg2,
		const double *I, const double *p) override;

	void cmptParamHessian(double *d2f_dp2, const double *d2f_dg2,
		const double *df_dg, const double *I, const double *p) override{
		cmptParamHessian(d2f_dp2, d2f_dg2, I, p);
	}
	void cmptPixHessian(double *d2f_dI2, const double *d2f_dg2,
		const double *df_dg, const double *I, const double *p) override{
		cmptPixHessian(d2f_dI2, d2f_dg2, I, p);
	}
	void cmptCrossHessian(double *d2f_dp_dI, const double *d2f_dg2,
		const double *df_dg, const double *I, const double *p) override;

	PixHessType getPixHessType() override;
	void parseSamplerSigma(VectorXd &out_sigma, const VectorXd &in_sigma) override;
	void parseSamplerMean(VectorXd &out_mean, const VectorXd &in_mean) override;
private:
	ParamType params;
	int state_size;
	int n_ctrl_pts;
	int region_size_x, region_size_y;
	int bias_id;
	MatrixX2i region_bounds_x, region_bounds_y;
	MatrixXi ctrl_pts_idx;//used for indexing the sub region locations
	MatrixX2d ctrl_pts;
	MatrixXd pix_wts;	
};

_MTF_END_NAMESPACE

#endif



