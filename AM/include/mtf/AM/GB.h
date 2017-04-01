#ifndef MTF_GB_H
#define MTF_GB_H

#define GB_ADDITIVE_UPDATE 0

#include "IlluminationModel.h"

_MTF_BEGIN_NAMESPACE
struct GBParams : ILMParams{
	bool additive_update;
	//! value constructor
	GBParams(ILMParams *ilm_params, bool _additive_update);
	//! default/copy constructor
	GBParams(const GBParams *params = nullptr);
};
// global Gain and Bias illumination model
class GB : public IlluminationModel{
public:
	typedef GBParams ParamType;
	GB(const ParamType *_gb_params = nullptr);
	virtual ~GB(){}
	void initialize(double *p) override{
		p[0] = p[1] = 0;		
	}
	int getStateSize() const override{ return 2; };
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

	PixHessType getPixHessType() override { return d2f_dg2_type; }
	void parseSamplerSigma(VectorXd &out_sigma, const VectorXd &in_sigma) override;
	void parseSamplerMean(VectorXd &out_mean, const VectorXd &in_mean) override;

protected:
	ParamType params;

};

_MTF_END_NAMESPACE

#endif



