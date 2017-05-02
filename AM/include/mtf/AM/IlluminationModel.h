#ifndef MTF_ILLUMINATION_MODEL_H
#define MTF_ILLUMINATION_MODEL_H

#include "mtf/Macros/common.h"
#include "mtf/Utilities/excpUtils.h"

#define ilm_not_implemeted(func_name) \
	throw mtf::utils::FunctonNotImplemented(cv::format("%s :: %s :: Not implemented Yet", name.c_str(), #func_name))

_MTF_BEGIN_NAMESPACE

struct ILMParams{
	//! horizontal and vertical sampling resolutions
	int resx, resy;
	ILMParams(int _resx, int _resy) :resx(_resx), resy(_resy){}
	ILMParams(const ILMParams *ilm_params = nullptr) :
		resx(MTF_RES), resy(MTF_RES){
		if(ilm_params){
			resx = ilm_params->resx;
			resy = ilm_params->resy;
		}
	}
};

/**
Illumination Model is a parametric function that transforms pixel values extracted
from a patch to account for illumination changes
g(I, a): R(N) x R(K) -> R(N)
*/
class IlluminationModel{

public:
	enum class PixHessType{ Constant, Diagonal, General };
	string name;
	bool apply_on_init;
	IlluminationModel(const ILMParams *ilm_params):
		resx(MTF_RES), resy(MTF_RES), d2f_dg2_type(PixHessType::General){
		if(ilm_params) {
			if(ilm_params->resx <= 0 || ilm_params->resy <= 0) {
				throw utils::InvalidArgument("IlluminationModel::Invalid sampling resolution provided");
			}
			resx = ilm_params->resx;
			resy = ilm_params->resy;			
		}
		n_pix = resx*resy;
	}
	virtual ~IlluminationModel(){}
	virtual int getStateSize() const = 0;
	virtual void initialize(double *p){}
	virtual void apply(double *g, const double *I, const double *p) = 0;
	virtual void invert(double *inv_p, const double *p) = 0;
	virtual void update(double *new_p, const double *old_p, const double *dp) = 0;
	/**
	ILM Jacobians
	*/
	virtual	void cmptParamJacobian(double *df_dp, const double *df_dg,
		const double *I, const double *p) = 0;
	virtual void cmptPixJacobian(double *df_dI, const double *df_dg,
		const double *I, const double *p) = 0;
	/**
	NULL/nullptr for d2f_dg2 implies that the corresponding matrix is identity;
	conversely a non negative return imples an identity d2f_dI2 scaled by the return value;
	these conventions are necessary to avoid unnecessary and very expensive matrix multiplications;
	*/
	/**
	Second order ILM hessians
	*/
	//! d2f_dp2 = dg_dp^T*d2f_dg2*dg_dp + df_dg*d2g_dp2
	virtual	void cmptParamHessian(double *d2f_dp2, const double *d2f_dg2,
		const double *df_dg, const double *I, const double *p) = 0;
	//! d2f_dI2 = dg_dI^T*d2f_dg2*dg_dI + df_dg*d2g_dI2
	virtual void cmptPixHessian(double *d2f_dI2, const double *d2f_dg2,
		const double *df_dg, const double *I, const double *p) = 0;
	//! d2f_dp_dI = dg_dp^T*d2f_dg2*dg_dI + df_dg*d2g_dpdI
	virtual void cmptCrossHessian(double *d2f_dp_dI, const double *d2f_dg2,
		const double *df_dg, const double *I, const double *p) = 0;
	/**
	First order ILM hessians
	*/
	//! d2f_dp2 = dg_dp^T*d2f_dg2*dg_dp
	virtual	void cmptParamHessian(double *d2f_dp2, const double *d2f_dg2,
		const double *I, const double *p) = 0;
	//! d2f_dI2 = dg_dI^T*d2f_dg2*dg_dI
	virtual void cmptPixHessian(double *d2f_dI2, const double *d2f_dg2,
		const double *I, const double *p) = 0;
	//! d2f_dp_dI = dg_dp^T*d2f_dg2*dg_dI
	virtual void cmptCrossHessian(double *d2f_dp_dI, const double *d2f_dg2,
		const double *I, const double *p) = 0;

	virtual void setPixHessType(PixHessType _d2f_dg2_type){
		d2f_dg2_type = _d2f_dg2_type;
	}
	virtual PixHessType getPixHessType() = 0;
	virtual void parseSamplerSigma(VectorXd &out_sigma, const VectorXd &in_sigma) = 0;
	virtual void parseSamplerMean(VectorXd &out_mean, const VectorXd &in_mean) = 0;

protected:

	int resx, resy, n_pix;
	PixHessType d2f_dg2_type;
};

_MTF_END_NAMESPACE

#endif



