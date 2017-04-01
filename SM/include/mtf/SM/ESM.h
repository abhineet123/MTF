#ifndef MTF_ESM_H
#define MTF_ESM_H

#include "SearchMethod.h"
#include "ESMParams.h"

_MTF_BEGIN_NAMESPACE

template<class AM, class SSM>
class ESM : public SearchMethod < AM, SSM > {

public:
	typedef ESMParams ParamType;

	typedef typename ParamType::JacType JacType;
	typedef typename ParamType::HessType HessType;

	using SearchMethod<AM, SSM> ::am;
	using SearchMethod<AM, SSM> ::ssm;
	using typename SearchMethod<AM, SSM> ::AMParams;
	using typename SearchMethod<AM, SSM> ::SSMParams;
	using SearchMethod<AM, SSM> ::cv_corners_mat;
	using SearchMethod<AM, SSM> ::name;
	using SearchMethod<AM, SSM> ::initialize;
	using SearchMethod<AM, SSM> ::update;	

	ESM(const ParamType *esm_params = nullptr,
		const AMParams *am_params = nullptr, 
		const SSMParams *ssm_params = nullptr);

	void initialize(const cv::Mat &corners) override;
	void update() override;
	void setRegion(const cv::Mat& corners) override;

protected:
	ParamType params;

	//! N x S jacobians of the pixel values w.r.t the SSM state vector where N = resx * resy
	//! is the no. of pixels in the object patch
	MatrixXd dI0_dpssm, dIt_dpssm, mean_dI_dpssm;
	MatrixXd d2I0_dpssm2, d2It_dpssm2, mean_d2I_dpssm2;

	//! 1 x S Jacobian of the AM error norm w.r.t. SSM state vector
	RowVectorXd df_dp;
	//! S x S Hessian of the AM error norm w.r.t. SSM state vector
	MatrixXd d2f_dp2, init_d2f_dp2;

	VectorXd state_update, ssm_update, am_update;
	VectorXd inv_ssm_update, inv_am_update;
	int state_size, ssm_state_size, am_state_size;

	int frame_id;

	Matrix24d prev_corners;

	init_profiling();
	char *time_fname;
	char *log_fname;
};
_MTF_END_NAMESPACE

#endif

