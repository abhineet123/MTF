#ifndef MTF_ESM_H
#define MTF_ESM_H

#include "SearchMethod.h"
#include "ESMParams.h"

_MTF_BEGIN_NAMESPACE

template<class AM, class SSM>
class ESM : public SearchMethod < AM, SSM > {

public:
	typedef ESMParams ParamType;
	ParamType params;

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

	int frame_id;

	Matrix24d prev_corners;

	//! N x S jacobians of the pixel values w.r.t the SSM state vector where N = resx * resy
	//! is the no. of pixels in the object patch
	MatrixXd init_pix_jacobian, curr_pix_jacobian, mean_pix_jacobian;
	MatrixXd init_pix_hessian, curr_pix_hessian, mean_pix_hessian;

	VectorXd ssm_update;

	//! 1 x S Jacobian of the AM error norm w.r.t. SSM state vector
	RowVectorXd jacobian;
	//! S x S Hessian of the AM error norm w.r.t. SSM state vector
	MatrixXd hessian, init_self_hessian;

	init_profiling();
	char *time_fname;
	char *log_fname;
};
_MTF_END_NAMESPACE

#endif

