#ifndef MTF_ICLK_H
#define MTF_ICLK_H

#include "SearchMethod.h"
#include "ICLKParams.h"

_MTF_BEGIN_NAMESPACE

template<class AM, class SSM>
class ICLK : public SearchMethod < AM, SSM > {

private:
	init_profiling();
	char *log_fname;
	char *time_fname;

public:
	typedef ICLKParams ParamType;
	typedef typename ParamType::HessType HessType;
	ParamType params;

	using SearchMethod<AM, SSM> ::am;
	using SearchMethod<AM, SSM> ::ssm;
	using typename SearchMethod<AM, SSM> ::AMParams;
	using typename SearchMethod<AM, SSM> ::SSMParams;
	using SearchMethod<AM, SSM> ::cv_corners_mat;
	using SearchMethod<AM, SSM> ::name;

	using SearchMethod<AM, SSM> ::initialize;
	using SearchMethod<AM, SSM> ::update;

	// Let S = size of SSM state vector and N = resx * resy = no. of pixels in the object patch
	//! 1 x S Jacobian of the AM error norm w.r.t. SSM state vector
	RowVectorXd jacobian;
	//! S x S Hessian of the AM error norm w.r.t. SSM state vector
	MatrixXd hessian;
	//! N x S jacobians of the pixel values w.r.t the SSM state vector 
	MatrixXd init_pix_jacobian, curr_pix_jacobian;
	//! N x S x S hessians of the pixel values w.r.t the SSM state vector stored as a (S*S) x N 2D matrix
	MatrixXd init_pix_hessian, curr_pix_hessian;

	Matrix24d prev_corners;
	VectorXd ssm_update, inv_update;
	int frame_id;

	ICLK(const ParamType *iclk_params = nullptr,
		const AMParams *am_params = nullptr, const SSMParams *ssm_params = nullptr);

	void initialize(const cv::Mat &corners) override;
	void update() override;
	void setRegion(const cv::Mat& corners) override;
};
_MTF_END_NAMESPACE

#endif

