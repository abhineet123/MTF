#ifndef MTF_IALK_H
#define MTF_IALK_H

#include "SearchMethod.h"
#include "IALKParams.h"

_MTF_BEGIN_NAMESPACE

template<class AM, class SSM>
class IALK : public SearchMethod < AM, SSM > {
public:

	typedef IALKParams ParamType;
	typedef typename ParamType::HessType HessType;

	using SearchMethod<AM, SSM> ::am;
	using SearchMethod<AM, SSM> ::ssm;
	using typename SearchMethod<AM, SSM> ::AMParams;
	using typename SearchMethod<AM, SSM> ::SSMParams;
	using SearchMethod<AM, SSM> ::cv_corners_mat;
	using SearchMethod<AM, SSM> ::name;

	using SearchMethod<AM, SSM> ::initialize;
	using SearchMethod<AM, SSM> ::update;	

	IALK(const ParamType *ialk_params = nullptr,
		const AMParams *am_params = nullptr, const SSMParams *ssm_params = nullptr);

	void initialize(const cv::Mat &corners) override;
	void update() override;

protected:

	ParamType params;

	// Let S = size of SSM state vector and N = resx * resy = no. of pixels in the object patch

	//! 1 x S Jacobian of the AM error norm w.r.t. SSM state vector
	RowVectorXd jacobian;
	//! S x S Hessian of the AM error norm w.r.t. SSM state vector
	MatrixXd init_self_hessian, hessian;
	//! N x S jacobians of the pix values w.r.t the SSM state vector where N = resx * resy
	//! is the no. of pixels in the object patch
	//! N x S jacobians of the pix values w.r.t the SSM state vector 
	MatrixXd init_pix_jacobian, curr_pix_jacobian;
	//! N x S x S hessians of the pixel values w.r.t the SSM state vector stored as a (S*S) x N 2D matrix
	MatrixXd init_pix_hessian, curr_pix_hessian;

	Matrix24d prev_corners;
	VectorXd ssm_update;
	Matrix3d warp_update;
	int frame_id;


	init_profiling();
	char *log_fname;
	char *time_fname;

};
_MTF_END_NAMESPACE

#endif

