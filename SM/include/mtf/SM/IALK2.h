#ifndef MTF_IALK2_H
#define MTF_IALK2_H

#include "SearchMethod.h"

#define IALK2_MAX_ITERS 10
#define IALK2_EPSILON 0.01
#define IALK2_HESS_TYPE 0
#define IALK2_SEC_ORD_HESS false
#define IALK2_DEBUG_MODE false

_MTF_BEGIN_NAMESPACE

struct IALK2Params{
	enum class HessType{ InitialSelf, CurrentSelf, Std };

	int max_iters; //! maximum iterations of the IALK2 algorithm to run for each frame
	double epsilon; //! maximum L1 norm of the state update vector at which to stop the iterations
	HessType hess_type;
	bool sec_ord_hess;
	bool debug_mode; //! decides whether logging data will be printed for debugging purposes; 
	//! only matters if logging is enabled at compile time

	IALK2Params(int _max_iters, double _epsilon, 
		HessType _hess_type, bool _sec_ord_hess,
		bool _debug_mode
		){
		max_iters = _max_iters;
		epsilon = _epsilon;
		hess_type = _hess_type;
		sec_ord_hess = _sec_ord_hess;
		debug_mode = _debug_mode;
	}
	IALK2Params(IALK2Params *params = nullptr) :
		max_iters(IALK2_MAX_ITERS), 
		epsilon(IALK2_EPSILON),
		hess_type(static_cast<HessType>(IALK2_HESS_TYPE)),
		sec_ord_hess(IALK2_SEC_ORD_HESS),
		debug_mode(IALK2_DEBUG_MODE){
		if(params){
			max_iters = params->max_iters;
			epsilon = params->epsilon;
			hess_type = params->hess_type;
			sec_ord_hess = params->sec_ord_hess;
			debug_mode = params->debug_mode;
		}
	}
};

template<class AM, class SSM>
class IALK2 : public SearchMethod < AM, SSM > {

public:
	typedef IALK2Params ParamType;
	typedef typename ParamType::HessType HessType;

	using SearchMethod<AM, SSM> ::am;
	using SearchMethod<AM, SSM> ::ssm;
	using typename SearchMethod<AM, SSM> ::AMParams;
	using typename SearchMethod<AM, SSM> ::SSMParams;
	using SearchMethod<AM, SSM> ::cv_corners_mat;
	using SearchMethod<AM, SSM> ::name;

	using SearchMethod<AM, SSM> ::initialize;
	using SearchMethod<AM, SSM> ::update;

	IALK2(const ParamType *iclk_params = NULL,
		const AMParams *am_params = NULL, const SSMParams *ssm_params = NULL);

	void initialize(const cv::Mat &corners) override;
	void update() override;

private:
	ParamType params;

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
	ClockType start_time, end_time;
	std::vector<double> proc_times;
	std::vector<char*> proc_labels;
	char *log_fname;
	char *time_fname;
	void updatePixJacobian();
	void updatePixHessian();

};
_MTF_END_NAMESPACE

#endif

