#ifndef MTF_HACLK_H
#define MTF_HACLK_H

#include "SearchMethod.h"
#include <vector>

#define HACLK_MAX_ITERS 10
#define HACLK_EPSILON 0.01
#define HACLK_REC_INIT_ERR_GRAD false
#define HACLK_DEBUG_MODE false
#define HACLK_HESS_TYPE 0

_MTF_BEGIN_NAMESPACE

struct HACLKParams{
	int max_iters; //! maximum iterations of the HACLK algorithm to run for each frame
	double epsilon; //! maximum L1 norm of the state update vector at which to stop the iterations
	bool rec_init_err_grad; //! decides if the gradient of the error vector w.r.t. initial pix values
	//! is recomputed every time the vector changes
	bool debug_mode; //! decides whether logging data will be printed for debugging purposes; 
	//! only matters if logging is enabled at compile time
	int hess_type;
	std::vector<cv::Mat> converged_corners;

	HACLKParams(int _max_iters, double _epsilon,
		bool _rec_init_err_grad, bool _debug_mode,
		int _hess_type, const std::vector<cv::Mat> &_converged_corners){
		this->max_iters = _max_iters;
		this->epsilon = _epsilon;
		this->rec_init_err_grad = _rec_init_err_grad;
		this->debug_mode = _debug_mode;
		this->hess_type = _hess_type;
		this->converged_corners = _converged_corners;
	}
	HACLKParams(HACLKParams *params = nullptr) :
		max_iters(HACLK_MAX_ITERS), 
		epsilon(HACLK_EPSILON),
		rec_init_err_grad(HACLK_REC_INIT_ERR_GRAD),
		debug_mode(HACLK_DEBUG_MODE),
		hess_type(HACLK_HESS_TYPE){
		if(params){
			max_iters = params->max_iters;
			epsilon = params->epsilon;
			rec_init_err_grad = params->rec_init_err_grad;
			debug_mode = params->debug_mode;
			hess_type = params->hess_type;
			converged_corners = params->converged_corners;
		}
	}
};
// Hessian After Convergence Lucas Kanade Search Method
template<class AM, class SSM>
class HACLK : public SearchMethod < AM, SSM > {
public:

	enum { GaussNewton, Newton, InitialNewton, CurrentNewton,  ConvergedNewton};

	typedef HACLKParams ParamType;

	using SearchMethod<AM, SSM> ::am;
	using SearchMethod<AM, SSM> ::ssm;
	using typename SearchMethod<AM, SSM> ::AMParams;
	using typename SearchMethod<AM, SSM> ::SSMParams;
	using SearchMethod<AM, SSM> ::cv_corners_mat;
	using SearchMethod<AM, SSM> ::name;
	using SearchMethod<AM, SSM> ::initialize;
	using SearchMethod<AM, SSM> ::update;

	HACLK(const ParamType *haclk_params = nullptr,
		const AMParams *am_params = nullptr, const SSMParams *ssm_params = nullptr);

	void initialize(const cv::Mat &corners) override;
	void update() override;

private:
	ParamType params;

	bool use_newton_method;
	// Let S = size of SSM state vector and N = resx * resy = no. of pixels in the object patch

	//! N x S jacobians of the pix values w.r.t the SSM state vector 
	MatrixXd init_pix_jacobian, curr_pix_jacobian;
	//! N x S x S hessians of the pixel values w.r.t the SSM state vector stored as a (S*S) x N 2D matrix
	MatrixXd init_pix_hessian, curr_pix_hessian;

	//! 1 x S Jacobian of the AM error norm w.r.t. SSM state vector
	RowVectorXd similarity_jacobian;
	//! S x S Hessian of the AM error norm w.r.t. SSM state vector
	MatrixXd hessian;

	Matrix24d curr_conv_corners;
	Matrix24d prev_corners;
	VectorXd ssm_update;
	Matrix3d warp_update;

	//VectorXd prev_pix_vals;
	//PixGradT prev_pix_grad;
	//MatrixXd prev_pix_jacobian;

	int n_frames;
	int frame_id;
	VectorXd inv_state;
	ClockType start_time, end_time;
	std::vector<double> proc_times;
	std::vector<char*> proc_labels;
	char *log_fname;
	char *time_fname;
	inline void updateSSM(VectorXd &state_update){
		ssm.compositionalUpdate(state_update);
	}
	inline void resetSSM(VectorXd &state_update){
		ssm.invertState(inv_state, state_update);
		ssm.compositionalUpdate(inv_state);
	}
};
_MTF_END_NAMESPACE

#endif

