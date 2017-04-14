#ifndef MTF_HESM_H
#define MTF_HESM_H

#include "SearchMethod.h"

#define _MAX_ITERS 10
#define _EPSILON 0.01
#define _REC_INIT_ERR_GRAD false
#define _DEBUG_MODE false

_MTF_BEGIN_NAMESPACE

struct HESMParams{
	int max_iters; //! maximum iterations of the HESM algorithm to run for each frame
	double epsilon; //! maximum L1 norm of the state update vector at which to stop the iterations
	bool rec_init_err_grad; //! decides if the gradient of the error vector w.r.t. initial pix values
	//! is recomputed every time the vector changes
	bool debug_mode; //! decides whether logging data will be printed for debugging purposes; 
	//! only matters if logging is enabled at compile time

	HESMParams(int _max_iters, double _epsilon,
		bool _rec_init_err_grad, bool _debug_mode){
		max_iters = _max_iters;
		epsilon = _epsilon;
		rec_init_err_grad = _rec_init_err_grad;
		debug_mode = _debug_mode;
	}
	HESMParams(HESMParams *params = nullptr) :
		max_iters(_MAX_ITERS),
		epsilon(_EPSILON),
		rec_init_err_grad(_REC_INIT_ERR_GRAD),
		debug_mode(_DEBUG_MODE){
		if(params){
			max_iters = params->max_iters;
			epsilon = params->epsilon;
			rec_init_err_grad = params->rec_init_err_grad;
			debug_mode = params->debug_mode;
		}
	}
};

struct SSMData{
	MatrixXd init_pix_jacobian, curr_pix_jacobian, mean_pix_jacobian;
	RowVectorXd similarity_jacobian;
	VectorXd ssm_update;
	MatrixXd hessian;
	SSMData(){}
	SSMData(int n_pix, int state_vec_size){
		resize(n_pix, state_vec_size);
	}
	void resize(int n_pix, int state_vec_size){
		ssm_update.resize(state_vec_size);
		init_pix_jacobian.resize(n_pix, state_vec_size);
		curr_pix_jacobian.resize(n_pix, state_vec_size);
		mean_pix_jacobian.resize(n_pix, state_vec_size);
		similarity_jacobian.resize(state_vec_size);
		hessian.resize(state_vec_size, state_vec_size);
	}
};

template<class AM, class SSM, class SSM2>
class HESM : public SearchMethod < AM, SSM > {

public:
	typedef HESMParams ParamType;

	using SearchMethod<AM, SSM> ::am;
	using SearchMethod<AM, SSM> ::ssm;
	using typename SearchMethod<AM, SSM> ::AMParams;
	using typename SearchMethod<AM, SSM> ::SSMParams;
	using SearchMethod<AM, SSM> ::resx;
	using SearchMethod<AM, SSM> ::resy;
	using SearchMethod<AM, SSM> ::n_pix;
	using SearchMethod<AM, SSM> ::cv_corners_mat;
	using SearchMethod<AM, SSM> ::cv_corners;
	using SearchMethod<AM, SSM> ::name;
	using SearchMethod<AM, SSM> ::curr_img;
	using SearchMethod<AM, SSM> ::img_height;
	using SearchMethod<AM, SSM> ::img_width;
	using SearchMethod<AM, SSM> ::initialize;
	using SearchMethod<AM, SSM> ::update;

	typedef typename SSM2::ParamType SSM2Params;


	HESM(const ParamType *nesm_params = NULL,
		const AMParams *am_params = NULL, const SSMParams *ssm_params = NULL,
		SSM2Params *ssm2_params = NULL);

	void initialize(const cv::Mat &corners) override;
	void update() override;
	template<class _SSM> void updateSSM(_SSM *_ssm, SSMData &_data);
	//template<class _SSM> void initializeSSM(_SSM *_ssm, SSMData &_data);

	//double computeSSMUpdate();
private:
	ParamType params;

	SSM2 *ssm2;
	SSMData ssm_data, ssm2_data;

	Matrix24d prev_corners;

	//! N x S jacobians of the pixel values w.r.t the SSM state vector where N = resx * resy
	//! is the no. of pixels in the object patch
	//MatrixXd init_pix_jacobian, curr_pix_jacobian, mean_pix_jacobian;
	//MatrixXd init_pix_jacobian2, curr_pix_jacobian2, mean_pix_jacobian2;

	//VectorXd ssm1_update, ssm2_update;

	Matrix3d ssm2_warp_update;

	////! 1 x S Jacobian of the AM error norm w.r.t. SSM state vector
	//RowVectorXd similarity_jacobian, similarity_jacobian2;
	////! S x S Hessian of the AM error norm w.r.t. SSM state vector
	//MatrixXd hessian, hessian2;

	//! A x S mean of the initial and current jacobians of the AM error vector w.r.t. the SSM state vector;
	//MatrixXd err_vec_jacobian;

	int frame_id;
	char *log_fname;
	char *time_fname;

	//! these share memory with their namesake 'Map' variables
	MatrixXd _init_jacobian, _curr_jacobian;
};
_MTF_END_NAMESPACE

#endif

