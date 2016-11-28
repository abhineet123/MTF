#ifndef MTF_FESMBASE_H
#define MTF_FESMBASE_H

#include "SearchMethod.h"

#define FESM_MAX_ITERS 10
#define FESM_UPD_THRESH 0.01
#define FESM_SEC_ORD_HESS false
#define FESM_ENABLE_SPI false
#define FESM_SPI_THRESH 10
#define FESM_DEBUG_MODE false

_MTF_BEGIN_NAMESPACE

struct FESMParams{

	enum class JacType{ Original, DiffOfJacs };
	enum class HessType {
		Original, SumOfStd, SumOfSelf,
		InitialSelf, CurrentSelf, Std
	};

	int max_iters; //! maximum iterations of the FESMBase algorithm to run for each frame
	double upd_thresh; //! maximum L1 norm of the state update vector at which to stop the iterations
	bool sec_ord_hess;

	bool enable_spi;
	double spi_thresh;
	bool debug_mode; //! decides whether logging data will be printed for debugging purposes; 
	//! only matters if logging is enabled at compile time

	// value constructor
	FESMParams(int _max_iters, double _upd_thresh,
		bool _sec_ord_hess,
		bool _enable_spi, double _spi_thresh,
		bool _debug_mode){
		max_iters = _max_iters;
		upd_thresh = _upd_thresh;
		sec_ord_hess = _sec_ord_hess;
		enable_spi = _enable_spi;
		spi_thresh = _spi_thresh;
		debug_mode = _debug_mode;
	}
	// default and copy constructor
	FESMParams(FESMParams *params = nullptr) :
		max_iters(FESM_MAX_ITERS),
		upd_thresh(FESM_UPD_THRESH),
		sec_ord_hess(FESM_SEC_ORD_HESS),
		enable_spi(FESM_ENABLE_SPI),
		spi_thresh(FESM_SPI_THRESH),
		debug_mode(FESM_DEBUG_MODE){
		if(params){
			max_iters = params->max_iters;
			upd_thresh = params->upd_thresh;
			sec_ord_hess = params->sec_ord_hess;
			enable_spi = params->enable_spi;
			spi_thresh = params->spi_thresh;
			debug_mode = params->debug_mode;
		}
	}
};

template<class AM, class SSM>
class FESMBase : public SearchMethod < AM, SSM > {
protected:
	init_profiling();
	char *time_fname;
	char *log_fname;

	string hess_order;

	void initializeSPIMask();
	void updateSPIMask();
	void showSPIMask();

public:
	typedef FESMParams ParamType;
	ParamType params;

	using SearchMethod<AM, SSM> ::am;
	using SearchMethod<AM, SSM> ::ssm;
	using typename SearchMethod<AM, SSM> ::AMParams;
	using typename SearchMethod<AM, SSM> ::SSMParams;
	using SearchMethod<AM, SSM> ::cv_corners_mat;
	using SearchMethod<AM, SSM> ::name;
	using SearchMethod<AM, SSM> ::initialize;
	using SearchMethod<AM, SSM> ::update;

	int frame_id;
	VectorXc pix_mask2;
	VectorXb pix_mask;
	VectorXd rel_pix_diff;
	cv::Mat pix_mask_img;
	double max_pix_diff;
	char* spi_win_name;

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

	FESMBase(const ParamType *nesm_params = nullptr,
		const AMParams *am_params = nullptr,
		const SSMParams *ssm_params = nullptr);

	void initialize(const cv::Mat &corners) override;
	void update() override;

	virtual void initializeHessian(){}
	virtual void updateJacobian();
	virtual void updateHessian();

	// functions re implemented by AFESMBase to get the additive variant
	void initializePixJacobian();
	void updatePixJacobian();
	void initializePixHessian();
	void updatePixHessian();
	void updateSSM();
};
_MTF_END_NAMESPACE

#endif

