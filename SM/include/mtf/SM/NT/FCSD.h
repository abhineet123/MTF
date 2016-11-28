#ifndef MTF_FCSD_NT_H
#define MTF_FCSD_NT_H

#include "SearchMethod.h"

#define FCSD_MAX_ITERS 10
#define FCSD_EPSILON 0.01
#define FCSD_REC_INIT_ERR_GRAD false
#define FCSD_DEBUG_MODE false
#define FCSD_HESS_TYPE 0

_MTF_BEGIN_NAMESPACE

struct FCSDParams{
	int max_iters; //! maximum iterations of the FCGD algorithm to run for each frame
	double epsilon; //! maximum L1 norm of the state update vector at which to stop the iterations
	double learning_rate; //! decides if the gradient of the error vector w.r.t. initial pix values
	//! is recomputed every time the vector changes
	bool debug_mode; //! decides whether logging data will be printed for debugging purposes; 
	//! only matters if logging is enabled at compile time
	int hess_type;

	FCSDParams(int _max_iters, double _epsilon,
		double _learning_rate, bool _debug_mode,
		int _hess_type);
	FCSDParams(const FCSDParams *params = nullptr);
};
// Forward Compositional Steepest Descent
namespace nt{

	class FCSD : public SearchMethod {
		init_profiling();
		char *log_fname;
		char *time_fname;

	public:
		typedef FCSDParams ParamType;

		ParamType params;

		using SearchMethod ::am;
		using SearchMethod ::ssm;
		using SearchMethod ::cv_corners_mat;
		using SearchMethod ::name;
		using SearchMethod ::initialize;
		using SearchMethod ::update;

		FCSD(AM _am, SSM _ssm, const ParamType *fcsd_params = nullptr);

		void initialize(const cv::Mat &corners) override;
		void update() override;

	protected:
		// Let S = size of SSM state vector and N = resx * resy = no. of pixels in the object patch

		//! 1 x S Jacobian of the AM error norm w.r.t. SSM state vector

		MatrixXd hessian;
		RowVectorXd jacobian;
		//! N x S jacobians of the pix values w.r.t the SSM state vector 
		MatrixXd init_pix_jacobian, curr_pix_jacobian;
		VectorXd ssm_update;

		Matrix24d prev_corners;
		Matrix3d warp_update;
		int frame_id;
		double learning_rate;

	};
}
_MTF_END_NAMESPACE

#endif

