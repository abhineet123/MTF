#ifndef MTF_ICLK_NT_H
#define MTF_ICLK_NT_H

#include "SearchMethod.h"
#include "mtf/SM/ICLKParams.h"


_MTF_BEGIN_NAMESPACE

namespace nt{

	class ICLK : public SearchMethod{

	public:
		typedef ICLKParams ParamType;
		typedef ParamType::HessType HessType;

		using SearchMethod::initialize;
		using SearchMethod::update;		

		ICLK(AM _am, SSM _ssm,
		 const ParamType *iclk_params = nullptr);

		void initialize(const cv::Mat &corners) override;
		void update() override;
		void setRegion(const cv::Mat& corners) override;

	protected:

		ParamType params;
		// Let S = size of SSM state vector and N = resx * resy = no. of pixels in the object patch
		//! 1 x S Jacobian of the AM error norm w.r.t. SSM state vector
		RowVectorXd df_dp;
		//! S x S Hessian of the AM error norm w.r.t. SSM state vector
		MatrixXd d2f_dp2, d2f_dp2_orig;
		//! N x S jacobians of the pixel values w.r.t the SSM state vector 
		MatrixXd dI0_dpssm, dIt_dpssm;
		//! N x S x S hessians of the pixel values w.r.t the SSM state vector stored as a (S*S) x N 2D matrix
		MatrixXd d2I0_dpssm2, d2It_dpssm2;

		Matrix24d prev_corners;
		VectorXd state_update, ssm_update, am_update;
		VectorXd inv_ssm_update, inv_am_update;

		int state_size, ssm_state_size, am_state_size;
		int frame_id;

	private:
		init_profiling();
		char *log_fname;
		char *time_fname;
	};
}
_MTF_END_NAMESPACE

#endif

