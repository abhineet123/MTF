#ifndef MTF_FCLK_NT_H
#define MTF_FCLK_NT_H

#include "SearchMethod.h"
#include "mtf/SM/FCLKParams.h"

_MTF_BEGIN_NAMESPACE
namespace nt{
	// Non templated implementation of FCLK
	class FCLK : public SearchMethod {
		init_profiling();
		char *log_fname;
		char *time_fname;

		cv::Mat curr_img_uchar;
		VectorXd curr_patch_eig;
		cv::Mat curr_patch, curr_patch_uchar, curr_patch_resized;

		std::string write_frame_dir;

		void drawGrid();

	public:

		typedef FCLKParams ParamType;
		typedef ParamType::HessType HessType;

		using SearchMethod::am;
		using SearchMethod::ssm;
		using SearchMethod::cv_corners_mat;
		using SearchMethod::name;
		using SearchMethod::initialize;
		using SearchMethod::update;		

		FCLK(AM _am, SSM _ssm, const ParamType *fclk_params = nullptr);
		void initialize(const cv::Mat &corners) override;
		void update() override;
		void setRegion(const cv::Mat& corners) override;

	protected:

		ParamType params;

		// Let S = size of SSM state vector and N = resx * resy = no. of pixels in the object patch

		//! 1 x S Jacobian of the appearance model w.r.t. SSM state vector
		RowVectorXd jacobian;
		//! S x S Hessian of the appearance model w.r.t. SSM state vector
		MatrixXd hessian, init_self_hessian;
		//! N x S jacobians of the pix values w.r.t the SSM state vector 
		MatrixXd init_pix_jacobian, curr_pix_jacobian;
		MatrixXd curr_pix_jacobian_new;

		//! N x S x S hessians of the pixel values w.r.t the SSM state vector stored as a (S*S) x N 2D matrix
		MatrixXd init_pix_hessian, curr_pix_hessian;
		CornersT prev_corners;

		VectorXd state_update, ssm_update, am_update;
		VectorXd inv_ssm_update, inv_am_update;

		VectorXd frame_ssm_update;

		int frame_id;

		int state_size, ssm_state_size, am_state_size;
	};
}



_MTF_END_NAMESPACE

#endif

