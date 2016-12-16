#ifndef MTF_FALK_NT_H
#define MTF_FALK_NT_H

#include "SearchMethod.h"
#include "mtf/SM/FALKParams.h"

_MTF_BEGIN_NAMESPACE
namespace nt{

	class FALK : public SearchMethod {

		init_profiling();
		char *log_fname;
		char *time_fname;

		cv::Mat curr_img_uchar;
		VectorXd curr_patch_eig;
		cv::Mat curr_patch, curr_patch_uchar, curr_patch_resized;

		std::string write_frame_dir;

		void drawGrid();

	public:
		typedef FALKParams ParamType;
		typedef ParamType::HessType HessType;

		ParamType params;
		using SearchMethod::initialize;
		using SearchMethod::update;		

		FALK(AM _am, SSM _ssm, const ParamType *fclk_params = nullptr);
		void initialize(const cv::Mat &corners) override;
		void update() override;	

	private:
		// Let S = size of SSM state vector and N = resx * resy = no. of pixels in the object patch

		//! 1 x S Jacobian of the AM similarity function w.r.t. SSM state vector
		RowVectorXd jacobian;
		//! S x S Hessian of the AM similarity function w.r.t. SSM state vector
		MatrixXd init_self_hessian, hessian;
		//! N x S jacobians of the pixel values w.r.t the SSM state vector 
		MatrixXd init_pix_jacobian, curr_pix_jacobian;
		//! N x S x S hessians of the pixel values w.r.t the SSM state vector stored as a (S*S) x N 2D matrix
		MatrixXd init_pix_hessian, curr_pix_hessian;

		Matrix24d prev_corners;
		VectorXd ssm_update;
		int frame_id;
	};
}
_MTF_END_NAMESPACE

#endif

