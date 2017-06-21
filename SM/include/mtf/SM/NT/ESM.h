#ifndef MTF_ESM_NT_H
#define MTF_ESM_NT_H

#include "SearchMethod.h"
#include "mtf/SM/ESMParams.h"
#include "mtf/Utilities/spiUtils.h"

_MTF_BEGIN_NAMESPACE
namespace nt{	
	class ESM : public SearchMethod {
	public:
		typedef ESMParams ParamType;
		typedef ParamType::JacType JacType;
		typedef ParamType::HessType HessType;
		typedef ParamType::SPIType SPIType;

		using SearchMethod::initialize;
		using SearchMethod::update;	

		ESM(AM _am, SSM _ssm,
		 const ParamType *esm_params = nullptr);

		void initialize(const cv::Mat &corners) override;
		void update() override;
		void setRegion(const cv::Mat& corners) override;

	protected:

		ParamType params;

		init_profiling();

		int frame_id;
		VectorXc pix_mask2;
		VectorXb pix_mask;
		cv::Mat pix_mask_img;
		char* spi_win_name;

		Matrix24d prev_corners;

		//! N x S jacobians of the pixel values w.r.t the SSM state vector where N = resx * resy
		//! is the no. of pixels in the object patch
		MatrixXd init_pix_jacobian, curr_pix_jacobian, mean_pix_jacobian;
		MatrixXd init_pix_hessian, curr_pix_hessian, mean_pix_hessian;

		VectorXd state_update, ssm_update, am_update;
		VectorXd inv_ssm_update, inv_am_update;
		unsigned int state_size, ssm_state_size, am_state_size;

		//! 1 x S Jacobian of the AM error norm w.r.t. SSM state vector
		RowVectorXd jacobian;
		//! S x S Hessian of the AM error norm w.r.t. SSM state vector
		MatrixXd hessian, init_self_hessian;

		bool spi_enabled;
		std::unique_ptr<utils::spi::Base> spi;

		char *time_fname;
		char *log_fname;

		void cmptJacobian();
		void cmptHessian();

		//! support for Selective Pixel Integration
		void initializeSPIMask();
		void updateSPIMask();
		void showSPIMask();

		//! functions re implemented by AESM to get the additive variant
		virtual void initializePixJacobian();
		virtual void updatePixJacobian();
		virtual void initializePixHessian();
		virtual void updatePixHessian();
		virtual void updateState();
	};
}
_MTF_END_NAMESPACE

#endif

