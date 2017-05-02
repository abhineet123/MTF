#include "FESMBase.h"
#include "../Utilities/miscUtils.h"
#include <time.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <stdexcept>

_MTF_BEGIN_NAMESPACE

template <class AM, class SSM>
FESMBase<AM, SSM >::FESMBase(const ParamType *esm_params,
	const AMParams *am_params, const SSMParams *ssm_params) :
	SearchMethod<AM, SSM>(am_params, ssm_params),
	params(esm_params){
	printf("\n");
	printf("Using FESM tracker with:\n");
	printf("max_iters: %d\n", params.max_iters);
	printf("epsilon: %f\n", params.epsilon);
	printf("sec_ord_hess: %d\n", params.sec_ord_hess);
	printf("enable_spi: %d\n", params.enable_spi);
	printf("spi_thresh: %f\n", params.spi_thresh);
	printf("debug_mode: %d\n", params.debug_mode);

	printf("appearance model: %s\n", am.name.c_str());
	printf("state space model: %s\n", ssm.name.c_str());
	printf("\n");

	name = "esm";
	log_fname = "log/mtf_esm_log.txt";
	time_fname = "log/mtf_esm_times.txt";

	hess_order = params.sec_ord_hess ? "Second" : "First";

	frame_id = 0;
	max_pix_diff = 0;

	ssm_update.resize(ssm.getStateSize());
	jacobian.resize(ssm.getStateSize());
	hessian.resize(ssm.getStateSize(), ssm.getStateSize());

	init_pix_jacobian.resize(am.getPixCount(), ssm.getStateSize());
	curr_pix_jacobian.resize(am.getPixCount(), ssm.getStateSize());
}

template <class AM, class SSM>
void FESMBase<AM, SSM >::initialize(const cv::Mat &corners){
	start_timer();

	frame_id = 0;
	ssm.initialize(corners);
	am.initializePixVals(ssm.getPts());

	if(params.enable_spi){ initializeSPIMask(); }

	initializePixJacobian();
	if(params.sec_ord_hess){
		initializePixHessian();
	}
	am.initializeSimilarity();
	am.initializeGrad();
	am.initializeHess();

	initializeHessian();

	ssm.getCorners(cv_corners_mat);

	end_timer();
	write_interval(time_fname, "w");
}

template <class AM, class SSM>
void FESMBase<AM, SSM >::update(){
	++frame_id;
	write_frame_id(frame_id);

	am.setFirstIter();
	for(int iter_id = 0; iter_id < params.max_iters; iter_id++){
		init_timer();

		// extract pixel values from the current image at the latest known position of the object
		am.updatePixVals(ssm.getPts());
		record_event("am.updatePixVals");

		if(params.enable_spi){ updateSPIMask(); }

		updatePixJacobian();

		// compute the prerequisites for the gradient functions
		am.updateSimilarity();
		record_event("am.update");

		// update the gradient of the error norm w.r.t. current pixel values
		am.updateCurrGrad();
		record_event("am.updateCurrGrad");

		// update the gradient of the error norm w.r.t. initial pixel values
		am.updateInitGrad();
		record_event("am.updateInitGrad");

		updateJacobian();
		updateHessian();

		ssm_update = -hessian.colPivHouseholderQr().solve(jacobian.transpose());
		record_event("ssm_update");

		prev_corners = ssm.getCorners();
		updateSSM();

		//double update_norm = ssm_update.lpNorm<1>();
		double update_norm = (prev_corners - ssm.getCorners()).squaredNorm();
		record_event("update_norm");

		write_data(time_fname);

		if(update_norm < params.epsilon){
			if(params.debug_mode){
				printf("n_iters: %d\n", iter_id + 1);
			}
			break;
		}

		if(params.enable_spi){ showSPIMask(); }

		am.clearFirstIter();
	}
	ssm.getCorners(cv_corners_mat);
}

template <class AM, class SSM>
void FESMBase<AM, SSM >::updateJacobian(){
	mean_pix_jacobian = (init_pix_jacobian + curr_pix_jacobian) / 2.0;
	record_event("mean_pix_jacobian");
	am.cmptCurrJacobian(jacobian, mean_pix_jacobian);
	record_event("am.cmptCurrJacobian");
}


template <class AM, class SSM>
void FESMBase<AM, SSM >::updateHessian(){
	if(params.sec_ord_hess){
		updatePixHessian();
		mean_pix_hessian = (init_pix_hessian + curr_pix_hessian) * 0.5;
		record_event("mean_pix_hessian");
		am.cmptCurrHessian(hessian, mean_pix_jacobian, mean_pix_hessian);
		record_event("am.cmptCurrHessian (second order)");
	} else{
		am.cmptCurrHessian(hessian, mean_pix_jacobian);
		record_event("am.cmptCurrHessian (first order)");
	}
}
template <class AM, class SSM>
void FESMBase<AM, SSM >::initializePixJacobian(){
	ssm.initializeGradPts(am.getGradOffset());
	am.initializePixGrad(ssm.getGradPts());
	ssm.cmptInitPixJacobian(init_pix_jacobian, am.getInitPixGrad());
}

template <class AM, class SSM>
void FESMBase<AM, SSM >::updatePixJacobian(){
	// compute pixel gradient of the current image warped with the current warp
	ssm.updateGradPts(am.getGradOffset());
	record_event("ssm.updateGradPts");

	am.updatePixGrad(ssm.getGradPts());
	record_event("am.updatePixGrad New");

	// multiply the pixel gradient with the SSM Jacobian to get the Jacobian of pixel values w.r.t. SSM parameters
	ssm.cmptInitPixJacobian(curr_pix_jacobian, am.getCurrPixGrad());
	record_event("ssm.cmptInitPixJacobian");
}

template <class AM, class SSM>
void FESMBase<AM, SSM >::initializePixHessian(){
	ssm.initializeHessPts(am.getHessOffset());
	am.initializePixHess(ssm.getPts(), ssm.getHessPts());
	ssm.cmptInitPixHessian(init_pix_hessian, am.getInitPixHess(), am.getInitPixGrad());
}

template <class AM, class SSM>
void FESMBase<AM, SSM >::updatePixHessian(){
	ssm.updateHessPts(am.getHessOffset());
	record_event("ssm.updateHessPts");

	am.updatePixHess(ssm.getPts(), ssm.getHessPts());
	record_event("am.updatePixHess New");

	ssm.cmptInitPixHessian(curr_pix_hessian, am.getCurrPixHess(), am.getCurrPixGrad());
	record_event("ssm.cmptInitPixHessian");
}

template <class AM, class SSM>
void FESMBase<AM, SSM >::updateSSM(){
	ssm.compositionalUpdate(ssm_update);
	record_event("ssm.compositionalUpdate");
}

// support for Selective Pixel Integration

template <class AM, class SSM>
void FESMBase<AM, SSM >::initializeSPIMask(){
#ifndef DISABLE_SPI
	if(!ssm.supportsSPI())
		throw utils::InvalidArgument("FESMBase::initialize : SSM does not support SPI");
	if(!am.supportsSPI())
		throw utils::InvalidArgument("FESMBase::initialize : AM does not support SPI");

	printf("Using Selective Pixel Integration\n");
	pix_mask.resize(am.getPixCount());
	ssm.setSPIMask(pix_mask.data());
	am.setSPIMask(pix_mask.data());

	rel_pix_diff.resize(am.getPixCount());
	pix_mask2.resize(am.getPixCount());
	pix_mask_img = cv::Mat(am.getResX(), am.getResY(), CV_8UC1, pix_mask2.data());
	spi_win_name = "pix_mask_img";
	cv::namedWindow(spi_win_name);
	max_pix_diff = am.getInitPixVals().maxCoeff() - am.getInitPixVals().minCoeff();
	utils::printScalar(max_pix_diff, "max_pix_diff");
#endif
}

template <class AM, class SSM>
void FESMBase<AM, SSM >::updateSPIMask(){
#ifndef DISABLE_SPI
	rel_pix_diff = (am.getInitPixVals() - am.getCurrPixVals()) / max_pix_diff;
	record_event("rel_pix_diff");

	pix_mask = rel_pix_diff.cwiseAbs().array() < params.spi_thresh;
	record_event("pix_mask");

	if(params.debug_mode){
		int active_pixels = pix_mask.count();
		utils::printScalar(active_pixels, "active_pixels", "%d");
	}
#endif	
}
template <class AM, class SSM>
void FESMBase<AM, SSM >::showSPIMask(){
#ifndef DISABLE_SPI
	if(params.enable_spi){
		for(int pix_id = 0; pix_id < am.getPixCount(); pix_id++){
			int x = pix_mask(pix_id);
			pix_mask2(pix_id) = x * 255;
		}
		cv::Mat pix_mask_img_resized;
		cv::resize(pix_mask_img, pix_mask_img_resized, cv::Size(300, 300));
		imshow(spi_win_name, pix_mask_img_resized);
	}
#endif
}
_MTF_END_NAMESPACE

#include "mtf/Macros/register.h"
_REGISTER_TRACKERS(FESMBase);