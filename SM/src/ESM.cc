#include "mtf/SM/ESM.h"
#include "mtf/Utilities/miscUtils.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdexcept>

_MTF_BEGIN_NAMESPACE

template <class AM, class SSM>
ESM<AM, SSM >::ESM(const ParamType *esm_params,
	const AMParams *am_params, const SSMParams *ssm_params) :
	SearchMethod<AM, SSM>(am_params, ssm_params),
	params(esm_params){
	printf("\n");
	printf("Using Efficient Second order Minimization SM with:\n");
	printf("max_iters: %d\n", params.max_iters);
	printf("epsilon: %f\n", params.epsilon);
	printf("jac_type: %d\n", params.jac_type);
	printf("hess_type: %d\n", params.hess_type);
	printf("sec_ord_hess: %d\n", params.sec_ord_hess);
	printf("chained_warp: %d\n", params.chained_warp);
	printf("debug_mode: %d\n", params.debug_mode);

	printf("appearance model: %s\n", am.name.c_str());
	printf("state space model: %s\n", ssm.name.c_str());
	printf("\n");

	name = "esm";
	log_fname = "log/esm_log.txt";
	time_fname = "log/esm_times.txt";

	frame_id = 0;

	printf("Using %s\n", ESMParams::toString(params.jac_type));
	const char *hess_order = params.sec_ord_hess ? "Second" : "First";
	printf("Using %s order %s\n", hess_order, ESMParams::toString(params.hess_type));

	ssm_update.resize(ssm.getStateSize());
	jacobian.resize(ssm.getStateSize());
	hessian.resize(ssm.getStateSize(), ssm.getStateSize());

	init_pix_jacobian.resize(am.getPatchSize(), ssm.getStateSize());
	curr_pix_jacobian.resize(am.getPatchSize(), ssm.getStateSize());

	if(params.jac_type == JacType::Original || params.hess_type == HessType::Original){
		mean_pix_jacobian.resize(am.getPatchSize(), ssm.getStateSize());
	}
	if(params.hess_type == HessType::SumOfSelf){
		init_self_hessian.resize(ssm.getStateSize(), ssm.getStateSize());
	}
	if(params.sec_ord_hess){
		init_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPatchSize());
		if(params.hess_type != HessType::InitialSelf){
			curr_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPatchSize());
			if(params.hess_type == HessType::Original){
				mean_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPatchSize());
			}
		}
	}
}

template <class AM, class SSM>
void ESM<AM, SSM >::initialize(const cv::Mat &corners){
	start_timer();

	am.clearInitStatus();
	ssm.clearInitStatus();

	frame_id = 0;

	ssm.initialize(corners, am.getNChannels());

	am.initializePixVals(ssm.getPts());
	am.initializeSimilarity();
	am.initializeGrad();
	am.initializeHess();

	if(params.chained_warp){
		am.initializePixGrad(ssm.getPts());
		ssm.cmptWarpedPixJacobian(init_pix_jacobian, am.getInitPixGrad());
	} else{
		ssm.initializeGradPts(am.getGradOffset());
		am.initializePixGrad(ssm.getGradPts());
		ssm.cmptInitPixJacobian(init_pix_jacobian, am.getInitPixGrad());
	}
	if(params.sec_ord_hess){
		if(params.chained_warp){
			am.initializePixHess(ssm.getPts());
			ssm.cmptWarpedPixHessian(init_pix_hessian, am.getInitPixHess(),
				am.getInitPixGrad());
		} else{
			ssm.initializeHessPts(am.getHessOffset());
			am.initializePixHess(ssm.getPts(), ssm.getHessPts());
			ssm.cmptInitPixHessian(init_pix_hessian, am.getInitPixHess(), am.getInitPixGrad());
		}		
	}
	if(params.hess_type == HessType::InitialSelf || params.hess_type == HessType::SumOfSelf){
		if(params.sec_ord_hess){
			am.cmptSelfHessian(hessian, init_pix_jacobian, init_pix_hessian);
		} else{
			am.cmptSelfHessian(hessian, init_pix_jacobian);
		}
		if(params.hess_type == HessType::SumOfSelf){
			init_self_hessian = hessian;
		}
	}
	ssm.getCorners(cv_corners_mat);

	end_timer();
	write_interval(time_fname, "w");
}

template <class AM, class SSM>
void ESM<AM, SSM >::setRegion(const cv::Mat& corners){
	ssm.setCorners(corners);
	// since the above command completely resets the SSM state including its initial points,
	// any quantities that depend on these, like init_pix_jacobian and init_pix_hessian,
	// must be recomputed along with quantities that depend on them in turn.
	ssm.cmptInitPixJacobian(init_pix_jacobian, am.getInitPixGrad());
	if(params.sec_ord_hess){
		ssm.cmptInitPixHessian(init_pix_hessian, am.getInitPixHess(), am.getInitPixGrad());
	}
	if(params.hess_type == HessType::InitialSelf || params.hess_type == HessType::SumOfSelf){
		if(params.sec_ord_hess){
			am.cmptSelfHessian(hessian, init_pix_jacobian, init_pix_hessian);
		} else{
			am.cmptSelfHessian(hessian, init_pix_jacobian);
		}
		if(params.hess_type == HessType::SumOfSelf){
			init_self_hessian = hessian;
		}
	}
	ssm.getCorners(cv_corners_mat);
}

template <class AM, class SSM>
void ESM<AM, SSM >::update(){
	++frame_id;
	write_frame_id(frame_id);

	am.setFirstIter();
	for(int iter_id = 0; iter_id < params.max_iters; iter_id++){
		init_timer();

		//! extract pixel values from the current image at the latest known position of the object
		am.updatePixVals(ssm.getPts());
		record_event("am.updatePixVals");

		//! compute the prerequisites for the gradient functions
		am.updateSimilarity();
		record_event("am.updateSimilarity");

		//! update the gradient of the error norm w.r.t. current pixel values
		am.updateCurrGrad();
		record_event("am.updateCurrGrad");

		//! update the gradient of the error norm w.r.t. initial pixel values
		am.updateInitGrad();
		record_event("am.updateInitGrad");

		if(params.chained_warp){
			am.updatePixGrad(ssm.getPts());
			record_event("am.updatePixGrad");
			ssm.cmptWarpedPixJacobian(curr_pix_jacobian, am.getCurrPixGrad());
			record_event("ssm.cmptWarpedPixJacobian");
		} else{
			// compute pixel Jacobian
			ssm.updateGradPts(am.getGradOffset());
			record_event("ssm.updateGradPts");
			// compute pixel gradient of the current image warped with the current warp
			am.updatePixGrad(ssm.getGradPts());
			record_event("am.updatePixGrad");
			// multiply the pixel gradient with the SSM Jacobian to get the Jacobian of pixel values w.r.t. SSM parameters
			ssm.cmptInitPixJacobian(curr_pix_jacobian, am.getCurrPixGrad());
			record_event("ssm.cmptInitPixJacobian");
		}		
		if(params.jac_type == JacType::Original || params.hess_type == HessType::Original){
			mean_pix_jacobian = (init_pix_jacobian + curr_pix_jacobian) / 2.0;
			record_event("mean_pix_jacobian");
		}
		if(params.sec_ord_hess && params.hess_type != HessType::InitialSelf){
			if(params.chained_warp){
				am.updatePixHess(ssm.getPts());
				record_event("am.updatePixHess");
				ssm.cmptWarpedPixHessian(curr_pix_hessian, am.getCurrPixHess(), am.getCurrPixGrad());
				record_event("ssm.cmptWarpedPixHessian");
			} else{
				ssm.updateHessPts(am.getHessOffset());
				record_event("ssm.updateHessPts");
				am.updatePixHess(ssm.getPts(), ssm.getHessPts());
				record_event("am.updatePixHess");
				ssm.cmptInitPixHessian(curr_pix_hessian, am.getCurrPixHess(), am.getCurrPixGrad());
				record_event("ssm.cmptInitPixHessian");
			}
		}
		//! compute the Jacobian of the similarity w.r.t. SSM parameters
		switch(params.jac_type){
		case JacType::Original:
			/**
			take the mean at the level of the jacobian of the pixel values wrt SSM parameters, 
			then use this mean jacobian to compute the Jacobian of the error norm wrt SSM parameters
			*/
			am.cmptCurrJacobian(jacobian, mean_pix_jacobian);
			record_event("am.cmptCurrJacobian");
			break;
		case JacType::DiffOfJacs:
			//! compute the mean difference between the Jacobians of the error norm w.r.t. initial AND current values of SSM parameters
			am.cmptDifferenceOfJacobians(jacobian, init_pix_jacobian, curr_pix_jacobian);
			jacobian *= 0.5;
			record_event("am.cmptDifferenceOfJacobians");
			break;
		}
		//! compute the Hessian of the similarity w.r.t. SSM parameters
		switch(params.hess_type){
		case HessType::InitialSelf:
			break;
		case HessType::Original:
			if(params.sec_ord_hess){
				mean_pix_hessian = (init_pix_hessian + curr_pix_hessian) / 2.0;
				record_event("mean_pix_hessian");
				am.cmptCurrHessian(hessian, mean_pix_jacobian, mean_pix_hessian);
				record_event("am.cmptCurrHessian (second order)");
			} else{
				am.cmptCurrHessian(hessian, mean_pix_jacobian);
				record_event("am.cmptCurrHessian (first order)");
			}
			break;
		case HessType::SumOfStd:
			if(params.sec_ord_hess){
				am.cmptSumOfHessians(hessian, init_pix_jacobian, curr_pix_jacobian,
					init_pix_hessian, curr_pix_hessian);
				record_event("am.cmptSumOfHessians (second order)");
			} else{
				am.cmptSumOfHessians(hessian, init_pix_jacobian, curr_pix_jacobian);
				record_event("am.cmptSumOfHessians (first order)");
			}
			hessian *= 0.5;
			break;
		case HessType::SumOfSelf:
			if(params.sec_ord_hess){
				am.cmptSelfHessian(hessian, curr_pix_jacobian, curr_pix_hessian);
				record_event("am.cmptSelfHessian (second order)");
			} else{
				am.cmptSelfHessian(hessian, curr_pix_jacobian);
				record_event("am.cmptSelfHessian (first order)");
			}
			hessian = (hessian + init_self_hessian) * 0.5;
			break;
		case HessType::CurrentSelf:
			if(params.sec_ord_hess){
				am.cmptSelfHessian(hessian, curr_pix_jacobian, curr_pix_hessian);
				record_event("am.cmptSelfHessian (second order)");
			} else{
				am.cmptSelfHessian(hessian, curr_pix_jacobian);
				record_event("am.cmptSelfHessian (first order)");
			}
			break;
		case HessType::Std:
			if(params.sec_ord_hess){
				am.cmptCurrHessian(hessian, curr_pix_jacobian, curr_pix_hessian);
				record_event("am.cmptCurrHessian (second order)");
			} else{
				am.cmptCurrHessian(hessian, curr_pix_jacobian);
				record_event("am.cmptCurrHessian (first order)");
			}
			break;
		}
		ssm_update = -hessian.colPivHouseholderQr().solve(jacobian.transpose());
		record_event("ssm_update");

		prev_corners = ssm.getCorners();

		ssm.compositionalUpdate(ssm_update);
		record_event("ssm.compositionalUpdate");

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
		am.clearFirstIter();
	}
	ssm.getCorners(cv_corners_mat);
}

_MTF_END_NAMESPACE

#ifndef HEADER_ONLY_MODE
#include "mtf/Macros/register.h"
_REGISTER_TRACKERS(ESM);
#endif