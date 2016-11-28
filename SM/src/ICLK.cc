#include "mtf/SM/ICLK.h"
#include "mtf/Utilities/miscUtils.h"
#include <stdexcept>

_MTF_BEGIN_NAMESPACE

template <class AM, class SSM>
ICLK<AM, SSM >::ICLK(const ParamType *iclk_params,
	const AMParams *am_params, const SSMParams *ssm_params) :
	SearchMethod<AM, SSM>(am_params, ssm_params),
	params(iclk_params){

	printf("\n");
	printf("Using Inverse Compositional Lucas Kanade SM with:\n");
	printf("max_iters: %d\n", params.max_iters);
	printf("epsilon: %f\n", params.epsilon);
	printf("update_ssm: %d\n", params.update_ssm);
	printf("chained_warp: %d\n", params.chained_warp);
	printf("hess_type: %d\n", params.hess_type);
	printf("sec_ord_hess: %d\n", params.sec_ord_hess);
	printf("enable_learning: %d\n", params.enable_learning);
	printf("debug_mode: %d\n", params.debug_mode);

	printf("appearance model: %s\n", am.name.c_str());
	printf("state space model: %s\n", ssm.name.c_str());
	printf("\n");

	name = "iclk";
	log_fname = "log/mtf_iclk_log.txt";
	time_fname = "log/mtf_iclk_times.txt";
	frame_id = 0;

	const char *hess_order = params.sec_ord_hess ? "Second" : "First";
	printf("Using %s order %s Hessian\n", 
		hess_order, ICLKParams::toString(params.hess_type));
	init_pix_jacobian.resize(am.getPatchSize(), ssm.getStateSize());
	if(params.hess_type == HessType::CurrentSelf){
		curr_pix_jacobian.resize(am.getPatchSize(), ssm.getStateSize());
	}
	jacobian.resize(ssm.getStateSize());
	hessian.resize(ssm.getStateSize(), ssm.getStateSize());
	ssm_update.resize(ssm.getStateSize());
	inv_update.resize(ssm.getStateSize());

	if(params.sec_ord_hess){
		if(params.hess_type == HessType::CurrentSelf){
			curr_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPatchSize());
		} else {
			init_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPatchSize());
		}
	}
}

template <class AM, class SSM>
void ICLK<AM, SSM >::initialize(const cv::Mat &corners){
	start_timer();

	am.clearInitStatus();
	ssm.clearInitStatus();

	ssm.initialize(corners, am.getNChannels());

	am.initializePixVals(ssm.getPts());
	am.initializeSimilarity();
	am.initializeGrad();
	am.initializeHess();

	if(params.chained_warp){
		am.initializePixGrad(ssm.getPts());
	} else{
		ssm.initializeGradPts(am.getGradOffset());
		am.initializePixGrad(ssm.getGradPts());
	}
	if(params.chained_warp){
		ssm.cmptWarpedPixJacobian(init_pix_jacobian, am.getInitPixGrad());
	} else{
		ssm.cmptInitPixJacobian(init_pix_jacobian, am.getInitPixGrad());
	}
	am.cmptInitJacobian(jacobian, init_pix_jacobian);

	if(params.sec_ord_hess){
		if(params.sec_ord_hess){
			if(params.chained_warp){
				am.initializePixHess(ssm.getPts());
			} else{
				ssm.initializeHessPts(am.getHessOffset());
				am.initializePixHess(ssm.getPts(), ssm.getHessPts());
			}
		}
		if(params.hess_type != HessType::CurrentSelf){
			if(params.chained_warp){
				ssm.cmptWarpedPixHessian(init_pix_hessian, am.getInitPixHess(), 
					am.getInitPixGrad());
			} else{
				ssm.cmptInitPixHessian(init_pix_hessian, am.getInitPixHess(),
					am.getInitPixGrad());
			}
		}
	}
	if(params.hess_type == HessType::InitialSelf){
		if(params.sec_ord_hess){
			am.cmptSelfHessian(hessian, init_pix_jacobian, init_pix_hessian);
		} else{
			am.cmptSelfHessian(hessian, init_pix_jacobian);
		}
	}
	ssm.getCorners(cv_corners_mat);

	end_timer();
	write_interval(time_fname, "w");
}

template <class AM, class SSM>
void ICLK<AM, SSM >::setRegion(const cv::Mat& corners){
	ssm.setCorners(corners);
	if(params.update_ssm){
		// since the above command completely resets the SSM state including its initial points,
		// any quantities that depend on these, like init_pix_jacobian and init_pix_hessian,
		// must be recomputed along with quantities that depend on these in turn.
		ssm.cmptWarpedPixJacobian(init_pix_jacobian, am.getInitPixGrad());
		am.cmptInitJacobian(jacobian, init_pix_jacobian);
		if(params.hess_type != HessType::CurrentSelf){
			if(params.sec_ord_hess){
				ssm.cmptWarpedPixHessian(init_pix_hessian, am.getInitPixHess(), am.getInitPixGrad());
			}
			if(params.hess_type == HessType::InitialSelf){
				if(params.sec_ord_hess){
					am.cmptSelfHessian(hessian, init_pix_jacobian, init_pix_hessian);
				} else{
					am.cmptSelfHessian(hessian, init_pix_jacobian);
				}
			}
		}
	}
	ssm.getCorners(cv_corners_mat);
}

template <class AM, class SSM>
void ICLK<AM, SSM >::update(){
	++frame_id;
	write_frame_id(frame_id);

	am.setFirstIter();
	for(int i = 0; i < params.max_iters; i++){
		init_timer();

		am.updatePixVals(ssm.getPts());
		record_event("am.updatePixVals");

		am.updateSimilarity();
		record_event("am.updateSimilarity");

		am.updateInitGrad();
		record_event("am.updateInitGrad");

		am.cmptInitJacobian(jacobian, init_pix_jacobian);
		record_event("am.cmptInitJacobian");

		switch(params.hess_type){
		case HessType::InitialSelf:
			break;
		case HessType::CurrentSelf:
			if(params.chained_warp){
				am.updatePixGrad(ssm.getPts());
				record_event("am.updatePixGrad");
				ssm.cmptWarpedPixJacobian(curr_pix_jacobian, am.getCurrPixGrad());
				record_event("ssm.cmptWarpedPixJacobian");
			} else{
				ssm.updateGradPts(am.getGradOffset());
				record_event("ssm.updateGradPts");
				am.updatePixGrad(ssm.getGradPts());
				record_event("am.updatePixGrad");
				ssm.cmptInitPixJacobian(curr_pix_jacobian, am.getCurrPixGrad());
				record_event("ssm.cmptInitPixJacobian");
			}
			if(params.sec_ord_hess){
				if(params.chained_warp){
					am.updatePixHess(ssm.getPts());
					record_event("am.updatePixHess");
					ssm.cmptWarpedPixHessian(curr_pix_hessian, am.getCurrPixHess(),
						am.getCurrPixGrad());
					record_event("ssm.cmptWarpedPixHessian");
				} else{
					ssm.updateHessPts(am.getHessOffset());
					record_event("ssm.updateHessPts");
					am.updatePixHess(ssm.getPts(), ssm.getHessPts());
					record_event("am.updatePixHess");
					ssm.cmptInitPixHessian(curr_pix_hessian, am.getCurrPixHess(), am.getCurrPixGrad());
					record_event("ssm.cmptInitPixHessian");
				}
				am.cmptSelfHessian(hessian, curr_pix_jacobian, curr_pix_hessian);
				record_event("am.cmptSelfHessian (second order)");
			} else{
				am.cmptSelfHessian(hessian, curr_pix_jacobian);
				record_event("am.cmptSelfHessian (first order)");
			}
			break;
		case HessType::Std:
			if(params.sec_ord_hess){
				am.cmptInitHessian(hessian, init_pix_jacobian, init_pix_hessian);
				record_event("am.cmptInitHessian (second order)");
			} else{
				am.cmptInitHessian(hessian, init_pix_jacobian);
				record_event("am.cmptInitHessian (first order)");
			}
			break;
		}

		ssm_update = -hessian.colPivHouseholderQr().solve(jacobian.transpose());
		record_event("ssm_update");

		ssm.invertState(inv_update, ssm_update);
		record_event("ssm.invertState");

		prev_corners = ssm.getCorners();

		ssm.compositionalUpdate(inv_update);
		record_event("ssm.compositionalUpdate");

		double update_norm = (prev_corners - ssm.getCorners()).squaredNorm();
		record_event("update_norm");

		if(update_norm < params.epsilon){
			if(params.debug_mode){
				printf("n_iters: %d\n", i + 1);
			}
			break;
		}
		am.clearFirstIter();
	}
	if(params.enable_learning){
		am.updateModel(ssm.getPts());
	}
	ssm.getCorners(cv_corners_mat);
}

_MTF_END_NAMESPACE

#ifndef HEADER_ONLY_MODE
#include "mtf/Macros/register.h"
_REGISTER_TRACKERS(ICLK);
#endif
