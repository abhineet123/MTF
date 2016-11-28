#include "mtf/SM/FCLK.h"
#include "mtf/Utilities/miscUtils.h"
#include <stdexcept>

_MTF_BEGIN_NAMESPACE

template <class AM, class SSM>
FCLK<AM, SSM >::FCLK(const ParamType *fclk_params,
	const AMParams *am_params, const SSMParams *ssm_params) :
	SearchMethod<AM, SSM>(am_params, ssm_params),
	params(fclk_params) {

	printf("\n");
	printf("Using Forward Compositional Lucas Kanade SM with:\n");
	printf("max_iters: %d\n", params.max_iters);
	printf("epsilon: %f\n", params.epsilon);
	printf("hess_type: %d\n", params.hess_type);
	printf("sec_ord_hess: %d\n", params.sec_ord_hess);
	printf("chained_warp: %d\n", params.chained_warp);
	printf("enable_learning: %d\n", params.enable_learning);
	printf("debug_mode: %d\n", params.debug_mode);
	printf("appearance model: %s\n", am.name.c_str());
	printf("state space model: %s\n", ssm.name.c_str());
	printf("\n");

	name = "fclk";
	log_fname = "log/mtf_fclk_log.txt";
	time_fname = "log/mtf_fclk_times.txt";
	frame_id = 0;

	const char *hess_order = params.sec_ord_hess ? "Second" : "First";
	printf("Using %s order %s Hessian\n", hess_order,
		FCLKParams::toString(params.hess_type));

	ssm_update.resize(ssm.getStateSize());
	curr_pix_jacobian.resize(am.getPatchSize(), ssm.getStateSize());
	curr_pix_jacobian_new.resize(am.getPatchSize(), ssm.getStateSize());
	jacobian.resize(ssm.getStateSize());
	hessian.resize(ssm.getStateSize(), ssm.getStateSize());

	if(params.hess_type == HessType::InitialSelf){
		init_pix_jacobian.resize(am.getPatchSize(), ssm.getStateSize());
		if(params.sec_ord_hess){
			init_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPatchSize());
		}
	} else if(params.sec_ord_hess){
		curr_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPatchSize());
	}
}

template <class AM, class SSM>
void FCLK<AM, SSM>::initialize(const cv::Mat &corners){
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
	if(params.sec_ord_hess){
		if(params.chained_warp){
			am.initializePixHess(ssm.getPts());
		} else{
			ssm.initializeHessPts(am.getHessOffset());
			am.initializePixHess(ssm.getPts(), ssm.getHessPts());
		}
	}
	if(params.hess_type == HessType::InitialSelf){
		if(params.chained_warp){
			ssm.cmptWarpedPixJacobian(init_pix_jacobian, am.getInitPixGrad());
		} else{
			ssm.cmptInitPixJacobian(init_pix_jacobian, am.getInitPixGrad());
		}
		if(params.sec_ord_hess){
			if(params.chained_warp){
				ssm.cmptWarpedPixHessian(init_pix_hessian, am.getInitPixHess(),
					am.getInitPixGrad());
			} else{
				ssm.cmptInitPixHessian(init_pix_hessian, am.getInitPixHess(),
					am.getInitPixGrad());
			}
			am.cmptSelfHessian(hessian, init_pix_jacobian, init_pix_hessian);
		} else{
			am.cmptSelfHessian(hessian, init_pix_jacobian);
		}
	}
	ssm.getCorners(cv_corners_mat);

	end_timer();
	write_interval(time_fname, "w");
	//#ifdef ENABLE_PROFILING
	//	utils::printScalarToFile("cmp_times", NULL, "log/ssm_cmp_times.txt", "%s", "w");
	//#endif
}

template <class AM, class SSM>
void FCLK<AM, SSM >::setRegion(const cv::Mat& corners){
	ssm.setCorners(corners);
	// since the above command completely resets the SSM state including its initial points,
	// any quantities that depend on these, like init_pix_jacobian and init_pix_hessian,
	// must be recomputed along with quantities that depend on these in turn.
	if(params.hess_type == HessType::InitialSelf){
		ssm.cmptInitPixJacobian(init_pix_jacobian, am.getInitPixGrad());
		if(params.sec_ord_hess){
			ssm.cmptInitPixHessian(init_pix_hessian, am.getInitPixHess(),
				am.getInitPixGrad());
			am.cmptSelfHessian(hessian, init_pix_jacobian, init_pix_hessian);
		} else{
			am.cmptSelfHessian(hessian, init_pix_jacobian);
		}
	}
	ssm.getCorners(cv_corners_mat);
}

template <class AM, class SSM>
void FCLK<AM, SSM>::update(){

	++frame_id;
	write_frame_id(frame_id);

	am.setFirstIter();
	for(int i = 0; i < params.max_iters; i++){
		init_timer();

		am.updatePixVals(ssm.getPts()); record_event("am.updatePixVals");
		am.updateSimilarity(); record_event("am.updateSimilarity");
		am.updateCurrGrad(); record_event("am.updateCurrGrad");

		if(params.chained_warp){
			am.updatePixGrad(ssm.getPts());
			record_event("am.updatePixGrad");
			ssm.cmptWarpedPixJacobian(curr_pix_jacobian, am.getCurrPixGrad());
			record_event("ssm.cmptWarpedPixJacobian");

		} else{
			// compute pixel Jacobian
			ssm.updateGradPts(am.getGradOffset());
			record_event("ssm.updateGradPts");
			am.updatePixGrad(ssm.getGradPts());
			record_event("am.updatePixGrad");
			ssm.cmptInitPixJacobian(curr_pix_jacobian, am.getCurrPixGrad());
			record_event("ssm.cmptInitPixJacobian");
		}

		//! compute pixel Hessian
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

		//! compute similarity Jacobian
		am.cmptCurrJacobian(jacobian, curr_pix_jacobian); 
		record_event("am.cmptCurrJacobian");

		//! compute similarity Hessian
		switch(params.hess_type){
		case HessType::InitialSelf:
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

		double update_norm = (prev_corners - ssm.getCorners()).squaredNorm();
		record_event("update_norm");

		write_data(time_fname);

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
_REGISTER_TRACKERS(FCLK);
#endif