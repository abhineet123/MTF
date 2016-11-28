#include "mtf/SM/IALK.h"
#include "mtf/Utilities/miscUtils.h"

_MTF_BEGIN_NAMESPACE

template <class AM, class SSM>
IALK<AM, SSM >::IALK(const ParamType *ialk_params,
	const AMParams *am_params, const SSMParams *ssm_params) :
	SearchMethod<AM, SSM>(am_params, ssm_params),
	params(ialk_params) {
	printf("\n");
	printf("Using Inverse Additive Lucas Kanade SM with:\n");
	printf("max_iters: %d\n", params.max_iters);
	printf("epsilon: %f\n", params.epsilon);
	printf("hess_type: %d\n", params.hess_type);
	printf("debug_mode: %d\n", params.debug_mode);
	printf("appearance model: %s\n", am.name.c_str());
	printf("state space model: %s\n", ssm.name.c_str());
	printf("\n");

	name = "ialk";
	log_fname = "log/mtf_ialk_log.txt";
	time_fname = "log/mtf_ialk_times.txt";
	frame_id = 0;

	const char *hess_order = params.sec_ord_hess ? "Second" : "First";
	switch(params.hess_type){
	case HessType::InitialSelf:
		printf("Using %s order Initial Self Hessian\n", hess_order);
		break;
	case HessType::CurrentSelf:
		printf("Using %s order Current Self Hessian\n", hess_order);
		break;
	case HessType::Std:
		printf("Using %s order Standard Hessian\n", hess_order);
		break;
	default:
		throw std::invalid_argument("Invalid Hessian type provided");
	}
	ssm_update.resize(ssm.getStateSize());
	curr_pix_jacobian.resize(am.getPatchSize(), ssm.getStateSize());
	jacobian.resize(ssm.getStateSize());
	hessian.resize(ssm.getStateSize(), ssm.getStateSize());
	if(params.hess_type == HessType::InitialSelf){
		init_pix_jacobian.resize(am.getPatchSize(), ssm.getStateSize());
	}
	if(params.sec_ord_hess){
		if(params.hess_type == HessType::InitialSelf){
			init_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPatchSize());
		} else {
			curr_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPatchSize());
		}
	}
}

template <class AM, class SSM>
void IALK<AM, SSM >::initialize(const cv::Mat &corners){
	start_timer();

	am.clearInitStatus();
	ssm.clearInitStatus();

	ssm.initialize(corners, am.getNChannels());
	am.initializePixVals(ssm.getPts());
	am.initializePixGrad(ssm.getPts());
	am.initializeSimilarity();
	am.initializeGrad();
	am.initializeHess();

	if(params.sec_ord_hess){
		am.initializePixHess(ssm.getPts());
	}
	if(params.hess_type == HessType::InitialSelf){
		ssm.cmptPixJacobian(init_pix_jacobian, am.getInitPixGrad());
		if(params.sec_ord_hess){
			ssm.cmptPixHessian(init_pix_hessian, am.getInitPixHess(), am.getInitPixGrad());
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
void IALK<AM, SSM >::update(){
	++frame_id;
	write_frame_id(frame_id);

	for(int i = 0; i < params.max_iters; i++){
		init_timer();

		am.updatePixVals(ssm.getPts());
		record_event("am.updatePixVals");

		ssm.cmptApproxPixJacobian(curr_pix_jacobian, am.getInitPixGrad());
		record_event("am.cmptApproxPixJacobian");

		am.updateSimilarity();
		record_event("am.updateSimilarity");

		am.updateCurrGrad();
		record_event("am.updateCurrGrad");

		am.cmptCurrJacobian(jacobian, curr_pix_jacobian);
		record_event("am.cmptCurrJacobian");

		//compute pixel Hessian
		if(params.sec_ord_hess && params.hess_type != HessType::InitialSelf){
			ssm.cmptApproxPixHessian(curr_pix_hessian,
				am.getInitPixHess(), am.getInitPixGrad());
			record_event("ssm.cmptApproxPixHessian");
		}
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
		ssm.additiveUpdate(ssm_update);
		record_event("ssm.additiveUpdate");

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
	ssm.getCorners(cv_corners_mat);

}

_MTF_END_NAMESPACE

#ifndef HEADER_ONLY_MODE
#include "mtf/Macros/register.h"
_REGISTER_TRACKERS(IALK);
#endif