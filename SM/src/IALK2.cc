#include "mtf/SM/IALK2.h"
#include "mtf/Utilities/miscUtils.h"
#include <time.h>
#include <stdexcept>

_MTF_BEGIN_NAMESPACE

template <class AM, class SSM>
IALK2<AM, SSM >::IALK2(const ParamType *ialk2_params,
	const AMParams *am_params, const SSMParams *ssm_params) :
	SearchMethod<AM, SSM>(am_params, ssm_params),
	params(ialk2_params){

	printf("\n");
	printf("Using Inverse Additive Lucas Kanade II SM with:\n");
	printf("max_iters: %d\n", params.max_iters);
	printf("epsilon: %f\n", params.epsilon);
	printf("hess_type: %d\n", params.hess_type);
	printf("sec_ord_hess: %d\n", params.sec_ord_hess);
	printf("debug_mode: %d\n", params.debug_mode);

	printf("appearance model: %s\n", am.name.c_str());
	printf("state space model: %s\n", ssm.name.c_str());
	printf("\n");

	name = "ialk2";
	log_fname = "log/mtf_ialk2_log.txt";
	time_fname = "log/mtf_ialk2_times.txt";
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
		throw utils::InvalidArgument("Invalid Hessian type provided");
	}
	init_pix_jacobian.resize(am.getNPix(), ssm.getStateSize());
	jacobian.resize(ssm.getStateSize());
	hessian.resize(ssm.getStateSize(), ssm.getStateSize());
	ssm_update.resize(ssm.getStateSize());
	inv_update.resize(ssm.getStateSize());

	if(params.sec_ord_hess){
		if(params.hess_type == HessType::CurrentSelf){
			curr_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getNPix());
		} else {
			init_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getNPix());
		}
	}
}

template <class AM, class SSM>
void IALK2<AM, SSM >::initialize(const cv::Mat &corners){
	start_timer();

	ssm.initialize(corners);
	am.initializePixVals(ssm.getPts());
	am.initializePixGrad(ssm.getPts());
	am.initializeSimilarity();
	am.initializeGrad();
	am.initializeHess();

	ssm.cmptPixJacobian(init_pix_jacobian, am.getInitPixGrad());
	am.cmptInitJacobian(jacobian, init_pix_jacobian);

	if(params.sec_ord_hess){
		am.initializePixHess(ssm.getPts());
		ssm.cmptPixHessian(init_pix_hessian, am.getInitPixHess(), am.getInitPixGrad());
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
void IALK2<AM, SSM >::update(){
	++frame_id;
	write_frame_id(frame_id);

	am.setFirstIter();
	for(int i = 0; i < params.max_iters; i++){
		init_timer();

		am.updatePixVals(ssm.getPts());
		record_event("am.updatePixVals");

		ssm.cmptPixJacobian(init_pix_jacobian, am.getInitPixGrad());
		record_event("ssm.cmptPixJacobian");

		am.updateSimilarity();
		record_event("am.update");

		am.updateInitGrad();
		record_event("am.updateInitGrad");

		am.cmptInitJacobian(jacobian, init_pix_jacobian);
		record_event("am.cmptInitJacobian");

		switch(params.hess_type){
		case HessType::Std:
			if(params.sec_ord_hess){
				ssm.cmptPixHessian(init_pix_hessian, am.getInitPixHess(), am.getInitPixGrad());
				record_event("ssm.cmptPixHessian");
				am.cmptInitHessian(hessian, init_pix_jacobian, init_pix_hessian);
				record_event("am.cmptInitHessian (second order)");
			} else{
				am.cmptInitHessian(hessian, init_pix_jacobian);
				record_event("am.cmptInitHessian (first order)");
			}
			break;
		case HessType::CurrentSelf:
			updatePixJacobian();
			if(params.sec_ord_hess){
				updatePixHessian();
				am.cmptSelfHessian(hessian, curr_pix_jacobian, curr_pix_hessian);
				record_event("am.cmptSelfHessian (second order)");
			} else{
				am.cmptSelfHessian(hessian, curr_pix_jacobian);
				record_event("am.cmptSelfHessian (first order)");
			}
			break;
		case HessType::InitialSelf:
			break;
		}

		ssm_update = -hessian.colPivHouseholderQr().solve(jacobian.transpose());
		record_event("ssm_update");

		inv_update = -ssm_update;

		prev_corners = ssm.getCorners();

		ssm.additiveUpdate(inv_update);
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
	ssm.getCorners(cv_corners_mat);
}

template <class AM, class SSM>
void IALK2<AM, SSM >::updatePixJacobian(){
	am.updatePixGrad(ssm.getPts());
	record_event("am.updatePixGrad");

	ssm.cmptPixJacobian(curr_pix_jacobian, am.getCurrPixGrad());
	record_event("ssm.cmptPixJacobian");
}

template <class AM, class SSM>
void IALK2<AM, SSM >::updatePixHessian(){
	am.updatePixHess(ssm.getPts());
	record_event("am.updatePixHess");

	ssm.cmptPixHessian(curr_pix_hessian, am.getCurrPixHess(), am.getCurrPixGrad());
	record_event("ssm.cmptPixHessian");
}

_MTF_END_NAMESPACE

#include "mtf/Macros/register.h"
_REGISTER_TRACKERS(IALK2);

