#include "mtf/SM/FALK.h"
#include "mtf/Utilities/miscUtils.h"

_MTF_BEGIN_NAMESPACE

template <class AM, class SSM>
FALK<AM, SSM >::FALK(const ParamType *falk_params,
	const AMParams *am_params, const SSMParams *ssm_params) :
	SearchMethod<AM, SSM>(am_params, ssm_params),
	params(falk_params) {

	printf("\n");
	printf("Using Forward Additive Lucas Kanade SM with:\n");
	printf("max_iters: %d\n", params.max_iters);
	printf("epsilon: %f\n", params.epsilon);
	printf("enable_learning: %d\n", params.enable_learning);
	printf("debug_mode: %d\n", params.debug_mode);

	printf("appearance model: %s\n", am.name.c_str());
	printf("state space model: %s\n", ssm.name.c_str());
	printf("\n");

	name = "falk";
	log_fname = "log/mtf_falk_log.txt";
	time_fname = "log/mtf_falk_times.txt";
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
		if(params.sec_ord_hess){
			init_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPatchSize());
		}
	} else if(params.sec_ord_hess){
		curr_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getPatchSize());
	}
}

template <class AM, class SSM>
void FALK<AM, SSM >::initialize(const cv::Mat &corners){
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
			ssm.cmptPixHessian(init_pix_hessian, am.getInitPixHess(),
				am.getInitPixGrad());
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
void FALK<AM, SSM >::update(){
	++frame_id;
	write_frame_id(frame_id);

	am.setFirstIter();
	for(int iter_id = 0; iter_id < params.max_iters; iter_id++){
		init_timer();

		am.updatePixVals(ssm.getPts());
		record_event("am.updatePixVals");

		am.updatePixGrad(ssm.getPts());
		record_event("am.updatePixGrad");

		ssm.cmptPixJacobian(curr_pix_jacobian, am.getCurrPixGrad());
		record_event("ssm.cmptPixJacobian");

		// compute the prerequisites for the gradient function
		am.updateSimilarity();
		record_event("am.updateSimilarity");

		am.updateCurrGrad();
		record_event("am.updateCurrGrad");

		am.cmptCurrJacobian(jacobian, curr_pix_jacobian);
		record_event("am.cmptCurrJacobian");

		//compute pixel Hessian
		if(params.sec_ord_hess && params.hess_type != HessType::InitialSelf){
			am.updatePixHess(ssm.getPts());
			record_event("am.updatePixHess");
			ssm.cmptPixHessian(curr_pix_hessian, am.getCurrPixHess(), am.getCurrPixGrad());
			record_event("ssm.cmptPixHessian");
		}

		// compute similarity Hessian
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
		//ssm_update = hessian.inverse() * jacobian.transpose();
		ssm_update = -hessian.colPivHouseholderQr().solve(jacobian.transpose());
		record_event("ssm_update");

		prev_corners = ssm.getCorners();
		ssm.additiveUpdate(ssm_update);
		record_event("ssm.additiveUpdate");

		double update_norm = (prev_corners - ssm.getCorners()).squaredNorm();
		record_event("update_norm");

		if(update_norm < params.epsilon){
			if(params.debug_mode){
				printf("n_iters: %d\n", iter_id + 1);
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
_REGISTER_TRACKERS(FALK);
#endif