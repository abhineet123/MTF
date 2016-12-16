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
	printf("leven_marq: %d\n", params.leven_marq);
	if(params.leven_marq){
		printf("lm_delta_init: %f\n", params.lm_delta_init);
		printf("lm_delta_update: %f\n", params.lm_delta_update);
	}
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
	if(params.leven_marq){
		printf("Using Levenberg Marquardt formulation...\n");
	}

	ssm_state_size = ssm.getStateSize();
	am_state_size = am.getStateSize();
	state_size = ssm_state_size + am_state_size;

	printf("ssm_state_size: %d am_state_size: %d state_size: %d\n",
		ssm_state_size, am_state_size, state_size);

	state_update.resize(state_size);
	ssm_update.resize(ssm_state_size);
	am_update.resize(am_state_size);
	inv_ssm_update.resize(ssm_state_size);
	inv_am_update.resize(am_state_size);

	dIt_dpssm.resize(am.getPatchSize(), ssm_state_size);
	df_dp.resize(state_size);
	d2f_dp2.resize(state_size, state_size);

	if(params.hess_type == HessType::InitialSelf){
		dI0_dpssm.resize(am.getPatchSize(), ssm_state_size);
		if(params.sec_ord_hess){
			d2I0_dpssm2.resize(ssm_state_size*ssm_state_size, am.getPatchSize());
		}
	} else if(params.sec_ord_hess){
		d2It_dpssm2.resize(ssm_state_size*ssm_state_size, am.getPatchSize());
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

	am.initializePixGrad(ssm.getPts());
	if(params.sec_ord_hess){
		am.initializePixHess(ssm.getPts());
	}
	if(params.hess_type == HessType::InitialSelf){
		ssm.cmptWarpedPixJacobian(dI0_dpssm, am.getInitPixGrad());
		if(params.sec_ord_hess){
			ssm.cmptWarpedPixHessian(d2I0_dpssm2, am.getInitPixHess(),
					am.getInitPixGrad());	
			am.cmptSelfHessian(d2f_dp2, dI0_dpssm, d2I0_dpssm2);
		} else{
			am.cmptSelfHessian(d2f_dp2, dI0_dpssm);
		}
		if(params.leven_marq){
			init_d2f_dp2 = d2f_dp2;
		}
	}
	ssm.getCorners(cv_corners_mat);

	end_timer();
	write_interval(time_fname, "w");
}

template <class AM, class SSM>
void FCLK<AM, SSM>::update(){

	++frame_id;
	write_frame_id(frame_id);

	double prev_f = 0;
	double lm_delta = params.lm_delta_init;
	bool state_reset = false;

	am.setFirstIter();
	for(int iter_id = 0; iter_id < params.max_iters; ++iter_id){
		init_timer();

		am.updatePixVals(ssm.getPts()); record_event("am.updatePixVals");
		am.updateSimilarity(false); record_event("am.updateSimilarity");

		if(params.leven_marq && !state_reset){
			double f = am.getSimilarity();
			if(iter_id > 0){
				if(f < prev_f){
					lm_delta *= params.lm_delta_update;
					//! undo the last update
					ssm.invertState(inv_ssm_update, ssm_update);
					ssm.compositionalUpdate(inv_ssm_update);
					am.invertState(inv_am_update, am_update);
					am.updateState(inv_am_update);

					state_reset = true;
					continue;
				}
				if(f > prev_f){
					lm_delta /= params.lm_delta_update;
				}
			}
			prev_f = f;
		}
		state_reset = false;

		am.updateCurrGrad(); record_event("am.updateCurrGrad");

		am.updatePixGrad(ssm.getPts());
		record_event("am.updatePixGrad");
		ssm.cmptWarpedPixJacobian(dIt_dpssm, am.getCurrPixGrad());
		record_event("ssm.cmptWarpedPixJacobian");

		//! compute pixel Hessian
		if(params.sec_ord_hess && params.hess_type != HessType::InitialSelf){
			am.updatePixHess(ssm.getPts());
			record_event("am.updatePixHess");
			ssm.cmptWarpedPixHessian(d2It_dpssm2, am.getCurrPixHess(), am.getCurrPixGrad());
			record_event("ssm.cmptWarpedPixHessian");
		}

		//! compute similarity Jacobian
		am.cmptCurrJacobian(df_dp, dIt_dpssm);
		record_event("am.cmptCurrJacobian");

		//! compute similarity Hessian
		switch(params.hess_type){
		case HessType::InitialSelf:
			if(params.leven_marq){
				d2f_dp2 = init_d2f_dp2;
			}
			break;
		case HessType::CurrentSelf:
			if(params.sec_ord_hess){
				am.cmptSelfHessian(d2f_dp2, dIt_dpssm, d2It_dpssm2);
				record_event("am.cmptSelfHessian (second order)");
			} else{
				am.cmptSelfHessian(d2f_dp2, dIt_dpssm);
				record_event("am.cmptSelfHessian (first order)");
			}
			break;
		case HessType::Std:
			if(params.sec_ord_hess){
				am.cmptCurrHessian(d2f_dp2, dIt_dpssm, d2It_dpssm2);
				record_event("am.cmptCurrHessian (second order)");
			} else{
				am.cmptCurrHessian(d2f_dp2, dIt_dpssm);
				record_event("am.cmptCurrHessian (first order)");
			}
			break;
		}

		if(params.leven_marq){
			MatrixXd diag_d2f_dp2 = d2f_dp2.diagonal().asDiagonal();
			d2f_dp2 += lm_delta*diag_d2f_dp2;
		}

		state_update = -d2f_dp2.colPivHouseholderQr().solve(df_dp.transpose());
		ssm_update = state_update.head(ssm_state_size);
		am_update = state_update.tail(am_state_size);

		record_event("ssm_update");

		prev_corners = ssm.getCorners();

		ssm.compositionalUpdate(ssm_update);
		record_event("ssm.compositionalUpdate");

		am.updateState(am_update);
		record_event("am.updateState");

		double update_norm = (prev_corners - ssm.getCorners()).squaredNorm();
		record_event("update_norm");

		write_data(time_fname);

		if(update_norm < params.epsilon){ break; }

		am.clearFirstIter();
	}

	if(params.enable_learning){
		am.updateModel(ssm.getPts());
	}
	ssm.getCorners(cv_corners_mat);
}

template <class AM, class SSM>
void FCLK<AM, SSM >::setRegion(const cv::Mat& corners){
	ssm.setCorners(corners);
	// since the above command completely resets the SSM state including its initial points,
	// any quantities that depend on these, like dI0_dpssm and d2I0_dpssm2,
	// must be recomputed along with quantities that depend on these in turn.
	if(params.hess_type == HessType::InitialSelf){
		ssm.cmptInitPixJacobian(dI0_dpssm, am.getInitPixGrad());
		if(params.sec_ord_hess){
			ssm.cmptInitPixHessian(d2I0_dpssm2, am.getInitPixHess(),
				am.getInitPixGrad());
			am.cmptSelfHessian(d2f_dp2, dI0_dpssm, d2I0_dpssm2);
		} else{
			am.cmptSelfHessian(d2f_dp2, dI0_dpssm);
		}
	}
	ssm.getCorners(cv_corners_mat);
}

_MTF_END_NAMESPACE

#ifndef HEADER_ONLY_MODE
#include "mtf/Macros/register.h"
_REGISTER_TRACKERS(FCLK);
#endif