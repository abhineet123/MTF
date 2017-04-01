#include "mtf/SM/ESM.h"
#include "mtf/Utilities/miscUtils.h"

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

	name = "esm";
	log_fname = "log/esm_log.txt";
	time_fname = "log/esm_times.txt";

	frame_id = 0;

	printf("Using %s\n", ESMParams::toString(params.jac_type));
	const char *hess_order = params.sec_ord_hess ? "Second" : "First";
	printf("Using %s order %s\n", hess_order, ESMParams::toString(params.hess_type));
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

	df_dp.resize(state_size);
	d2f_dp2.resize(state_size, state_size);
	if(params.hess_type == HessType::SumOfSelf){
		init_d2f_dp2.resize(state_size, state_size);
	}

	dI0_dpssm.resize(am.getPatchSize(), ssm_state_size);
	dIt_dpssm.resize(am.getPatchSize(), ssm_state_size);

	if(params.jac_type == JacType::Original || params.hess_type == HessType::Original){
		mean_dI_dpssm.resize(am.getPatchSize(), ssm_state_size);
	}
	if(params.sec_ord_hess){
		d2I0_dpssm2.resize(ssm_state_size*ssm_state_size, am.getPatchSize());
		if(params.hess_type != HessType::InitialSelf){
			d2It_dpssm2.resize(ssm_state_size*ssm_state_size, am.getPatchSize());
			if(params.hess_type == HessType::Original){
				mean_d2I_dpssm2.resize(ssm_state_size*ssm_state_size, am.getPatchSize());
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

	am.initializePixGrad(ssm.getPts());
	ssm.cmptWarpedPixJacobian(dI0_dpssm, am.getInitPixGrad());

	if(params.sec_ord_hess){
		am.initializePixHess(ssm.getPts());
		ssm.cmptWarpedPixHessian(d2I0_dpssm2, am.getInitPixHess(),
			am.getInitPixGrad());

	}
	if(params.hess_type == HessType::InitialSelf || params.hess_type == HessType::SumOfSelf){
		if(params.sec_ord_hess){
			am.cmptSelfHessian(d2f_dp2, dI0_dpssm, d2I0_dpssm2);
		} else{
			am.cmptSelfHessian(d2f_dp2, dI0_dpssm);
		}
		if(params.hess_type == HessType::SumOfSelf){
			init_d2f_dp2 = d2f_dp2;
		}
	}
	ssm.getCorners(cv_corners_mat);

	end_timer();
	write_interval(time_fname, "w");
}

template <class AM, class SSM>
void ESM<AM, SSM >::update(){
	++frame_id;
	write_frame_id(frame_id);

	double prev_f = 0;
	double lm_delta = params.lm_delta_init;
	bool state_reset = false;

	am.setFirstIter();
	for(int iter_id = 0; iter_id < params.max_iters; ++iter_id){
		init_timer();

		//! extract pixel values from the current image at the latest known position of the object
		am.updatePixVals(ssm.getPts());
		record_event("am.updatePixVals");

		//! compute f and the prerequisites for its gradients
		am.updateSimilarity(false);
		record_event("am.updateSimilarity");

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

		//! update the gradient of f w.r.t. It
		am.updateCurrGrad();
		record_event("am.updateCurrGrad");

		//! update the gradient of f w.r.t. I0
		am.updateInitGrad();
		record_event("am.updateInitGrad");

		am.updatePixGrad(ssm.getPts());
		record_event("am.updatePixGrad");
		ssm.cmptWarpedPixJacobian(dIt_dpssm, am.getCurrPixGrad());
		record_event("ssm.cmptWarpedPixJacobian");

		if(params.jac_type == JacType::Original || params.hess_type == HessType::Original){
			mean_dI_dpssm = (dI0_dpssm + dIt_dpssm) / 2.0;
			record_event("mean_dI_dpssm");
		}
		if(params.sec_ord_hess && params.hess_type != HessType::InitialSelf){
			am.updatePixHess(ssm.getPts());
			record_event("am.updatePixHess");
			ssm.cmptWarpedPixHessian(d2It_dpssm2, am.getCurrPixHess(), am.getCurrPixGrad());
			record_event("ssm.cmptWarpedPixHessian");
		}
		//! compute the Jacobian of the similarity w.r.t. SSM parameters
		switch(params.jac_type){
		case JacType::Original:
			/**
			take the mean at the level of the df_dp of the pixel values wrt SSM parameters,
			then use this mean df_dp to compute the Jacobian of the error norm wrt SSM parameters
			*/
			am.cmptCurrJacobian(df_dp, mean_dI_dpssm);
			record_event("am.cmptCurrJacobian");
			break;
		case JacType::DiffOfJacs:
			//! compute the mean difference between the Jacobians of the error norm w.r.t. initial AND current values of SSM parameters
			am.cmptDifferenceOfJacobians(df_dp, dI0_dpssm, dIt_dpssm);
			df_dp *= 0.5;
			record_event("am.cmptDifferenceOfJacobians");
			break;
		}
		//! compute the Hessian of the similarity w.r.t. SSM parameters
		switch(params.hess_type){
		case HessType::InitialSelf:
			if(params.leven_marq){
				d2f_dp2 = init_d2f_dp2;
			}
			break;
		case HessType::Original:
			if(params.sec_ord_hess){
				mean_d2I_dpssm2 = (d2I0_dpssm2 + d2It_dpssm2) / 2.0;
				record_event("mean_d2I_dpssm2");
				am.cmptCurrHessian(d2f_dp2, mean_dI_dpssm, mean_d2I_dpssm2);
				record_event("am.cmptCurrHessian (second order)");
			} else{
				am.cmptCurrHessian(d2f_dp2, mean_dI_dpssm);
				record_event("am.cmptCurrHessian (first order)");
			}
			break;
		case HessType::SumOfStd:
			if(params.sec_ord_hess){
				am.cmptSumOfHessians(d2f_dp2, dI0_dpssm, dIt_dpssm,
					d2I0_dpssm2, d2It_dpssm2);
				record_event("am.cmptSumOfHessians (second order)");
			} else{
				am.cmptSumOfHessians(d2f_dp2, dI0_dpssm, dIt_dpssm);
				record_event("am.cmptSumOfHessians (first order)");
			}
			d2f_dp2 *= 0.5;
			break;
		case HessType::SumOfSelf:
			if(params.sec_ord_hess){
				am.cmptSelfHessian(d2f_dp2, dIt_dpssm, d2It_dpssm2);
				record_event("am.cmptSelfHessian (second order)");
			} else{
				am.cmptSelfHessian(d2f_dp2, dIt_dpssm);
				record_event("am.cmptSelfHessian (first order)");
			}
			d2f_dp2 = (d2f_dp2 + init_d2f_dp2) * 0.5;
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
			MatrixXd diag_hessian = d2f_dp2.diagonal().asDiagonal();
			d2f_dp2 += lm_delta*diag_hessian;
		}

		state_update = -d2f_dp2.colPivHouseholderQr().solve(df_dp.transpose());
		record_event("state_update");

		ssm_update = state_update.head(ssm_state_size);
		am_update = state_update.tail(am_state_size);

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
void ESM<AM, SSM >::setRegion(const cv::Mat& corners){
	ssm.setCorners(corners);
	// since the above command completely resets the SSM state including its initial points,
	// any quantities that depend on these, like dI0_dpssm and d2I0_dpssm2,
	// must be recomputed along with quantities that depend on them in turn.
	ssm.cmptInitPixJacobian(dI0_dpssm, am.getInitPixGrad());
	if(params.sec_ord_hess){
		ssm.cmptInitPixHessian(d2I0_dpssm2, am.getInitPixHess(), am.getInitPixGrad());
	}
	if(params.hess_type == HessType::InitialSelf || params.hess_type == HessType::SumOfSelf){
		if(params.sec_ord_hess){
			am.cmptSelfHessian(d2f_dp2, dI0_dpssm, d2I0_dpssm2);
		} else{
			am.cmptSelfHessian(d2f_dp2, dI0_dpssm);
		}
		if(params.hess_type == HessType::SumOfSelf){
			init_d2f_dp2 = d2f_dp2;
		}
	}
	ssm.getCorners(cv_corners_mat);
}

_MTF_END_NAMESPACE

#ifndef HEADER_ONLY_MODE
#include "mtf/Macros/register.h"
_REGISTER_TRACKERS(ESM);
#endif