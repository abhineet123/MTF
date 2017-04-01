#include "mtf/SM/NT/ICLK.h"
#include "mtf/Utilities/miscUtils.h"
#include <stdexcept>

_MTF_BEGIN_NAMESPACE
namespace nt{
	
	ICLK::ICLK(AM _am, SSM _ssm, const ParamType *iclk_params) :
		SearchMethod(_am, _ssm), params(iclk_params){
		printf("\n");
		printf("Using Inverse Compositional Lucas Kanade (NT) SM with:\n");
		printf("max_iters: %d\n", params.max_iters);
		printf("epsilon: %f\n", params.epsilon);
		printf("update_ssm: %d\n", params.update_ssm);
		printf("chained_warp: %d\n", params.chained_warp);
		printf("leven_marq: %d\n", params.leven_marq);
		if(params.leven_marq){
			printf("lm_delta_init: %f\n", params.lm_delta_init);
			printf("lm_delta_update: %f\n", params.lm_delta_update);
		}
		printf("enable_learning: %d\n", params.enable_learning);
		printf("hess_type: %d\n", params.hess_type);
		printf("sec_ord_hess: %d\n", params.sec_ord_hess);
		printf("debug_mode: %d\n", params.debug_mode);

		printf("appearance model: %s\n", am->name.c_str());
		printf("state space model: %s\n", ssm->name.c_str());
		printf("\n");

		name = "iclk_nt";
		log_fname = "log/mtf_iclk_nt_log.txt";
		time_fname = "log/mtf_iclk_nt_times.txt";
		frame_id = 0;

		const char *hess_order = params.sec_ord_hess ? "Second" : "First";
		printf("Using %s order %s Hessian\n", hess_order, 
			ICLKParams::toString(params.hess_type));
		if(params.leven_marq){
			printf("Using Levenberg Marquardt formulation...\n");
		}
		ssm_state_size = ssm->getStateSize();
		am_state_size = am->getStateSize();
		state_size = ssm_state_size + am_state_size;
		printf("ssm_state_size: %d am_state_size: %d state_size: %d\n",
			ssm_state_size, am_state_size, state_size);

		dI0_dpssm.resize(am->getPatchSize(), ssm_state_size);
		if(params.hess_type == HessType::CurrentSelf){
			dIt_dpssm.resize(am->getPatchSize(), ssm_state_size);
		}
		df_dp.resize(state_size);
		d2f_dp2.resize(state_size, state_size);

		state_update.resize(state_size);
		ssm_update.resize(ssm_state_size);
		am_update.resize(am_state_size);

		inv_ssm_update.resize(ssm_state_size);
		inv_am_update.resize(am_state_size);

		if(params.sec_ord_hess){
			if(params.hess_type == HessType::CurrentSelf){
				d2It_dpssm2.resize(ssm_state_size*ssm_state_size, am->getPatchSize());
			} else {
				d2I0_dpssm2.resize(ssm_state_size*ssm_state_size, am->getPatchSize());
			}
		}
	}

	
	void ICLK::initialize(const cv::Mat &corners){
		start_timer();

		am->clearInitStatus();
		ssm->clearInitStatus();

		ssm->initialize(corners, am->getNChannels());
		am->initializePixVals(ssm->getPts());
		if(params.chained_warp){
			am->initializePixGrad(ssm->getPts());
		} else{
			ssm->initializeGradPts(am->getGradOffset());
			am->initializePixGrad(ssm->getGradPts());
		}
		am->initializeSimilarity();
		am->initializeGrad();
		am->initializeHess();

		if(params.chained_warp){
			ssm->cmptWarpedPixJacobian(dI0_dpssm, am->getInitPixGrad());
		} else{
			ssm->cmptInitPixJacobian(dI0_dpssm, am->getInitPixGrad());
		}
		am->cmptInitJacobian(df_dp, dI0_dpssm);

		if(params.sec_ord_hess){
			if(params.sec_ord_hess){
				if(params.chained_warp){
					am->initializePixHess(ssm->getPts());
				} else{
					ssm->initializeHessPts(am->getHessOffset());
					am->initializePixHess(ssm->getPts(), ssm->getHessPts());
				}
			}
			if(params.hess_type != HessType::CurrentSelf){
				if(params.chained_warp){
					ssm->cmptWarpedPixHessian(d2I0_dpssm2, am->getInitPixHess(),
						am->getInitPixGrad());
				} else{
					ssm->cmptInitPixHessian(d2I0_dpssm2, am->getInitPixHess(),
						am->getInitPixGrad());
				}
			}
		}
		if(params.hess_type == HessType::InitialSelf){
			if(params.sec_ord_hess){
				am->cmptSelfHessian(d2f_dp2, dI0_dpssm, d2I0_dpssm2);
			} else{
				am->cmptSelfHessian(d2f_dp2, dI0_dpssm);
			}
			if(params.leven_marq){
				d2f_dp2_orig = d2f_dp2;
			}
		}
		ssm->getCorners(cv_corners_mat);

		end_timer();
		write_interval(time_fname, "w");
	}

	
	void ICLK::setRegion(const cv::Mat& corners){
		ssm->setCorners(corners);
		if(params.update_ssm){
			// since the above command completely resets the SSM state including its initial points,
			// any quantities that depend on these, like dI0_dpssm and d2I0_dpssm2,
			// must be recomputed along with quantities that depend on these in turn.
			ssm->cmptWarpedPixJacobian(dI0_dpssm, am->getInitPixGrad());
			am->cmptInitJacobian(df_dp, dI0_dpssm);
			if(params.hess_type != HessType::CurrentSelf){
				if(params.sec_ord_hess){
					ssm->cmptWarpedPixHessian(d2I0_dpssm2, am->getInitPixHess(), am->getInitPixGrad());
				}
				if(params.hess_type == HessType::InitialSelf){
					if(params.sec_ord_hess){
						am->cmptSelfHessian(d2f_dp2, dI0_dpssm, d2I0_dpssm2);
					} else{
						am->cmptSelfHessian(d2f_dp2, dI0_dpssm);
					}
					if(params.leven_marq){
						d2f_dp2_orig = d2f_dp2;
					}
				}
			}
		}
		ssm->getCorners(cv_corners_mat);
	}

	
	void ICLK::update(){
		++frame_id;
		write_frame_id(frame_id);

		double prev_similarity = 0;
		double leven_marq_delta = params.lm_delta_init;
		bool state_reset = false;

		am->setFirstIter();
		for(int iter_id = 0; iter_id < params.max_iters; iter_id++){
			init_timer();

			am->updatePixVals(ssm->getPts());
			record_event("am->updatePixVals");

			am->updateSimilarity(false);
			record_event("am->updateSimilarity");

			if(params.leven_marq && !state_reset){
				double curr_similarity = am->getSimilarity();
				if(iter_id > 0){
					if(curr_similarity < prev_similarity){
						leven_marq_delta *= params.lm_delta_update;
						//! undo the last update
						ssm->compositionalUpdate(ssm_update);
						am->updateState(am_update);
						state_reset = true;
						continue;
					}
					if(curr_similarity > prev_similarity){
						leven_marq_delta /= params.lm_delta_update;
					}
				}
				prev_similarity = curr_similarity;
			}
			state_reset = false;

			am->updateInitGrad();
			record_event("am->updateInitGrad");

			am->cmptInitJacobian(df_dp, dI0_dpssm);
			record_event("am->cmptInitJacobian");

			switch(params.hess_type){
			case HessType::InitialSelf:
				if(params.leven_marq){
					d2f_dp2 = d2f_dp2_orig;
				}
				break;
			case HessType::CurrentSelf:
				if(params.chained_warp){
					am->updatePixGrad(ssm->getPts());
					record_event("am->updatePixGrad");
					ssm->cmptWarpedPixJacobian(dIt_dpssm, am->getCurrPixGrad());
					record_event("ssm->cmptWarpedPixJacobian");
				} else{
					ssm->updateGradPts(am->getGradOffset());
					record_event("ssm->updateGradPts");
					am->updatePixGrad(ssm->getGradPts());
					record_event("am->updatePixGrad");
					ssm->cmptInitPixJacobian(dIt_dpssm, am->getCurrPixGrad());
					record_event("ssm->cmptInitPixJacobian");
				}
				if(params.sec_ord_hess){
					if(params.chained_warp){
						am->updatePixHess(ssm->getPts());
						record_event("am->updatePixHess");
						ssm->cmptWarpedPixHessian(d2It_dpssm2, am->getCurrPixHess(),
							am->getCurrPixGrad());
						record_event("ssm->cmptWarpedPixHessian");
					} else{
						ssm->updateHessPts(am->getHessOffset());
						record_event("ssm->updateHessPts");
						am->updatePixHess(ssm->getPts(), ssm->getHessPts());
						record_event("am->updatePixHess");
						ssm->cmptInitPixHessian(d2It_dpssm2, am->getCurrPixHess(), am->getCurrPixGrad());
						record_event("ssm->cmptInitPixHessian");
					}
					am->cmptSelfHessian(d2f_dp2, dIt_dpssm, d2It_dpssm2);
					record_event("am->cmptSelfHessian (second order)");
				} else{
					am->cmptSelfHessian(d2f_dp2, dIt_dpssm);
					record_event("am->cmptSelfHessian (first order)");
				}
				break;
			case HessType::Std:
				if(params.sec_ord_hess){
					am->cmptInitHessian(d2f_dp2, dI0_dpssm, d2I0_dpssm2);
					record_event("am->cmptInitHessian (second order)");
				} else{
					am->cmptInitHessian(d2f_dp2, dI0_dpssm);
					record_event("am->cmptInitHessian (first order)");
				}
				break;
			}

			if(params.leven_marq){
				//utils::printMatrix(hessian, "original hessian");
				MatrixXd diag_d2f_dp2 = d2f_dp2.diagonal().asDiagonal();
				//utils::printMatrix(diag_hessian, "diag_hessian");
				d2f_dp2 += leven_marq_delta*diag_d2f_dp2;
				//utils::printMatrix(hessian, "LM hessian");
			}

			state_update = -d2f_dp2.colPivHouseholderQr().solve(df_dp.transpose());
			record_event("ssm_update");

			ssm_update = state_update.head(ssm_state_size);
			am_update = state_update.tail(am_state_size);

			prev_corners = ssm->getCorners();

			ssm->invertState(inv_ssm_update, ssm_update);
			record_event("ssm->invertState");

			ssm->compositionalUpdate(inv_ssm_update);
			record_event("ssm->compositionalUpdate");

			am->invertState(inv_am_update, am_update);
			record_event("am->invertState");

			am->updateState(inv_am_update);
			record_event("am->updateState");

			double update_norm = (prev_corners - ssm->getCorners()).squaredNorm();
			record_event("update_norm");

			if(update_norm < params.epsilon){
				if(params.debug_mode){
					printf("n_iters: %d\n", iter_id + 1);
				}
				break;
			}
			am->clearFirstIter();
		}
		if(params.enable_learning){
			am->updateModel(ssm->getPts());
		}
		ssm->getCorners(cv_corners_mat);
	}
}
_MTF_END_NAMESPACE
