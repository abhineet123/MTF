#include "mtf/SM/NT/IALK.h"
#include "mtf/Utilities/miscUtils.h"

_MTF_BEGIN_NAMESPACE

namespace nt{

	IALK::IALK(AM _am, SSM _ssm, const ParamType *ialk_params) :
		SearchMethod(_am, _ssm),
		params(ialk_params) {
		printf("\n");
		printf("Using Inverse Additive Lucas Kanade (NT) SM with:\n");
		printf("max_iters: %d\n", params.max_iters);
		printf("epsilon: %f\n", params.epsilon);
		printf("hess_type: %d\n", params.hess_type);
		printf("leven_marq: %d\n", params.leven_marq);
		if(params.leven_marq){
			printf("lm_delta_init: %f\n", params.lm_delta_init);
			printf("lm_delta_update: %f\n", params.lm_delta_update);
		}
		printf("debug_mode: %d\n", params.debug_mode);
		printf("appearance model: %s\n", am->name.c_str());
		printf("state space model: %s\n", ssm->name.c_str());
		printf("\n");

		name = "ialk";
		log_fname = "log/mtf_ialk_log.txt";
		time_fname = "log/mtf_ialk_times.txt";
		frame_id = 0;

		const char *hess_order = params.sec_ord_hess ? "Second" : "First";
		printf("Using %s order %s Hessian\n", hess_order,
			IALKParams::toString(params.hess_type));
		if(params.leven_marq){
			printf("Using Levenberg Marquardt formulation...\n");
		}

		ssm_update.resize(ssm->getStateSize());
		curr_pix_jacobian.resize(am->getPatchSize(), ssm->getStateSize());
		jacobian.resize(ssm->getStateSize());
		hessian.resize(ssm->getStateSize(), ssm->getStateSize());
		if(params.hess_type == HessType::InitialSelf){
			init_pix_jacobian.resize(am->getPatchSize(), ssm->getStateSize());
		}
		if(params.sec_ord_hess){
			if(params.hess_type == HessType::InitialSelf){
				init_pix_hessian.resize(ssm->getStateSize()*ssm->getStateSize(), am->getPatchSize());
			} else {
				curr_pix_hessian.resize(ssm->getStateSize()*ssm->getStateSize(), am->getPatchSize());
			}
		}
	}

	
	void IALK::initialize(const cv::Mat &corners){
		start_timer();

		am->clearInitStatus();
		ssm->clearInitStatus();

		ssm->initialize(corners, am->getNChannels());
		am->initializePixVals(ssm->getPts());
		am->initializePixGrad(ssm->getPts());
		am->initializeSimilarity();
		am->initializeGrad();
		am->initializeHess();

		if(params.sec_ord_hess){
			am->initializePixHess(ssm->getPts());
		}
		if(params.hess_type == HessType::InitialSelf){
			ssm->cmptPixJacobian(init_pix_jacobian, am->getInitPixGrad());
			if(params.sec_ord_hess){
				ssm->cmptPixHessian(init_pix_hessian, am->getInitPixHess(), am->getInitPixGrad());
				am->cmptSelfHessian(hessian, init_pix_jacobian, init_pix_hessian);
			} else{
				am->cmptSelfHessian(hessian, init_pix_jacobian);
			}
			if(params.leven_marq){
				init_self_hessian = hessian;
			}
		}

		ssm->getCorners(cv_corners_mat);

		end_timer();
		write_interval(time_fname, "w");
	}
	
	void IALK::update(){
		++frame_id;
		write_frame_id(frame_id);

		double prev_similarity = 0;
		double leven_marq_delta = params.lm_delta_init;
		bool state_reset = false;

		for(int iter_id = 0; iter_id < params.max_iters; ++iter_id){
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
						VectorXd inv_ssm_update = -ssm_update;
						ssm->additiveUpdate(inv_ssm_update);

						if(params.debug_mode){
							printf("curr_similarity: %20.14f\t prev_similarity: %20.14f\n",
								curr_similarity, prev_similarity);
							utils::printScalar(leven_marq_delta, "leven_marq_delta");
						}
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

			ssm->cmptApproxPixJacobian(curr_pix_jacobian, am->getInitPixGrad());
			record_event("am->cmptApproxPixJacobian");

			am->updateCurrGrad();
			record_event("am->updateCurrGrad");

			am->cmptCurrJacobian(jacobian, curr_pix_jacobian);
			record_event("am->cmptCurrJacobian");

			//compute pixel Hessian
			if(params.sec_ord_hess && params.hess_type != HessType::InitialSelf){
				ssm->cmptApproxPixHessian(curr_pix_hessian,
					am->getInitPixHess(), am->getInitPixGrad());
				record_event("ssm->cmptApproxPixHessian");
			}
			switch(params.hess_type){
			case HessType::InitialSelf:
				if(params.leven_marq){
					hessian = init_self_hessian;
				}
				break;
			case HessType::CurrentSelf:
				if(params.sec_ord_hess){
					am->cmptSelfHessian(hessian, curr_pix_jacobian, curr_pix_hessian);
					record_event("am->cmptSelfHessian (second order)");
				} else{
					am->cmptSelfHessian(hessian, curr_pix_jacobian);
					record_event("am->cmptSelfHessian (first order)");
				}
				break;
			case HessType::Std:
				if(params.sec_ord_hess){
					am->cmptCurrHessian(hessian, curr_pix_jacobian, curr_pix_hessian);
					record_event("am->cmptCurrHessian (second order)");
				} else{
					am->cmptCurrHessian(hessian, curr_pix_jacobian);
					record_event("am->cmptCurrHessian (first order)");
				}
				break;
			}
			if(params.leven_marq){
				//utils::printMatrix(hessian, "original hessian");
				MatrixXd diag_hessian = hessian.diagonal().asDiagonal();
				//utils::printMatrix(diag_hessian, "diag_hessian");
				hessian += leven_marq_delta*diag_hessian;
				//utils::printMatrix(hessian, "LM hessian");
			}

			ssm_update = -hessian.colPivHouseholderQr().solve(jacobian.transpose());
			record_event("ssm_update");

			prev_corners = ssm->getCorners();
			ssm->additiveUpdate(ssm_update);
			record_event("ssm->additiveUpdate");

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
		ssm->getCorners(cv_corners_mat);
	}
}
_MTF_END_NAMESPACE
