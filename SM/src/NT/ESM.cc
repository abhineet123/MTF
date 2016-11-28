#include "mtf/SM/NT/ESM.h"
#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/excpUtils.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

_MTF_BEGIN_NAMESPACE

namespace nt{	

	ESM::ESM(AM _am, SSM _ssm, const ParamType *esm_params) :
		SearchMethod(_am, _ssm), params(esm_params){
		printf("\n");
		printf("Using Efficient Second order Minimization (NT) SM with:\n");
		printf("max_iters: %d\n", params.max_iters);
		printf("epsilon: %f\n", params.epsilon);
		printf("jac_type: %d\n", params.jac_type);
		printf("hess_type: %d\n", params.hess_type);
		printf("sec_ord_hess: %d\n", params.sec_ord_hess);
		printf("chained_warp: %d\n", params.chained_warp);
		printf("leven_marq: %d\n", params.leven_marq);
		if(params.leven_marq){
			printf("lm_delta_init: %f\n", params.lm_delta_init);
			printf("lm_delta_update: %f\n", params.lm_delta_update);
		}
		printf("enable_spi: %d\n", params.enable_spi);
		printf("spi_thresh: %f\n", params.spi_thresh);
		printf("debug_mode: %d\n", params.debug_mode);

		printf("appearance model: %s\n", am->name.c_str());
		printf("state space model: %s\n", ssm->name.c_str());
		printf("\n");

		name = "esm_nt";
		log_fname = "log/mtf_esm_log.txt";
		time_fname = "log/mtf_esm_times.txt";

		frame_id = 0;
		max_pix_diff = 0;

#ifndef DISABLE_SPI
		if(params.enable_spi){
			if(!ssm->supportsSPI())
				throw utils::FunctonNotImplemented("ESM::SSM does not support SPI");
			if(!am->supportsSPI())
				throw utils::FunctonNotImplemented("ESM::AM does not support SPI");

			printf("Using Selective Pixel Integration\n");
			pix_mask.resize(am->getPatchSize());
			ssm->setSPIMask(pix_mask.data());
			am->setSPIMask(pix_mask.data());
			rel_pix_diff.resize(am->getPatchSize());
			pix_mask2.resize(am->getPatchSize());
			pix_mask_img = cv::Mat(am->getResX(), am->getResY(), CV_8UC1, pix_mask2.data());
			spi_win_name = "pix_mask_img";
		}
#endif
		printf("Using %s\n", ESMParams::toString(params.jac_type));
		const char *hess_order = params.sec_ord_hess ? "Second" : "First";
		printf("Using %s order %s\n", hess_order, ESMParams::toString(params.hess_type));
		if(params.leven_marq){
			printf("Using Levenberg Marquardt formulation...\n");
		}
		ssm_state_size = ssm->getStateSize();
		am_state_size = am->getStateSize();
		state_size = ssm_state_size + am_state_size;
		printf("ssm_state_size: %d am_state_size: %d state_size: %d\n",
			ssm_state_size, am_state_size, state_size);

		state_update.resize(state_size);
		ssm_update.resize(ssm_state_size);
		am_update.resize(am_state_size);
		inv_ssm_update.resize(ssm_state_size);
		inv_am_update.resize(am_state_size);

		jacobian.resize(state_size);
		hessian.resize(state_size, state_size);
		if(params.hess_type == HessType::SumOfSelf){
			init_self_hessian.resize(state_size, state_size);
		}

		init_pix_jacobian.resize(am->getPatchSize(), ssm_state_size);
		curr_pix_jacobian.resize(am->getPatchSize(), ssm_state_size);

		if(params.jac_type == JacType::Original || params.hess_type == HessType::Original){
			mean_pix_jacobian.resize(am->getPatchSize(), ssm_state_size);
		}
		if(params.sec_ord_hess){
			init_pix_hessian.resize(ssm_state_size*ssm_state_size, am->getPatchSize());
			if(params.hess_type != HessType::InitialSelf){
				curr_pix_hessian.resize(ssm_state_size*ssm_state_size, am->getPatchSize());
				if(params.hess_type == HessType::Original){
					mean_pix_hessian.resize(ssm_state_size*ssm_state_size, am->getPatchSize());
				}
			}
		}
	}

	void ESM::initialize(const cv::Mat &corners){
		start_timer();

		am->clearInitStatus();
		ssm->clearInitStatus();

		frame_id = 0;
		ssm->initialize(corners, am->getNChannels());
		am->initializePixVals(ssm->getPts());

		if(params.enable_spi){ initializeSPIMask(); }

		initializePixJacobian();
		if(params.sec_ord_hess){
			initializePixHessian();
		}
		am->initializeSimilarity();
		am->initializeGrad();
		am->initializeHess();

		if(params.hess_type == HessType::InitialSelf || params.hess_type == HessType::SumOfSelf){
			if(params.sec_ord_hess){
				am->cmptSelfHessian(hessian, init_pix_jacobian, init_pix_hessian);
			} else{
				am->cmptSelfHessian(hessian, init_pix_jacobian);
			}
			init_self_hessian = hessian;
		}
		ssm->getCorners(cv_corners_mat);

		end_timer();
		write_interval(time_fname, "w");
	}
	
	void ESM::setRegion(const cv::Mat& corners){
		ssm->setCorners(corners);
		// since the above command completely resets the SSM state including its initial points,
		// any quantities that depend on these, like init_pix_jacobian and init_pix_hessian,
		// must be recomputed along with quantities that depend on them in turn.
		ssm->cmptInitPixJacobian(init_pix_jacobian, am->getInitPixGrad());
		if(params.sec_ord_hess){
			ssm->cmptInitPixHessian(init_pix_hessian, am->getInitPixHess(), am->getInitPixGrad());
		}
		if(params.hess_type == HessType::InitialSelf || params.hess_type == HessType::SumOfSelf){
			if(params.sec_ord_hess){
				am->cmptSelfHessian(hessian, init_pix_jacobian, init_pix_hessian);
			} else{
				am->cmptSelfHessian(hessian, init_pix_jacobian);
			}
			init_self_hessian = hessian;
		}
		ssm->getCorners(cv_corners_mat);
	}

	void ESM::update(){
		++frame_id;
		write_frame_id(frame_id);

		double prev_similarity = 0;
		double leven_marq_delta = params.lm_delta_init;
		bool state_reset = false;

		am->setFirstIter();
		for(int iter_id = 0; iter_id < params.max_iters; iter_id++){
			init_timer();

			//! extract pixel values from the current image at the latest known position of the object
			am->updatePixVals(ssm->getPts());
			record_event("am->updatePixVals");

			if(params.enable_spi){ updateSPIMask(); }

			//! compute the prerequisites for the gradient functions
			am->updateSimilarity(false);
			record_event("am->update");

			if(params.leven_marq && !state_reset){
				double curr_similarity = am->getSimilarity();
				if(iter_id > 0){
					if(curr_similarity < prev_similarity){
						leven_marq_delta *= params.lm_delta_update;
						//! undo the last update
						ssm->invertState(inv_ssm_update, ssm_update);
						ssm->compositionalUpdate(inv_ssm_update);
						am->invertState(inv_am_update, am_update);
						am->updateState(inv_am_update);

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

			updatePixJacobian();
			if(params.jac_type == JacType::Original || params.hess_type == HessType::Original){
				mean_pix_jacobian = (init_pix_jacobian + curr_pix_jacobian) / 2.0;
				record_event("mean_pix_jacobian");
			}
			if(params.sec_ord_hess && params.hess_type != HessType::InitialSelf){
				updatePixHessian();
			}

			// update the gradient of the error norm w.r.t. current pixel values
			am->updateCurrGrad();
			record_event("am->updateCurrGrad");

			// update the gradient of the error norm w.r.t. initial pixel values
			am->updateInitGrad();
			record_event("am->updateInitGrad");

			cmptJacobian();
			cmptHessian();

			if(params.leven_marq){
				//utils::printMatrix(hessian, "original hessian");
				MatrixXd diag_hessian = hessian.diagonal().asDiagonal();
				//utils::printMatrix(diag_hessian, "diag_hessian");
				hessian += leven_marq_delta*diag_hessian;
				//utils::printMatrix(hessian, "LM hessian");
			}

			state_update = -hessian.colPivHouseholderQr().solve(jacobian.transpose());
			record_event("state_update");

			ssm_update = state_update.head(ssm_state_size);
			am_update = state_update.tail(am_state_size);

			prev_corners = ssm->getCorners();
			updateState();

			//double update_norm = ssm_update.lpNorm<1>();
			double update_norm = (prev_corners - ssm->getCorners()).squaredNorm();
			record_event("update_norm");

			write_data(time_fname);

			if(update_norm < params.epsilon){
				if(params.debug_mode){
					printf("n_iters: %d\n", iter_id + 1);
				}
				break;
			}

			if(params.enable_spi){ showSPIMask(); }

			am->clearFirstIter();
		}
		ssm->getCorners(cv_corners_mat);
	}
	
	void ESM::cmptJacobian(){
		switch(params.jac_type){
		case JacType::Original:
			// take the mean at the level of the jacobian of the pixel values wrt SSM parameters, 
			//then use this mean jacobian to compute the Jacobian of the error norm wrt SSM parameters
			am->cmptCurrJacobian(jacobian, mean_pix_jacobian);
			record_event("am->cmptCurrJacobian");
			break;
		case JacType::DiffOfJacs:
			// compute the mean difference between the Jacobians of the error norm w.r.t. initial AND current values of SSM parameters
			am->cmptDifferenceOfJacobians(jacobian, init_pix_jacobian, curr_pix_jacobian);
			jacobian *= 0.5;
			record_event("am->cmptDifferenceOfJacobians");
			break;
		}
	}

	void ESM::cmptHessian(){
		switch(params.hess_type){
		case HessType::InitialSelf:
			if(params.leven_marq){
				hessian = init_self_hessian;
			}
			break;
		case HessType::Original:
			if(params.sec_ord_hess){
				mean_pix_hessian = (init_pix_hessian + curr_pix_hessian) / 2.0;
				record_event("mean_pix_hessian");
				am->cmptCurrHessian(hessian, mean_pix_jacobian, mean_pix_hessian);
				record_event("am->cmptCurrHessian (second order)");
			} else{
				am->cmptCurrHessian(hessian, mean_pix_jacobian);
				record_event("am->cmptCurrHessian (first order)");
			}
			break;
		case HessType::SumOfStd:
			if(params.sec_ord_hess){
				am->cmptSumOfHessians(hessian, init_pix_jacobian, curr_pix_jacobian,
					init_pix_hessian, curr_pix_hessian);
				record_event("am->cmptSumOfHessians (second order)");
			} else{
				am->cmptSumOfHessians(hessian, init_pix_jacobian, curr_pix_jacobian);
				record_event("am->cmptSumOfHessians (first order)");
			}
			hessian *= 0.5;
			break;
		case HessType::SumOfSelf:
			if(params.sec_ord_hess){
				am->cmptSelfHessian(hessian, curr_pix_jacobian, curr_pix_hessian);
				record_event("am->cmptSelfHessian (second order)");
			} else{
				am->cmptSelfHessian(hessian, curr_pix_jacobian);
				record_event("am->cmptSelfHessian (first order)");
			}
			hessian = (hessian + init_self_hessian) * 0.5;
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
	}
	
	void ESM::initializePixJacobian(){
		if(params.chained_warp){
			am->initializePixGrad(ssm->getPts());
			ssm->cmptWarpedPixJacobian(init_pix_jacobian, am->getInitPixGrad());
		} else{
			ssm->initializeGradPts(am->getGradOffset());
			am->initializePixGrad(ssm->getGradPts());
			ssm->cmptInitPixJacobian(init_pix_jacobian, am->getInitPixGrad());
		}
	}
	
	void ESM::updatePixJacobian(){
		//! compute the Jacobian of pixel values w.r.t. SSM parameters
		if(params.chained_warp){
			am->updatePixGrad(ssm->getPts());
			record_event("am->updatePixGrad");
			ssm->cmptWarpedPixJacobian(curr_pix_jacobian, am->getCurrPixGrad());
			record_event("ssm->cmptWarpedPixJacobian");
		} else{
			ssm->updateGradPts(am->getGradOffset());
			record_event("ssm->updateGradPts");
			// compute pixel gradient of the current image warped with the current warp
			am->updatePixGrad(ssm->getGradPts());
			record_event("am->updatePixGrad");
			// multiply the pixel gradient with the SSM Jacobian to get the Jacobian of pixel values w.r.t. SSM parameters
			ssm->cmptInitPixJacobian(curr_pix_jacobian, am->getCurrPixGrad());
			record_event("ssm->cmptInitPixJacobian");
		}
	}
	
	void ESM::initializePixHessian(){
		if(params.chained_warp){
			am->initializePixHess(ssm->getPts());
			ssm->cmptWarpedPixHessian(init_pix_hessian, am->getInitPixHess(),
				am->getInitPixGrad());
		} else{
			ssm->initializeHessPts(am->getHessOffset());
			am->initializePixHess(ssm->getPts(), ssm->getHessPts());
			ssm->cmptInitPixHessian(init_pix_hessian, am->getInitPixHess(), am->getInitPixGrad());
		}
	}
	
	void ESM::updatePixHessian(){
		if(params.chained_warp){
			am->updatePixHess(ssm->getPts());
			record_event("am->updatePixHess");
			ssm->cmptWarpedPixHessian(curr_pix_hessian, am->getCurrPixHess(), am->getCurrPixGrad());
			record_event("ssm->cmptWarpedPixHessian");
		} else{
			ssm->updateHessPts(am->getHessOffset());
			record_event("ssm->updateHessPts");
			am->updatePixHess(ssm->getPts(), ssm->getHessPts());
			record_event("am->updatePixHess");
			ssm->cmptInitPixHessian(curr_pix_hessian, am->getCurrPixHess(), am->getCurrPixGrad());
			record_event("ssm->cmptInitPixHessian");
		}
	}

	void ESM::updateState(){
		ssm->compositionalUpdate(ssm_update);
		record_event("ssm->compositionalUpdate");
		am->updateState(am_update);
		record_event("am->updateState");
	}

	// support for Selective Pixel Integration	
	void ESM::initializeSPIMask(){
		cv::namedWindow(spi_win_name);
		max_pix_diff = am->getInitPixVals().maxCoeff() - am->getInitPixVals().minCoeff();
		utils::printScalar(max_pix_diff, "max_pix_diff");
	}
	
	void ESM::updateSPIMask(){
#ifndef DISABLE_SPI
		rel_pix_diff = (am->getInitPixVals() - am->getCurrPixVals()) / max_pix_diff;
		record_event("rel_pix_diff");

		pix_mask = rel_pix_diff.cwiseAbs().array() < params.spi_thresh;
		record_event("pix_mask");

		if(params.debug_mode){
			int active_pixels = pix_mask.count();
			utils::printScalar(active_pixels, "active_pixels", "%d");
		}
#endif	
	}
	
	void ESM::showSPIMask(){
#ifndef DISABLE_SPI
		if(params.enable_spi){
			for(int pix_id = 0; pix_id < am->getPatchSize(); pix_id++){
				int x = pix_mask(pix_id);
				pix_mask2(pix_id) = x * 255;
			}
			cv::Mat pix_mask_img_resized;
			cv::resize(pix_mask_img, pix_mask_img_resized, cv::Size(300, 300));
			imshow(spi_win_name, pix_mask_img_resized);
		}
#endif
	}
}
_MTF_END_NAMESPACE
