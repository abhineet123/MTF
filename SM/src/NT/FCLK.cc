#include "mtf/SM/NT/FCLK.h"
#include "mtf/Utilities/miscUtils.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdexcept>

_MTF_BEGIN_NAMESPACE
namespace nt{
	FCLK::FCLK(AM _am, SSM _ssm,
	 const ParamType *fclk_params) :
		SearchMethod(_am, _ssm),
		params(fclk_params) {
		printf("\n");
		printf("Using Forward Compositional Lucas Kanade (NT) SM with:\n");
		printf("max_iters: %d\n", params.max_iters);
		printf("epsilon: %f\n", params.epsilon);
		printf("hess_type: %d\n", params.hess_type);
		printf("sec_ord_hess: %d\n", params.sec_ord_hess);
		printf("chained_warp: %d\n", params.chained_warp);
		printf("leven_marq: %d\n", params.leven_marq);
		if(params.leven_marq){
			printf("lm_delta_init: %f\n", params.lm_delta_init);
			printf("lm_delta_update: %f\n", params.lm_delta_update);
		}
		printf("enable_learning: %d\n", params.enable_learning);
		printf("show_grid: %d\n", params.show_grid);
		printf("show_patch: %d\n", params.show_patch);
		if(params.show_patch){
			printf("patch_resize_factor: %f\n", params.patch_resize_factor);
		}
		printf("debug_mode: %d\n", params.debug_mode);

		printf("appearance model: %s\n", am->name.c_str());
		printf("state space model: %s\n", ssm->name.c_str());
		printf("\n");

		name = "fclk_nt";
		log_fname = "log/mtf_fclk_nt_log.txt";
		time_fname = "log/mtf_fclk_nt_times.txt";
		frame_id = 0;

		const char *hess_order = params.sec_ord_hess ? "Second" : "First";
		printf("Using %s order %s Hessian\n", hess_order,
			FCLKParams::toString(params.hess_type));
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

		curr_pix_jacobian.resize(am->getPatchSize(), ssm_state_size);
		curr_pix_jacobian_new.resize(am->getPatchSize(), ssm_state_size);
		jacobian.resize(state_size);
		hessian.resize(state_size, state_size);

		if(params.hess_type == HessType::InitialSelf){
			init_pix_jacobian.resize(am->getPatchSize(), ssm_state_size);
			if(params.sec_ord_hess){
				init_pix_hessian.resize(ssm_state_size*ssm_state_size, am->getPatchSize());
			}
		} else if(params.sec_ord_hess){
			curr_pix_hessian.resize(ssm_state_size*ssm_state_size, am->getPatchSize());
		}	

		if(params.show_grid){
			curr_img_uchar.create(am->getCurrImg().rows, am->getCurrImg().cols, CV_8UC3);
			if(params.show_patch){
				curr_patch_eig.resize(am->getPatchSize());	
				printf("curr_patch_eig size: %ld\n", curr_patch_eig.size());
				if(am->getCurrImg().type() == CV_32FC1){
					printf("Using single channel images\n");
					curr_patch = cv::Mat(am->getResY(), am->getResX(), CV_64FC1, curr_patch_eig.data());
					curr_patch_uchar.create(am->getResY(), am->getResX(), CV_8UC1);
					curr_patch_resized.create(am->getResY()*params.patch_resize_factor,
						am->getResX()*params.patch_resize_factor, CV_8UC1);
				} else{
					printf("Using multi channel images\n");
					curr_patch = cv::Mat(am->getResY(), am->getResX(), CV_64FC3, curr_patch_eig.data());
					curr_patch_uchar.create(am->getResY(), am->getResX(), CV_8UC3);
					curr_patch_resized.create(am->getResY()*params.patch_resize_factor,
						am->getResX()*params.patch_resize_factor, CV_8UC3);
				}				

			}
			cv::namedWindow("FCLK::Grid");
		}
	}

	void FCLK::initialize(const cv::Mat &corners){
		start_timer();

		am->clearInitStatus();
		ssm->clearInitStatus();

		ssm->initialize(corners, am->getNChannels());
		am->initializePixVals(ssm->getPts());

		am->initializeSimilarity();
		am->initializeGrad();
		am->initializeHess();

		if(params.chained_warp){
			am->initializePixGrad(ssm->getPts());
		} else{
			ssm->initializeGradPts(am->getGradOffset());
			am->initializePixGrad(ssm->getGradPts());
		}
		if(params.sec_ord_hess){
			if(params.chained_warp){
				am->initializePixHess(ssm->getPts());
			} else{
				ssm->initializeHessPts(am->getHessOffset());
				am->initializePixHess(ssm->getPts(), ssm->getHessPts());
			}
		}
		if(params.hess_type == HessType::InitialSelf){
			if(params.chained_warp){
				ssm->cmptWarpedPixJacobian(init_pix_jacobian, am->getInitPixGrad());
			} else{
				ssm->cmptInitPixJacobian(init_pix_jacobian, am->getInitPixGrad());
			}
			if(params.sec_ord_hess){
				if(params.chained_warp){
					ssm->cmptWarpedPixHessian(init_pix_hessian, am->getInitPixHess(),
						am->getInitPixGrad());
				} else{
					ssm->cmptInitPixHessian(init_pix_hessian, am->getInitPixHess(),
						am->getInitPixGrad());
				}
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
		//#ifdef ENABLE_PROFILING
		//	utils::printScalarToFile("cmp_times", NULL, "log/ssm_cmp_times.txt", "%s", "w");
		//#endif
		if(params.write_ssm_updates){
			printf("Writing SSM updates to log/fc_ssm_updates.txt and log/fc_frame_ssm_updates.txt\n");
			fclose(fopen("log/fc_ssm_updates.txt", "w"));
			fclose(fopen("log/fc_frame_ssm_updates.txt", "w"));
			frame_ssm_update.resize(ssm_state_size);
		}
		if(params.debug_mode){
			fclose(fopen("log/fc_debug.txt", "w"));
		}
		if(params.show_grid){ drawGrid(); }

	}

	void FCLK::update(){

		++frame_id;
		write_frame_id(frame_id);

		am->setFirstIter();
		int iter_id = 0;
		double prev_similarity = 0;
		double leven_marq_delta = params.lm_delta_init;
		bool state_reset = false;

		if(params.write_ssm_updates){
			ssm->getIdentityWarp(frame_ssm_update);
			utils::printScalarToFile(frame_id, nullptr, "log/fc_ssm_updates.txt",
				"%d", "a", "\t", "\n");
		}
		while(iter_id < params.max_iters){
			init_timer();

			am->updatePixVals(ssm->getPts()); record_event("am->updatePixVals");
			am->updateSimilarity(false); record_event("am->updateSimilarity");

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

			am->updateCurrGrad(); record_event("am->updateCurrGrad");

			//! compute pixel Jacobian
			if(params.chained_warp){
				am->updatePixGrad(ssm->getPts());
				record_event("am->updatePixGrad");
				ssm->cmptWarpedPixJacobian(curr_pix_jacobian, am->getCurrPixGrad());
				record_event("ssm->cmptWarpedPixJacobian");
			} else{
				ssm->updateGradPts(am->getGradOffset());
				record_event("ssm->updateGradPts");
				am->updatePixGrad(ssm->getGradPts());
				record_event("am->updatePixGrad");
				ssm->cmptInitPixJacobian(curr_pix_jacobian, am->getCurrPixGrad());
				record_event("ssm->cmptInitPixJacobian");
			}

			//double pix_jacobian_diff = (curr_pix_jacobian_new - curr_pix_jacobian).squaredNorm();
			//printf("pix_jacobian_diff: %f\n", pix_jacobian_diff);
			//utils::printMatrixToFile(curr_pix_jacobian.transpose(), "curr_pix_jacobian", log_fname);
			//utils::printMatrixToFile(curr_pix_jacobian_new.transpose(), "curr_pix_jacobian_new", log_fname);

			//! compute pixel Hessian
			if(params.sec_ord_hess && params.hess_type != HessType::InitialSelf){
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

			//! compute similarity Jacobian
			am->cmptCurrJacobian(jacobian, curr_pix_jacobian);
			record_event("am->cmptCurrJacobian");

			//! compute similarity Hessian
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

			state_update = -hessian.colPivHouseholderQr().solve(jacobian.transpose());
			ssm_update = state_update.head(ssm_state_size);
			am_update = state_update.tail(am_state_size);

			record_event("state_update");

			if(params.debug_mode){
				utils::printMatrixToFile(jacobian, "jacobian", "log/fc_debug.txt");
				utils::printMatrixToFile(hessian, "hessian", "log/fc_debug.txt");
				utils::printMatrixToFile(state_update.transpose(), "state_update", "log/fc_debug.txt");
				utils::printMatrixToFile(ssm_update.transpose(), "ssm_update", "log/fc_debug.txt");
				utils::printMatrixToFile(am_update.transpose(), "am_update", "log/fc_debug.txt");
			}

			prev_corners = ssm->getCorners();

			ssm->compositionalUpdate(ssm_update);
			record_event("ssm->compositionalUpdate");

			am->updateState(am_update);
			record_event("ssm->updateParam");


			double update_norm = (prev_corners - ssm->getCorners()).squaredNorm();
			record_event("update_norm");

			write_data(time_fname);

			if(params.write_ssm_updates){
				ssm->composeWarps(frame_ssm_update, ssm_update, frame_ssm_update);
				utils::printScalarToFile(iter_id, nullptr, "log/fc_ssm_updates.txt",
					"%d", "a", "\t", "\t");
				utils::printMatrixToFile(ssm_update.transpose(), nullptr, "log/fc_ssm_updates.txt",
					"%15.9f", "a","\t", "\t");
				utils::printScalarToFile(update_norm, nullptr, "log/fc_ssm_updates.txt",
					"%e", "a", "\t", "\t");
				utils::printScalarToFile(ssm_update.squaredNorm(), nullptr, "log/fc_ssm_updates.txt",
					"%e", "a", "\t", "\n");
			}

			if(update_norm < params.epsilon){
				break;
			}
			am->clearFirstIter();
			++iter_id;
		}

		if(params.write_ssm_updates){	
			utils::printScalarToFile(frame_id, nullptr, "log/fc_frame_ssm_updates.txt",
				"%d", "a", "\t", "\t");
			utils::printMatrixToFile(frame_ssm_update.transpose(), nullptr, "log/fc_frame_ssm_updates.txt");
		}
		if(params.debug_mode){
			printf("n_iters: %d\n", iter_id + 1);
		}
		if(params.enable_learning){
			am->updateModel(ssm->getPts());
		}
		ssm->getCorners(cv_corners_mat);

		if(params.show_grid){ drawGrid(); }
	}

	void FCLK::setRegion(const cv::Mat& corners){
		ssm->setCorners(corners);
		// since the above command completely resets the SSM state including its initial points,
		// any quantities that depend on these, like init_pix_jacobian and init_pix_hessian,
		// must be recomputed along with quantities that depend on these in turn.
		if(params.hess_type == HessType::InitialSelf){
			ssm->cmptInitPixJacobian(init_pix_jacobian, am->getInitPixGrad());
			if(params.sec_ord_hess){
				ssm->cmptInitPixHessian(init_pix_hessian, am->getInitPixHess(),
					am->getInitPixGrad());
				am->cmptSelfHessian(hessian, init_pix_jacobian, init_pix_hessian);
			} else{
				am->cmptSelfHessian(hessian, init_pix_jacobian);
			}
		}
		ssm->getCorners(cv_corners_mat);
	}

	void FCLK::drawGrid(){
		am->getCurrImg().convertTo(curr_img_uchar, curr_img_uchar.type());
		if(curr_img_uchar.type() == CV_8UC1){
			cv::cvtColor(curr_img_uchar, curr_img_uchar, CV_GRAY2BGR);
		}		
		utils::drawGrid(curr_img_uchar, ssm->getPts(), am->getResX(), am->getResY());
		if(params.show_patch){
			am->extractPatch(curr_patch_eig, ssm->getPts());
			curr_patch.convertTo(curr_patch_uchar, curr_patch_uchar.type());
			cv::resize(curr_patch_uchar, curr_patch_resized,
				cv::Size(curr_patch_resized.cols, curr_patch_resized.rows));
			utils::drawPatch<uchar, uchar>(curr_img_uchar, curr_patch_resized, am->getNChannels());
		}
		cv::imshow("FCLK::Grid", curr_img_uchar);
	}
}

_MTF_END_NAMESPACE