#include "mtf/SM/NT/FALK.h"
#include "mtf/Utilities/miscUtils.h"
#include <time.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "boost/filesystem/operations.hpp"

_MTF_BEGIN_NAMESPACE
namespace nt{
	FALK::FALK(AM _am, SSM _ssm,
	 const ParamType *falk_params) :
		SearchMethod(_am, _ssm),
		params(falk_params) {

		printf("\n");
		printf("Using Forward Additive Lucas Kanade (NT) SM with:\n");
		printf("max_iters: %d\n", params.max_iters);
		printf("epsilon: %f\n", params.epsilon);
		printf("show_grid: %d\n", params.show_grid);
		printf("show_patch: %d\n", params.show_patch);
		printf("patch_resize_factor: %f\n", params.patch_resize_factor);
		printf("write_frames: %d\n", params.write_frames);
		printf("enable_learning: %d\n", params.enable_learning);
		printf("leven_marq: %d\n", params.leven_marq);
		if(params.leven_marq){
			printf("lm_delta_init: %f\n", params.lm_delta_init);
			printf("lm_delta_update: %f\n", params.lm_delta_update);
		}
		printf("debug_mode: %d\n", params.debug_mode);

		printf("appearance model: %s\n", am->name.c_str());
		printf("state space model: %s\n", ssm->name.c_str());
		printf("\n");

		name = "falk_nt";
		log_fname = "log/mtf_falk_nt_log.txt";
		time_fname = "log/mtf_falk_nt_times.txt";
		frame_id = 0;

		const char *hess_order = params.sec_ord_hess ? "Second" : "First";
		printf("Using %s order %s Hessian\n", hess_order,
			FALKParams::toString(params.hess_type));
		if(params.leven_marq){
			printf("Using Levenberg Marquardt formulation...\n");
		}

		ssm_update.resize(ssm->getStateSize());
		curr_pix_jacobian.resize(am->getPatchSize(), ssm->getStateSize());
		jacobian.resize(ssm->getStateSize());
		hessian.resize(ssm->getStateSize(), ssm->getStateSize());

		if(params.hess_type == HessType::InitialSelf){
			init_pix_jacobian.resize(am->getPatchSize(), ssm->getStateSize());
			if(params.sec_ord_hess){
				init_pix_hessian.resize(ssm->getStateSize()*ssm->getStateSize(), am->getPatchSize());
			}
		} else if(params.sec_ord_hess){
			curr_pix_hessian.resize(ssm->getStateSize()*ssm->getStateSize(), am->getPatchSize());
		}
		if(params.show_grid){
			curr_img_uchar.create(am->getCurrImg().rows, am->getCurrImg().cols, CV_8UC3);
			if(params.show_patch){
				curr_patch_eig.resize(am->getPatchSize());
				switch(am->getNChannels()){
				case 1:
					curr_patch = cv::Mat(am->getResY(), am->getResX(), CV_64FC1, curr_patch_eig.data());
					curr_patch_uchar.create(am->getResY(), am->getResX(), CV_8UC1);
					break;
				case 3:
					curr_patch = cv::Mat(am->getResY(), am->getResX(), CV_64FC3, curr_patch_eig.data());
					curr_patch_uchar.create(am->getResY(), am->getResX(), CV_8UC3);
					break;
				}
				curr_patch_resized.create(am->getResY()*params.patch_resize_factor,
					am->getResX()*params.patch_resize_factor, curr_patch_uchar.type());
				if(params.write_frames){
					write_frame_dir = cv::format("log/fa_%s_%s_%d_%d", am->name.c_str(),
						ssm->name.c_str(), am->getResX(), am->getResY());
					if(!boost::filesystem::exists(write_frame_dir)){
						printf("FALK::Frame directory: %s does not exist. Creating it...\n", write_frame_dir.c_str());
						boost::filesystem::create_directories(write_frame_dir);
					}
				}
			}
			cv::namedWindow("Grid");
		}
	}


	void FALK::initialize(const cv::Mat &corners){
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
				ssm->cmptPixHessian(init_pix_hessian, am->getInitPixHess(),
					am->getInitPixGrad());
				am->cmptSelfHessian(hessian, init_pix_jacobian, init_pix_hessian);
			} else{
				am->cmptSelfHessian(hessian, init_pix_jacobian);
			}
		}
		if(params.leven_marq){
			init_self_hessian = hessian;
		}
		ssm->getCorners(cv_corners_mat);

		if(params.show_grid){ drawGrid(); }

		end_timer();
		write_interval(time_fname, "w");
	}

	void FALK::update(){
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

			// compute the prerequisites for the gradient function
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

			am->updatePixGrad(ssm->getPts());
			record_event("am->updatePixGrad");

			ssm->cmptPixJacobian(curr_pix_jacobian, am->getCurrPixGrad());
			record_event("ssm->cmptPixJacobian");

			am->updateCurrGrad();
			record_event("am->updateCurrGrad");

			am->cmptCurrJacobian(jacobian, curr_pix_jacobian);
			record_event("am->cmptCurrJacobian");

			//compute pixel Hessian
			if(params.sec_ord_hess && params.hess_type != HessType::InitialSelf){
				am->updatePixHess(ssm->getPts());
				record_event("am->updatePixHess");
				ssm->cmptPixHessian(curr_pix_hessian, am->getCurrPixHess(), am->getCurrPixGrad());
				record_event("ssm->cmptPixHessian");
			}

			// compute similarity Hessian
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
			//utils::printMatrix(curr_pix_hessian, "curr_pix_hessian");
			//utils::printMatrix(hessian, "hessian");
			//utils::printMatrix(jacobian, "jacobian");
			//utils::printMatrix(ssm_update, "ssm_update");

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
		if(params.enable_learning){
			am->updateModel(ssm->getPts());
		}
		ssm->getCorners(cv_corners_mat);

		if(params.show_grid){ drawGrid(); }
	}
	void FALK::drawGrid(){
		am->getCurrImg().convertTo(curr_img_uchar, curr_img_uchar.type());
		cv::cvtColor(curr_img_uchar, curr_img_uchar, CV_GRAY2BGR);
		utils::drawGrid(curr_img_uchar, ssm->getPts(), am->getResX(), am->getResY());
		if(params.show_patch){
			am->extractPatch(curr_patch_eig, ssm->getPts());
			curr_patch.convertTo(curr_patch_uchar, curr_patch_uchar.type());
			cv::resize(curr_patch_uchar, curr_patch_resized,
				cv::Size(curr_patch_resized.cols, curr_patch_resized.rows));
			utils::drawPatch<uchar, uchar>(curr_img_uchar, curr_patch_resized, am->getNChannels());
		}
		cv::imshow("Grid", curr_img_uchar);
		if(params.write_frames){
			cv::imwrite(cv::format("%s/frame%05d.jpg", write_frame_dir.c_str(), frame_id),
				curr_img_uchar);
		}
	}
}

_MTF_END_NAMESPACE