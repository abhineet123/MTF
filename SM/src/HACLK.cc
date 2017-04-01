#include "mtf/SM/HACLK.h"
#include "mtf/Utilities/miscUtils.h"
#include <time.h>
#include <stdexcept>


_MTF_BEGIN_NAMESPACE

template <class AM, class SSM>
HACLK<AM, SSM >::HACLK(const ParamType *haclk_params,
	const AMParams *am_params, const SSMParams *ssm_params) : 
	SearchMethod<AM, SSM>(am_params, ssm_params),
	params(haclk_params) {

	printf("\n");
	printf("Using HAC Lucas Kanade tracker with:\n");
	printf("max_iters: %d\n", params.max_iters);
	printf("epsilon: %f\n", params.epsilon);
	printf("rec_init_err_grad: %d\n", params.rec_init_err_grad);
	printf("debug_mode: %d\n", params.debug_mode);
	printf("hess_type: %d\n", params.hess_type);
	printf("appearance model: %s\n", am.name.c_str());
	printf("state space model: %s\n", ssm.name.c_str());
	printf("\n");

	name = "haclk";
	log_fname = "log/mtf_haclk_log.txt";
	time_fname = "log/mtf_haclk_times.txt";
	frame_id = 0;
	n_frames = params.converged_corners.size();


	printf("ssm.getStateSize(): %d\n", ssm.getStateSize());
	printf("n_frames: %d\n", n_frames);

	use_newton_method = false;

	if(params.hess_type != GaussNewton){
		use_newton_method = true;
		if(params.hess_type == Newton){
			printf("Using Newton method with standard Hessian\n");
		} else if(params.hess_type == InitialNewton){
			printf("Using Newton method with initial Self Hessian\n");
		} else if(params.hess_type == CurrentNewton){
			printf("Using Newton method with current Self Hessian\n");
		} else if(params.hess_type == ConvergedNewton){
			printf("Using Newton method with Hessian at convergence\n");
		} else{
			throw std::invalid_argument("HACLK :: Invalid Hessian type provided");
		}
	} else{
		printf("Using Gauss Newton method\n");
	}

	inv_state.resize(ssm.getStateSize());
	ssm_update.resize(ssm.getStateSize());
	curr_pix_jacobian.resize(am.getNPix(), ssm.getStateSize());
	similarity_jacobian.resize(ssm.getStateSize());
	hessian.resize(ssm.getStateSize(), ssm.getStateSize());
}

template <class AM, class SSM>
void HACLK<AM, SSM >::initialize(const cv::Mat &corners){

	if(use_newton_method){
		if(params.hess_type == InitialNewton){
			init_pix_jacobian.resize(am.getNPix(), ssm.getStateSize());
			init_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getNPix());
		} else{
			curr_pix_hessian.resize(ssm.getStateSize()*ssm.getStateSize(), am.getNPix());
		}
	}
#ifdef LOG_HACLK_TIMES
	INIT_TIMER(start_time);
#endif
	ssm.initialize(corners);
	am.initializePixVals(ssm.getPts());

	ssm.initializeGradPts(am.getGradOffset());
	am.initializePixGrad(ssm.getGradPts());
	ssm.initializeHessPts(am.getHessOffset());
	am.initializePixHess(ssm.getPts(), ssm.getHessPts());

	am.initializeSimilarity();
	am.initializeGrad();
	am.initializeHess();
	ssm.getCorners(cv_corners_mat);

#ifdef LOG_HACLK_TIMES
	double init_time;
	END_TIMER(start_time, end_time, init_time);
	utils::printScalarToFile(init_time, "init_time", time_fname, "%15.9f", "w");
#endif
#ifdef LOG_HACLK_DATA
	if(params.debug_mode){
		utils::printScalarToFile("initialize", "function", log_fname, "%s", "w");
}
#endif
}

template <class AM, class SSM>
void HACLK<AM, SSM >::update(){
	++frame_id;
#ifdef LOG_HACLK_DATA
	if(params.debug_mode){		
		am.iter_id = 0;
	}
#endif
#ifdef LOG_HACLK_TIMES
	utils::printScalarToFile(frame_id, "\n\nframe_id", time_fname, "%6d", "a");
	double upd_time = 0;
#endif

	if(frame_id < n_frames){
		for(int i = 0; i < 4; i++){
			curr_conv_corners(0, i) = params.converged_corners[frame_id].at<double>(0, i);
			curr_conv_corners(1, i) = params.converged_corners[frame_id].at<double>(1, i);
		}
		prev_corners = ssm.getCorners();
		if(params.debug_mode){
			utils::printMatrix(prev_corners, "corners before update");
		}
	} else{
		printf("Ran out of converged corners so using the last known position instead\n");
		curr_conv_corners = ssm.getCorners();
	}
	if(params.hess_type == ConvergedNewton){
		//utils::printMatrix(curr_conv_corners, "curr_conv_corners");
		//utils::printMatrix(prev_corners, "corners before update");

		//update state and compute hessian at the ground truth location
		ssm.setCorners(curr_conv_corners);
		am.updatePixVals(ssm.getPts());
		ssm.updateGradPts(am.getGradOffset());
		am.updatePixGrad(ssm.getGradPts());

		ssm.updateHessPts(am.getHessOffset());
		am.updatePixHess(ssm.getPts(), ssm.getHessPts());

		ssm.cmptInitPixJacobian(curr_pix_jacobian, am.getCurrPixGrad());
		ssm.cmptInitPixHessian(curr_pix_hessian, am.getCurrPixHess(), am.getCurrPixGrad());
		am.updateSimilarity();
		am.updateCurrGrad();
		am.cmptCurrHessian(hessian, curr_pix_jacobian, curr_pix_hessian);

		//utils::printMatrix(ssm.getCorners(), "corners before reset");
		// reset state to the previous one
		ssm.setCorners(prev_corners);

		//utils::printMatrix(ssm.getCorners(), "corners after reset");

		//am.setCurrPixGrad();
		//am.setCurrPixGrad();
		//am.setCurrPixGrad();
	} else{
		ssm.setCorners(curr_conv_corners);
		am.initializePixVals(ssm.getPts());
		ssm.initializeGradPts(am.getGradOffset());
		am.initializePixGrad(ssm.getGradPts());

		am.initializeSimilarity();
		am.initializeGrad();

		if(use_newton_method){
			ssm.initializeHessPts(am.getHessOffset());
			am.initializePixHess(ssm.getPts(), ssm.getHessPts());
			am.initializeHess();
			if(params.hess_type == InitialNewton){
				ssm.cmptInitPixJacobian(init_pix_jacobian, am.getInitPixGrad());
				ssm.cmptInitPixHessian(init_pix_hessian, am.getInitPixHess(), am.getInitPixGrad());
				am.cmptCurrHessian(hessian, init_pix_jacobian, init_pix_hessian);
			}
		}
	}
	am.setFirstIter();
	for(int i = 0; i < params.max_iters; i++){
#ifdef LOG_HACLK_TIMES
		proc_times.clear();
		proc_labels.clear();
		INIT_TIMER(start_time);
#endif
		am.updatePixVals(ssm.getPts());
#ifdef LOG_HACLK_TIMES
		RECORD_EVENT(start_time, end_time, "am.updatePixVals", proc_times, proc_labels);
#endif		
		ssm.updateGradPts(am.getGradOffset());
#ifdef LOG_HACLK_TIMES
		RECORD_EVENT(start_time, end_time, "ssm.updateGradPts", proc_times, proc_labels);
#endif	
		am.updatePixGrad(ssm.getGradPts());
#ifdef LOG_HACLK_TIMES
		RECORD_EVENT(start_time, end_time, "am.updatePixGrad New", proc_times, proc_labels);
#endif	
		ssm.cmptInitPixJacobian(curr_pix_jacobian, am.getCurrPixGrad());
#ifdef LOG_HACLK_TIMES
		RECORD_EVENT(start_time, end_time, "ssm.cmptInitPixJacobian", proc_times, proc_labels);
#endif	
		am.updateSimilarity();
#ifdef LOG_HACLK_TIMES
		RECORD_EVENT(start_time, end_time, "am.update", proc_times, proc_labels);
#endif
		am.updateCurrGrad();
#ifdef LOG_HACLK_TIMES
		RECORD_EVENT(start_time, end_time, "am.updateCurrGrad", proc_times, proc_labels);
#endif
		am.cmptCurrJacobian(similarity_jacobian, curr_pix_jacobian);
#ifdef LOG_HACLK_TIMES
		RECORD_EVENT(start_time, end_time, "am.cmptCurrJacobian", proc_times, proc_labels);
#endif	
		if(use_newton_method){
			if(params.hess_type != InitialNewton && params.hess_type != ConvergedNewton){
				ssm.updateHessPts(am.getHessOffset());
#ifdef LOG_HACLK_TIMES
				RECORD_EVENT(start_time, end_time, "ssm.updateHessPts", proc_times, proc_labels);
#endif
				am.updatePixHess(ssm.getPts(), ssm.getHessPts());
#ifdef LOG_HACLK_TIMES
				RECORD_EVENT(start_time, end_time, "am.updatePixHess New", proc_times, proc_labels);
#endif
				ssm.cmptInitPixHessian(curr_pix_hessian, am.getCurrPixHess(), am.getCurrPixGrad());
#ifdef LOG_HACLK_TIMES
				RECORD_EVENT(start_time, end_time, "ssm.cmptInitPixHessian", proc_times, proc_labels);
#endif
				if(params.hess_type == Newton){
					am.cmptCurrHessian(hessian, curr_pix_jacobian, curr_pix_hessian);
#ifdef LOG_HACLK_TIMES
					RECORD_EVENT(start_time, end_time, "am.cmptCurrHessian (second order)", proc_times, proc_labels);
#endif
				} else if(params.hess_type == CurrentNewton){
					am.cmptSelfHessian(hessian, curr_pix_jacobian, curr_pix_hessian);
#ifdef LOG_HACLK_TIMES
					RECORD_EVENT(start_time, end_time, "am.cmptSelfHessian (second order)", proc_times, proc_labels);
#endif				
				}
		}
	} else{
			am.cmptCurrHessian(hessian, curr_pix_jacobian);
#ifdef LOG_HACLK_TIMES
			RECORD_EVENT(start_time, end_time, "am.cmptCurrHessian (first order)", proc_times, proc_labels);
#endif
		}

		ssm_update = -hessian.colPivHouseholderQr().solve(similarity_jacobian.transpose());
#ifdef LOG_HACLK_TIMES
		RECORD_EVENT(start_time, end_time, "ssm_update", proc_times, proc_labels);
#endif
		prev_corners = ssm.getCorners();
		ssm.compositionalUpdate(ssm_update);
#ifdef LOG_HACLK_TIMES
		RECORD_EVENT(start_time, end_time, "ssm.compositionalUpdate", proc_times, proc_labels);
#endif
		double update_norm = (prev_corners - ssm.getCorners()).squaredNorm();
#ifdef LOG_HACLK_TIMES
		RECORD_EVENT(start_time, end_time, "update_norm", proc_times, proc_labels);
		MatrixXd iter_times(proc_times.size(), 2);
		for(int proc_id = 0; proc_id < proc_times.size(); proc_id++){
			iter_times(proc_id, 0) = proc_times[proc_id];
		}
		double total_iter_time = iter_times.col(0).sum();
		iter_times.col(1) = (iter_times.col(0) / total_iter_time) * 100;
		utils::printScalarToFile(i, "iteration", time_fname, "%6d", "a");
		utils::printMatrixToFile(iter_times, "iter_times", time_fname, "%15.9f", "a",
			"\t", "\n", &proc_labels[0]);
		utils::printScalarToFile(total_iter_time, "total_iter_time", time_fname, "%15.9f", "a");
		upd_time+=total_iter_time;
#endif

#ifdef LOG_HACLK_DATA
		if(params.debug_mode){
			utils::printScalarToFile(frame_id, "\nframe_id", log_fname, "%6d", "a");
			utils::printScalarToFile(i, "iter", log_fname, "%6d", "a");	

			utils::printMatrixToFile(am.getCurrPixGrad().transpose().eval(), "am.curr_pix_grad", log_fname, "%15.9f", "a");
			utils::printMatrixToFile(curr_pix_jacobian.transpose().eval(), "curr_pix_jacobian", log_fname, "%15.9f", "a");
			utils::printMatrixToFile(similarity_jacobian, "similarity_jacobian", log_fname, "%15.9f", "a");

			utils::printMatrixToFile(am.getCurrErrVecGrad(), "am.curr_err_vec_grad", log_fname, "%15.9f", "a");
			utils::printMatrixToFile(am.getCurrGrad(), "am.curr_grad", log_fname, "%15.9f", "a");
			utils::printMatrix(similarity_jacobian, "similarity_jacobian");
			utils::printMatrixToFile(hessian, "hessian", log_fname, "%15.9f", "a");
			utils::printMatrixToFile(ssm_update, "ssm_update", log_fname, "%15.9f", "a");
			utils::printMatrixToFile(ssm.getCurrWarp(), "curr_warp", log_fname, "%15.9f", "a");

			am.updateSimilarity();
			double similarity_diff = am.similarity - am.init_similarity ;
			utils::printScalarToFile(am.getInitSimilarity(), "am.init_similarity", log_fname, "%15.9f", "a");
			utils::printScalarToFile(am.getCurrSimilarity(), "am.similarity", log_fname, "%15.9f", "a");
			utils::printScalarToFile(update_norm, "update_norm", log_fname, "%15.9f", "a");
			printf("frame: %3d\t iter: %3d\t update_norm: %15.12f\t init_similarity: %15.12f\t similarity: %15.12f\t similarity_diff: %15.12f\n",
				frame_id, i, update_norm, am.init_similarity, am.similarity, similarity_diff);
			am.iter_id++;
			}
#endif
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

#include "mtf/Macros/register.h"
_REGISTER_TRACKERS(HACLK);
