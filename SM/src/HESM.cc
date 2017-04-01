#include "mtf/SM/HESM.h"
#include "mtf/Utilities/miscUtils.h"
#include <time.h>

_MTF_BEGIN_NAMESPACE

template <class AM, class SSM, class SSM2>
HESM<AM, SSM, SSM2>::HESM(const ParamType *hesm_params,
	const AMParams *am_params,
	const SSMParams *ssm_params, SSM2Params *ssm2_params) :
	SearchMethod<AM, SSM>(am_params, ssm_params),
	params(hesm_params){

	ssm2 = new SSM2(resx, resy, ssm2_params);

	ssm_data.resize(n_pix, ssm.getStateSize());
	ssm2_data.resize(n_pix, ssm2->state_vec_size);

	printf("\n");
	printf("Using Hierarchical ESM SM with:\n");
	printf("max_iters: %d\n", params.max_iters);
	printf("epsilon: %f\n", params.epsilon);
	printf("rec_init_err_grad: %d\n", params.rec_init_err_grad);
	printf("debug_mode: %d\n", params.debug_mode);
	printf("appearance model: %s\n", am.name.c_str());
	printf("primary state space model: %s\n", ssm.name.c_str());
	printf("secondary state space model: %s\n", ssm2->name.c_str());
	printf("\n");

	name = "hesm";
	log_fname = "log/mtf_hesm_log.txt";
	time_fname = "log/mtf_hesm_times.txt";
	frame_id = 0;

	if(ssm2->state_vec_size >= ssm.getStateSize()){
		printf("Warning: Secondary SSM has a state size not smaller than the primary SSM\n");
	}

	//init_pix_jacobian.resize(n_pix, ssm.getStateSize());
	//curr_pix_jacobian.resize(n_pix, ssm.getStateSize());
	//similarity_jacobian.resize(ssm.getStateSize());
	//hessian.resize(ssm.getStateSize(), ssm.getStateSize());

	//init_pix_jacobian2.resize(n_pix, ssm.getStateSize());
	//curr_pix_jacobian2.resize(n_pix, ssm.getStateSize());
	//similarity_jacobian2.resize(ssm.getStateSize());
	//hessian2.resize(ssm.getStateSize(), ssm.getStateSize());
	//err_vec_jacobian.resize(am.err_vec_size, ssm.getStateSize());
	//ssm1_update.resize(ssm.getStateSize());
	//ssm2_update.resize(ssm2->state_vec_size);

}

template <class AM, class SSM, class SSM2>
void HESM<AM, SSM, SSM2>::initialize(const cv::Mat &corners){
#ifdef LOG_HESM_TIMES
	clock_t start_time=clock();
#endif
	ssm.initialize(corners);
	ssm2->initialize(corners);

	//initialize the appearance model only with the primary SSM since initial location of both SSMs is same anyway
	am.initializePixVals(curr_img, ssm.curr_pts, img_height, img_width);
	am.initializePixGrad(curr_img, ssm.getPtsHom(), ssm.getInitWarp(), img_height, img_width);
#ifdef USE_AM_ERR_VEC
	am.initializeErrVec();
	am.initializeErrVecGrad();
	am.initializeSimilarity();
#endif
	am.initializeGrad();

	ssm.cmptInitPixJacobian(ssm_data.init_pix_jacobian, am.getInitPixGrad());
	ssm2->cmptInitPixJacobian(ssm2_data.init_pix_jacobian, am.getInitPixGrad());

	ssm.getCorners(cv_corners_mat);

#ifdef LOG_HESM_TIMES
	clock_t end_time = clock();
	double init_time = ((double) (end_time - start_time))/CLOCKS_PER_SEC;
	utils::printScalarToFile(init_time, "init_time", time_fname, "%15.9f", "w");
#endif
#ifdef LOG_HESM_DATA
	if(params.debug_mode){
		utils::printScalarToFile("initialize", "function", log_fname, "%s", "w");
		utils::printMatrixToFile(init_pix_jacobian, "init_pix_jacobian", log_fname, "%15.9f", "a");
		utils::printMatrixToFile(am.init_err_grad, "am.init_err_grad", log_fname, "%15.9f", "a");
		utils::printMatrixToFile(am.init_err_vec, "am.init_err_vec", log_fname, "%15.9f", "a");
		am.initSimilarity();
		utils::printScalarToFile(am.init_similarity, " am.init_similarity", log_fname, "%15.9f", "a");
		printf("init_similarity: %15.12f\n", am.init_similarity);
		//utils::printMatrixToFile(am.init_pix_vals, "init_pix_vals", log_fname, "%15.9f", "a");
		//utils::printMatrixToFile(ssm.curr_warp, "init_warp", log_fname, "%15.9f", "a");
		//utils::printMatrixToFile(am.init_warped_pix_grad, "init_warped_pix_grad", 
		//	log_fname, "%15.9f", "a");
		//utils::printScalarToFile(params.epsilon, "params.epsilon", log_fname, "%15.9f", "a");
		//utils::printMatrixToFile(curr_img, "init_img", "log/mtf_img.txt", "%5.1f");
	}
#endif
}
template <class AM, class SSM, class SSM2>
template<class _SSM> void  HESM<AM, SSM, SSM2>::updateSSM(_SSM *_ssm, SSMData &ssm_data){
#ifdef LOG_HESM_TIMES
	INIT_TIMER(start_time);
#endif
	// extract pixel values from the current image at the latest known position of the object
	am.updatePixVals(curr_img, _ssm.getPts(), img_height, img_width);
#ifdef LOG_HESM_TIMES
	TIME_EVENT(start_time, end_time, iter_times(0, 0));
#endif
	// compute pixel gradient of the current image warped with the current warp
	am.updatePixGrad(curr_img, ssm.getPtsHom(), ssm.getCurrWarp(), img_height, img_width);
#ifdef LOG_HESM_TIMES
	TIME_EVENT(start_time, end_time, iter_times(1, 0));
#endif
	// multiply the pixel gradient with the SSM Jacobian to get the Jacobian of pixel values w.r.t. SSM parameters
	_ssm.cmptInitPixJacobian(ssm_data.curr_pix_jacobian, am.getCurrPixGrad());
#ifdef LOG_HESM_TIMES
	TIME_EVENT(start_time, end_time, iter_times(2, 0));
#endif
#ifdef USE_AM_ERR_VEC
	// compute the error vector between current and initial pixel values
	am.updateCurrErrVec();

#ifdef LOG_HESM_TIMES
	TIME_EVENT(start_time, end_time, iter_times(3, 0));
#endif		
	// update the gradient of the error vector w.r.t. current pixel values
	am.updateCurrErrVecGrad();
#ifdef LOG_HESM_TIMES
	TIME_EVENT(start_time, end_time, iter_times(4, 0));
#endif	
	// update the gradient of the error vector w.r.t. initial pixel values
	am.updateInitErrVecGrad();
	//am.lmultCurrSimGrad(similarity_jacobian, mean_jacobian);

#ifdef LOG_HESM_TIMES
	TIME_EVENT(start_time, end_time, iter_times(6, 0));
#endif
#endif
	// update the gradient of the error norm w.r.t. current pixel values
	am.updateCurrGrad();

#ifdef LOG_HESM_TIMES
	TIME_EVENT(start_time, end_time, iter_times(5, 0));
#endif
	// update the gradient of the error norm w.r.t. initial pixel values
	am.updateInitGrad();

#ifdef LOG_HESM_TIMES
	TIME_EVENT(start_time, end_time, iter_times(6, 0));
#endif
	// compute the mean difference between the Jacobians of the error norm w.r.t. current and initial values of SSM parameters
	//mean_jacobian = (init_jacobian + curr_jacobian) / 2;
	am.cmptDifferenceOfJacobians(ssm_data.similarity_jacobian, ssm_data.mean_pix_jacobian,
		ssm_data.init_pix_jacobian, ssm_data.curr_pix_jacobian);

#ifdef LOG_HESM_TIMES
	TIME_EVENT(start_time, end_time, iter_times(7, 0));
#endif
	// compute the Hessian of the error norm w.r.t. mean of the current and initial values of SSM parameters
	//hessian = mean_jacobian.transpose() * mean_jacobian;
	am.cmptMeanSimHessian(ssm_data.hessian, ssm_data.mean_pix_jacobian,
		ssm_data.init_pix_jacobian, ssm_data.curr_pix_jacobian);

#ifdef LOG_HESM_TIMES
	TIME_EVENT(start_time, end_time, iter_times(8, 0));
#endif	

	ssm_data.ssm_update = ssm_data.hessian.colPivHouseholderQr().solve(ssm_data.similarity_jacobian.transpose());

#ifdef LOG_HESM_TIMES
	TIME_EVENT(start_time, end_time, iter_times(9, 0))
#endif
		_ssm.compositionalUpdate(ssm_data.ssm_update);
}

template <class AM, class SSM, class SSM2>
void HESM<AM, SSM, SSM2>::update(){
	am.frame_id = ++frame_id;
	am.setFirstIter();
	for(int i = 0; i < params.max_iters; i++){
		updateSSM<SSM2>(ssm2, ssm2_data);
		prev_corners = ssm2->getCorners();
		double update_norm = (prev_corners - ssm2->getCorners()).squaredNorm();
		if(update_norm < params.epsilon)
			break;
		am.clearFirstIter();
	}
	ssm2->getWarpFromState(ssm2_warp_update, ssm2_data.ssm_update);
	ssm.compositionalUpdate(ssm2_warp_update);
	am.setFirstIter();
	for(int i = 0; i < params.max_iters; i++){
		prev_corners = ssm.getCorners();
		updateSSM<SSM>(ssm, ssm_data);
		double update_norm = (prev_corners - ssm.getCorners()).squaredNorm();
		if(update_norm < params.epsilon)
			break;
		am.clearFirstIter();
	}
	ssm.getCorners(cv_corners_mat);
}

_MTF_END_NAMESPACE

#include "mtf/Macros/register.h"
_REGISTER_HTRACKERS(HESM);
