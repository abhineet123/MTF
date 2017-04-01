#include "mtf/Test/Diagnostics.h"
#include "mtf/Utilities/miscUtils.h"

_MTF_BEGIN_NAMESPACE

void Diagnostics::generateNumericalData(VectorXd &param_range_vec,
	int n_pts, NDT data_type, const char* fname, double grad_diff){
	assert(param_range_vec.size() == state_size);

	//printf("Computing numerical %s %s data for %d points with grad_diff: %f and ",
	//	update_name, data_name, n_pts, grad_diff);
	//utils::printMatrix(param_range_vec, "parameter range");


	//VectorXd param_range_vec = param_range_vec.array() / ssm_grad_norm_mean.array();
	VectorXd param_range_vec_norm = param_range_vec;

	//VectorXd base_state = ssm->getInitState();
	VectorXd min_state = -param_range_vec_norm;
	VectorXd max_state = param_range_vec_norm;

	VectorXd state_update(state_size), grad_update(state_size);
	diagnostics_data.resize(n_pts, 2 * state_size);

	double grad_mult_factor = 1.0 / (2 * grad_diff);
	double data_val;
	for(int state_id = 0; state_id < state_size; state_id++){
		//printf("Processing state parameter %d....\n", state_id);

		state_update.setZero();
		grad_update.setZero();
		diagnostics_data.col(2 * state_id) = VectorXd::LinSpaced(n_pts, min_state(state_id), max_state(state_id));
		for(int pt_id = 0; pt_id < n_pts; pt_id++){
			state_update(state_id) = diagnostics_data(pt_id, 2 * state_id);

			updateState(state_update, state_id);

			switch(data_type){
			case NDT::Jacobian:
			{
				grad_update(state_id) = grad_diff;
				updateState(grad_update, state_id);
				updateCurrSimilarity();
				double norm_inc = am->getSimilarity();	
				resetState(grad_update, state_id);

				grad_update(state_id) = -grad_diff;
				updateState(grad_update, state_id);
				updateCurrSimilarity();
				double norm_dec = am->getSimilarity();
				resetState(grad_update, state_id);

				data_val = (norm_inc - norm_dec) * grad_mult_factor;
				break;
			}
			case NDT::Hessian:
			{
				grad_update(state_id) = grad_diff;
				updateState(grad_update, state_id);
				updateCurrPixJacobian();
				updateCurrGrad();
				am->cmptCurrJacobian(similarity_jacobian, curr_pix_jacobian);
				double jacobian_inc = similarity_jacobian(state_id);
				resetState(grad_update, state_id);

				grad_update(state_id) = -grad_diff;
				updateState(grad_update, state_id);
				updateCurrPixJacobian();
				updateCurrGrad();
				am->cmptCurrJacobian(similarity_jacobian, curr_pix_jacobian);
				double jacobian_dec = similarity_jacobian(state_id);
				resetState(grad_update, state_id);

				data_val = (jacobian_inc - jacobian_dec) * grad_mult_factor;
				break;
			}
			case NDT::NHessian:
			{
				updateCurrSimilarity();
				double norm = am->getSimilarity();

				grad_update(state_id) = grad_diff;
				updateState(grad_update, state_id);
				updateCurrSimilarity();
				double norm_inc = am->getSimilarity();
				resetState(grad_update, state_id);

				grad_update(state_id) = -grad_diff;
				updateState(grad_update, state_id);
				updateCurrSimilarity();
				double norm_dec = am->getSimilarity();
				resetState(grad_update, state_id);

				data_val = (norm_inc + norm_dec - 2 * norm) / (grad_diff * grad_diff);
				break;
			}
			default:
				throw std::invalid_argument("Diagnostics :: Invalid numerical data type specified");
			}
			diagnostics_data(pt_id, 2 * state_id + 1) = data_val;
			resetState(state_update, state_id);
			//if((pt_id + 1) % 50 == 0){ printf("Done %d points\n", pt_id + 1); }
		}
	}
	if(params.show_data){
		utils::printMatrix(diagnostics_data, cv::format("%s data", toString(data_type)).c_str());
	}
	if(fname){
		printf("Writing diagnostics data to: %s\n", fname);
		utils::printMatrixToFile(diagnostics_data, "diagnostics_data", fname, "%15.9f", "w");
	}
}


void Diagnostics::generateInverseNumericalData(VectorXd &param_range_vec,
	int n_pts, NDT data_type, const char* fname, double grad_diff){
	assert(param_range_vec.size() == state_size);

	//printf("Computing numerical %s %s data for %d points with grad_diff: %f and ",
	//	update_name, data_name, n_pts, grad_diff);
	//utils::printMatrix(param_range_vec, "parameter range");

	am->updatePixVals(ssm->getPts());

	ssm->initialize(init_corners);

	//VectorXd base_state = ssm->getInitState();
	VectorXd min_state = -param_range_vec;
	VectorXd max_state = param_range_vec;

	VectorXd state_update(state_size), grad_update(state_size);
	diagnostics_data.resize(n_pts, 2 * state_size);

	double grad_mult_factor = 1.0 / (2 * grad_diff);
	double data_val;

	// backup the current image
	curr_img_cv = am->getCurrImg();

	// all further updates need to be done using the initial image;
	// so the current image  is not needed anymore in this call
	am->setCurrImg(init_img_cv);

	for(int state_id = 0; state_id < ssm_state_size; state_id++){
		//printf("Processing state parameter %d....\n", state_id);

		state_update.setZero();
		grad_update.setZero();
		diagnostics_data.col(2 * state_id) = VectorXd::LinSpaced(n_pts, min_state(state_id), max_state(state_id));
		for(int pt_id = 0; pt_id < n_pts; pt_id++){
			state_update(state_id) = diagnostics_data(pt_id, 2 * state_id);

			updateState(state_update, state_id);

			switch(data_type){
			case NDT::Jacobian:
			{
				grad_update(state_id) = grad_diff;
				updateState(grad_update, state_id);
				updateInitSimilarity();
				double norm_inc = am->getSimilarity();
				resetState(grad_update, state_id);

				grad_update(state_id) = -grad_diff;
				updateState(grad_update, state_id);
				updateInitSimilarity();
				double norm_dec = am->getSimilarity();
				resetState(grad_update, state_id);

				data_val = (norm_inc - norm_dec) * grad_mult_factor;
				break;
			}
			case NDT::Hessian:
			{
				grad_update(state_id) = grad_diff;
				updateState(grad_update, state_id);
				updateInitPixJacobian();
				updateInitSimilarity();
				updateInitGrad();
				am->cmptInitJacobian(similarity_jacobian, init_pix_jacobian);
				double jacobian_inc = similarity_jacobian(state_id);
				resetState(grad_update, state_id);

				grad_update(state_id) = -grad_diff;
				updateState(grad_update, state_id);
				updateInitPixJacobian();
				updateInitSimilarity();
				updateInitGrad();
				am->cmptInitJacobian(similarity_jacobian, init_pix_jacobian);
				double jacobian_dec = similarity_jacobian(state_id);
				resetState(grad_update, state_id);

				data_val = (jacobian_inc - jacobian_dec) * grad_mult_factor;
				break;
			}
			case NDT::NHessian:
			{
				updateInitSimilarity();
				double norm = am->getSimilarity();

				grad_update(state_id) = grad_diff;
				updateState(grad_update, state_id);
				updateInitSimilarity();
				double norm_inc = am->getSimilarity();
				resetState(grad_update, state_id);

				grad_update(state_id) = -grad_diff;
				updateState(grad_update, state_id);
				updateInitSimilarity();
				double norm_dec = am->getSimilarity();
				resetState(grad_update, state_id);

				data_val = (norm_inc + norm_dec - 2 * norm) / (grad_diff * grad_diff);
				break;
			}
			default:
				throw std::invalid_argument("Diagnostics :: Invalid numerical data type specified");
			}
			diagnostics_data(pt_id, 2 * state_id + 1) = data_val;
			resetState(state_update, state_id);
			//if((pt_id + 1) % 50 == 0){ printf("Done %d points\n", pt_id + 1); }
		}
	}
	// restore the current image
	am->setCurrImg(curr_img_cv);
	if(fname){
		printf("Writing diagnostics data to: %s\n", fname);
		utils::printMatrixToFile(diagnostics_data, "diagnostics_data", fname, "%15.9f", "w");
	}
}

_MTF_END_NAMESPACE
