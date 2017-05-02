#include "mtf/Test/Diagnostics.h"
#include "mtf/Utilities/miscUtils.h"
#include "opencv2/highgui/highgui.hpp"

_MTF_BEGIN_NAMESPACE


void Diagnostics::generateInverseAnalyticalData(VectorXd &param_range_vec,
int n_pts, ADT data_type, const char* fname){
	assert(param_range_vec.size() == state_size);

	am->updatePixVals(ssm->getPts());
	//updateCurrPixJacobian();
	//updateCurrPixHessian();

	if(data_type == ADT::CurrSelf){
		updateCurrPixJacobian();
		am->cmptSelfHessian(curr_self_hessian, curr_pix_jacobian);
	} else if(data_type == ADT::CurrSelf2){
		updateCurrPixJacobian();
		updateCurrPixHessian();
		am->cmptSelfHessian(curr_self_hessian2, curr_pix_jacobian, curr_pix_hessian);
	} else if(data_type == ADT::FeatNorm){
		am->updateDistFeat(curr_dist_vec.data());
	}

	if(params.show_patches){
		curr_patch.convertTo(curr_patch_uchar, curr_patch_uchar.type());
		//utils::printMatrix<double>(curr_patch, "curr_patch");
		//utils::printMatrix<uchar>(curr_patch_uchar, "curr_patch_uchar", "%d");
		cv::imshow(curr_patch_win_name, curr_patch_uchar);
	}

	ssm->initialize(init_corners);

	VectorXd min_state = -param_range_vec;
	VectorXd max_state = param_range_vec;

	VectorXd state_update(state_size);
	diagnostics_data.resize(n_pts, 2 * state_size);

	// all further updates need to be done using the initial image;
	// as the current image  is not needed anymore, there is no need to back it up;
	curr_img_cv = am->getCurrImg();
	am->setCurrImg(init_img_cv);

	for(unsigned int state_id = 0; state_id < state_size; ++state_id){
		//printf("Processing state parameter %d....\n", state_id);

		state_update.setZero();
		diagnostics_data.col(2 * state_id) = VectorXd::LinSpaced(n_pts, min_state(state_id), max_state(state_id));
		
		for(int pt_id = 0; pt_id < n_pts; pt_id++){

			state_update(state_id) = diagnostics_data(pt_id, 2 * state_id);		

			updateState(state_update, state_id);
			//utils::printMatrix(state_update, "state_update");
			//utils::printMatrix(ssm->getCorners(), "curr_corners");				
			
			diagnostics_data(pt_id, 2 * state_id + 1) = getInvADTVal(data_type, state_id);				

			if(params.show_patches){
				init_patch.convertTo(init_patch_uchar, init_patch_uchar.type());
				cv::imshow(init_patch_win_name, init_patch_uchar);

				curr_patch.convertTo(curr_patch_uchar, curr_patch_uchar.type());
				cv::imshow(curr_patch_win_name, curr_patch_uchar);

				if(cv::waitKey(1) == 27)
					exit(0);
			}
			if(params.show_corners){
				init_img_cv.convertTo(init_img_cv_uchar, init_img_cv_uchar.type());
				putText(init_img_cv_uchar, cv::format("param: %d range: %f", state_id, param_range_vec(state_id)),
					cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(255, 255, 255));
				drawCurrCorners(init_img_cv_uchar);
				cv::namedWindow(init_img_win_name);
				imshow(init_img_win_name, init_img_cv_uchar);
				if(cv::waitKey(1) == 27)
					exit(0);
			}
			resetState(state_update, state_id);
			//if((pt_id + 1) % 50 == 0){ printf("Done %d points\n", pt_id + 1); }
		}
	}
	am->setCurrImg(curr_img_cv);
	if(params.show_data){
		utils::printMatrix(diagnostics_data, cv::format("%s data", toString(data_type)).c_str());
	}
	if(fname){
		printf("Writing diagnostics data to: %s\n", fname);
		utils::printMatrixToFile(diagnostics_data, "diagnostics_data", fname, "%15.9f", "w");
	}
}

double Diagnostics::getInvADTVal(ADT data_type, int state_id){
	switch(data_type){
	case  ADT::Norm:
		updateInitSimilarity();
		return am->getSimilarity();		
	case  ADT::Likelihood:
		updateInitSimilarity();
		return am->getLikelihood();		
	case  ADT::FeatNorm:
		// don't need curr_pix_vals anymore so can safely overwrite with the values extracted from init_img
		am->updatePixVals(ssm->getPts());
		am->updateDistFeat();
		return (*dist_func)(am->getDistFeat(), curr_dist_vec.data(), am_dist_size);
	case  ADT::StdJac:
		updateInitGrad();
		updateInitPixJacobian();
		am->cmptInitJacobian(similarity_jacobian, init_pix_jacobian);
		return similarity_jacobian(state_id);		
	case  ADT::ESMJac:
		updateInitGrad();
		updateInitPixJacobian();
		mean_pix_jacobian = (init_pix_jacobian + curr_pix_jacobian) / 2.0;
		am->cmptInitJacobian(similarity_jacobian, mean_pix_jacobian);
		return similarity_jacobian(state_id);		
	case  ADT::DiffOfJacs:
		updateInitGrad();
		updateInitPixJacobian();
		am->updateCurrGrad();
		am->cmptDifferenceOfJacobians(similarity_jacobian, init_pix_jacobian, curr_pix_jacobian);
		return similarity_jacobian(state_id)*0.5;		
	case  ADT::Std:
		updateInitHess();
		updateInitPixJacobian();
		//utils::printMatrix(init_pix_jacobian, cv::format("%d init_pix_jacobian", state_id).c_str());
		am->cmptInitHessian(hessian, init_pix_jacobian);
		//utils::printMatrix(hessian, cv::format("%d hessian", state_id).c_str());
		return hessian(state_id, state_id);		
	case  ADT::Std2:
		updateInitHess();
		updateInitPixJacobian();
		updateInitPixHessian();
		am->cmptInitHessian(hessian, init_pix_jacobian,
			init_pix_hessian);
		return hessian(state_id, state_id);		
	case  ADT::ESM:
		updateInitHess();
		updateInitPixJacobian();
		mean_pix_jacobian = (init_pix_jacobian + curr_pix_jacobian) / 2.0;
		am->cmptInitHessian(hessian, mean_pix_jacobian);
		return hessian(state_id, state_id);		
	case  ADT::ESM2:
		updateInitHess();
		updateInitPixJacobian();
		updateInitPixHessian();
		mean_pix_jacobian = (init_pix_jacobian + curr_pix_jacobian) / 2.0;
		mean_pix_hessian = (init_pix_hessian + curr_pix_hessian) / 2.0;
		am->cmptCurrHessian(hessian, mean_pix_jacobian,
			mean_pix_hessian);
		return hessian(state_id, state_id);		
	case  ADT::InitSelf:
		updateInitSelfHess();
		updateInitPixJacobian();
		am->cmptSelfHessian(hessian, init_pix_jacobian);
		return hessian(state_id, state_id);		
	case  ADT::InitSelf2:
		updateInitSelfHess();
		updateInitPixJacobian();
		updateInitPixHessian();
		am->cmptSelfHessian(hessian, init_pix_jacobian, init_pix_hessian);
		return hessian(state_id, state_id);		
	case  ADT::CurrSelf:
		return curr_self_hessian(state_id, state_id);		
	case  ADT::CurrSelf2:
		return curr_self_hessian2(state_id, state_id);		
	case  ADT::SumOfStd:
		updateInitHess();
		updateInitPixJacobian();
		am->updateCurrGrad();
		am->cmptSumOfHessians(hessian, init_pix_jacobian, curr_pix_jacobian);
		return hessian(state_id, state_id) / 2.0;		
	case  ADT::SumOfStd2:
		updateInitHess();
		updateInitPixJacobian();
		updateInitPixHessian();
		am->updateCurrGrad();
		am->cmptSumOfHessians(hessian, init_pix_jacobian, curr_pix_jacobian,
			init_pix_hessian, curr_pix_hessian);
		return hessian(state_id, state_id) / 2.0;		
	case  ADT::SumOfSelf:
		updateInitSelfHess();
		updateInitPixJacobian();
		am->cmptSelfHessian(hessian, init_pix_jacobian);
		hessian += curr_self_hessian;
		return hessian(state_id, state_id) / 2.0;		
	case  ADT::SumOfSelf2:
		updateInitSelfHess();
		updateInitPixJacobian();
		updateInitPixHessian();
		am->cmptSelfHessian(hessian, init_pix_jacobian, init_pix_hessian);
		hessian += curr_self_hessian2;
		return hessian(state_id, state_id) / 2.0;		
	default:
		throw utils::InvalidArgument("Diagnostics :: Invalid data type specified");
	}
}


_MTF_END_NAMESPACE
