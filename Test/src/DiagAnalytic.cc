#include "mtf/Test/Diagnostics.h"
#include "mtf/Utilities/miscUtils.h"
#include "opencv2/highgui/highgui.hpp"

_MTF_BEGIN_NAMESPACE

void Diagnostics::generateAnalyticalData(VectorXd &param_range_vec,
int n_pts, ADT data_type, const char* fname){
	assert(param_range_vec.size() == state_size);

	//const char *data_name = getADTName(data_type);
	//printf("Computing analytical %s %s data for %d points with ", update_name, data_name, n_pts);
	//utils::printMatrix(param_range_vec, "parameter range");
	//utils::printMatrixToFile(init_dist_vec.transpose().eval(), "init_dist_vec", "log/diag_log.txt");

	//VectorXd param_range_vec = param_range_vec.array() / ssm_grad_norm_mean.array();
	VectorXd param_range_vec_norm = param_range_vec;

	//utils::printMatrix(param_range_vec_norm.transpose(), "param_range_vec_norm");
	//VectorXd base_state = ssm->getInitState();
	VectorXd min_state = -param_range_vec_norm;
	VectorXd max_state = param_range_vec_norm;

	//utils::printMatrix(param_range_vec_norm, "param_range_vec_norm");
	//utils::printMatrix(min_state, "min_state");
	//utils::printMatrix(max_state, "max_state");


	VectorXd state_update(state_size);
	diagnostics_data.resize(n_pts, 2 * state_size);
	//if(data_type == iHessian){
	//	am->cmptCurrHessian(hessian, init_pix_jacobian, init_pix_hessian);		
	//}

	//utils::printScalarToFile(frame_id, "frame", "log/diagnostics/extreme_corners.txt", "%d");
	//printf("img_height: %d img_width: %d\n", am->getImgHeight(), am->getImgWidth());
	//cv::Mat curr_img_cv_out(am->getImgHeight(), am->getImgWidth(), CV_8UC3);	
	//cv::Mat corners(2, 4, CV_64FC1); 
	//ssm->getCorners(corners);
	//cv::Rect roi = utils::getBestFitRectangle<int>(corners);
	//roi.x -= 40;
	//roi.y -= 40;
	//roi.width += 80;
	//roi.height += 80;

	for(unsigned int state_id = 0; state_id < state_size; ++state_id){
		//printf("Processing state parameter %d....\n", state_id);

		//if(data_type == ADT::Norm){			
		//	//am->getCurrImg().convertTo(curr_img_cv_out, curr_img_cv_out.type());
		//	curr_img_cv_out = CV_RGB(255, 255, 255);
		//	utils::printScalarToFile(state_id, "state", "log/diagnostics/extreme_corners.txt", "%d");
		//	utils::printMatrixToFile(ssm->getCorners(), nullptr, "log/diagnostics/extreme_corners.txt");
		//	drawCurrCorners(curr_img_cv_out, state_id, 1, false, CV_RGB(0, 0, 0));
		//}

		state_update.setZero();
		diagnostics_data.col(2 * state_id) = VectorXd::LinSpaced(n_pts, min_state(state_id), max_state(state_id));
		for(int pt_id = 0; pt_id < n_pts; pt_id++){

			state_update(state_id) = diagnostics_data(pt_id, 2 * state_id);

			//utils::printMatrix(ssm->getCorners(), "init_corners");

			updateState(state_update, state_id);

			//utils::printMatrix(state_update, "state_update");
			//utils::printMatrix(ssm->getCorners(), "curr_corners");

			double data_val = getADTVal(data_type, state_id);

			if(params.show_patches){
				curr_patch.convertTo(curr_patch_uchar, curr_patch_uchar.type());
				imshow(curr_patch_win_name, curr_patch_uchar);
				if(cv::waitKey(1) == 27){
					exit(0);
				}
			}
			if(params.show_corners){
				curr_img_cv.convertTo(curr_img_cv_uchar, curr_img_cv_uchar.type());
				putText(curr_img_cv_uchar, cv::format("param: %d range: %f", state_id, param_range_vec_norm(state_id)),
					cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(255, 255, 255));
				drawCurrCorners(curr_img_cv_uchar, state_id);
				imshow(curr_img_win_name, curr_img_cv_uchar);
				if(cv::waitKey(1) == 27){ exit(0); }
			}
			//if(data_type == ADT::Norm){
			//	if(pt_id == 0){
			//		utils::printMatrixToFile(ssm->getCorners(), nullptr, "log/diagnostics/extreme_corners.txt");
			//		drawCurrCorners(curr_img_cv_out, state_id, 1, false, CV_RGB(255, 0, 0));
			//	} else if(pt_id == n_pts - 1){
			//		utils::printMatrixToFile(ssm->getCorners(), nullptr, "log/diagnostics/extreme_corners.txt");
			//		drawCurrCorners(curr_img_cv_out, state_id, 1, false, CV_RGB(0, 255, 0));
			//	}
			//}
			diagnostics_data(pt_id, 2 * state_id + 1) = data_val;

			resetState(state_update, state_id);

			//if((pt_id + 1) % 50 == 0){ printf("Done %d points\n", pt_id + 1); }
		}
		//if(data_type == ADT::Norm){
		//	//imshow("curr_img_cv_out", curr_img_cv_out);
		//	//if(cv::waitKey(0) == 27){ exit(0); }
		//	cv::imwrite(cv::format("log/diagnostics/frame_%d_state_%d.bmp", frame_id, state_id), curr_img_cv_out(roi));
		//}
	}
	if(params.show_data){
		utils::printMatrix(diagnostics_data, cv::format("%s data", toString(data_type)).c_str());
	}
	if(fname){
		printf("Writing diagnostics data to: %s\n", fname);
		utils::printMatrixToFile(diagnostics_data, "diagnostics_data", fname, "%15.9f", "w");
	}
}
void Diagnostics::generateAnalyticalData3D(VectorXd &x_vec, VectorXd &y_vec,
	const VectorXd &param_range_vec, int n_pts, const Vector2i &state_ids,
	ADT data_type, const char* fname){
	assert(param_range_vec.size() == state_size);

	//const char *data_name = getADTName(data_type);
	//printf("Computing analytical %s %s data for %d points with ", update_name, data_name, n_pts);
	//utils::printMatrix(param_range_vec, "parameter range");
	//utils::printMatrixToFile(init_dist_vec.transpose().eval(), "init_dist_vec", "log/diag_log.txt");

	if((state_ids[0] < static_cast<int>(ssm_state_size)) !=
		(state_ids[1] < static_cast<int>(ssm_state_size))){
		throw utils::InvalidArgument("Diagnostics::generateAnalyticalData3D::Both state IDs must be either for SSM or AM\n");
	}

	//VectorXd param_range_vec = param_range_vec.array() / ssm_grad_norm_mean.array();
	VectorXd param_range_vec_norm = param_range_vec;

	utils::printMatrix(param_range_vec_norm.transpose(), "param_range_vec_norm");
	//VectorXd base_state = ssm->getInitState();
	VectorXd min_state = -param_range_vec_norm;
	VectorXd max_state = param_range_vec_norm;

	//utils::printMatrix(param_range_vec_norm, "param_range_vec_norm");
	//utils::printMatrix(min_state, "min_state");
	//utils::printMatrix(max_state, "max_state");


	VectorXd state_update(state_size);
	diagnostics_data.resize(n_pts, n_pts);
	//if(data_type == iHessian){
	//	am->cmptCurrHessian(hessian, init_pix_jacobian, init_pix_hessian);		
	//}
	x_vec = VectorXd::LinSpaced(n_pts, min_state(state_ids[0]), max_state(state_ids[0]));
	y_vec = VectorXd::LinSpaced(n_pts, min_state(state_ids[1]), max_state(state_ids[1]));

	state_update.setZero();
	for(int pt_id1 = 0; pt_id1 < n_pts; pt_id1++){
		//printf("Processing state parameter %d....\n", state_id);		
		state_update(state_ids[0]) = x_vec(pt_id1);
		for(int pt_id2 = 0; pt_id2 < n_pts; pt_id2++){

			state_update(state_ids[1]) = y_vec(pt_id2);

			//utils::printMatrix(ssm->getCorners(), "init_corners");
			updateState(state_update, state_ids[0]);
			//utils::printMatrix(state_update, "state_update");
			//utils::printMatrix(ssm->getCorners(), "curr_corners");
			diagnostics_data(pt_id1, pt_id2) = getADTVal(data_type, state_ids[1]);

			if(params.show_patches){
				curr_patch.convertTo(curr_patch_uchar, curr_patch_uchar.type());
				imshow(curr_patch_win_name, curr_patch_uchar);
				if(cv::waitKey(1) == 27)
					exit(0);
			}
			if(params.show_corners){
				curr_img_cv.convertTo(curr_img_cv_uchar, curr_img_cv_uchar.type());
				putText(curr_img_cv_uchar, cv::format("param: %d range: %f", state_ids[1], param_range_vec_norm(state_ids[1])),
					cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(255, 255, 255));
				drawCurrCorners(curr_img_cv_uchar, state_ids[1]);
				imshow(curr_img_win_name, curr_img_cv_uchar);
				if(cv::waitKey(1) == 27)
					exit(0);
			}
			resetState(state_update, state_ids[0]);
			//if((pt_id + 1) % 50 == 0){ printf("Done %d points\n", pt_id + 1); }
		}
	}
	if(fname){
		printf("Writing diagnostics data to: %s\n", fname);
		utils::printMatrixToFile(diagnostics_data, "diagnostics_data", fname, "%15.9f", "w");
	}
}
double Diagnostics::getADTVal(ADT data_type, int state_id){
	switch(data_type){
	case  ADT::Norm:
		updateCurrSimilarity();
		return am->getSimilarity();
	case  ADT::Likelihood:
		updateCurrSimilarity();
		return am->getLikelihood();
	case  ADT::FeatNorm:
		am->updatePixVals(ssm->getPts());
		am->updateDistFeat();
		//utils::printMatrixToFile(am->updateDistFeat().transpose().eval(), "curr_dist_vec", "log/diag_log.txt");
		return (*dist_func)(init_dist_vec.data(), am->getDistFeat(), am_dist_size);
	case  ADT::StdJac:
		updateCurrGrad();
		updateCurrPixJacobian();
		am->cmptCurrJacobian(similarity_jacobian, curr_pix_jacobian);
		return similarity_jacobian(state_id);
	case  ADT::ESMJac:
		updateCurrGrad();
		updateCurrPixJacobian();
		mean_pix_jacobian = (init_pix_jacobian + curr_pix_jacobian) / 2.0;
		am->cmptCurrJacobian(similarity_jacobian, mean_pix_jacobian);
		return similarity_jacobian(state_id);
	case  ADT::DiffOfJacs:
		updateCurrGrad();
		am->updateInitGrad();
		updateCurrPixJacobian();
		am->cmptDifferenceOfJacobians(similarity_jacobian, init_pix_jacobian, curr_pix_jacobian);
		return similarity_jacobian(state_id)*0.5;
	case  ADT::Std:
		updateCurrPixJacobian();
		updateCurrGrad();
		am->cmptCurrHessian(hessian, curr_pix_jacobian);
		return hessian(state_id, state_id);
	case  ADT::ESM:
		updateCurrGrad();
		updateCurrPixJacobian();
		mean_pix_jacobian = (init_pix_jacobian + curr_pix_jacobian) / 2.0;
		am->cmptCurrHessian(hessian, mean_pix_jacobian);
		return hessian(state_id, state_id);
	case  ADT::InitSelf:
		return init_self_hessian(state_id, state_id);
	case  ADT::CurrSelf:
		updateCurrSelfHess();
		updateCurrPixJacobian();
		am->cmptSelfHessian(hessian, curr_pix_jacobian);
		return hessian(state_id, state_id);
	case  ADT::Std2:
		updateCurrGrad();
		updateCurrPixJacobian();
		updateCurrPixHessian();
		am->cmptCurrHessian(hessian, curr_pix_jacobian,
			curr_pix_hessian);
		return hessian(state_id, state_id);
	case  ADT::ESM2:
		updateCurrGrad();
		updateCurrPixJacobian();
		updateCurrPixHessian();
		mean_pix_jacobian = (init_pix_jacobian + curr_pix_jacobian) / 2.0;
		mean_pix_hessian = (init_pix_hessian + curr_pix_hessian) / 2.0;
		am->cmptCurrHessian(hessian, mean_pix_jacobian,
			mean_pix_hessian);
		return hessian(state_id, state_id);
	case  ADT::InitSelf2:
		return init_self_hessian2(state_id, state_id);
	case  ADT::CurrSelf2:
		updateCurrSelfHess();
		updateCurrPixJacobian();
		updateCurrPixHessian();
		am->cmptSelfHessian(hessian, curr_pix_jacobian,
			curr_pix_hessian);
		return hessian(state_id, state_id);
	case  ADT::SumOfStd:
		updateCurrGrad();
		am->updateInitGrad();
		updateCurrPixJacobian();
		am->cmptSumOfHessians(hessian, init_pix_jacobian, curr_pix_jacobian);
		return hessian(state_id, state_id) / 2.0;
	case  ADT::SumOfStd2:
		updateCurrGrad();
		am->updateInitGrad();
		updateCurrPixJacobian();
		updateCurrPixHessian();
		am->cmptSumOfHessians(hessian, init_pix_jacobian, curr_pix_jacobian,
			init_pix_hessian, curr_pix_hessian);
		return hessian(state_id, state_id) / 2.0;
	case  ADT::SumOfSelf:
		updateCurrSelfHess();
		updateCurrPixJacobian();
		am->cmptSelfHessian(hessian, curr_pix_jacobian);
		hessian += init_self_hessian;
		return hessian(state_id, state_id) / 2.0;
	case  ADT::SumOfSelf2:
		updateCurrSelfHess();
		updateCurrPixJacobian();
		updateCurrPixHessian();
		am->cmptSelfHessian(hessian, curr_pix_jacobian,
			curr_pix_hessian);
		hessian += init_self_hessian2;
		return hessian(state_id, state_id) / 2.0;
	default:
		throw utils::InvalidArgument(cv::format("Diagnostics :: Invalid data type specified: %d", data_type));
	}
}
_MTF_END_NAMESPACE
