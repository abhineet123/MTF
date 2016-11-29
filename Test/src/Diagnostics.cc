#include "mtf/Test/Diagnostics.h"
#include "mtf/Utilities/miscUtils.h"
#include <time.h>
#include <stdexcept>
#if CV_MAJOR_VERSION == 3
#include "opencv2/imgproc/imgproc.hpp"
#endif

#include "DiagAnalytic.cc"
#include "DiagInvAnalytic.cc"
#include "DiagNumeric.cc"
#include "DiagHelper.cc"

_MTF_BEGIN_NAMESPACE

DiagnosticsParams::DiagnosticsParams(UpdateType _update_type,
bool _show_data, bool _show_corners,
bool _show_patches, bool _enable_validation,
double _validation_prec):
update_type(_update_type),
show_data(_show_data),
show_corners(_show_corners),
show_patches(_show_patches),
enable_validation(_enable_validation),
validation_prec(_validation_prec){}

DiagnosticsParams::DiagnosticsParams(const DiagnosticsParams *params) :
update_type(static_cast<UpdateType>(DIAG_UPDATE_TYPE)),
show_data(DIAG_SHOW_DATA),
show_corners(DIAG_SHOW_CORNERS),
show_patches(DIAG_SHOW_PATCHES),
enable_validation(DIAG_ENABLE_VALIDATION),
validation_prec(DIAG_VALIDATION_PREC){
	if(params){
		update_type = params->update_type;
		show_data = params->show_data;
		show_corners = params->show_corners;
		show_patches = params->show_patches;
		enable_validation = params->enable_validation;
		validation_prec = params->validation_prec;
	}
}
Diagnostics::~Diagnostics(){
	if(params.show_corners){
		cv::destroyWindow(init_img_win_name);
		cv::destroyWindow(curr_img_win_name);
	}
	if(params.show_patches){
		cv::destroyWindow(init_patch_win_name);
		cv::destroyWindow(curr_patch_win_name);
	}
}
Diagnostics::Diagnostics(AM _am, SSM _ssm,
	const ParamType *diag_params) :
	params(diag_params), am(_am), ssm(_ssm),
	init_img(0, 0, 0){

	switch(params.update_type){
	case UpdateType::Additive:
		update_name = "Additive";
		break;
	case UpdateType::Compositional:
		update_name = "Compositional";
		break;
	default:
		throw std::invalid_argument("Diagnostics :: Invalid update type specified");
	}

	n_pix = am->getNPix();
	am_dist_size = am->getDistFeatSize();
	ssm_state_size = ssm->getStateSize();
	am_state_size = am->getStateSize();
	state_size = ssm_state_size + am_state_size;

	frame_id = 0;

	printf("\n");
	printf("Using Diagnostics with :\n");
	printf("appearance model: %s\n", am->name.c_str());
	printf("state space model: %s\n", ssm->name.c_str());
	printf("update type: %s\n", update_name);
	printf("am_dist_size: %d\n", am_dist_size);
	printf("ssm_state_size: %d\n", ssm_state_size);
	printf("am_state_size: %d\n", am_state_size);
	printf("state_size: %d\n", state_size);
	printf("\n");

	inv_ssm_state.resize(ssm_state_size);
	inv_am_state.resize(am_state_size);

	init_pix_jacobian.resize(n_pix, ssm->getStateSize());
	init_pix_hessian.resize(ssm_state_size*ssm_state_size, n_pix);
	curr_pix_jacobian.resize(n_pix, ssm_state_size);
	curr_pix_hessian.resize(ssm_state_size*ssm_state_size, n_pix);
	mean_pix_jacobian.resize(n_pix, ssm_state_size);
	mean_pix_hessian.resize(ssm_state_size*ssm_state_size, n_pix);

	similarity_jacobian.resize(state_size);
	hessian.resize(state_size, state_size);
	init_hessian.resize(state_size, state_size);

	init_self_hessian.resize(state_size, state_size);
	init_self_hessian2.resize(state_size, state_size);

	init_dist_vec.resize(am_dist_size);
	curr_dist_vec.resize(am_dist_size);

	ssm_grad_norm.resize(n_pix, ssm_state_size);
	ssm_grad_norm_mean.resize(ssm_state_size);

	if(params.show_corners){
		init_img_win_name = "Initial Image";
		curr_img_win_name = "Current Image";
	}

	if(params.show_patches){
		init_patch_win_name = "Initial Patch";
		curr_patch_win_name = "Current Patch";
	}
}


bool Diagnostics::validateHessians(const MatrixXd &self_hessian){
	bool matching_hess = true;
	double init_hessian_diff_norm = (self_hessian - init_hessian).squaredNorm();
	if(init_hessian_diff_norm > params.validation_prec){
		printf("Diagnostics:: Init Self Hessian does not match Init Hessian\n");
		utils::printMatrix(self_hessian, "self_hessian");
		utils::printMatrix(init_hessian, "init_hessian");
		utils::printScalar(init_hessian_diff_norm, "init_hessian_diff_norm", "%e");
		double relative_diff = 2 * init_hessian_diff_norm / (init_hessian.squaredNorm() + self_hessian.squaredNorm());
		utils::printScalar(relative_diff, "relative_diff", "%e");
		matching_hess = false;
		//throw std::logic_error("Diagnostics:: Init Self Hessian does not match Init Hessian");
	}
	double curr_hessian_diff_norm = (self_hessian - hessian).squaredNorm();
	if(curr_hessian_diff_norm > params.validation_prec){
		printf("Diagnostics:: Init Self Hessian does not match Curr Hessian\n");
		utils::printMatrix(self_hessian, "self_hessian");
		utils::printMatrix(hessian, "hessian");
		utils::printScalar(curr_hessian_diff_norm, "curr_hessian_diff_norm", "%e");
		double relative_diff = 2 * curr_hessian_diff_norm / (hessian.squaredNorm() + self_hessian.squaredNorm());
		utils::printScalar(relative_diff, "relative_diff", "%e");
		matching_hess = false;
	}
	double inter_hessian_diff_norm = (init_hessian - hessian).squaredNorm();
	if(inter_hessian_diff_norm > params.validation_prec){
		printf("Diagnostics:: Curr and Init Hessians do not match\n");
		utils::printMatrix(init_hessian, "init_hessian");
		utils::printMatrix(hessian, "hessian");
		utils::printScalar(inter_hessian_diff_norm, "inter_hessian_diff_norm", "%e");
		double relative_diff = 2 * inter_hessian_diff_norm / (init_hessian.squaredNorm() + hessian.squaredNorm());
		utils::printScalar(relative_diff, "relative_diff", "%e");
		matching_hess = false;
	}
	return matching_hess;
}


void Diagnostics::initialize(const cv::Mat &corners, 
	const bool *gen_flags){

	ssm->initialize(corners);
	am->initializePixVals(ssm->getPts());
	am->initializeSimilarity();

	if(gen_flags[0] || gen_flags[1] || gen_flags[2]){
		initializePixJacobian();
		am->initializeGrad();
		ssm->cmptInitPixJacobian(init_pix_jacobian, am->getInitPixGrad());
		if(gen_flags[1] || gen_flags[2]){
			am->initializeHess();
		}
		if(gen_flags[1]){
			am->cmptSelfHessian(init_self_hessian, init_pix_jacobian);
		}
		if(gen_flags[2]){
			initializePixHessian();
			ssm->cmptInitPixHessian(init_pix_hessian, am->getInitPixHess(), am->getInitPixGrad());
			am->cmptSelfHessian(init_self_hessian2, init_pix_jacobian, init_pix_hessian);
		}
	}
	if(params.enable_validation){
		am->cmptInitHessian(init_hessian, init_pix_jacobian);
		am->cmptCurrHessian(hessian, init_pix_jacobian);
		if(!validateHessians(init_self_hessian)){
			throw std::logic_error("Diagnostics:: First Order Hessian Mismatch Encountered");
		}
		am->cmptInitHessian(init_hessian, init_pix_jacobian, init_pix_hessian);
		am->cmptCurrHessian(hessian, init_pix_jacobian, init_pix_hessian);
		if(!validateHessians(init_self_hessian2)){
			throw std::logic_error("Diagnostics:: Second Order Hessian Mismatch Encountered");
		}
	}
	if(gen_flags[3]){
		am->initializeDistFeat();
		am->updateDistFeat(init_dist_vec.data());
	}
	init_img_cv = am->getCurrImg().clone();
	new (&init_img) EigImgT((EigPixT*)(init_img_cv.data), init_img_cv.rows, init_img_cv.cols);

	init_corners.create(2, 4, CV_64FC1);
	corners.copyTo(init_corners);

	if(params.show_corners){
		init_img_cv_uchar.create(init_img.rows(), init_img.cols(), CV_8UC3);
		init_img_cv.convertTo(init_img_cv_uchar, init_img_cv_uchar.type());
		drawCurrCorners(init_img_cv_uchar);
		cv::namedWindow(init_img_win_name);
		imshow(init_img_win_name, init_img_cv_uchar);

		curr_img_cv = am->getCurrImg();
		curr_img_cv_uchar.create(am->getImgHeight(), am->getImgWidth(), CV_8UC3);
		curr_img_cv.convertTo(curr_img_cv_uchar, curr_img_cv_uchar.type());
		cv::namedWindow(curr_img_win_name);
		imshow(curr_img_win_name, curr_img_cv_uchar);
		if(cv::waitKey(1) == 27){
			exit(0);
		}			
	}
	if(params.show_patches){
		init_patch = cv::Mat(am->getResY(), am->getResX(), CV_64FC1,
			const_cast<double*>(am->getInitPixVals().data()));
		init_patch_uchar.create(am->getResY(), am->getResX(), CV_8UC1);
		init_patch.convertTo(init_patch_uchar, init_patch_uchar.type());

		cv::namedWindow(init_patch_win_name);
		imshow(init_patch_win_name, init_patch_uchar);

		curr_patch = cv::Mat(am->getResY(), am->getResX(), CV_64FC1,
			const_cast<double*>(am->getCurrPixVals().data()));
		curr_patch_uchar.create(am->getResY(), am->getResX(), CV_8UC1);
		curr_patch.convertTo(curr_patch_uchar, curr_patch_uchar.type());

		cv::namedWindow(curr_patch_win_name);
		imshow(curr_patch_win_name, curr_patch_uchar);
		if(cv::waitKey(1) == 27){
			exit(0);
		}
	}
}


void Diagnostics::update(const cv::Mat &corners){
	//Matrix24d in_corners;
	//for(int i = 0; i < 4; i++){
	//	in_corners(0, i) = corners.at<double>(0, i);
	//	in_corners(1, i) = corners.at<double>(1, i);
	//}
	++frame_id;

	ssm->initialize(corners);
	am->setFirstIter();

	//Matrix2Xd pix_ssm_grad;
	//pix_ssm_grad.resize(Eigen::NoChange, ssm_state_size);
	//for(int pix_id = 0; pix_id < n_pix; pix_id++){
	//	ssm->getCurrPixGrad(pix_ssm_grad, pix_id);
	//	ssm_grad_norm.row(pix_id) = pix_ssm_grad.colwise().norm();
	//}
	//ssm_grad_norm_mean = ssm_grad_norm.colwise().mean();
	//utils::printMatrix(ssm_grad_norm_mean, "ssm_grad_norm_mean");

	//double basis_scale_factor = sqrt((ssm->getCorners().array() / ssm->getCorners().array()).matrix().squaredNorm() / 8);
	
	//utils::printScalar(basis_scale_factor, "basis_scale_factor");

	//ssm_grad_norm_mean *= basis_scale_factor;
	//utils::printMatrix(ssm_grad_norm_mean, "ssm_grad_norm_mean scaled");


	//VectorXd state_update(ssm->getStateSize());
	//ssm->estimateWarpFromCorners(state_update, ssm->getCorners(), in_corners);
	//ssm->setState(state_update);
	//Matrix24d out_corners = ssm->getCorners();

	//utils::printMatrix(in_corners, "in corners");
	//utils::printMatrix(ssm->getCorners(), "out corners");
}


void Diagnostics::generateSSMParamData(VectorXd &param_range_vec,
	int n_pts, const char* fname){
	assert(param_range_vec.size() == ssm_state_size);

	//printf("Computing %s ssm data for %d points with ",
	//	update_name, n_pts);
	//utils::printMatrix(param_range_vec, "parameter range");


	VectorXd min_state = -param_range_vec;
	VectorXd max_state = param_range_vec;

	//utils::printMatrix(min_state, "min_state");
	//utils::printMatrix(max_state, "max_state");

	VectorXd state_update(ssm_state_size), grad_update(ssm_state_size);
	diagnostics_data.resize(n_pts, 2 * ssm_state_size);

	double corner_change_norm;
	Matrix24d corner_change;
	for(int state_id = 0; state_id < ssm_state_size; state_id++){
		printf("Processing state parameter %d....\n", state_id);

		state_update.setZero();
		diagnostics_data.col(2 * state_id) = VectorXd::LinSpaced(n_pts, min_state(state_id), max_state(state_id));
		for(int pt_id = 0; pt_id < n_pts; pt_id++){
			state_update(state_id) = diagnostics_data(pt_id, 2 * state_id);

			updateSSM(state_update);
			corner_change = ssm->getCorners() - ssm->getCorners();
			//utils::printMatrix(ssm->getCorners(), "ssm->getCorners()");
			//utils::printMatrix(ssm->getCorners(), "ssm->getCorners()");
			//utils::printMatrix(corner_change, "corner_change");
			corner_change_norm = corner_change.lpNorm<1>();
			diagnostics_data(pt_id, 2 * state_id + 1) = corner_change_norm;
			resetSSM(state_update);
			if((pt_id + 1) % 50 == 0){ printf("Done %d points\n", pt_id + 1); }
		}
	}
	if(fname){
		printf("Writing diagnostics data to: %s\n", fname);
		utils::printMatrixToFile(diagnostics_data, "diagnostics_data", fname, "%15.9f", "w");
	}
}

const char*  Diagnostics::toString(ADT data_type){
	switch(data_type){
	case ADT::Norm:
		return "Norm";
	case ADT::Likelihood:
		return "Likelihood";
	case ADT::FeatNorm:
		return "FeatNorm";
	case ADT::StdJac:
		return "StdJac";
	case ADT::ESMJac:
		return "ESMJac";
	case ADT::DiffOfJacs:
		return "DiffOfJacs";
	case ADT::Std:
		return "Std";
	case ADT::ESM:
		return "ESM";
	case ADT::InitSelf:
		return "InitSelf";
	case ADT::CurrSelf:
		return "CurrSelf";
	case ADT::Std2:
		return "Std2";
	case ADT::ESM2:
		return "ESM2";
	case ADT::InitSelf2:
		return "InitSelf2";
	case ADT::CurrSelf2:
		return "CurrSelf2";
	case ADT::SumOfStd:
		return "SumOfStd";
	case ADT::SumOfStd2:
		return "SumOfStd2";
	case ADT::SumOfSelf:
		return "SumOfSelf";
	case ADT::SumOfSelf2:
		return "SumOfSelf2";
	default:
		printf("Data type: %d\n", data_type);
		throw std::invalid_argument("Diagnostics :: Invalid analytical  data type specified");
	}
}
const char*  Diagnostics::toString(NDT data_type){
	switch(data_type){
	case NDT::Jacobian:
		return "Jacobian";
	case NDT::Hessian:
		return "Hessian";
	case NDT::NHessian:
		return "NHessian";
	default:
		printf("Data type: %d\n", data_type);
		throw std::invalid_argument("Diagnostics :: Invalid numerical data type specified");
	}
}


void Diagnostics::drawCurrCorners(cv::Mat &img, int state_id, 
	int thickness, bool write_text, cv::Scalar corners_col){
	ssm->getCorners(curr_corners_cv);
	line(img, curr_corners_cv[0], curr_corners_cv[1], corners_col, thickness, CV_AA);
	line(img, curr_corners_cv[1], curr_corners_cv[2], corners_col, thickness, CV_AA);
	line(img, curr_corners_cv[2], curr_corners_cv[3], corners_col, thickness, CV_AA);
	line(img, curr_corners_cv[3], curr_corners_cv[0], corners_col, thickness, CV_AA);
	if(write_text && state_id >= 0){
		stringstream state_id_txt;
		state_id_txt << "state param: " << state_id;
		putText(img, state_id_txt.str(), curr_corners_cv[0],
			cv::FONT_HERSHEY_SIMPLEX, 0.5, corners_col);
	}

}

_MTF_END_NAMESPACE
