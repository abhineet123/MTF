#include "mtf/SSM/Spline.h"
#include "mtf/Utilities/warpUtils.h"
#include "mtf/Utilities/miscUtils.h"


_MTF_BEGIN_NAMESPACE

SplineParams::SplineParams(const SSMParams *ssm_params,
int _control_size_x, int _control_size_y,
double _control_overlap, InterpolationType _interp_type,
bool _static_wts, bool _debug_mode) :
SSMParams(ssm_params),
control_size_x(_control_size_x),
control_size_y(_control_size_y),
control_overlap(_control_overlap),
interp_type(_interp_type),
static_wts(_static_wts),
debug_mode(_debug_mode){}

SplineParams::SplineParams(const SplineParams *params) :
SSMParams(params),
control_size_x(SPLINE_CONTROL_SIZE),
control_size_y(SPLINE_CONTROL_SIZE),
control_overlap(SPLINE_CONTROL_OVERLAP),
interp_type(SPLINE_INTERP_TYPE),
static_wts(SPLINE_STATIC_WTS),
debug_mode(SPLINE_DEBUG_MODE){
	if(params){
		control_size_x = params->control_size_x;
		control_size_y = params->control_size_y;
		control_overlap = params->control_overlap;
		interp_type = params->interp_type;
		static_wts = params->static_wts;
		debug_mode = params->debug_mode;
	}
}
const char*  SplineParams::toString(InterpolationType interp_type){
	switch(interp_type){
	case InterpolationType::Bilinear:
		return "Bilinear";
	case InterpolationType::Biquadratic:
		return "Biquadratic";
	case InterpolationType::Bicubic:
		return "Bicubic";
	default:
		throw utils::InvalidArgument(
			cv::format("SplineParams :: Invalid interpolation type specified: %d", interp_type)
			);
	}
}
Spline::Spline(
	const ParamType *spl_params) : StateSpaceModel(spl_params),
	params(spl_params){
	printf("\n");
	printf("Using Spline based SSM with:\n");
	printf("resx: %d\n", resx);
	printf("resy: %d\n", resy);
	printf("control_size: %dx%d\n", params.control_size_x, params.control_size_y);
	printf("control_overlap: %f\n", params.control_overlap);
	printf("interp_type: %s\n", SplineParams::toString(params.interp_type));
	printf("static_wts: %d\n", params.static_wts);
	printf("debug_mode: %d\n", params.debug_mode);

	name = "spline";

	int mod_x = resx % params.control_size_x;
	int mod_y = resy % params.control_size_y;
	if(mod_x != 0 || mod_y != 0){
		throw utils::InvalidArgument(
			cv_format("Sampling resolution %dx%d is not divided evenly by the control patch size %dx%d\n",
			resx, resy, params.control_size_x, params.control_size_y));
	}
	control_res_x = resx / params.control_size_x;
	control_res_y = resy / params.control_size_y;

	printf("Spline control grid resolution: %dx%d\n", control_res_x, control_res_y);
	max_dist_x = (params.control_size_x - 1) / 2.0;
	max_dist_y = (params.control_size_y - 1) / 2.0;


	// location of spline control points - each point is located at the center of its patch;
	ctrl_idx = VectorXd::LinSpaced(control_res_x, max_dist_x, resx - 1 - max_dist_x);
	ctrl_idy = VectorXd::LinSpaced(control_res_y, max_dist_y, resy - 1 - max_dist_y);

	utils::printMatrix(ctrl_idx, "ctrl_idx");
	utils::printMatrix(ctrl_idy, "ctrl_idy");

	n_control_pts = control_res_x*control_res_y;
	state_size = 2 * n_control_pts;
	printf("state_size: %d\n", state_size);
	n_bounding_pts = 2 * (resx + resy - 2);

	dist_norm_x.resize(n_control_pts);
	dist_norm_y.resize(n_control_pts);

	norm_dist_x.resize(n_pts, n_control_pts);
	norm_dist_y.resize(n_pts, n_control_pts);
	interp_wts.resize(n_pts, n_control_pts);

	curr_state.resize(state_size);
	norm_pts.resize(Eigen::NoChange, n_pts);
	init_control_pts.resize(Eigen::NoChange, n_control_pts);
	curr_control_pts.resize(Eigen::NoChange, n_control_pts);
	ssm_grad.resize(Eigen::NoChange, state_size);
	utils::getNormUnitSquarePts(norm_pts, norm_corners, resx, resy);

}
void Spline::setCorners(const CornersT& corners){
	curr_corners = corners;
	utils::getPtsFromCorners(curr_pts, corners, norm_pts, norm_corners);

	for(unsigned int pt_id = 0; pt_id < n_control_pts; ++pt_id){
		unsigned int pt_id_x = pt_id % control_res_x;
		unsigned int pt_id_y = pt_id / control_res_x;

		int ctrl_idx_floor = static_cast<int>(floor(ctrl_idx(pt_id_x)));
		double ctrl_idx_frac = ctrl_idx(pt_id_x) - ctrl_idx_floor;
		int ctrl_idy_floor = static_cast<int>(floor(ctrl_idy(pt_id_y)));
		double ctrl_idy_frac = ctrl_idy(pt_id_y) - ctrl_idy_floor;
		curr_control_pts(0, pt_id) =
			curr_pts(0, ctrl_idx_floor) * (1 - ctrl_idx_frac) +
			curr_pts(0, ctrl_idx_floor + 1) * ctrl_idx_frac;
		curr_control_pts(1, pt_id) =
			curr_pts(1, ctrl_idx_floor) * (1 - ctrl_idy_frac) +
			curr_pts(1, ctrl_idx_floor + 1) * ctrl_idy_frac;
	}
	init_corners = curr_corners;
	init_control_pts = curr_control_pts;
	init_pts = curr_pts;
	curr_state.fill(0);
	initInterpolationWeights();

	if(params.debug_mode){
		utils::printMatrix(curr_pts, "curr_pts");
		utils::printMatrix(curr_control_pts, "curr_control_pts");
		utils::printMatrix(dist_norm_x, "dist_norm_x");
		utils::printMatrix(dist_norm_y, "dist_norm_y");
		utils::printMatrix(norm_dist_x, "norm_dist_x");
		utils::printMatrix(norm_dist_y, "norm_dist_y");
		utils::printMatrix(interp_wts, "interp_wts");
	}

}

void Spline::getCorners(cv::Mat &cv_corners){
	cv_corners.create(2, n_bounding_pts, CV_64FC1);
	utils::getBoundingPts(cv_corners, curr_pts, resx, resy);
	for(unsigned int corner_id = 0; corner_id < 4; ++corner_id){
		cv_corners.at<double>(0, corner_id) = curr_corners(0, corner_id);
		cv_corners.at<double>(1, corner_id) = curr_corners(1, corner_id);
	}
}

void Spline::compositionalUpdate(const VectorXd& state_update){
	validate_ssm_state(state_update);
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
		double pt_disp_x = 0, pt_disp_y = 0;
		int state_id = -1;
		for(unsigned int ctrl_pt_id = 0; ctrl_pt_id < n_control_pts; ++ctrl_pt_id){
			pt_disp_x += interp_wts(pt_id, ctrl_pt_id)*state_update(++state_id);
			pt_disp_y += interp_wts(pt_id, ctrl_pt_id)*state_update(++state_id);
		}
		curr_pts(0, pt_id) += pt_disp_x;
		curr_pts(1, pt_id) += pt_disp_y;
	}
	curr_state += state_update;
	int state_id = -1;
	for(unsigned int ctrl_pt_id = 0; ctrl_pt_id < n_control_pts; ++ctrl_pt_id){
		curr_control_pts(0, ctrl_pt_id) += state_update(++state_id);
		curr_control_pts(1, ctrl_pt_id) += state_update(++state_id);
	}
	if(!params.static_wts){
		updateInterpolationWeights();
	}
	curr_corners.col(0) = curr_pts.col(0);
	curr_corners.col(1) = curr_pts.col(resx - 1);
	curr_corners.col(2) = curr_pts.col(n_pts - 1);
	curr_corners.col(3) = curr_pts.col(n_pts - resx);

}
void Spline::initInterpolationWeights(){
	for(unsigned int ctrl_pt_id = 0; ctrl_pt_id < n_control_pts; ++ctrl_pt_id){
		unsigned int ctrl_pt_id_x = ctrl_pt_id % control_res_x;
		unsigned int ctrl_pt_id_y = ctrl_pt_id / control_res_x;
		int max_pt_id_x = ctrl_pt_id_x*params.control_size_x;
		int max_pt_id_y = ctrl_pt_id_y*params.control_size_y;
		dist_norm_x(ctrl_pt_id) = abs(max_pt_id_x - ctrl_idx(ctrl_pt_id_x)) + params.control_overlap;
		dist_norm_y(ctrl_pt_id) = abs(max_pt_id_y - ctrl_idy(ctrl_pt_id_y)) + params.control_overlap;
		for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
			int pt_id_x = pt_id % resx;
			int pt_id_y = pt_id / resx;
			norm_dist_x(pt_id, ctrl_pt_id) = (pt_id_x - ctrl_idx(ctrl_pt_id_x)) / dist_norm_x(ctrl_pt_id);
			norm_dist_y(pt_id, ctrl_pt_id) = (pt_id_y - ctrl_idy(ctrl_pt_id_y)) / dist_norm_y(ctrl_pt_id);
			interp_wts(pt_id, ctrl_pt_id) = getWeight(norm_dist_x(pt_id, ctrl_pt_id), norm_dist_y(pt_id, ctrl_pt_id));
		}
	}
	interp_wts = (interp_wts.array().colwise() / interp_wts.rowwise().sum().array()).matrix();
}

void Spline::updateInterpolationWeights(){
	for(unsigned int ctrl_pt_id = 0; ctrl_pt_id < n_control_pts; ++ctrl_pt_id){
		unsigned int ctrl_pt_id_x = ctrl_pt_id % control_res_x;
		unsigned int ctrl_pt_id_y = ctrl_pt_id / control_res_x;
		int max_pt_id_x = static_cast<int>(ctrl_idx(ctrl_pt_id_x) - max_dist_x);
		int max_pt_id_y = static_cast<int>(ctrl_idy(ctrl_pt_id_y) - max_dist_y);
		int max_pt_id = max_pt_id_y*resx + max_pt_id_x;
		dist_norm_x(ctrl_pt_id) = abs(curr_pts(0, max_pt_id) - curr_control_pts(0, ctrl_pt_id)) + params.control_overlap;
		dist_norm_y(ctrl_pt_id) = abs(curr_pts(1, max_pt_id) - curr_control_pts(1, ctrl_pt_id)) + params.control_overlap;
		for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
			norm_dist_x(pt_id, ctrl_pt_id) = (curr_pts(0, pt_id) - curr_control_pts(0, ctrl_pt_id)) / dist_norm_x(ctrl_pt_id);
			norm_dist_y(pt_id, ctrl_pt_id) = (curr_pts(1, pt_id) - curr_control_pts(1, ctrl_pt_id)) / dist_norm_y(ctrl_pt_id);
			interp_wts(pt_id, ctrl_pt_id) = getWeight(norm_dist_x(pt_id, ctrl_pt_id), norm_dist_y(pt_id, ctrl_pt_id));
		}
	}
	interp_wts = (interp_wts.array().colwise() / interp_wts.rowwise().sum().array()).matrix();
}

double Spline::getWeight(double x, double y){
	switch(params.interp_type){
	case InterpolationType::Bilinear:
		if((x < -1 || x > 1) || (y < -1 || y > 1)){
			return 0;
		}
		return (1 - abs(x))*(1 - abs(y));
	case InterpolationType::Biquadratic:
		throw utils::FunctonNotImplemented("Spline::Biquadratic interpolation is not implemented yet");
	case InterpolationType::Bicubic:
		throw utils::FunctonNotImplemented("Spline::Bicubic interpolation is not implemented yet");
	default:
		throw utils::InvalidArgument(
			cv::format("Spline::getWeight :: Invalid interpolation type specified: %d", params.interp_type));
	}
}

void Spline::setState(const VectorXd& state){
	curr_state = state;
	int state_id = -1;
	for(unsigned int ctrl_pt_id = 0; ctrl_pt_id < n_control_pts; ++ctrl_pt_id){
		curr_control_pts(0, ctrl_pt_id) = init_control_pts(0, ctrl_pt_id) + curr_state(++state_id);
		curr_control_pts(1, ctrl_pt_id) = init_control_pts(1, ctrl_pt_id) + curr_state(++state_id);
	}
}
void Spline::invertState(VectorXd& inv_state, const VectorXd& state){
	inv_state = -state;
}

void Spline::cmptInitPixJacobian(MatrixXd &jacobian_prod,
	const PixGradT &pix_jacobian){
	validate_ssm_jacobian(jacobian_prod, pix_jacobian);

	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
		getInitPixGrad(ssm_grad, pt_id);
		jacobian_prod.row(pt_id) = pix_jacobian.row(pt_id)*ssm_grad;
	}
}

void Spline::cmptPixJacobian(MatrixXd &jacobian_prod,
	const PixGradT &pix_jacobian){
	validate_ssm_jacobian(jacobian_prod, pix_jacobian);

	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
		getCurrPixGrad(ssm_grad, pt_id);
		jacobian_prod.row(pt_id) = pix_jacobian.row(pt_id)*ssm_grad;
	}
	//jacobian_prod.array().colwise() /= curr_pts_hm.array().row(2).transpose();
}

void Spline::cmptWarpedPixJacobian(MatrixXd &jacobian_prod,
	const PixGradT &pix_grad) {
	validate_ssm_jacobian(jacobian_prod, pix_grad);
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id) {

	}
}

void Spline::cmptWarpedPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
	const PixGradT &pix_grad) {
	validate_ssm_hessian(pix_hess_ssm, pix_hess_coord, pix_grad);

	Matrix28d ssm_jacobian;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id) {
	}
}

void Spline::getInitPixGrad(Matrix2Xd &ssm_grad, int pt_id) {
	getCurrPixGrad(ssm_grad, pt_id);
}

void Spline::getCurrPixGrad(Matrix2Xd &_ssm_grad, int pt_id) {
	_ssm_grad.setZero();
	int state_id = -1;
	for(unsigned int ctrl_pt_id = 0; ctrl_pt_id < n_control_pts; ++ctrl_pt_id){
		_ssm_grad(0, ++state_id) += interp_wts(pt_id, ctrl_pt_id);
		_ssm_grad(1, ++state_id) += interp_wts(pt_id, ctrl_pt_id);
	}

}

void Spline::cmptApproxPixJacobian(MatrixXd &jacobian_prod,
	const PixGradT &pix_grad) {
	validate_ssm_jacobian(jacobian_prod, pix_grad);

}

void Spline::cmptApproxPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
	const PixGradT &pix_grad) {
	validate_ssm_hessian(pix_hess_ssm, pix_hess_coord, pix_grad);

	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id) {

	}
}

void Spline::cmptInitPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
	const PixGradT &pix_grad){
	validate_ssm_hessian(pix_hess_ssm, pix_hess_coord, pix_grad);
	Matrix28d ssm_jacobian;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){

	}
}

void Spline::cmptPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
	const PixGradT &pix_grad){
	validate_ssm_hessian(pix_hess_ssm, pix_hess_coord, pix_grad);

	Matrix28d ssm_jacobian;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){

	}
}

void Spline::updateGradPts(double grad_eps){

	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
		grad_pts(0, pt_id) = curr_pts(0, pt_id) + grad_eps;
		grad_pts(1, pt_id) = curr_pts(1, pt_id);

		grad_pts(2, pt_id) = curr_pts(0, pt_id) - grad_eps;
		grad_pts(3, pt_id) = curr_pts(1, pt_id);

		grad_pts(4, pt_id) = curr_pts(0, pt_id);
		grad_pts(5, pt_id) = curr_pts(1, pt_id) + grad_eps;

		grad_pts(6, pt_id) = curr_pts(0, pt_id);
		grad_pts(7, pt_id) = curr_pts(1, pt_id) - grad_eps;
	}
}

void Spline::updateHessPts(double hess_eps){
	double hess_eps2 = 2 * hess_eps;
	for(unsigned int pt_id = 0; pt_id < n_pts; ++pt_id){
	}
}

void Spline::generatePerturbation(VectorXd &state_update){
}


_MTF_END_NAMESPACE

