#include "mtf/SSM/Similitude.h"
#include "mtf/SSM/SSMEstimator.h"
#include "mtf/Utilities/warpUtils.h"
#include "mtf/Utilities/miscUtils.h"

_MTF_BEGIN_NAMESPACE

SimilitudeParams::SimilitudeParams(const SSMParams *ssm_params, 
bool _normalized_init, bool _geom_sampling,
int _pt_based_sampling, int _n_model_pts,
bool _debug_mode) :
SSMParams(ssm_params),
normalized_init(_normalized_init),
geom_sampling(_geom_sampling),
pt_based_sampling(_pt_based_sampling),
n_model_pts(_n_model_pts),
debug_mode(_debug_mode){}

SimilitudeParams::SimilitudeParams(const SimilitudeParams *params) :
SSMParams(params),
normalized_init(SIM_NORMALIZED_INIT),
geom_sampling(SIM_GEOM_SAMPLING),
pt_based_sampling(SIM_PT_BASED_SAMPLING),
n_model_pts(SIM_N_MODEL_PTS),
debug_mode(SIM_DEBUG_MODE){
	if(params){
		normalized_init = params->normalized_init;
		geom_sampling = params->geom_sampling;
		pt_based_sampling = params->pt_based_sampling;
		n_model_pts = params->n_model_pts;
		debug_mode = params->debug_mode;
	}
}

Similitude::Similitude(const ParamType *_params) :
ProjectiveBase(_params),
params(_params){

	printf("\n");
	printf("Using Similitude SSM with:\n");
	printf("resx: %d\n", resx);
	printf("resy: %d\n", resy);
	printf("normalized_init: %d\n", params.normalized_init);
	printf("geom_sampling: %d\n", params.geom_sampling);
	printf("pt_based_sampling: %d\n", params.pt_based_sampling);
	printf("n_model_pts: %d\n", params.n_model_pts);
	printf("debug_mode: %d\n", params.debug_mode);

	name = "similitude";
	state_size = 4;
	curr_state.resize(state_size);

	utils::getNormUnitSquarePts(norm_pts, norm_corners, resx, resy,
		1 - resx / 2.0, 1 - resy / 2.0, resx / 2.0, resy / 2.0);
	utils::homogenize(norm_pts, norm_pts_hm);
	utils::homogenize(norm_corners, norm_corners_hm);

	init_corners = getNormCorners();
	init_corners_hm = getHomNormCorners();
	init_pts = getNormPts();
	init_pts_hm = getHomNormPts();
}

void Similitude::setCorners(const CornersT& corners){
	if(params.normalized_init){
		curr_warp = utils::computeSimilitudeDLT(init_corners, corners);
		getStateFromWarp(curr_state, curr_warp);

		curr_pts.noalias() = curr_warp.topRows<2>() * init_pts_hm;
		curr_corners.noalias() = curr_warp.topRows<2>() * init_corners_hm;

		utils::homogenize(curr_pts, curr_pts_hm);
		utils::homogenize(curr_corners, curr_corners_hm);
	} else {
		curr_corners = corners;
		utils::homogenize(curr_corners, curr_corners_hm);

		getPtsFromCorners(curr_warp, curr_pts, curr_pts_hm, curr_corners);

		init_corners = curr_corners;
		init_pts = curr_pts;
		utils::homogenize(init_corners, init_corners_hm);
		utils::homogenize(init_pts, init_pts_hm);

		curr_warp = Matrix3d::Identity();
		curr_state.fill(0);
	}
}

void Similitude::setState(const VectorXd &ssm_state){
	validate_ssm_state(ssm_state);
	curr_state = ssm_state;
	getWarpFromState(curr_warp, curr_state);
	curr_pts.noalias() = curr_warp.topRows<2>() * init_pts_hm;
	curr_corners.noalias() = curr_warp.topRows<2>() * init_corners_hm;
}

void Similitude::compositionalUpdate(const VectorXd& state_update){
	validate_ssm_state(state_update);

	getWarpFromState(warp_update_mat, state_update);
	curr_warp = curr_warp * warp_update_mat;

	getStateFromWarp(curr_state, curr_warp);

	curr_pts.noalias() = curr_warp.topRows<2>() * init_pts_hm;
	curr_corners.noalias() = curr_warp.topRows<2>() * init_corners_hm;
}

void Similitude :: getWarpFromState(Matrix3d &warp_mat, 
	const VectorXd& ssm_state){
	validate_ssm_state(ssm_state);

	double tx = ssm_state(0);
	double ty = ssm_state(1);
	double a = ssm_state(2);
	double b = ssm_state(3);

	warp_mat(0,0) = 1 + a;
	warp_mat(0,1) = -b;
	warp_mat(0,2) = tx;
	warp_mat(1,0) = b;
	warp_mat(1, 1) = 1 + a;
	warp_mat(1,2) = ty;
	warp_mat(2,0) = 0;
	warp_mat(2,1) = 0;
	warp_mat(2,2) = 1;
}


void Similitude :: getStateFromWarp(VectorXd &state_vec, 
	const Matrix3d& sim_mat){
	validate_ssm_state(state_vec);
	validate_sim_warp(sim_mat);

	state_vec(0) = sim_mat(0, 2);
	state_vec(1) = sim_mat(1, 2);
	state_vec(2) = sim_mat(0, 0) - 1;
	state_vec(3) = sim_mat(1, 0);
}

void Similitude::getInitPixGrad(Matrix2Xd &dI_dp, int pt_id) {
	double x = init_pts(0, pt_id);
	double y = init_pts(1, pt_id);
	dI_dp <<
		1, 0, x, -y,
		0, 1, y, x;
}

void Similitude::cmptInitPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dx){
	validate_ssm_jacobian(dI_dp, dI_dx);

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; ++pt_id){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = dI_dx(ch_pt_id, 0);
			double Iy = dI_dx(ch_pt_id, 1);

			dI_dp(ch_pt_id, 0) = Ix;
			dI_dp(ch_pt_id, 1) = Iy;
			dI_dp(ch_pt_id, 2) = Ix*x + Iy*y;
			dI_dp(ch_pt_id, 3) = Iy*x - Ix*y;
			++ch_pt_id;
		}
	}
}

void Similitude::cmptWarpedPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dx){
	validate_ssm_jacobian(dI_dp, dI_dx);
	double a = curr_state(2) + 1, b = -curr_state(3);
	double c = curr_state(3), d = curr_state(2) + 1;

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; pt_id++){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = a*dI_dx(ch_pt_id, 0) + c*dI_dx(ch_pt_id, 1);
			double Iy = b*dI_dx(ch_pt_id, 0) + d*dI_dx(ch_pt_id, 1);

			dI_dp(ch_pt_id, 0) = Ix;
			dI_dp(ch_pt_id, 1) = Iy;
			dI_dp(ch_pt_id, 2) = Ix*x + Iy*y;
			dI_dp(ch_pt_id, 3) = Iy*x - Ix*y;
			++ch_pt_id;
		}
	}
}

void Similitude::cmptApproxPixJacobian(MatrixXd &dI_dp, const PixGradT &dI_dx) {
	validate_ssm_jacobian(dI_dp, dI_dx);
	double a_plus_1 = curr_state(2) + 1, b = curr_state(3);
	double inv_det = 1.0 / (a_plus_1*a_plus_1 + b*b);

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; pt_id++){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = dI_dx(ch_pt_id, 0);
			double Iy = dI_dx(ch_pt_id, 1);

			dI_dp(ch_pt_id, 0) = (Ix*a_plus_1 - Iy*b) * inv_det;
			dI_dp(ch_pt_id, 1) = (Ix*b + Iy*a_plus_1) * inv_det;
			dI_dp(ch_pt_id, 2) = (Ix*(x*a_plus_1 + y*b) + Iy*(y*a_plus_1 - x*b)) * inv_det;
			dI_dp(ch_pt_id, 3) = (Ix*(-y*a_plus_1 + x*b) + Iy*(x*a_plus_1 + y*b)) * inv_det;
			++ch_pt_id;
		}
	}
}

void Similitude::cmptInitPixHessian(MatrixXd &d2I_dp2, const PixHessT &d2I_dw2,
	const PixGradT &dI_dw){
	validate_ssm_hessian(d2I_dp2, d2I_dw2, dI_dw);
	
	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; pt_id++){
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		Matrix24d dw_dp;
		dw_dp <<
			1, 0, x, -y,
			0, 1, y, x;
		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			Map<Matrix4d>(d2I_dp2.col(ch_pt_id).data()) = dw_dp.transpose()*
				Map<const Matrix2d>(d2I_dw2.col(ch_pt_id).data())*dw_dp;
			++ch_pt_id;
		}
	}
}
void Similitude::cmptWarpedPixHessian(MatrixXd &d2I_dp2, const PixHessT &d2I_dw2,
	const PixGradT &dI_dw) {
	validate_ssm_hessian(d2I_dp2, d2I_dw2, dI_dw);
	double a2 = curr_state(2) + 1, a3 = curr_state(3);
	Matrix2d dw_dx;
	dw_dx <<
		a2, -a3,
		a3, a2;

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; ++pt_id) {
		spi_pt_check_mc(spi_mask, pt_id, ch_pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		Matrix24d dw_dp;
		dw_dp <<
			1, 0, x, -y,
			0, 1, y, x;

		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			Map<Matrix4d>(d2I_dp2.col(ch_pt_id).data()) = dw_dp.transpose()*
				dw_dx.transpose()*Map<const Matrix2d>(d2I_dw2.col(ch_pt_id).data())*dw_dx*dw_dp;
			++ch_pt_id;
		}
	}
}

void Similitude::estimateWarpFromCorners(VectorXd &state_update, const Matrix24d &in_corners,
	const Matrix24d &out_corners){
	validate_ssm_state(state_update);

	Matrix3d warp_update_mat = utils::computeSimilitudeDLT(in_corners, out_corners);
	getStateFromWarp(state_update, warp_update_mat);
}

void Similitude::estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
	const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
	const EstimatorParams &est_params){
	cv::Mat sim_params = estimateSimilitude(in_pts, out_pts, mask, est_params);
	state_update(0) = sim_params.at<double>(0, 1);
	state_update(1) = sim_params.at<double>(1, 1);
	state_update(2) = sim_params.at<double>(0, 0) - 1;
	state_update(3) = sim_params.at<double>(1, 0);
}


void Similitude::updateGradPts(double grad_eps){
	Vector2d diff_vec_x_warped = curr_warp.topRows<2>().col(0) * grad_eps;
	Vector2d diff_vec_y_warped = curr_warp.topRows<2>().col(1) * grad_eps;

	for(int pt_id = 0; pt_id < n_pts; pt_id++){
		spi_pt_check(spi_mask, pt_id);

		grad_pts(0, pt_id) = curr_pts(0, pt_id) + diff_vec_x_warped(0);
		grad_pts(1, pt_id) = curr_pts(1, pt_id) + diff_vec_x_warped(1);

		grad_pts(2, pt_id) = curr_pts(0, pt_id) - diff_vec_x_warped(0);
		grad_pts(3, pt_id) = curr_pts(1, pt_id) - diff_vec_x_warped(1);

		grad_pts(4, pt_id) = curr_pts(0, pt_id) + diff_vec_y_warped(0);
		grad_pts(5, pt_id) = curr_pts(1, pt_id) + diff_vec_y_warped(1);

		grad_pts(6, pt_id) = curr_pts(0, pt_id) - diff_vec_y_warped(0);
		grad_pts(7, pt_id) = curr_pts(1, pt_id) - diff_vec_y_warped(1);
	}
}


void Similitude::updateHessPts(double hess_eps){
	double hess_eps2 = 2 * hess_eps;

	Vector2d diff_vec_xx_warped = curr_warp.topRows<2>().col(0) * hess_eps2;
	Vector2d diff_vec_yy_warped = curr_warp.topRows<2>().col(1) * hess_eps2;
	Vector2d diff_vec_xy_warped = (curr_warp.topRows<2>().col(0) + curr_warp.topRows<2>().col(1)) * hess_eps;
	Vector2d diff_vec_yx_warped = (curr_warp.topRows<2>().col(0) - curr_warp.topRows<2>().col(1)) * hess_eps;

	for(int pt_id = 0; pt_id < n_pts; pt_id++){

		spi_pt_check(spi_mask, pt_id);

		hess_pts(0, pt_id) = curr_pts(0, pt_id) + diff_vec_xx_warped(0);
		hess_pts(1, pt_id) = curr_pts(1, pt_id) + diff_vec_xx_warped(1);

		hess_pts(2, pt_id) = curr_pts(0, pt_id) - diff_vec_xx_warped(0);
		hess_pts(3, pt_id) = curr_pts(1, pt_id) - diff_vec_xx_warped(1);

		hess_pts(4, pt_id) = curr_pts(0, pt_id) + diff_vec_yy_warped(0);
		hess_pts(5, pt_id) = curr_pts(1, pt_id) + diff_vec_yy_warped(1);

		hess_pts(6, pt_id) = curr_pts(0, pt_id) - diff_vec_yy_warped(0);
		hess_pts(7, pt_id) = curr_pts(1, pt_id) - diff_vec_yy_warped(1);

		hess_pts(8, pt_id) = curr_pts(0, pt_id) + diff_vec_xy_warped(0);
		hess_pts(9, pt_id) = curr_pts(1, pt_id) + diff_vec_xy_warped(1);

		hess_pts(10, pt_id) = curr_pts(0, pt_id) - diff_vec_xy_warped(0);
		hess_pts(11, pt_id) = curr_pts(1, pt_id) - diff_vec_xy_warped(1);

		hess_pts(12, pt_id) = curr_pts(0, pt_id) + diff_vec_yx_warped(0);
		hess_pts(13, pt_id) = curr_pts(1, pt_id) + diff_vec_yx_warped(1);

		hess_pts(14, pt_id) = curr_pts(0, pt_id) - diff_vec_yx_warped(0);
		hess_pts(15, pt_id) = curr_pts(1, pt_id) - diff_vec_yx_warped(1);
	}
}
void Similitude::applyWarpToCorners(Matrix24d &warped_corners, const Matrix24d &orig_corners,
	const VectorXd &ssm_state){
	getWarpFromState(warp_mat, ssm_state);
	for(int corner_id = 0; corner_id < 4; corner_id++){
		warped_corners(0, corner_id) = warp_mat(0, 0)*orig_corners(0, corner_id) + warp_mat(0, 1)*orig_corners(1, corner_id) +
			warp_mat(0, 2);
		warped_corners(1, corner_id) = warp_mat(1, 0)*orig_corners(0, corner_id) + warp_mat(1, 1)*orig_corners(1, corner_id) +
			warp_mat(1, 2);
	}
}

void Similitude::applyWarpToPts(Matrix2Xd &warped_pts, const Matrix2Xd &orig_pts,
	const VectorXd &ssm_state){
	getWarpFromState(warp_mat, ssm_state);
	int n_pts = orig_pts.cols();
	for(int pt_id = 0; pt_id < n_pts; pt_id++){
		warped_pts(0, pt_id) = warp_mat(0, 0)*orig_pts(0, pt_id) + warp_mat(0, 1)*orig_pts(1, pt_id) +
			warp_mat(0, 2);
		warped_pts(1, pt_id) = warp_mat(1, 0)*orig_pts(0, pt_id) + warp_mat(1, 1)*orig_pts(1, pt_id) +
			warp_mat(1, 2);
	}
}

Vector4d Similitude::geomToState(const Vector4d &geom){
	double s = geom[3], theta = geom[2];
	double cos_theta = cos(theta), sin_theta = sin(theta);
	Vector4d state;
	state[0] = geom[0];
	state[1] = geom[1];
	state[2] = (s + 1)*cos_theta - 1;
	state[3] = (s + 1)*sin_theta;
	return state;
}
Vector4d Similitude::stateToGeom(const Vector4d &state){
	double a = state[2], b = state[3];
	Vector4d geom;
	geom[0] = state[0];
	geom[1] = state[1];
	geom[2] = atan2(b, a + 1);
	geom[3] = sqrt((a + 1)*(a + 1) + b*b) - 1;
	return geom;
}

void Similitude::generatePerturbation(VectorXd &perturbation){
	assert(perturbation.size() == state_size);
	if(params.geom_sampling){
		//! perform geometric perturbation
		Vector4d geom_perturbation;
		for(int state_id = 0; state_id < 4; state_id++){
			geom_perturbation(state_id) = rand_dist[state_id](rand_gen[state_id]);
		}
		perturbation = geomToState(geom_perturbation);
	} else if(params.pt_based_sampling){
		PtsT orig_pts, perturbed_pts;
		orig_pts.resize(Eigen::NoChange, 2);
		perturbed_pts.resize(Eigen::NoChange, 2);
		orig_pts.col(0) = init_corners.col(0);
		orig_pts.col(1) = init_corners.col(2);
		if(params.pt_based_sampling == 1){
			perturbed_pts(0, 0) = orig_pts(0, 0) + rand_dist[0](rand_gen[0]);
			perturbed_pts(1, 0) = orig_pts(1, 0) + rand_dist[1](rand_gen[1]);
			perturbed_pts(0, 1) = orig_pts(0, 1) + rand_dist[2](rand_gen[2]);
			perturbed_pts(1, 1) = orig_pts(1, 1) + rand_dist[3](rand_gen[3]);
		} else {
			//! different perturbation for x,y coordinates of each corner
			//! followed by consistent translational perturbation to all corners
			perturbed_pts(0, 0) = orig_pts(0, 0) + rand_dist[1](rand_gen[1]);
			perturbed_pts(1, 0) = orig_pts(1, 0) + rand_dist[1](rand_gen[1]);
			perturbed_pts(0, 1) = orig_pts(0, 1) + rand_dist[1](rand_gen[1]);
			perturbed_pts(1, 1) = orig_pts(1, 1) + rand_dist[1](rand_gen[1]);
			perturbed_pts = perturbed_pts.colwise() + Vector2d(rand_dist[0](rand_gen[0]), rand_dist[0](rand_gen[0]));
		}
		Matrix3d sim_warp = utils::computeSimilitudeDLT(orig_pts, perturbed_pts);
		getStateFromWarp(perturbation, sim_warp);
	} else{
		ProjectiveBase::generatePerturbation(perturbation);
	}
}

// use Random Walk model to generate perturbed sample
void Similitude::additiveRandomWalk(VectorXd &perturbed_state,
	const VectorXd &base_state){
	if(params.geom_sampling){
		Vector4d geom_perturbation;
		for(int state_id = 0; state_id < 4; ++state_id){
			geom_perturbation(state_id) = rand_dist[state_id](rand_gen[state_id]);
		}
		Vector4d base_geom = stateToGeom(base_state);
		Vector4d perturbed_geom = base_geom + geom_perturbation;
		perturbed_state = geomToState(perturbed_geom);
	} else{
		ProjectiveBase::additiveRandomWalk(perturbed_state, base_state);
	}
}

// use first order Auto Regressive model to generate perturbed sample
void Similitude::additiveAutoRegression1(VectorXd &perturbed_state, VectorXd &perturbed_ar,
	const VectorXd &base_state, const VectorXd &base_ar, double a){
	if(params.geom_sampling){
		Vector4d geom_perturbation;
		for(int state_id = 0; state_id < 4; ++state_id){
			geom_perturbation(state_id) = rand_dist[state_id](rand_gen[state_id]);
		}
		Vector4d base_geom = stateToGeom(base_state);
		Vector4d base_ar_geom = stateToGeom(base_ar);
		Vector4d perturbed_geom = base_geom + base_ar_geom + geom_perturbation;
		Vector4d perturbed_ar_geom = a*(perturbed_geom - base_geom);
		perturbed_state = geomToState(perturbed_geom);
		perturbed_ar = geomToState(perturbed_ar_geom);
	} else{
		ProjectiveBase::additiveAutoRegression1(perturbed_state, perturbed_ar, 
			base_state, base_ar, a);
	}
}
void Similitude::compositionalRandomWalk(VectorXd &perturbed_state,
	const VectorXd &base_state){
	if(params.geom_sampling){
		throw mtf::utils::FunctonNotImplemented("Similitude::compositionalRandomWalk :: geometric sampling is not implemented yet");
	} else{
		ProjectiveBase::compositionalRandomWalk(perturbed_state, base_state);
	}
}


SimilitudeEstimator::SimilitudeEstimator(int _modelPoints, bool _use_boost_rng)
	: SSMEstimator(_modelPoints, cvSize(2, 2), 1, _use_boost_rng) {
	assert(_modelPoints >= 2);
	checkPartialSubsets = false;
}

int SimilitudeEstimator::runKernel(const CvMat* m1, const CvMat* m2, CvMat* H) {
	int n_pts = m1->rows * m1->cols;

	//if(n_pts != 3) {
	//    throw invalid_argument(cv::format("Invalid no. of points: %d provided", n_pts));
	//}
	const CvPoint2D64f* M = (const CvPoint2D64f*)m1->data.ptr;
	const CvPoint2D64f* m = (const CvPoint2D64f*)m2->data.ptr;

	Matrix2Xd in_pts, out_pts;
	in_pts.resize(Eigen::NoChange, n_pts);
	out_pts.resize(Eigen::NoChange, n_pts);
	for(int pt_id = 0; pt_id < n_pts; pt_id++) {
		in_pts(0, pt_id) = M[pt_id].x;
		in_pts(1, pt_id) = M[pt_id].y;

		out_pts(0, pt_id) = m[pt_id].x;
		out_pts(1, pt_id) = m[pt_id].y;
	}
	Matrix3d sim_mat = utils::computeSimilitudeDLT(in_pts, out_pts);

	double *H_ptr = H->data.db;
	H_ptr[0] = sim_mat(0, 0);
	H_ptr[1] = sim_mat(0, 2);
	H_ptr[2] = sim_mat(1, 0);
	H_ptr[3] = sim_mat(1, 2);
	return 1;
}


void SimilitudeEstimator::computeReprojError(const CvMat* m1, const CvMat* m2,
	const CvMat* model, CvMat* _err) {
	int n_pts = m1->rows * m1->cols;
	const CvPoint2D64f* M = (const CvPoint2D64f*)m1->data.ptr;
	const CvPoint2D64f* m = (const CvPoint2D64f*)m2->data.ptr;
	const double* H = model->data.db;
	float* err = _err->data.fl;

	for(int pt_id = 0; pt_id < n_pts; pt_id++) {
		double dx = (H[0] * M[pt_id].x - H[2] * M[pt_id].y + H[1]) - m[pt_id].x;
		double dy = (H[2] * M[pt_id].x + H[0] * M[pt_id].y + H[3]) - m[pt_id].y;
		err[pt_id] = (float)(dx * dx + dy * dy);
	}
}

bool SimilitudeEstimator::refine(const CvMat* m1, const CvMat* m2,
	CvMat* model, int maxIters) {
	LevMarq solver(4, 0, cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, maxIters, DBL_EPSILON));
	int n_pts = m1->rows * m1->cols;
	const CvPoint2D64f* M = (const CvPoint2D64f*)m1->data.ptr;
	const CvPoint2D64f* m = (const CvPoint2D64f*)m2->data.ptr;
	CvMat modelPart = cvMat(solver.param->rows, solver.param->cols, model->type, model->data.ptr);
	cvCopy(&modelPart, solver.param);

	for(;;)	{
		const CvMat* _param = 0;
		CvMat *_JtJ = 0, *_JtErr = 0;
		double* _errNorm = 0;

		if(!solver.updateAlt(_param, _JtJ, _JtErr, _errNorm))
			break;

		for(int pt_id = 0; pt_id < n_pts; pt_id++)	{
			const double* h = _param->data.db;
			double Mx = M[pt_id].x, My = M[pt_id].y;
			double _xi = (h[0] * Mx - h[2] * My + h[1]);
			double _yi = (h[2] * Mx + h[0] * My + h[3]);
			double err[] = { _xi - m[pt_id].x, _yi - m[pt_id].y };
			if(_JtJ || _JtErr) {
				double J[][4] = {
					{ Mx, 1, -My, 0 },
					{ My, 0, Mx, 1 }
				};
				for(int j = 0; j < 4; j++) {
					for(int k = j; k < 4; k++)
						_JtJ->data.db[j * 4 + k] += J[0][j] * J[0][k] + J[1][j] * J[1][k];
					_JtErr->data.db[j] += J[0][j] * err[0] + J[1][j] * err[1];
				}
			}
			if(_errNorm)
				*_errNorm += err[0] * err[0] + err[1] * err[1];
		}
	}

	cvCopy(solver.param, &modelPart);
	return true;
}

_MTF_END_NAMESPACE

