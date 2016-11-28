#include "mtf/SSM/SL3.h"
#include "mtf/SSM/SSMEstimator.h"
#include "mtf/Utilities/warpUtils.h"
#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/excpUtils.h"

#include <unsupported/Eigen/MatrixFunctions>
#include "opencv2/calib3d/calib3d.hpp"
#include <boost/random/random_device.hpp>
#include <boost/random/seed_seq.hpp>

#define MAX_VALID_VAL 1e50
#define is_unbounded(eig_mat) (eig_mat.array().cwiseAbs().eval() > MAX_VALID_VAL).any()

_MTF_BEGIN_NAMESPACE

SL3Params::SL3Params(const SSMParams *ssm_params,
bool _normalized_init, bool _iterative_sample_mean,
int _sample_mean_max_iters, double _sample_mean_eps,
bool _debug_mode) :
SSMParams(ssm_params),
normalized_init(_normalized_init),
iterative_sample_mean(_iterative_sample_mean),
sample_mean_max_iters(_sample_mean_max_iters),
sample_mean_eps(_sample_mean_eps),
debug_mode(_debug_mode){}

//! copy/default constructor
SL3Params::SL3Params(const SL3Params *params) :
SSMParams(params),
normalized_init(SL3_NORMALIZED_BASIS),
iterative_sample_mean(SL3_ITERATIVE_SAMPLE_MEAN),
sample_mean_max_iters(SL3_SAMPLE_MEAN_MAX_ITERS),
sample_mean_eps(SL3_SAMPLE_MEAN_EPS),
debug_mode(SL3_DEBUG_MODE){
	if(params){
		normalized_init = params->normalized_init;
		iterative_sample_mean = params->iterative_sample_mean;
		sample_mean_max_iters = params->sample_mean_max_iters;
		sample_mean_eps = params->sample_mean_eps;
		debug_mode = params->debug_mode;
	}
}

SL3::SL3(
	const ParamType *_params) :
	ProjectiveBase(_params), params(_params){

	printf("\n");
	printf("Using SL3 SSM with:\n");
	printf("resx: %d\n", resx);
	printf("resy: %d\n", resy);
	printf("normalized_init: %d\n", params.normalized_init);
	printf("iterative_sample_mean: %d\n", params.iterative_sample_mean);
	printf("sample_mean_max_iters: %d\n", params.sample_mean_max_iters);
	printf("sample_mean_eps: %f\n", params.sample_mean_eps);
	printf("debug_mode: %d\n", params.debug_mode);


	name = "sl3";
	state_size = 8;
	curr_state.resize(state_size);

	lie_alg_mat = Matrix3d::Zero();
	warp_mat = Matrix3d::Identity();

	lieAlgBasis[0] <<
		1, 0, 0,
		0, -1, 0,
		0, 0, 0;
	lieAlgBasis[1] <<
		0, 0, 0,
		0, -1, 0,
		0, 0, 1;
	lieAlgBasis[2] <<
		0, -1, 0,
		1, 0, 0,
		0, 0, 0;
	lieAlgBasis[3] <<
		0, 1, 0,
		1, 0, 0,
		0, 0, 0;
	lieAlgBasis[4] <<
		0, 0, 1,
		0, 0, 0,
		0, 0, 0;
	lieAlgBasis[5] <<
		0, 0, 0,
		0, 0, 1,
		0, 0, 0;
	lieAlgBasis[6] <<
		0, 0, 0,
		0, 0, 0,
		1, 0, 0;
	lieAlgBasis[7] <<
		0, 0, 0,
		0, 0, 0,
		0, 1, 0;

	utils::getNormUnitSquarePts(norm_pts, norm_corners, resx, resy,
		-static_cast<double>(resx) / 2.0, -static_cast<double>(resy) / 2.0,
		static_cast<double>(resx) / 2.0, static_cast<double>(resy) / 2.0);
	utils::homogenize(norm_corners, norm_corners_hm);
	utils::homogenize(norm_pts, norm_pts_hm);

	init_corners = norm_corners;
	init_corners_hm = norm_corners_hm;
	init_pts = norm_pts;
	init_pts_hm = norm_pts_hm;

	if(params.debug_mode){
		fclose(fopen("log/sl3.txt", "w"));
	}
}

void SL3::setState(const VectorXd &ssm_state){
	validate_ssm_state(ssm_state);
	//utils::printMatrix(ssm_state.transpose(), " SL3::setState :: ssm_state");
	if(!ssm_state.allFinite()){
		utils::printMatrix(ssm_state.transpose(), "ssm_state");
		throw mtf::utils::InvalidTrackerState("SL3::setState::Invalid state provided");
	}
	curr_state = ssm_state;
	getWarpFromState(curr_warp, curr_state);
	curr_pts_hm.noalias() = curr_warp * init_pts_hm;
	curr_corners_hm.noalias() = curr_warp * init_corners_hm;
	utils::dehomogenize(curr_pts_hm, curr_pts);
	utils::dehomogenize(curr_corners_hm, curr_corners);
	if(params.debug_mode){
		utils::printMatrixToFile(curr_warp, "setState::curr_warp", "log/sl3.txt");
		utils::printMatrixToFile(curr_corners, "setState::curr_corners", "log/sl3.txt");
	}
}

void SL3::setCorners(const CornersT& corners){
	if(!corners.allFinite() || is_unbounded(corners)){
		utils::printMatrix(corners, "corners");
		throw mtf::utils::InvalidTrackerState("SL3::setCorners::Invalid corners provided");
	}
	curr_corners = corners;
	curr_warp = utils::computeHomographyDLT(norm_corners, curr_corners);
	double warp_det = curr_warp.determinant();
	if(!warp_mat.allFinite() || is_unbounded(warp_mat) || warp_det == 0 || std::isnan(warp_det) || std::isinf(warp_det)){
		printf("SL3::Cannot set SSM to the provided corners as the corresponding warp matrix is invalid: \n");
		utils::printMatrix(corners, "corners");
		utils::printMatrix(curr_warp, "warp");
		utils::printScalar(warp_det, "warp_det");
		throw mtf::utils::InvalidTrackerState("SL3::setCorners :: Cannot set SSM to the provided points as the corresponding warp matrix is invalid");
	}
	utils::homogenize(curr_corners, curr_corners_hm);
	curr_pts_hm = curr_warp * norm_pts_hm;
	utils::dehomogenize(curr_pts_hm, curr_pts);
	curr_warp = curr_warp / cbrt(warp_det);
	if(params.normalized_init){
		getStateFromWarp(curr_state, curr_warp);
	} else{
		init_corners = curr_corners;
		init_corners_hm = curr_corners_hm;
		init_pts = curr_pts;
		init_pts_hm = curr_pts_hm;
		curr_warp = Matrix3d::Identity();
		curr_state.fill(0);
	}
	if(params.debug_mode){
		utils::printMatrixToFile(init_pts, "setCorners::init_pts", "log/sl3.txt");
		utils::printMatrixToFile(init_corners, "setCorners::init_corners", "log/sl3.txt");
		utils::printMatrixToFile(curr_warp, "setCorners::curr_warp", "log/sl3.txt");
	}
}

void SL3::compositionalUpdate(const VectorXd& state_update){
	validate_ssm_state(state_update);

	getWarpFromState(warp_update_mat, state_update);
	curr_warp = curr_warp * warp_update_mat;
	if(!curr_warp.allFinite() || is_unbounded(curr_warp)){
		utils::printMatrix(state_update.transpose(), "state_update");
		utils::printMatrix(curr_warp, "curr_warp");
		utils::printMatrix(warp_update_mat, "warp_update_mat");
		throw mtf::utils::InvalidTrackerState("SL3::compositionalUpdate::Invalid state update provided");
		
	}
	curr_pts_hm.noalias() = curr_warp * init_pts_hm;
	curr_corners_hm.noalias() = curr_warp * init_corners_hm;

	utils::dehomogenize(curr_pts_hm, curr_pts);
	utils::dehomogenize(curr_corners_hm, curr_corners);

	getStateFromWarp(curr_state, curr_warp);

}

void SL3::getWarpFromState(Matrix3d &warp_mat,
	const VectorXd& ssm_state){
	validate_ssm_state(ssm_state);
	assert(ssm_state.size() == state_size);
	if(!ssm_state.allFinite() || is_unbounded(ssm_state)){
		utils::printMatrix(ssm_state.transpose(), "ssm_state");
		throw mtf::utils::InvalidTrackerState("SL3::getWarpFromState::Invalid state provided");
	}
	getLieAlgMatFromState(lie_alg_mat, ssm_state);
	if(!lie_alg_mat.allFinite() || is_unbounded(lie_alg_mat)){
		utils::printMatrix(ssm_state.transpose(), "ssm_state");
		utils::printMatrix(lie_alg_mat, "lie_alg_mat");
		throw mtf::utils::InvalidTrackerState("SL3::getWarpFromState::Invalid sl3 matrix corresponds to the given state");
	}
	warp_mat = lie_alg_mat.exp();
}

void SL3::getStateFromWarp(VectorXd &state_vec,
	const Matrix3d& warp_mat){
	validate_ssm_state(state_vec);
	double warp_det = warp_mat.determinant();
	if(!warp_mat.allFinite() || is_unbounded(warp_mat) || warp_det == 0 || std::isnan(warp_det) || std::isinf(warp_det)){
		utils::printMatrix(warp_mat, "warp_mat");
		utils::printScalar(warp_det, "warp_det");
		throw mtf::utils::InvalidTrackerState("SL3::getStateFromWarp :: Invalid warp matrix provided");
	}
	Matrix3d norm_warp_mat = warp_mat / cbrt(warp_det);
	if(!norm_warp_mat.allFinite() || is_unbounded(norm_warp_mat)){
		utils::printMatrix(warp_mat, "warp_mat");
		utils::printScalar(warp_det, "warp_det");
		utils::printMatrix(norm_warp_mat, "norm_warp_mat");
		throw mtf::utils::InvalidTrackerState("SL3::getStateFromWarp :: Invalid normalized warp matrix found");
	}
	lie_alg_mat = norm_warp_mat.log();
	getStateFromLieAlgMat(state_vec, lie_alg_mat);
}

void SL3::getLieAlgMatFromState(Matrix3d& lie_alg_mat,
	const VectorXd& ssm_state){
	validate_ssm_state(ssm_state);
	assert(ssm_state.size() == state_size);
	lie_alg_mat(0, 0) = ssm_state(0);
	lie_alg_mat(0, 1) = ssm_state(3) - ssm_state(2);
	lie_alg_mat(0, 2) = ssm_state(4);
	lie_alg_mat(1, 0) = ssm_state(3) + ssm_state(2);
	lie_alg_mat(1, 1) = -ssm_state(1) - ssm_state(0);
	lie_alg_mat(1, 2) = ssm_state(5);
	lie_alg_mat(2, 0) = ssm_state(6);
	lie_alg_mat(2, 1) = ssm_state(7);
	lie_alg_mat(2, 2) = ssm_state(1);
}

void SL3::getStateFromLieAlgMat(VectorXd &ssm_state,
	const Matrix3d& lie_alg_mat){
	validate_ssm_state(ssm_state);
	ssm_state(0) = lie_alg_mat(0, 0);
	ssm_state(1) = -lie_alg_mat(1, 1) - ssm_state(0);
	ssm_state(2) = (lie_alg_mat(1, 0) - lie_alg_mat(0, 1)) / 2.0;
	ssm_state(3) = (lie_alg_mat(1, 0) + lie_alg_mat(0, 1)) / 2.0;
	ssm_state(4) = lie_alg_mat(0, 2);
	ssm_state(5) = lie_alg_mat(1, 2);
	ssm_state(6) = lie_alg_mat(2, 0);
	ssm_state(7) = lie_alg_mat(2, 1);
	if(params.debug_mode){
		utils::printMatrixToFile(ssm_state.transpose(), "getStateFromLieAlgMat :: ssm_state", "log/sl3.txt");
		utils::printMatrixToFile(lie_alg_mat, "getStateFromLieAlgMat :: lie_alg_mat", "log/sl3.txt");
	}
}

void SL3::initializeSampler(const VectorXd &state_sigma, 
	const VectorXd &state_mean){
	if(state_sigma.size() != state_size){
		throw std::invalid_argument(
			cv::format("SL3::initializeSampler :: SSM sigma has invalid size: %d\n",
			state_sigma.size()));
	}
	covariance_mat = state_sigma.asDiagonal();
	printf("Using SL3 sampler with sigma:\n");
	utils::printMatrix(state_sigma.transpose(), nullptr, "%e");

	state_perturbation.resize(state_size);
	rand_gen.resize(1);
	rand_dist.resize(1);

	boost::random_device r;
	for(int state_id = 0; state_id < 1; ++state_id) {
		boost::random::seed_seq seed{ r(), r(), r(), r(), r(), r(), r(), r() };
		rand_gen[state_id] = SampleGenT(seed);
		rand_dist[state_id] = SampleDistT(0, 1);
	}
	if(params.debug_mode){
		utils::printMatrixToFile(covariance_mat, "covariance_mat", "log/sl3.txt");
	}
	is_initialized.sampler = true;
}

void SL3::setSampler(const VectorXd &state_sigma,
	const VectorXd &state_mean){
	assert(state_sigma.size() == state_size);
	covariance_mat = state_sigma.asDiagonal();
}

VectorXd SL3::getSamplerSigma(){
	VectorXd sampler_sigma = covariance_mat.diagonal();
	return sampler_sigma;
}

void SL3::generatePerturbation(VectorXd &perturbation){
	assert(perturbation.size() == state_size);
	VectorXd rand_vec(state_size);
	for(int state_id = 0; state_id < state_size; state_id++){
		rand_vec(state_id) = rand_dist[0](rand_gen[0]);
	}
	perturbation = covariance_mat*rand_vec;
}

// use Random Walk model to generate perturbed sample
void SL3::compositionalRandomWalk(VectorXd &perturbed_state,
	const VectorXd &base_state){
	generatePerturbation(state_perturbation);
	perturbed_state = base_state + state_perturbation;
}
// use first order Auto Regressive model to generate perturbed sample
//void SL3::compositionalAutoRegression1(VectorXd &perturbed_state, VectorXd &perturbed_ar,
//	const VectorXd &base_state, const VectorXd &base_ar, double a){
//	ProjWarpT warp_perturbation, sl3_perturbation, sl3_base_ar, base_warp;
//	generatePerturbation(state_perturbation);
//	getLieAlgMatFromState(warp_perturbation, state_perturbation);
//	getLieAlgMatFromState(sl3_base_ar, base_ar);
//	getWarpFromState(base_warp, base_state);
//	sl3_perturbation = warp_perturbation + a*sl3_base_ar;
//	ProjWarpT SL3_perturbation = sl3_perturbation.exp();
//	ProjWarpT perturbed_warp = base_warp*SL3_perturbation;
//	ProjWarpT base_warp_inv = base_warp.inverse();
//	if(params.debug_mode){
//		utils::printMatrixToFile(state_perturbation.transpose(), "rand_perturbation", "log/sl3.txt");
//		utils::printMatrixToFile(warp_perturbation.transpose(), "warp_perturbation", "log/sl3.txt");
//		utils::printMatrixToFile(sl3_perturbation, "sl3_perturbation", "log/sl3.txt");
//		utils::printMatrixToFile(base_ar.transpose(), "base_ar", "log/sl3.txt");
//		utils::printMatrixToFile(sl3_base_ar, "sl3_base_ar", "log/sl3.txt");
//		utils::printMatrixToFile(SL3_perturbation, "SL3_perturbation", "log/sl3.txt");
//		utils::printMatrixToFile(base_warp, "base_warp", "log/sl3.txt");
//		utils::printMatrixToFile(base_warp_inv, "base_warp_inv", "log/sl3.txt");
//		utils::printMatrixToFile(perturbed_warp, "perturbed_warp", "log/sl3.txt");
//		utils::printMatrixToFile(perturbed_state.transpose(), "perturbed_state", "log/sl3.txt");
//		utils::printMatrixToFile(perturbed_ar.transpose(), "perturbed_ar", "log/sl3.txt");
//	}
//	ProjWarpT SL3_perturbed_ar = a*(base_warp_inv*perturbed_warp);
//	ProjWarpT sl3_perturbed_ar = SL3_perturbed_ar.log();
//	if(params.debug_mode){
//		utils::printMatrixToFile(sl3_perturbed_ar, "sl3_perturbed_ar", "log/sl3.txt");
//	}
//	getStateFromWarp(perturbed_state, perturbed_warp);
//	getStateFromLieAlgMat(perturbed_ar, sl3_perturbed_ar);
//
//}
void SL3::compositionalAutoRegression1(VectorXd &perturbed_state, VectorXd &perturbed_ar,
	const VectorXd &base_state, const VectorXd &base_ar, double a){
	ProjWarpT sl3_perturbation, lie_alg_base_ar, base_warp;
	generatePerturbation(state_perturbation);
	getLieAlgMatFromState(sl3_perturbation, state_perturbation);
	getLieAlgMatFromState(lie_alg_base_ar, base_ar);
	getWarpFromState(base_warp, base_state);
	ProjWarpT SL3_perturbation = (a*lie_alg_base_ar + sl3_perturbation).exp();
	ProjWarpT perturbed_warp = base_warp*SL3_perturbation;
	ProjWarpT base_warp_inv = base_warp.inverse();
	ProjWarpT lie_alg_perturbed_ar = a*(base_warp_inv*perturbed_warp).log();
	getStateFromWarp(perturbed_state, perturbed_warp);
	getStateFromLieAlgMat(perturbed_ar, lie_alg_perturbed_ar);
	if(params.debug_mode){
		utils::printMatrixToFile(state_perturbation.transpose(), "state_perturbation", "log/sl3.txt");
		utils::printMatrixToFile(sl3_perturbation, "sl3_perturbation", "log/sl3.txt");
		utils::printMatrixToFile(base_ar.transpose(), "base_ar", "log/sl3.txt");
		utils::printMatrixToFile(lie_alg_base_ar, "lie_alg_base_ar", "log/sl3.txt");
		utils::printMatrixToFile(SL3_perturbation, "SL3_perturbation", "log/sl3.txt");
		utils::printMatrixToFile(base_warp, "base_warp", "log/sl3.txt");
		utils::printMatrixToFile(perturbed_warp, "perturbed_warp", "log/sl3.txt");
		utils::printMatrixToFile(lie_alg_perturbed_ar, "lie_alg_perturbed_ar", "log/sl3.txt");
	}
}

void SL3::getInitPixGrad(Matrix2Xd &ssm_grad, int pt_id) {
	double x = init_pts(0, pt_id);
	double y = init_pts(1, pt_id);

	ssm_grad <<
		x, -x, -y, y, 1, 0, -x*x, -x*y,
		-x, -2*y, x, x, 0, 1, -y*x, -y*y;
}


void SL3::getCurrPixGrad(Matrix2Xd &ssm_grad, int pt_id) {
	double x = init_pts(0, pt_id);
	double y = init_pts(1, pt_id);

	double curr_x = curr_pts(0, pt_id);
	double curr_y = curr_pts(1, pt_id);

	double xx = x*curr_x, xy = x*curr_y, yy = y*curr_y, yx = y*curr_x;

	Matrix3d curr_warp;
	getWarpFromState(curr_warp, curr_state);
	double a1 = curr_warp(0, 0), a2 = curr_warp(0, 1), a3 = curr_warp(0, 2);
	double a4 = curr_warp(1, 0), a5 = curr_warp(1, 1), a6 = curr_warp(1, 2);
	double a7 = curr_warp(2, 0), a8 = curr_warp(2, 1), a9 = curr_warp(2, 2);

	double p11 = a1*x - a2*y - a7*xx + a8*yx;
	double p12 = a1*y - a7*yx;
	double p13 = a1 - a7*curr_x;
	double p14 = a2*x - a8*xx;
	double p15 = a2*y - a3 - a8*yx + a9*curr_x;
	double p16 = a2 - a8*curr_x;
	double p17 = a3*x - a9*xx;
	double p18 = a3*y - a9*yx;

	double p21 = a4*x - a5*y - a7*xy + a8*yy;
	double p22 = a4*y - a7*yy;
	double p23 = a4 - a7*curr_y;
	double p24 = a5*x - a8*xy;
	double p25 = a5*y - a6 - a8*yy + a9*curr_y;
	double p26 = a5 - a8*curr_y;
	double p27 = a6*x - a9*xy;
	double p28 = a6*y - a9*yy;

	ssm_grad <<
		p11, p12, p13, p14, p15, p16, p17, p18,
		p21, p22, p23, p24, p25, p26, p27, p28;

	double inv_d = 1.0 / curr_pts_hm(2, pt_id);
	ssm_grad *= inv_d;
}

void SL3::cmptInitPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dw){
	validate_ssm_jacobian(dI_dp, dI_dw);

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; pt_id++){
		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = dI_dw(ch_pt_id, 0);
			double Iy = dI_dw(ch_pt_id, 1);

			double Ixx = Ix * x;
			double Iyy = Iy * y;
			double Ixy = Ix * y;
			double Iyx = Iy * x;

			dI_dp(ch_pt_id, 0) = Ixx - Iyx;
			dI_dp(ch_pt_id, 1) = -(2 * Iyy + Ixx);
			dI_dp(ch_pt_id, 2) = Iyx - Ixy;
			dI_dp(ch_pt_id, 3) = Iyx + Ixy;
			dI_dp(ch_pt_id, 4) = Ix;
			dI_dp(ch_pt_id, 5) = Iy;
			dI_dp(ch_pt_id, 6) = -Ixx*x - Iyy*x;
			dI_dp(ch_pt_id, 7) = -Ixx*y - Iyy*y;

			++ch_pt_id;
		}
	}
}

void SL3::cmptWarpedPixJacobian(MatrixXd &dI_dp,
	const PixGradT &dI_dw) {
	validate_ssm_jacobian(dI_dp, dI_dw);

	double a00 = curr_warp(0, 0);
	double a01 = curr_warp(0, 1);
	double a10 = curr_warp(1, 0);
	double a11 = curr_warp(1, 1);
	double a20 = curr_warp(2, 0);
	double a21 = curr_warp(2, 1);

	int ch_pt_id = 0;
	for(int pt_id = 0; pt_id < n_pts; ++pt_id) {
		double w_x = curr_pts(0, pt_id);
		double w_y = curr_pts(1, pt_id);

		double D = curr_pts_hm(2, pt_id);
		double inv_det = 1.0 / D;

		double dwx_dx = (a00 - a20*w_x);
		double dwx_dy = (a01 - a21*w_x);
		double dwy_dx = (a10 - a20*w_y);
		double dwy_dy = (a11 - a21*w_y);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		//double Ix = pix_grad(pt_id, 0);
		//double Iy = pix_grad(pt_id, 1);
		for(int ch_id = 0; ch_id < n_channels; ++ch_id){
			double Ix = (dwx_dx*dI_dw(ch_pt_id, 0) + dwy_dx*dI_dw(ch_pt_id, 1))*inv_det;
			double Iy = (dwx_dy*dI_dw(ch_pt_id, 0) + dwy_dy*dI_dw(ch_pt_id, 1))*inv_det;

			double Ixx = Ix * x;
			double Ixy = Ix * y;
			double Iyy = Iy * y;
			double Iyx = Iy * x;

			dI_dp(ch_pt_id, 0) = Ixx - Iyx;
			dI_dp(ch_pt_id, 1) = -(2 * Iyy + Ixx);
			dI_dp(ch_pt_id, 2) = Iyx - Ixy;
			dI_dp(ch_pt_id, 3) = Iyx + Ixy;
			dI_dp(ch_pt_id, 4) = Ix;
			dI_dp(ch_pt_id, 5) = Iy;
			dI_dp(ch_pt_id, 6) = -Ixx*x - Iyy*x;
			dI_dp(ch_pt_id, 7) = -Ixx*y - Iyy*y;

			++ch_pt_id;
		}
	}
}

void SL3::cmptPixJacobian(MatrixXd &dI_dp,
	const PixGradT &pix_jacobian){
	validate_ssm_jacobian(dI_dp, pix_jacobian);

	for(int pt_id = 0; pt_id < n_pts; pt_id++){
		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);
		double Ix = pix_jacobian(pt_id, 0);
		double Iy = pix_jacobian(pt_id, 1);

		double curr_x = curr_pts(0, pt_id);
		double curr_y = curr_pts(1, pt_id);
		double inv_d = 1.0 / curr_pts_hm(2, pt_id);

		double Ixx = Ix * x;
		double Iyy = Iy * y;
		double Ixy = Ix * y;
		double Iyx = Iy * x;

		dI_dp(pt_id, 0) = (Ixx - Iyy) * inv_d;
		dI_dp(pt_id, 1) = Ixy * inv_d;
		dI_dp(pt_id, 2) = Ix * inv_d;
		dI_dp(pt_id, 3) = Iyx * inv_d;
		dI_dp(pt_id, 4) = (Ix*curr_x + Iy*(y + curr_y)) * inv_d;
		dI_dp(pt_id, 5) = Iy * inv_d;
		dI_dp(pt_id, 6) = (-Ixx*curr_x - Iyx*curr_y) * inv_d;
		dI_dp(pt_id, 7) = (-Ixy*curr_x - Iyy*curr_y) * inv_d;
	}
	//dI_dp.array().colwise() /= curr_pts_hm.array().row(2).transpose();
}


void SL3::cmptApproxPixJacobian(MatrixXd &dI_dp,
	const PixGradT &pix_jacobian) {
	validate_ssm_jacobian(dI_dp, pix_jacobian);

	curr_warp /= curr_warp(2, 2);

	double h00_plus_1 = curr_warp(0, 0);
	double h01 = curr_warp(0, 1);
	double h10 = curr_warp(1, 0);
	double h11_plus_1 = curr_warp(1, 1);
	double h20 = curr_warp(2, 0);
	double h21 = curr_warp(2, 1);

	for(int i = 0; i < n_pts; i++){

		double Nx = curr_pts_hm(0, i);
		double Ny = curr_pts_hm(1, i);
		double D = curr_pts_hm(2, i);
		double D_sqr_inv = 1.0 / (D*D);

		double a = (h00_plus_1*D - h21*Nx) * D_sqr_inv;
		double b = (h01*D - h21*Nx) * D_sqr_inv;
		double c = (h10*D - h20*Ny) * D_sqr_inv;
		double d = (h11_plus_1*D - h21*Ny) * D_sqr_inv;
		double inv_det = 1.0 / ((a*d - b*c)*D);

		double x = init_pts(0, i);
		double y = init_pts(1, i);

		double curr_x = curr_pts(0, i);
		double curr_y = curr_pts(1, i);

		double Ix = pix_jacobian(i, 0);
		double Iy = pix_jacobian(i, 1);

		double Ixx = Ix * x;
		double Ixy = Ix * y;
		double Iyy = Iy * y;
		double Iyx = Iy * x;

		double factor1 = b*curr_y - d*curr_x;
		double factor2 = c*curr_x - a*curr_y;

		dI_dp(i, 0) = (Ixx*d + Ixy*b - Iyx*c - Iyy*a) * inv_det;
		dI_dp(i, 1) = (Ixy*d - Iyy*c) * inv_det;
		dI_dp(i, 2) = (Ix*d - Iy*c) * inv_det;
		dI_dp(i, 3) = (Iyx*a - Ixx*b) * inv_det;
		dI_dp(i, 4) = (Iyy*a - Ix*factor1 - Ixy*b - Iy*factor2) * inv_det;
		dI_dp(i, 5) = (Iy*a - Ix*b) * inv_det;
		dI_dp(i, 6) = (Ixx*factor1 + Iyx*factor2) * inv_det;
		dI_dp(i, 7) = (Ixy*factor1 + Iyy*factor2) * inv_det;

		//dI_dp(i, 0) = (Ix*(d*x + b*y) - Iy*(c*x + a*y)) * inv_det;
		//dI_dp(i, 1) = (Ix*d*y - Iy*c*y) * inv_det;
		//dI_dp(i, 2) = (Ix*d - Iy*c) * inv_det;
		//dI_dp(i, 3) = (Iy*a*x - Ix*b*x) * inv_det;
		//dI_dp(i, 4) = (Ix*(d*curr_x - b*(y + curr_y)) + Iy*(-c*curr_x + a*(y + curr_y))) * inv_det;
		//dI_dp(i, 5) = (Iy*a - Ix*b) * inv_det;
		//dI_dp(i, 6) = (Ix*(-d*x*curr_x + b*x*curr_y) + Iy*(c*x*curr_x - a*x*curr_y)) * inv_det;
		//dI_dp(i, 7) = (Ix*(-d*y*curr_x + b*y*curr_y) + Iy*(c*y*curr_x - a*y*curr_y)) * inv_det;
	}
}

void SL3::estimateWarpFromCorners(VectorXd &state_update, const Matrix24d &in_corners,
	const Matrix24d &out_corners){
	validate_ssm_state(state_update);
	Matrix3d warp_update_mat = utils::computeHomographyDLT(in_corners, out_corners);
	getStateFromWarp(state_update, warp_update_mat);
}

void SL3::estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
	const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
	const EstimatorParams &est_params){
	cv::Mat warp_mat_cv = estimateHomography(in_pts, out_pts, mask, est_params);
	utils::copyCVToEigen<double, Matrix3d>(warp_mat, warp_mat_cv);
	getStateFromWarp(state_update, warp_mat);
}

void SL3::estimateMeanOfSamples(VectorXd &sample_mean,
	const std::vector<VectorXd> &samples, int n_samples){
	if(params.iterative_sample_mean){
		vector<ProjWarpT> lie_group_samples;
		lie_group_samples.resize(n_samples);
		// convert state vectors to SL3 matrices
		for(int sample_id = 0; sample_id < n_samples; sample_id++){
			getWarpFromState(lie_group_samples[sample_id], samples[sample_id]);
		}
		ProjWarpT lie_group_mean = lie_group_samples[0];
		ProjWarpT lie_group_mean_inv = lie_group_mean.inverse();
		for(int iter_id = 0; iter_id < params.sample_mean_max_iters; ++iter_id){
			ProjWarpT lie_algebra_mean = ProjWarpT::Zero();
			for(int sample_id = 0; sample_id < n_samples; ++sample_id){
				lie_algebra_mean += (lie_group_mean_inv*lie_group_samples[sample_id]).log();
			}
			lie_algebra_mean /= n_samples;
			ProjWarpT lie_group_mean_upd = lie_algebra_mean.exp();
			lie_group_mean = lie_group_mean*lie_group_mean_upd;
			lie_group_mean_inv = lie_group_mean.inverse();
			double upd_norm = lie_group_mean_upd.squaredNorm();
			if(upd_norm < params.sample_mean_eps){
				break;
			}
		}
		getStateFromWarp(sample_mean, lie_group_mean);
	} else{
		ProjectiveBase::estimateMeanOfSamples(sample_mean, samples, n_samples);
	}
}


_MTF_END_NAMESPACE

