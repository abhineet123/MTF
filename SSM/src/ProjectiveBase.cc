#include "mtf/SSM/ProjectiveBase.h"
#include <boost/random/random_device.hpp>
#include <boost/random/seed_seq.hpp>
#include "mtf/Utilities/warpUtils.h"
#include "mtf/Utilities/miscUtils.h"

_MTF_BEGIN_NAMESPACE

ProjectiveBase::ProjectiveBase(const SSMParams *params) :
StateSpaceModel(params){
	init_pts_hm.resize(Eigen::NoChange, n_pts);
	curr_pts_hm.resize(Eigen::NoChange, n_pts);
	norm_pts.resize(Eigen::NoChange, n_pts);
	norm_pts_hm.resize(Eigen::NoChange, n_pts);
	utils::getNormUnitSquarePts(norm_pts, norm_corners, resx, resy);
	utils::homogenize(norm_pts, norm_pts_hm);
	utils::homogenize(norm_corners, norm_corners_hm);
}

void ProjectiveBase::getPtsFromCorners(ProjWarpT &warp, PtsT &pts, HomPtsT &pts_hm,
	const CornersT &corners){
	warp = utils::computeHomographyDLT(norm_corners, corners);
	pts_hm = warp * norm_pts_hm;
	utils::dehomogenize(pts_hm, pts);
}

void ProjectiveBase::setCorners(const CornersT& corners){
	curr_corners = corners;
	getPtsFromCorners(curr_warp, curr_pts, curr_pts_hm, curr_corners);
	utils::homogenize(curr_corners, curr_corners_hm);

	init_corners = curr_corners;
	init_pts = curr_pts;
	init_corners_hm = curr_corners_hm;
	utils::homogenize(init_pts, init_pts_hm);

	curr_warp = Matrix3d::Identity();
	curr_state.fill(0);
}

void ProjectiveBase::setState(const VectorXd &ssm_state){
	validate_ssm_state(ssm_state);
	curr_state = ssm_state;
	getWarpFromState(curr_warp, curr_state);
	curr_pts_hm.noalias() = curr_warp * init_pts_hm;
	curr_corners_hm.noalias() = curr_warp * init_corners_hm;
	utils::dehomogenize(curr_pts_hm, curr_pts);
	utils::dehomogenize(curr_corners_hm, curr_corners);
}

void ProjectiveBase::additiveUpdate(const VectorXd& state_update){
	validate_ssm_state(state_update);
	curr_state += state_update;
	setState(curr_state);
}

void ProjectiveBase::invertState(VectorXd& inv_state, const VectorXd& state){
	getWarpFromState(warp_mat, state);
	inv_warp_mat = warp_mat.inverse();
	inv_warp_mat /= inv_warp_mat(2, 2);
	getStateFromWarp(inv_state, inv_warp_mat);
}

void ProjectiveBase::updateGradPts(double grad_eps){
	Vector3d diff_vec_x_warped = curr_warp.col(0) * grad_eps;
	Vector3d diff_vec_y_warped = curr_warp.col(1) * grad_eps;

	Vector3d pt_inc_warped, pt_dec_warped;
	for(int pix_id = 0; pix_id < n_pts; pix_id++){
		pt_inc_warped = curr_pts_hm.col(pix_id) + diff_vec_x_warped;
		grad_pts(0, pix_id) = pt_inc_warped(0) / pt_inc_warped(2);
		grad_pts(1, pix_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pix_id) - diff_vec_x_warped;
		grad_pts(2, pix_id) = pt_dec_warped(0) / pt_dec_warped(2);
		grad_pts(3, pix_id) = pt_dec_warped(1) / pt_dec_warped(2);

		pt_inc_warped = curr_pts_hm.col(pix_id) + diff_vec_y_warped;
		grad_pts(4, pix_id) = pt_inc_warped(0) / pt_inc_warped(2);
		grad_pts(5, pix_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pix_id) - diff_vec_y_warped;
		grad_pts(6, pix_id) = pt_dec_warped(0) / pt_dec_warped(2);
		grad_pts(7, pix_id) = pt_dec_warped(1) / pt_dec_warped(2);
	}
}

void ProjectiveBase::updateHessPts(double hess_eps){
	double hess_eps2 = 2 * hess_eps;

	Vector3d diff_vec_xx_warped = curr_warp.col(0) * hess_eps2;
	Vector3d diff_vec_yy_warped = curr_warp.col(1) * hess_eps2;
	Vector3d diff_vec_xy_warped = (curr_warp.col(0) + curr_warp.col(1)) * hess_eps;
	Vector3d diff_vec_yx_warped = (curr_warp.col(0) - curr_warp.col(1)) * hess_eps;

	Vector3d pt_inc_warped, pt_dec_warped;

	for(int pix_id = 0; pix_id < n_pts; pix_id++){

		pt_inc_warped = curr_pts_hm.col(pix_id) + diff_vec_xx_warped;
		hess_pts(0, pix_id) = pt_inc_warped(0) / pt_inc_warped(2);
		hess_pts(1, pix_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pix_id) - diff_vec_xx_warped;
		hess_pts(2, pix_id) = pt_dec_warped(0) / pt_dec_warped(2);
		hess_pts(3, pix_id) = pt_dec_warped(1) / pt_dec_warped(2);

		pt_inc_warped = curr_pts_hm.col(pix_id) + diff_vec_yy_warped;
		hess_pts(4, pix_id) = pt_inc_warped(0) / pt_inc_warped(2);
		hess_pts(5, pix_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pix_id) - diff_vec_yy_warped;
		hess_pts(6, pix_id) = pt_dec_warped(0) / pt_dec_warped(2);
		hess_pts(7, pix_id) = pt_dec_warped(1) / pt_dec_warped(2);

		pt_inc_warped = curr_pts_hm.col(pix_id) + diff_vec_xy_warped;
		hess_pts(8, pix_id) = pt_inc_warped(0) / pt_inc_warped(2);
		hess_pts(9, pix_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pix_id) - diff_vec_xy_warped;
		hess_pts(10, pix_id) = pt_dec_warped(0) / pt_dec_warped(2);
		hess_pts(11, pix_id) = pt_dec_warped(1) / pt_dec_warped(2);

		pt_inc_warped = curr_pts_hm.col(pix_id) + diff_vec_yx_warped;
		hess_pts(12, pix_id) = pt_inc_warped(0) / pt_inc_warped(2);
		hess_pts(13, pix_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pix_id) - diff_vec_yx_warped;
		hess_pts(14, pix_id) = pt_dec_warped(0) / pt_dec_warped(2);
		hess_pts(15, pix_id) = pt_dec_warped(1) / pt_dec_warped(2);
	}
}

void ProjectiveBase::applyWarpToCorners(Matrix24d &warped_corners, const Matrix24d &orig_corners,
	const VectorXd &ssm_state){
	getWarpFromState(warp_mat, ssm_state);
	for(int corner_id = 0; corner_id < 4; corner_id++){
		applyWarpToPt(warped_corners(0, corner_id), warped_corners(1, corner_id),
			orig_corners(0, corner_id), orig_corners(1, corner_id), warp_mat);
	}
}
void ProjectiveBase::applyWarpToPts(Matrix2Xd &warped_pts, const Matrix2Xd &orig_pts,
	const VectorXd &ssm_state){
	getWarpFromState(warp_mat, ssm_state);
	int n_pts = orig_pts.cols();
	for(int pt_id = 0; pt_id < n_pts; pt_id++){
		applyWarpToPt(warped_pts(0, pt_id), warped_pts(1, pt_id),
			orig_pts(0, pt_id), orig_pts(1, pt_id), warp_mat);
	}
}

void ProjectiveBase::applyWarpToPt(double &warped_x, double &warped_y, double x, double y,
	const ProjWarpT &warp){
	double discr = warp(2, 0)*x + warp(2, 1)*y + warp(2, 2);
	warped_x = (warp(0, 0)*x + warp(0, 1)*y + warp(0, 2)) / discr;
	warped_y = (warp(1, 0)*x + warp(1, 1)*y + warp(1, 2)) / discr;
}

// -------------------------------------------------------------------------- //
// --------------------------- Stochastic Sampler --------------------------- //
// -------------------------------------------------------------------------- //

void ProjectiveBase::initializeSampler(const VectorXd &state_sigma,
	const VectorXd &state_mean){
	if(state_sigma.size() != state_size){		
		throw std::invalid_argument(
			cv::format("ProjectiveBase::initializeSampler :: SSM sigma has invalid size %d\n",
			state_sigma.size()));
	}
	if(state_mean.size() != state_size){
		throw std::invalid_argument(
			cv::format("ProjectiveBase::initializeSampler :: SSM mean has invalid size %d\n",
			state_mean.size()));
	}
	printf("Initializing %s sampler with sigma: ", name.c_str());
	utils::printMatrix(state_sigma.transpose(), nullptr, "%e");

	state_perturbation.resize(state_size);
	rand_gen.resize(state_size);
	rand_dist.resize(state_size);

	boost::random_device r;
	for(int state_id = 0; state_id < state_size; state_id++) {
		boost::random::seed_seq seed{ r(), r(), r(), r(), r(), r(), r(), r() };
		rand_gen[state_id] = SampleGenT(seed);
		rand_dist[state_id] = SampleDistT(state_mean[state_id], state_sigma[state_id]);
	}
	is_initialized.sampler = true;
}

void ProjectiveBase::estimateStateSigma(VectorXd &state_sigma, double pix_sigma){
	MatrixXd ssm_grad_norm(n_pts, state_size);
	Matrix2Xd pix_ssm_grad;
	pix_ssm_grad.resize(Eigen::NoChange, state_size);
	for(int pt_id = 0; pt_id < n_pts; pt_id++){
		getCurrPixGrad(pix_ssm_grad, pt_id);
		ssm_grad_norm.row(pt_id) = pix_ssm_grad.colwise().norm();
	}
	VectorXd ssm_grad_norm_mean = ssm_grad_norm.colwise().mean();
	for(int state_id = 0; state_id < state_size; state_id++){
		state_sigma(state_id) = pix_sigma / ssm_grad_norm_mean(state_id);
	}
}

void ProjectiveBase::setSampler(const VectorXd &state_sigma,
	const VectorXd &state_mean){
	assert(state_sigma.size() == state_size);
	assert(state_mean.size() == state_size);
	for(int state_id = 0; state_id < state_size; state_id++){
		rand_dist[state_id].param(DistParamT(state_mean[state_id], state_sigma[state_id]));
	}
}

void ProjectiveBase::setSamplerMean(const VectorXd &mean){
	for(int state_id = 0; state_id < state_size; state_id++){
		double state_sigma = rand_dist[state_id].sigma();
		rand_dist[state_id].param(DistParamT(mean[state_id], state_sigma));
	}
}
void ProjectiveBase::setSamplerSigma(const VectorXd &std){
	for(int state_id = 0; state_id < state_size; state_id++){
		double mean = rand_dist[state_id].mean();
		rand_dist[state_id].param(DistParamT(mean, std[state_id]));
	}
}

VectorXd ProjectiveBase::getSamplerSigma(){
	VectorXd sampler_sigma(state_size);
	for(int state_id = 0; state_id < state_size; state_id++){
		sampler_sigma(state_id) = rand_dist[state_id].sigma();
	}
	return sampler_sigma;
}
VectorXd ProjectiveBase::getSamplerMean(){
	VectorXd sampler_mean(state_size);
	for(int state_id = 0; state_id < state_size; state_id++){
		sampler_mean(state_id) = rand_dist[state_id].mean();
	}
	return sampler_mean;
}

// use Random Walk model to generate perturbed sample
void ProjectiveBase::additiveRandomWalk(VectorXd &perturbed_state,
	const VectorXd &base_state){
	generatePerturbation(state_perturbation);
	perturbed_state = base_state + state_perturbation;
}
void ProjectiveBase::compositionalRandomWalk(VectorXd &perturbed_state,
	const VectorXd &base_state){
	generatePerturbation(state_perturbation);
	ProjWarpT base_warp, warp_perturbation;
	getWarpFromState(base_warp, base_state);
	getWarpFromState(warp_perturbation, state_perturbation);
	ProjWarpT perturbed_warp = base_warp * warp_perturbation;
	getStateFromWarp(perturbed_state, perturbed_warp);
}
// use first order Auto Regressive model to generate perturbed sample
void ProjectiveBase::additiveAutoRegression1(VectorXd &perturbed_state, VectorXd &perturbed_ar,
	const VectorXd &base_state, const VectorXd &base_ar, double a){
	generatePerturbation(state_perturbation);
	perturbed_state = base_state + base_ar + state_perturbation;
	perturbed_ar = a*(perturbed_state - base_state);
}
void ProjectiveBase::compositionalAutoRegression1(VectorXd &perturbed_state, VectorXd &perturbed_ar,
	const VectorXd &base_state, const VectorXd &base_ar, double a){
	generatePerturbation(state_perturbation);
	ProjWarpT base_warp, warp_perturbation, warp_ar;
	getWarpFromState(base_warp, base_state);
	getWarpFromState(warp_perturbation, state_perturbation);
	getWarpFromState(warp_ar, base_ar);
	ProjWarpT perturbed_warp = base_warp * warp_ar * warp_perturbation;
	ProjWarpT perturbed_ar_warp = base_warp.inverse() * perturbed_warp;	
	//utils::printMatrix(base_warp, "base_warp");
	//utils::printMatrix(warp_ar, "warp_ar");
	//utils::printMatrix(warp_perturbation, "warp_perturbation");
	//utils::printMatrix(perturbed_warp, "perturbed_warp");
	getStateFromWarp(perturbed_state, perturbed_warp);
	getStateFromWarp(perturbed_ar, perturbed_ar_warp);
	perturbed_ar *= a;
}

void ProjectiveBase::generatePerturbation(VectorXd &perturbation){
	assert(perturbation.size() == state_size);
	for(int state_id = 0; state_id < state_size; state_id++){
		perturbation(state_id) = rand_dist[state_id](rand_gen[state_id]);
	}
}
void ProjectiveBase::generatePerturbedPts(VectorXd &perturbed_pts){
	VectorXd state_update(state_size);
	generatePerturbation(state_update);
	getPerturbedPts(perturbed_pts, state_update);
}

void ProjectiveBase::getPerturbedPts(VectorXd &perturbed_pts,
	const VectorXd &state_perturbation){
	Matrix3d warp_perturbation;
	getWarpFromState(warp_perturbation, state_perturbation);
	utils::dehomogenize(curr_warp * warp_perturbation * init_pts_hm, perturbed_pts);
}

void ProjectiveBase::estimateMeanOfSamples(VectorXd &sample_mean,
	const std::vector<VectorXd> &samples, int n_samples){
	sample_mean.setZero();
	for(int sample_id = 0; sample_id < n_samples; sample_id++){
		sample_mean += (samples[sample_id] - sample_mean) / (sample_id + 1);
	}
}

void ProjectiveBase::getIdentityWarp(VectorXd &identity_warp){
	identity_warp.setZero();
}
void ProjectiveBase::composeWarps(VectorXd &composed_state, const VectorXd &state_1,
	const VectorXd &state_2){
	ProjWarpT warp_1, warp_2;
	getWarpFromState(warp_1, state_1);
	getWarpFromState(warp_2, state_2);
	ProjWarpT composed_warp = warp_2*warp_1;
	getStateFromWarp(composed_state, composed_warp);
}
_MTF_END_NAMESPACE

