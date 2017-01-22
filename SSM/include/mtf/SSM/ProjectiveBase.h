#ifndef MTF_PROJECTIVE_BASE_H
#define MTF_PROJECTIVE_BASE_H

#include "StateSpaceModel.h"

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>

_MTF_BEGIN_NAMESPACE
/**
base class for all SSMs that can be expressed by homogeneous
multiplication with a 3x3 projective transformation matrix
*/
class ProjectiveBase : public StateSpaceModel{

private:
	stringstream err_msg;

protected:

	ProjWarpT warp_mat, warp_update_mat;
	ProjWarpT inv_warp_mat;

	// let N = no. of pixels and S = no. of state space parameters
	ProjWarpT curr_warp; //! 3 x 3 projective warp matrix 

	// both homogeneous versions are stored too for convenience
	HomPtsT init_pts_hm, curr_pts_hm;
	HomCornersT init_corners_hm, curr_corners_hm;

	PtsT norm_pts;
	HomPtsT norm_pts_hm;
	CornersT norm_corners;
	HomCornersT norm_corners_hm;

	const PtsT& getNormPts() const{ return norm_pts; }
	const HomPtsT& getHomNormPts() const{ return norm_pts_hm; }
	const CornersT& getNormCorners() const{ return norm_corners; }
	const HomCornersT& getHomNormCorners() const{ return norm_corners_hm; }

	// get uniformly spaced grid points corresponding to the given corners
	void getPtsFromCorners(ProjWarpT &warp, PtsT &pts, HomPtsT &pts_hm,
		const CornersT &corners);

	// convert the state vector to the 3x3 warp matrix and vice versa
	virtual void getWarpFromState(Matrix3d &warp_mat, const VectorXd& ssm_state) = 0;
	virtual void getStateFromWarp(VectorXd &state_vec, const Matrix3d& warp_mat) = 0;

public:
	// for the overloaded version that takes OpenCV Mat inut
	using StateSpaceModel::setCorners;

	ProjectiveBase(const SSMParams *params);
	virtual ~ProjectiveBase(){}

	void setCorners(const CornersT& corners) override;
	void setState(const VectorXd &ssm_state) override;
	void additiveUpdate(const VectorXd& state_update) override;
	void invertState(VectorXd& inv_state, const VectorXd& state) override;
	void updateGradPts(double grad_eps) override;
	void updateHessPts(double hess_eps) override;
	void applyWarpToCorners(Matrix24d &warped_corners, const Matrix24d &orig_corners,
		const VectorXd &state_update) override;
	void applyWarpToPts(Matrix2Xd &warped_pts, const Matrix2Xd &orig_pts,
		const VectorXd &state_update) override;
	void applyWarpToPt(double &warped_x, double &warped_y, double x, double y,
		const ProjWarpT &warp);

	// -------------------------------------------------------------------------- //
	// --------------------------- Stochastic Sampler --------------------------- //
	// -------------------------------------------------------------------------- //

	typedef boost::mt11213b SampleGenT;
	typedef boost::normal_distribution<double> SampleDistT;
	typedef SampleDistT::param_type DistParamT;

	std::vector<SampleGenT> rand_gen;
	std::vector<SampleDistT> rand_dist;
	VectorXd state_perturbation;

	void initializeSampler(const VectorXd &state_sigma,
		const VectorXd &state_mean) override;
	void setSampler(const VectorXd &state_sigma,
		const VectorXd &state_mean) override;
	void setSamplerMean(const VectorXd &mean) override;
	void setSamplerSigma(const VectorXd &std) override;

	VectorXd getSamplerSigma() override;
	VectorXd getSamplerMean() override;

	void additiveRandomWalk(VectorXd &perturbed_state,
		const VectorXd &base_state) override;
	void compositionalRandomWalk(VectorXd &perturbed_state,
		const VectorXd &base_state) override;

	void additiveAutoRegression1(VectorXd &perturbed_state, VectorXd &perturbed_ar,
		const VectorXd &base_state, const VectorXd &base_ar, double a = 0.5) override;
	void compositionalAutoRegression1(VectorXd &perturbed_state, VectorXd &perturbed_ar,
		const VectorXd &base_state, const VectorXd &base_ar, double a = 0.5) override;

	void generatePerturbation(VectorXd &perturbation) override;
	void generatePerturbedPts(VectorXd &perturbed_pts) override;
	void estimateMeanOfSamples(VectorXd &sample_mean,
		const std::vector<VectorXd> &samples, int n_samples) override;
	void getPerturbedPts(VectorXd &perturbed_pts,
		const VectorXd &state_perturbation) override;
	void estimateStateSigma(VectorXd &state_sigma, double pix_sigma) override;

	void getIdentityWarp(VectorXd &identity_warp) override;
	void composeWarps(VectorXd &composed_state, const VectorXd &state_1,
		const VectorXd &state_2) override;
};
_MTF_END_NAMESPACE

#endif
