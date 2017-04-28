#ifndef MTF_SL3_H
#define MTF_SL3_H

#include "ProjectiveBase.h"

_MTF_BEGIN_NAMESPACE

struct SL3Params : SSMParams{
	bool normalized_init;
	bool iterative_sample_mean;
	int sample_mean_max_iters;
	double sample_mean_eps;
	bool debug_mode;
	//! value constructor
	SL3Params(const SSMParams *ssm_params,
		bool _normalized_init, bool _iterative_sample_mean,
		int _sample_mean_max_iters, double _sample_mean_eps,
		bool _debug_mode);
	//! copy/default constructor
	SL3Params(const SL3Params *params = nullptr);
};

class SL3 : public ProjectiveBase{
public:

	typedef SL3Params ParamType;

	SL3( const ParamType *params_in = nullptr);

	void setState(const VectorXd &ssm_state) override;
	void setCorners(const Matrix24d &corners) override;
	using StateSpaceModel::setCorners;
	void compositionalUpdate(const VectorXd& state_update) override;
	void getInitPixGrad(Matrix2Xd &ssm_grad, int pix_id) override;
	void getCurrPixGrad(Matrix2Xd &ssm_grad, int pix_id) override;

	void cmptInitPixJacobian(MatrixXd &jacobian_prod, const PixGradT &pix_jacobian) override;
	void cmptWarpedPixJacobian(MatrixXd &dI_dp,	const PixGradT &dI_dw) override;
	void cmptPixJacobian(MatrixXd &jacobian_prod, const PixGradT &pix_jacobian) override;
	void cmptApproxPixJacobian(MatrixXd &jacobian_prod,
		const PixGradT &pix_jacobian) override;

	void estimateWarpFromCorners(VectorXd &state_update, const Matrix24d &in_corners,
		const Matrix24d &out_corners) override;

	void estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
		const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
		const EstimatorParams &est_params) override;

	void invertState(VectorXd& inv_state, const VectorXd& state) override{
		inv_state = -state;
	}
	void initializeSampler(const VectorXd &state_sigma,
		const VectorXd &state_mean) override;
	void setSampler(const VectorXd &state_sigma,
		const VectorXd &state_mean) override;
	using ProjectiveBase::initializeSampler;
	VectorXd getSamplerSigma() override;
	void generatePerturbation(VectorXd &perturbation) override;

	void compositionalRandomWalk(VectorXd &perturbed_state,
		const VectorXd &base_state) override;
	void compositionalAutoRegression1(VectorXd &perturbed_state, VectorXd &perturbed_ar,
		const VectorXd &base_state, const VectorXd &base_ar, double a = 0.5) override;
	void estimateMeanOfSamples(VectorXd &sample_mean,
		const std::vector<VectorXd> &samples, int n_samples) override;
private:

	ParamType params;

	Matrix3d lieAlgBasis[8];
	Matrix3d lie_alg_mat;
	ProjWarpT lie_group_perturbation;

	Matrix8d covariance_mat;

	char *log_fname;

	void getWarpFromState(Matrix3d &warp_mat, const VectorXd& ssm_state) override;
	void getStateFromWarp(VectorXd &state_vec, const Matrix3d& warp_mat) override;
	void getLieAlgMatFromState(Matrix3d& lie_alg_mat, const VectorXd& ssm_state);
	void getStateFromLieAlgMat(VectorXd &state_vec,
		const Matrix3d& lie_alg_mat);
};

_MTF_END_NAMESPACE

#endif
