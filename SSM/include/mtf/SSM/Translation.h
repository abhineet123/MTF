#ifndef MTF_TRANSLATION_H
#define MTF_TRANSLATION_H

#define VALIDATE_TRANS_WARP(warp) \
	assert(warp(0, 0) == 1 && warp(1, 1) == 1 && warp(2, 2) == 1); \
	assert(warp(0, 1) == 0 && warp(1, 0) == 0); \
	assert(warp(2, 0) == 0 && warp(2, 1) == 0)

#define TRANS_DEBUG_MODE 0

#include "ProjectiveBase.h"

_MTF_BEGIN_NAMESPACE

struct TranslationParams : SSMParams{
	bool debug_mode;
	TranslationParams(const SSMParams *ssm_params,
		bool _debug_mode);
	TranslationParams(const TranslationParams *params = nullptr);
};

class Translation : public ProjectiveBase{
public:

	using ProjectiveBase::setCorners;

	typedef TranslationParams ParamType;
	typedef EstimatorParams::EstType EstType;
		 
	Translation( const ParamType *params_in = nullptr);

	void setCorners(const Matrix24d &corners) override;
	void setState(const VectorXd &ssm_state) override;
	void compositionalUpdate(const VectorXd& state_update) override;

	void estimateWarpFromCorners(VectorXd &state_update, const Matrix24d &in_corners,
		const Matrix24d &out_corners) override;
	void cmptInitPixJacobian(MatrixXd &pix_jacobian_ssm, 
		const PixGradT &pix_jacobian_coord) override{
		validate_ssm_jacobian(pix_jacobian_ssm, pix_jacobian_coord);
		pix_jacobian_ssm = pix_jacobian_coord.leftCols<2>();
	}

	void cmptWarpedPixJacobian(MatrixXd &pix_jacobian_ssm,
		const PixGradT &pixel_grad) override {
		pix_jacobian_ssm = pixel_grad.leftCols<2>();
	}
	void cmptApproxPixJacobian(MatrixXd &pix_jacobian_ssm,
		const PixGradT &pixel_grad) override {
		pix_jacobian_ssm = pixel_grad.leftCols<2>();
	}
	// Jacobian is independent of the current values of the warp parameters
	void cmptPixJacobian(MatrixXd &pix_jacobian_ssm, 
		const PixGradT &pix_jacobian_coord) override{
		cmptInitPixJacobian(pix_jacobian_ssm, pix_jacobian_coord);
	}
	void cmptInitPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_jacobian_coord) override{
		validate_ssm_hessian(pix_hess_ssm, pix_hess_coord, pix_jacobian_coord);
		pix_hess_ssm = pix_hess_coord;
	}
	void cmptWarpedPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_jacobian_coord) override{
		validate_ssm_hessian(pix_hess_ssm, pix_hess_coord, pix_jacobian_coord);
		pix_hess_ssm = pix_hess_coord;
	}
	void cmptApproxPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_jacobian_coord) override{
		validate_ssm_hessian(pix_hess_ssm, pix_hess_coord, pix_jacobian_coord);
		pix_hess_ssm = pix_hess_coord;
	}
	// Hessian is independent of the current values of the warp parameters
	void cmptPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_jacobian_coord) override{
		cmptInitPixHessian(pix_hess_ssm, pix_hess_coord, pix_jacobian_coord);
	}

	void getInitPixGrad(Matrix2Xd &ssm_grad, int pix_id) override{
		ssm_grad = Matrix2d::Identity();
	}

	void getCurrPixGrad(Matrix2Xd &ssm_grad, int pix_id) override{
		ssm_grad = Matrix2d::Identity();
	}

	void invertState(VectorXd& inv_state, const VectorXd& state) override;

	void updateGradPts(double grad_eps) override;
	void updateHessPts(double hess_eps) override;

	void applyWarpToCorners(Matrix24d &warped_corners, const Matrix24d &orig_corners,
		const VectorXd &state_update) override;
	void applyWarpToPts(Matrix2Xd &warped_pts, const Matrix2Xd &orig_pts,
		const VectorXd &state_update) override;

	bool supportsSPI() override{ return true; }

	void compositionalRandomWalk(VectorXd &perturbed_state,
		const VectorXd &base_state) override;
	void compositionalAutoRegression1(VectorXd &perturbed_state, VectorXd &perturbed_ar,
		const VectorXd &base_state, const VectorXd &base_ar, double a = 0.5) override;
	void estimateStateSigma(VectorXd &state_sigma, double pix_sigma) override {
		state_sigma[0] = state_sigma[1] = pix_sigma;
	}
	void estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
		const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
		const EstimatorParams &est_params) override;

	void getWarpFromState(Matrix3d &warp_mat, const VectorXd& ssm_state) override;
	void getStateFromWarp(VectorXd &state_vec, const Matrix3d& warp_mat) override;

protected:

	ParamType params;
};

_MTF_END_NAMESPACE

#endif
