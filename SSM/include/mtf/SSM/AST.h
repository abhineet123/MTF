#ifndef MTF_AST_H
#define MTF_AST_H

#define VALIDATE_AST_WARP(warp) \
	assert(warp(0, 1) == 0 && warp(1, 0) == 0); \
	assert(warp(2, 0) == 0 && warp(2, 1) == 0); \
	assert(warp(2, 2) == 1)

#define AST_DEBUG_MODE 0

#include "ProjectiveBase.h"

_MTF_BEGIN_NAMESPACE

struct ASTParams : SSMParams{
	bool debug_mode;
	ASTParams(const SSMParams *ssm_params, bool _debug_mode);
	ASTParams(const ASTParams *params = nullptr);
};

//! Anisotropic Scaling and Translation
class AST : public ProjectiveBase{
public:

	typedef ASTParams ParamType;
	ParamType params;

	AST(const ParamType *params_in = nullptr);

	void setState(const VectorXd &ssm_state) override;
	void compositionalUpdate(const VectorXd& state_update) override;

	void getInitPixGrad(Matrix2Xd &ssm_grad, int pix_id) override;
	// Gradient is independent of the current values of the warp parameters
	void getCurrPixGrad(Matrix2Xd &ssm_grad, int pix_id) override {
		getInitPixGrad(ssm_grad, pix_id);
	}void cmptInitPixJacobian(MatrixXd &jacobian_prod, const PixGradT &am_jacobian) override;
	// Jacobian is independent of the current values of the warp parameters
	void cmptPixJacobian(MatrixXd &jacobian_prod, const PixGradT &am_jacobian) override{
		cmptInitPixJacobian(jacobian_prod, am_jacobian);
	}
	void cmptApproxPixJacobian(MatrixXd &jacobian_prod,
		const PixGradT &pix_jacobian) override;
	void cmptWarpedPixJacobian(MatrixXd &dI_dp, const PixGradT &dI_dx) override;
	
	
	void cmptInitPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) override;
	// Hessian is independent of the current values of the warp parameters
	void cmptPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) override {
		cmptInitPixHessian(pix_hess_ssm, pix_hess_coord, pix_grad);
	}
	void cmptWarpedPixHessian(MatrixXd &d2I_dp2, const PixHessT &d2I_dw2,
		const PixGradT &dI_dw) override;

	void estimateWarpFromCorners(VectorXd &state_update, const Matrix24d &in_corners,
		const Matrix24d &out_corners) override;
	void estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
		const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
		const EstimatorParams &est_params) override;

	void updateGradPts(double grad_eps) override;
	void updateHessPts(double hess_eps) override;

	void applyWarpToCorners(Matrix24d &warped_corners, const Matrix24d &orig_corners,
		const VectorXd &state_update) override;
	void applyWarpToPts(Matrix2Xd &warped_pts, const Matrix2Xd &orig_pts,
		const VectorXd &state_update) override;

	void getWarpFromState(Matrix3d &warp_mat, const VectorXd& ssm_state) override;
	void getStateFromWarp(VectorXd &state_vec, const Matrix3d& warp_mat) override;

#ifndef DISABLE_SPI
	bool supportsSPI() override{ return true; }
#endif

};



_MTF_END_NAMESPACE

#endif
