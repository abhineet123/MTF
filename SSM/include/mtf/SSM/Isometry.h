#ifndef MTF_ISOMETRY_H
#define MTF_ISOMETRY_H

#define validate_iso_warp(warp) \
	assert(warp(0, 0) == warp(1, 1)); \
	assert(warp(0, 1) == -warp(1, 0)); \
	assert(warp(0, 0) >= -1 && warp(0, 0) <= 1); \
	assert(warp(1, 0) >= -1 && warp(1, 0) <= 1); \
	assert(warp(2, 0) == 0); \
	assert(warp(2, 1) == 0); \
	assert(warp(2, 2) == 1)

#define validate_iso_state(state) \
	assert(state.size() == 3)

#define ISO_PT_BASED_SAMPLING true

#include "ProjectiveBase.h"

_MTF_BEGIN_NAMESPACE

struct IsometryParams : SSMParams{
	int pt_based_sampling;
	IsometryParams(const SSMParams *ssm_params, int _pt_based_sampling);
	IsometryParams(const IsometryParams *params = nullptr);
};

class Isometry : public ProjectiveBase{
public:
	typedef IsometryParams ParamType;

	Isometry( const ParamType *params_in = nullptr);

	void setState(const VectorXd &ssm_state) override;
	void compositionalUpdate(const VectorXd& state_update) override;

	void getInitPixGrad(Matrix2Xd &ssm_grad, int pix_id) override;
	void getCurrPixGrad(Matrix2Xd &ssm_grad, int pix_id) override;

	void cmptInitPixJacobian(MatrixXd &jacobian_prod, const PixGradT &am_jacobian) override;
	void cmptPixJacobian(MatrixXd &jacobian_prod, const PixGradT &am_jacobian) override;
	void cmptApproxPixJacobian(MatrixXd &jacobian_prod,
		const PixGradT &pix_jacobian) override;
	void cmptWarpedPixJacobian(MatrixXd &dI_dp,	const PixGradT &dI_dx) override;
	void cmptInitPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) override;
	void cmptPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) override;
	void cmptWarpedPixHessian(MatrixXd &d2I_dp2, const PixHessT &d2I_dw2,
		const PixGradT &dI_dw) override;

	void estimateWarpFromCorners(VectorXd &state_update, const Matrix24d &in_corners,
		const Matrix24d &out_corners) override;
	void estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
		const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
		const EstimatorParams &est_params) override;

	void invertState(VectorXd& inv_state, const VectorXd& state) override {
		inv_state = -state;
	}
	void updateGradPts(double grad_eps) override;
	void updateHessPts(double hess_eps) override;

	void applyWarpToCorners(Matrix24d &warped_corners, const Matrix24d &orig_corners,
		const VectorXd &state_update) override;
	void applyWarpToPts(Matrix2Xd &warped_pts, const Matrix2Xd &orig_pts,
		const VectorXd &state_update) override;
	void generatePerturbation(VectorXd &perturbation) override;

#ifndef DISABLE_SPI
	bool supportsSPI() override{ return true; }
#endif

private:
	ParamType params;

	void getWarpFromState(Matrix3d &warp_mat, const VectorXd& ssm_state) override;
	void getStateFromWarp(VectorXd &state_vec, const Matrix3d& warp_mat) override;
	double getAngleOfRotation(double sin_theta, double cos_theta);

};



_MTF_END_NAMESPACE

#endif
