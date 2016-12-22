#ifndef MTF_AFFINE_H
#define MTF_AFFINE_H

#define VALIDATE_AFFINE_WARP(warp)\
	assert(warp(2, 0) == 0.0 && warp(2, 1) == 0.0);\
	assert(warp(2, 2) == 1.0)

#define AFF_NORMALIZED_INIT 0
#define AFF_PT_BASED_SAMPLING 0
#define AFF_DEBUG_MODE 0

#include "ProjectiveBase.h"

_MTF_BEGIN_NAMESPACE

struct AffineParams : SSMParams{
	bool normalized_init;
	int pt_based_sampling;
	bool debug_mode;
	AffineParams(const SSMParams *ssm_params,
		bool _normalized_init, int _pt_based_sampling,
		bool _debug_mode);
	AffineParams(const AffineParams *params = nullptr);
};

class Affine : public ProjectiveBase{
public:

	typedef AffineParams ParamType;
	ParamType params;

	Affine( const ParamType *params_in = nullptr);

	using ProjectiveBase::setCorners;
	void setCorners(const CornersT &corners) override;

	void getInitPixGrad(Matrix2Xd &ssm_grad, int pix_id) override;
	// Gradient is independent of the current values of the warp parameters
	void getCurrPixGrad(Matrix2Xd &ssm_grad, int pix_id) override {
		getInitPixGrad(ssm_grad, pix_id);
	}
	void cmptInitPixJacobian(MatrixXd &jacobian_prod, const PixGradT &am_jacobian) override;
	// Jacobian is independent of the current values of the warp parameters
	void cmptPixJacobian(MatrixXd &jacobian_prod, const PixGradT &am_jacobian) override{
		cmptInitPixJacobian(jacobian_prod, am_jacobian);
	}
	void cmptApproxPixJacobian(MatrixXd &jacobian_prod, 
		const PixGradT &pix_jacobian) override;
	void cmptWarpedPixJacobian(MatrixXd &jacobian_prod,
		const PixGradT &pix_jacobian) override;

	void cmptInitPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) override;
	// Hessian is independent of the current values of the warp parameters
	void cmptPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) override {
		cmptInitPixHessian(pix_hess_ssm, pix_hess_coord, pix_grad);
	}
	void cmptWarpedPixHessian(MatrixXd &_d2I_dp2, const PixHessT &_d2I_dw2,
		const PixGradT &dI_dw) override;

	void compositionalUpdate(const VectorXd& state_update) override;
	void estimateWarpFromCorners(VectorXd &state_update, const CornersT &in_corners,
		const CornersT &out_corners) override;

	void estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
		const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
		const EstimatorParams &est_params) override;

	void invertState(VectorXd& inv_state, const VectorXd& state) override;
	void updateGradPts(double grad_eps) override;
	void updateHessPts(double hess_eps) override;

	void setState(const VectorXd &ssm_state) override;
	void applyWarpToCorners(CornersT &warped_corners, const CornersT &orig_corners,
		const VectorXd &state_update) override;
	void applyWarpToPts(Matrix2Xd &warped_pts, const Matrix2Xd &orig_pts,
		const VectorXd &state_update) override;

	void getWarpFromState(Matrix3d &warp_mat, const VectorXd& ssm_state) override;
	void getStateFromWarp(VectorXd &state_vec, const Matrix3d& warp_mat) override;

	void generatePerturbation(VectorXd &perturbation) override;
	void compositionalRandomWalk(VectorXd &perturbed_state,
		const VectorXd &base_state) override;
	void additiveRandomWalk(VectorXd &perturbed_state,
		const VectorXd &base_state) override;
	void additiveAutoRegression1(VectorXd &perturbed_state, VectorXd &perturbed_ar,
		const VectorXd &base_state, const VectorXd &base_ar, double a = 0.5) override;

#ifndef DISABLE_SPI
	bool supportsSPI() override{ return true; }
#endif

protected:

	Vector6d stateToGeom(const Vector6d &state);
	Vector6d geomToState(const Vector6d &geom);

};

_MTF_END_NAMESPACE

#endif
