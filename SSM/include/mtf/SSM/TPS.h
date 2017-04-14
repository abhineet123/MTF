#ifndef MTF_TPS_H
#define MTF_TPS_H

#define TPS_DIRECT_SAMPLES 0
#define TPS_NORMALIZED_BASIS false
#define TPS_N_CONTROL_PTS 2
#define TPS_DEBUG_MODE 0

#include "StateSpaceModel.h"

_MTF_BEGIN_NAMESPACE

struct TPSParams : SSMParams{
	int control_pts_resx, control_pts_resy;
	// resolution of control point grid
	// use normalized unit square points/corners as the initial points/corners
	bool normalized_init;	
	// generate samples by ading perturbations directly to the corners
	// and using DLT method to estimate the corresponding warp
	bool direct_samples;
	bool debug_mode;

	TPSParams(const SSMParams *ssm_params, bool _normalized_init,
		bool _direct_samples, bool _debug_mode);
	TPSParams(TPSParams *params = nullptr);
};
//! Thin plate splines
class TPS : public StateSpaceModel{
public:	
	typedef TPSParams ParamType;
	ParamType params;

	TPS(TPSParams *params_in=nullptr);
	void compositionalUpdate(const VectorXd& state_update) override;

	void cmptInitPixJacobian(MatrixXd &jacobian_prod, const PixGradT &am_jacobian) override;
	void cmptPixJacobian(MatrixXd &jacobian_prod, const PixGradT &am_jacobian) override;
	void cmptWarpedPixJacobian(MatrixXd &jacobian_prod,
		const PixGradT &pix_jacobian) override;

	void cmptApproxPixJacobian(MatrixXd &jacobian_prod,
		const PixGradT &pix_jacobian) override;
	void estimateWarpFromCorners(VectorXd &state_update, const CornersT &in_corners,
		const CornersT &out_corners) override;
	void cmptInitPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) override;

	void cmptWarpedPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) override;
	void cmptApproxPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) override;
	void cmptPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) override;

	void setCorners(const CornersT& corners) override;
	void estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
		const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
		int estimation_method, double ransac_reproj_thresh) override;

	void invertState(VectorXd& inv_state, const VectorXd& state) override;

	void updateGradPts(double grad_eps) override;
	void updateHessPts(double hess_eps) override;

	bool supportsSPI() override{ return true; }

	void getInitPixGrad(Matrix2Xd &jacobian_prod, int pix_id) override;
	void getCurrPixGrad(Matrix2Xd &jacobian_prod, int pix_id) override;

	void generatePerturbation(VectorXd &state_update) override;

	void getWarpFromState(Matrix3d &warp_mat, const VectorXd& ssm_state) override;
	void getStateFromWarp(VectorXd &state_vec, const Matrix3d& warp_mat) override;

private:
	PtsT norm_pts;
	CornersT norm_corners;
	PtsT corner_control_pts;
	MatrixX2dM tps_params;
	VectorXi ctrl_idx, ctrl_idy;

	CornersT rand_d;
	Vector2d rand_t;
	CornersT disturbed_corners;
};

_MTF_END_NAMESPACE

#endif
