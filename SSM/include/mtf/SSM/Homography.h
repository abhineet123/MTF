#ifndef MTF_HOMOGRAPHY_H
#define MTF_HOMOGRAPHY_H

#define HOM_DIRECT_SAMPLES 0
#define HOM_NORMALIZED_BASIS false
#define HOM_DEBUG_MODE 0

#include "ProjectiveBase.h"

_MTF_BEGIN_NAMESPACE

struct HomographyParams : SSMParams{
	// use normalized unit square points/corners as the initial points/corners
	bool normalized_init;	
	// generate samples by ading perturbations directly to the corners
	// and using DLT method to estimate the corresponding warp
	bool corner_based_sampling;
	bool debug_mode;

	HomographyParams(const SSMParams *ssm_params,
		bool _normalized_init,
		bool _corner_based_sampling, bool _debug_mode);
	HomographyParams(const HomographyParams *params = nullptr);
};

class Homography : public ProjectiveBase{
public:
	
	typedef HomographyParams ParamType;
	ParamType params;

	using ProjectiveBase::setCorners;

	Homography( const ParamType *params_in = nullptr);
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
	void cmptWarpedPixHessian2(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad);
	void cmptApproxPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) override;
	void cmptPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) override;

	void setCorners(const CornersT& corners) override;
	void estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
		const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
		const EstimatorParams &est_params) override;

	void invertState(VectorXd& inv_state, const VectorXd& state) override;

	void updateGradPts(double grad_eps) override;
	void updateHessPts(double hess_eps) override;

	void getInitPixGrad(Matrix2Xd &jacobian_prod, int pix_id) override;
	void getCurrPixGrad(Matrix2Xd &jacobian_prod, int pix_id) override;

	void generatePerturbation(VectorXd &state_update) override;
	void compositionalRandomWalk(VectorXd &perturbed_state,
		const VectorXd &base_state) override;
	void compositionalAutoRegression1(VectorXd &perturbed_state, VectorXd &perturbed_ar,
		const VectorXd &base_state, const VectorXd &base_ar, double a = 0.5) override;

	void getWarpFromState(Matrix3d &warp_mat, const VectorXd& ssm_state) override;
	void getStateFromWarp(VectorXd &state_vec, const Matrix3d& warp_mat) override;

#ifndef DISABLE_SPI
	bool supportsSPI() override{ return true; }
#endif

private:
	CornersT rand_d;
	Vector2d rand_t;
	CornersT disturbed_corners;
};

_MTF_END_NAMESPACE

#endif
