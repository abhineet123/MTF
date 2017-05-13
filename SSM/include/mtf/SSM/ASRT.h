#ifndef MTF_ASRT_H
#define MTF_ASRT_H

#include "ProjectiveBase.h"
#include "SSMEstimator.h"
#include "SSMEstimatorParams.h"

_MTF_BEGIN_NAMESPACE

struct ASRTParams : SSMParams{
	bool normalized_init;
	bool geom_sampling;
	int pt_based_sampling;
	bool debug_mode;
	ASRTParams(const SSMParams *ssm_params, 
		bool _normalized_init, bool _geom_sampling,
		int pt_based_sampling, bool _debug_mode);
	ASRTParams(const ASRTParams *params = nullptr);
};

class ASRTEstimator : public SSMEstimator{
public:
	ASRTEstimator(int modelPoints, bool _use_boost_rng);

	int runKernel(const CvMat* m1, const CvMat* m2, CvMat* model) override;
	bool refine(const CvMat* m1, const CvMat* m2,
		CvMat* model, int maxIters) override;
protected:
	void computeReprojError(const CvMat* m1, const CvMat* m2,
		const CvMat* model, CvMat* error) override;
};


//! Anisotropic Scaling, Rotation and Translation
class ASRT : public ProjectiveBase{
public:
	typedef ASRTParams ParamType;
	ASRT( const ParamType *params_in = nullptr);
	void setCorners(const CornersT& corners) override;
	using ProjectiveBase::setCorners;

	void setState(const VectorXd &ssm_state) override;
	void compositionalUpdate(const VectorXd& state_update) override;

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
	void cmptApproxPixJacobian(MatrixXd &jacobian_prod, const PixGradT &pix_jacobian) override;
	void cmptWarpedPixJacobian(MatrixXd &jacobian_prod,
		const PixGradT &pix_grad) override;

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

	void generatePerturbation(VectorXd &perturbation) override;

	void getWarpFromState(Matrix3d &warp_mat, const VectorXd& ssm_state) override;
	void getStateFromWarp(VectorXd &state_vec, const Matrix3d& warp_mat) override;

#ifndef DISABLE_SPI
	bool supportsSPI() override{ return true; }
#endif

private:
	ParamType params;

	cv::Mat estimateASRT(cv::InputArray _points1, cv::InputArray _points2,
		cv::OutputArray _mask, const SSMEstimatorParams &est_params);
	int	estimateASRT(const CvMat* in_pts, const CvMat* out_pts,
		CvMat* __H, CvMat* mask, const SSMEstimatorParams &est_params);

};



_MTF_END_NAMESPACE

#endif
