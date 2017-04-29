#ifndef MTF_LIE_AFFINE_H
#define MTF_LIE_AFFINE_H

#include "ProjectiveBase.h"

_MTF_BEGIN_NAMESPACE

struct LieAffineParams : SSMParams{
	bool normalized_init;
	double grad_eps;
	bool debug_mode;
	//! value constructor
	LieAffineParams(const SSMParams *ssm_params,
		bool _normalized_init, double _grad_eps, bool _debug_mode);
	//! copy/default constructor
	LieAffineParams(const LieAffineParams *params = nullptr);
};

class LieAffine : public ProjectiveBase{
public:

	typedef LieAffineParams ParamType;

	using StateSpaceModel::setCorners;

	LieAffine(const ParamType *params_in = nullptr);

	void setCorners(const Matrix24d &corners) override;
	void compositionalUpdate(const VectorXd& state_update) override;

	void getInitPixGrad(Matrix2Xd &ssm_grad, int pix_id) override;
	void getCurrPixGrad(Matrix2Xd &ssm_grad, int pix_id) override;

	void cmptInitPixJacobian(MatrixXd &dI_dp, const PixGradT &dI_dw) override;
	void cmptWarpedPixJacobian(MatrixXd &dI_dp, const PixGradT &dI_dw) override;
	void cmptPixJacobian(MatrixXd &dI_dp, const PixGradT &dI_dw) override;	
	void cmptApproxPixJacobian(MatrixXd &dI_dp,
		const PixGradT &dI_dw) override;

	void estimateWarpFromCorners(VectorXd &state_update, const Matrix24d &in_corners,
		const Matrix24d &out_corners) override;

	void estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
		const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
		const EstimatorParams &est_params) override;

	void invertState(VectorXd& inv_state, const VectorXd& state) override{
		inv_state = -state;
	}

	void getWarpFromState(Matrix3d &warp_mat, const VectorXd& ssm_state) override;
	void getStateFromWarp(VectorXd &state_vec, const Matrix3d& warp_mat) override;

protected:
	ParamType params;

	Matrix3d lieAlgBasis[6];
	RowVector3d zero_vec;
	Matrix3d lie_alg_mat;

	void computeJacobian(MatrixXd &jacobian, Matrix3Xd &basis_pts_hm);
};

_MTF_END_NAMESPACE

#endif
