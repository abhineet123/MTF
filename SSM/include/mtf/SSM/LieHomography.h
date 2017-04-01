#ifndef MTF_LIE_HOMOGRAPHY_H
#define MTF_LIE_HOMOGRAPHY_H

#define VALIDATE_LIE_HOM_WARP(warp) \
	assert(warp.determinant() == 1.0);


#define LHOM_NORMALIZED_INIT 0
#define LHOM_GRAD_EPS 1e-8
#define LHOM_DEBUG_MODE 0

#include "ProjectiveBase.h"

_MTF_BEGIN_NAMESPACE

struct LieHomographyParams : SSMParams{
	bool normalized_init;
	double grad_eps;
	bool debug_mode;
	//! value constructor
	LieHomographyParams(const SSMParams *ssm_params,
		bool _normalized_init, double _grad_eps,
		bool _debug_mode);
	//! copy/default constructor
	LieHomographyParams(const LieHomographyParams *params = nullptr);
};

class LieHomography : public ProjectiveBase{
public:

	typedef LieHomographyParams ParamType;
	ParamType params;

	Matrix3d lieAlgBasis[8];
	RowVector3d zero_vec;
	Matrix3d lie_alg_mat;

	using StateSpaceModel::setCorners;

	LieHomography( const ParamType *params_in = nullptr);

	void setCorners(const Matrix24d &corners) override;
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
	void cmptInitPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) override;

	void getWarpFromState(Matrix3d &warp_mat, const VectorXd& ssm_state) override;
	void getStateFromWarp(VectorXd &state_vec, const Matrix3d& warp_mat) override;

private:
	void computeJacobian(MatrixXd &jacobian, Matrix3Xd &basis_pts_hm);



};

_MTF_END_NAMESPACE

#endif
