#ifndef MTF_LIE_AFFINE_H
#define MTF_LIE_AFFINE_H

#define VALIDATE_LIE_HOM_WARP(warp) \
	assert(warp.determinant() == 1.0);

#define LAFF_NORMALIZED_BASIS 0
#define LAFF_DEBUG_MODE 0

#include "ProjectiveBase.h"

_MTF_BEGIN_NAMESPACE

struct LieAffineParams : SSMParams{
	bool normalized_init;
	bool debug_mode;
	//! value constructor
	LieAffineParams(const SSMParams *ssm_params,
		bool _init_as_basis, bool _debug_mode);
	//! copy/default constructor
	LieAffineParams(LieAffineParams *params = nullptr);
};

class LieAffine : public ProjectiveBase{
public:

	typedef LieAffineParams ParamType;
	ParamType params;

	Matrix3d lieAlgBasis[6];
	RowVector3d zero_vec;
	Matrix3d lie_alg_mat;

	using StateSpaceModel::setCorners;


	LieAffine( LieAffineParams *params_in = nullptr);

	void setCorners(const Matrix24d &corners) override;
	void compositionalUpdate(const VectorXd& state_update) override;

	void cmptInitPixJacobian(MatrixXd &jacobian_prod, const PixGradT &pix_jacobian) override;
	void cmptPixJacobian(MatrixXd &jacobian_prod, const PixGradT &pix_jacobian) override;
	void getInitPixGrad(Matrix2Xd &ssm_grad, int pix_id) override;
	void getCurrPixGrad(Matrix2Xd &ssm_grad, int pix_id) override;
	
	void cmptApproxPixJacobian(MatrixXd &jacobian_prod,
		const PixGradT &pix_jacobian) override;

	void estimateWarpFromCorners(VectorXd &state_update, const Matrix24d &in_corners,
		const Matrix24d &out_corners) override;

	void estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
		const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
		int estimation_method, double ransac_reproj_thresh);

	void invertState(VectorXd& inv_state, const VectorXd& state) override{
		inv_state = -state;
	}

	void getWarpFromState(Matrix3d &warp_mat, const VectorXd& ssm_state) override;
	void getStateFromWarp(VectorXd &state_vec, const Matrix3d& warp_mat) override;





};

_MTF_END_NAMESPACE

#endif
