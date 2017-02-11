#ifndef MTF_LIE_ISOMETRY_H
#define MTF_LIE_ISOMETRY_H

#define VALIDATE_LIE_ISO_WARP(warp) \
	assert(warp.determinant() == 1.0);

#define LISO_DEBUG_MODE 0
#define LISO_INIT_AS_BASIS 0

#include "ProjectiveBase.h"

_MTF_BEGIN_NAMESPACE

struct LieIsometryParams{
	bool debug_mode;
	bool init_as_basis;

	//! value constructor
	LieIsometryParams( bool _debug_mode, bool _init_as_basis) : c(_c),
		debug_mode(_debug_mode), init_as_basis(_init_as_basis){}
	//! copy constructor
	LieIsometryParams(LieIsometryParams *params = nullptr) : 
		debug_mode(LISO_DEBUG_MODE),
		init_as_basis(LISO_INIT_AS_BASIS){
		if(params){
			c = params->c;
			debug_mode = params->debug_mode;
			init_as_basis = params->init_as_basis;
		}
	}
};

class LieIsometry : public ProjectiveBase{
public:

	typedef LieIsometryParams ParamType;
	ParamType params;

	Matrix3d lieAlgBasis[3];
	RowVector3d zero_vec;
	Matrix3d lie_alg_mat;

	LieIsometry( LieIsometryParams *params_in = nullptr);

	void compositionalUpdate(const VectorXd& state_update) override;

	void cmptInitPixJacobian(MatrixXd &jacobian_prod, const PixGradT &pix_jacobian) override;
	void cmptPixJacobian(MatrixXd &jacobian_prod, const PixGradT &pix_jacobian) override;
	void cmptApproxPixJacobian(MatrixXd &jacobian_prod,
		const PixGradT &pix_jacobian) override;

	void estimateWarpFromCorners(VectorXd &state_update, const Matrix24d &in_corners,
		const Matrix24d &out_corners) override;

	void estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
		const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
		int estimation_method, double ransac_reproj_thresh) override;

	void invertState(VectorXd& inv_state, const VectorXd& state) override;

	void updateGradPts(double grad_eps) override;
	void updateHessPts(double hess_eps) override;

	void applyWarpToCorners(Matrix24d &warped_corners, const Matrix24d &orig_corners,
		const VectorXd &state_update) override;
	void applyWarpToPts(Matrix2Xd &warped_pts, const Matrix2Xd &orig_pts,
		const VectorXd &state_update) override;

	void getWarpFromState(Matrix3d &warp_mat, const VectorXd& ssm_state) override;
	void getStateFromWarp(VectorXd &state_vec, const Matrix3d& warp_mat) override;
};

_MTF_END_NAMESPACE

#endif
