#ifndef MTF_SPLINE_H
#define MTF_SPLINE_H

#define SPLINE_CONTROL_SIZE 10
#define SPLINE_CONTROL_OVERLAP 1.0
#define SPLINE_INTERP_TYPE InterpolationType::Bilinear
#define SPLINE_STATIC_WTS true
#define SPLINE_DEBUG_MODE false

#include "StateSpaceModel.h"

_MTF_BEGIN_NAMESPACE

struct SplineParams : SSMParams{

	enum class InterpolationType {
		Bilinear, Biquadratic, Bicubic
	};
	static const char* toString(InterpolationType data_type);

	// resolution of spline control patch, i.e. each spline control point
	// influences control_size_x*control_size_y pixels around it;
	int control_size_x, control_size_y;
	// overlap in pixels between the influence regions of neighboring spline control points;
	double control_overlap;
	// interpolation method to compute the displacements of individual pixels
	// based on those of the spline control points;
	InterpolationType interp_type;
	bool static_wts;
	bool debug_mode;

	SplineParams(const SSMParams *ssm_params,
		int _control_size_x, int _control_size_y,
		double _control_overlap,
		InterpolationType _interp_type,
		bool _static_wts, bool _debug_mode);
	SplineParams(const SplineParams *params = nullptr);
};

class Spline : public StateSpaceModel{

public:
	
	typedef SplineParams ParamType;
	typedef SplineParams::InterpolationType InterpolationType;

	Spline( const ParamType *params_in = nullptr);

	void getCorners(cv::Mat &cv_corners) override;
	using StateSpaceModel::getCorners;

	void additiveUpdate(const VectorXd& state_update) override{
		compositionalUpdate(state_update);
	}
	void setCorners(const CornersT& corners) override;
	using StateSpaceModel::setCorners;

	void compositionalUpdate(const VectorXd& state_update) override;
	void setState(const VectorXd& state) override;

	void cmptInitPixJacobian(MatrixXd &jacobian_prod, const PixGradT &am_jacobian) override;
	void cmptPixJacobian(MatrixXd &jacobian_prod, const PixGradT &am_jacobian) override;
	void cmptWarpedPixJacobian(MatrixXd &jacobian_prod,
		const PixGradT &pix_jacobian) override;

	void cmptApproxPixJacobian(MatrixXd &jacobian_prod,
		const PixGradT &pix_jacobian) override;

	void cmptInitPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) override;

	void cmptWarpedPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) override;
	void cmptApproxPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) override;
	void cmptPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
		const PixGradT &pix_grad) override;

	void invertState(VectorXd& inv_state, const VectorXd& state) override;

	void updateGradPts(double grad_eps) override;
	void updateHessPts(double hess_eps) override;

	bool supportsSPI() override{ return true; }

	void getInitPixGrad(Matrix2Xd &jacobian_prod, int pix_id) override;
	void getCurrPixGrad(Matrix2Xd &jacobian_prod, int pix_id) override;

	void generatePerturbation(VectorXd &state_update) override;

protected:
	ParamType params;

	PtsT norm_pts;
	CornersT norm_corners;
	PtsT init_control_pts, curr_control_pts;
	VectorXd ctrl_idx, ctrl_idy;
	// resolution of spline control point grid
	int control_res_x, control_res_y;
	int n_control_pts;
	//! no. of pts that lie on the region boundary
	int n_bounding_pts;
	VectorXd dist_norm_x, dist_norm_y;
	MatrixXd norm_dist_x, norm_dist_y, interp_wts;
	Matrix2Xd ssm_grad;
	double max_dist_x, max_dist_y;
	void initInterpolationWeights();
	void updateInterpolationWeights();
	double getWeight(double x, double y);
	CornersT rand_d;
	Vector2d rand_t;
	CornersT disturbed_corners;
};

_MTF_END_NAMESPACE

#endif
