#include "mtf/SSM/TPS.h"
#include "mtf/Utilities/warpUtils.h"
#include "mtf/Utilities/miscUtils.h"
#include "opencv2/calib3d/calib3d.hpp"

#ifndef DISABLE_SPI
#define hom_spi(pt_id) if(spi_mask && !spi_mask[pt_id]){continue;}
#else
#define hom_spi() 
#endif

_MTF_BEGIN_NAMESPACE

TPSParams::TPSParams(const SSMParams *ssm_params, 
bool _normalized_init,
bool _direct_samples, bool _debug_mode) :
SSMParams(ssm_params),
normalized_init(_normalized_init),
direct_samples(_direct_samples),
debug_mode(_debug_mode){}

TPSParams::TPSParams(TPSParams *params) :
SSMParams(params),
normalized_init(TPS_NORMALIZED_BASIS),
direct_samples(TPS_DIRECT_SAMPLES),
debug_mode(TPS_DEBUG_MODE){
	if(params){
		direct_samples = params->direct_samples;
		normalized_init = params->normalized_init;
		debug_mode = params->debug_mode;
	}
}

TPS::TPS(
TPSParams *_params) : StateSpaceModel(_params),
params(_params), tps_params(0, 0, 0){
	printf("\n");
	printf("Using Thin Plate Spline SSM with:\n");
	printf("resx: %d\n", resx);
	printf("resy: %d\n", resy);
	printf("control_pts_resx: %d\n", params.control_pts_resx);
	printf("direct_samples: %d\n", params.direct_samples);
	printf("normalized_init: %d\n", params.normalized_init);
	printf("debug_mode: %d\n", params.debug_mode);

	name = "tps";
	state_size = 2 * params.control_pts_resx*params.control_pts_resy + 6;
	ctrl_idx = VectorXi::LinSpaced(params.control_pts_resx, 0, resx - 1);
	ctrl_idy = VectorXi::LinSpaced(params.control_pts_resy, 0, resx - 1);

	curr_state.resize(state_size);
	corner_control_pts.resize(4, Eigen::NoChange);
	utils::getNormUnitSquarePts(norm_pts, norm_corners, resx, resy);
	corner_control_pts = norm_pts;
}

void TPS::setCorners(const CornersT& corners){
	curr_corners = corners;
	utils::getPtsFromCorners(curr_pts, corners, norm_pts, norm_corners);
	MatrixX2d tps_params = utils::computeTPS(norm_corners, corners);
	utils::applyTPS(curr_pts, norm_pts, corner_control_pts, tps_params);

	if(params.normalized_init){
		init_corners = norm_corners;
		init_pts = norm_pts;
	} else{
		init_corners = curr_corners;
		init_pts = curr_pts;
		curr_state.fill(0);
	}
}

void TPS::compositionalUpdate(const VectorXd& state_update){
	validate_ssm_state(state_update);

}

void TPS::getWarpFromState(Matrix3d &warp_mat,
	const VectorXd& ssm_state){
	validate_ssm_state(ssm_state);

	warp_mat(0, 0) = 1 + ssm_state(0);
	warp_mat(0, 1) = ssm_state(1);
	warp_mat(0, 2) = ssm_state(2);
	warp_mat(1, 0) = ssm_state(3);
	warp_mat(1, 1) = 1 + ssm_state(4);
	warp_mat(1, 2) = ssm_state(5);
	warp_mat(2, 0) = ssm_state(6);
	warp_mat(2, 1) = ssm_state(7);
	warp_mat(2, 2) = 1;
}

void TPS::invertState(VectorXd& inv_state,
	const VectorXd& state){}

void TPS::getStateFromWarp(VectorXd &state_vec,
	const Matrix3d& warp_mat){
	validate_ssm_state(state_vec);
	// since homography matrix is defined only up to a scale factor, this function assumes that 
	// the provided warp matrix has its bottom left entry as 1
	//utils::printMatrix(warp_mat, "warp_mat");
	assert(warp_mat(2, 2) == 1.0);

	state_vec(0) = warp_mat(0, 0) - 1;
	state_vec(1) = warp_mat(0, 1);
	state_vec(2) = warp_mat(0, 2);
	state_vec(3) = warp_mat(1, 0);
	state_vec(4) = warp_mat(1, 1) - 1;
	state_vec(5) = warp_mat(1, 2);
	state_vec(6) = warp_mat(2, 0);
	state_vec(7) = warp_mat(2, 1);
}

void TPS::cmptInitPixJacobian(MatrixXd &jacobian_prod,
	const PixGradT &pix_jacobian){
	validate_ssm_jacobian(jacobian_prod, pix_jacobian);

	for(int pt_id = 0; pt_id < n_pts; pt_id++){

		hom_spi(pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		double Ix = pix_jacobian(pt_id, 0);
		double Iy = pix_jacobian(pt_id, 1);

		double Ixx = Ix * x;
		double Iyy = Iy * y;
		double Ixy = Ix * y;
		double Iyx = Iy * x;

		jacobian_prod(pt_id, 0) = Ixx;
		jacobian_prod(pt_id, 1) = Ixy;
		jacobian_prod(pt_id, 2) = Ix;
		jacobian_prod(pt_id, 3) = Iyx;
		jacobian_prod(pt_id, 4) = Iyy;
		jacobian_prod(pt_id, 5) = Iy;
		jacobian_prod(pt_id, 6) = -x*Ixx - y*Iyx;
		jacobian_prod(pt_id, 7) = -x*Ixy - y*Iyy;
	}
}

void TPS::cmptPixJacobian(MatrixXd &jacobian_prod,
	const PixGradT &pix_jacobian){
	validate_ssm_jacobian(jacobian_prod, pix_jacobian);

	for(int pt_id = 0; pt_id < n_pts; pt_id++){
		hom_spi(pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		double curr_x = curr_pts(0, pt_id);
		double curr_y = curr_pts(1, pt_id);
		double inv_d = 1.0 / curr_pts_hm(2, pt_id);

		double Ix = pix_jacobian(pt_id, 0);
		double Iy = pix_jacobian(pt_id, 1);

		double Ixx = Ix * x;
		double Iyy = Iy * y;
		double Ixy = Ix * y;
		double Iyx = Iy * x;

		jacobian_prod(pt_id, 0) = Ixx * inv_d;
		jacobian_prod(pt_id, 1) = Ixy * inv_d;
		jacobian_prod(pt_id, 2) = Ix * inv_d;
		jacobian_prod(pt_id, 3) = Iyx * inv_d;
		jacobian_prod(pt_id, 4) = Iyy * inv_d;
		jacobian_prod(pt_id, 5) = Iy * inv_d;
		jacobian_prod(pt_id, 6) = (-curr_x*Ixx - curr_y*Iyx) * inv_d;
		jacobian_prod(pt_id, 7) = (-curr_x*Ixy - curr_y*Iyy) * inv_d;
	}
	//jacobian_prod.array().colwise() /= curr_pts_hm.array().row(2).transpose();
}

void TPS::cmptWarpedPixJacobian(MatrixXd &jacobian_prod,
	const PixGradT &pix_grad) {
	validate_ssm_jacobian(jacobian_prod, pix_grad);

	double h00_plus_1 = curr_warp(0, 0);
	double h01 = curr_warp(0, 1);
	double h10 = curr_warp(1, 0);
	double h11_plus_1 = curr_warp(1, 1);
	double h20 = curr_warp(2, 0);
	double h21 = curr_warp(2, 1);

	for(int pt_id = 0; pt_id < n_pts; pt_id++) {
		hom_spi(pt_id);

		double curr_x = curr_pts(0, pt_id);
		double curr_y = curr_pts(1, pt_id);

		double D = curr_pts_hm(2, pt_id);
		double inv_det = 1.0 / D;

		double a = (h00_plus_1 - h20*curr_x);
		double b = (h01 - h21*curr_x);
		double c = (h10 - h20*curr_y);
		double d = (h11_plus_1 - h21*curr_y);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		//double Ix = pix_grad(pt_id, 0);
		//double Iy = pix_grad(pt_id, 1);

		double Ix = (a*pix_grad(pt_id, 0) + c*pix_grad(pt_id, 1))*inv_det;
		double Iy = (b*pix_grad(pt_id, 0) + d*pix_grad(pt_id, 1))*inv_det;

		double Ixx = Ix * x;
		double Ixy = Ix * y;
		double Iyy = Iy * y;
		double Iyx = Iy * x;

		//jacobian_prod(pt_id, 0) = (Ixx*a + Iyx*c) * inv_det2;
		//jacobian_prod(pt_id, 1) = (Ixy*a + Iyy*c) * inv_det2;
		//jacobian_prod(pt_id, 2) = (Ix*a + Iy*c) * inv_det2;
		//jacobian_prod(pt_id, 3) = (Ixx*b + Iyx*d) * inv_det2;
		//jacobian_prod(pt_id, 4) = (Ixy*b + Iyy*d) * inv_det2;
		//jacobian_prod(pt_id, 5) = (Ix*b + Iy*d) * inv_det2;
		//jacobian_prod(pt_id, 6) = -(Ixx*(a*x + b*y) +
		//	Iyx*(c*x + d*y)) * inv_det2;
		//jacobian_prod(pt_id, 7) = -(Ixy*(a*x + b*y) +
		//	Iyy*(c*x + d*y)) * inv_det2;

		jacobian_prod(pt_id, 0) = Ixx;
		jacobian_prod(pt_id, 1) = Ixy;
		jacobian_prod(pt_id, 2) = Ix;
		jacobian_prod(pt_id, 3) = Iyx;
		jacobian_prod(pt_id, 4) = Iyy;
		jacobian_prod(pt_id, 5) = Iy;
		jacobian_prod(pt_id, 6) = -x*Ixx - y*Iyx;
		jacobian_prod(pt_id, 7) = -x*Ixy - y*Iyy;
	}
}

void TPS::cmptWarpedPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
	const PixGradT &pix_grad) {
	validate_ssm_hessian(pix_hess_ssm, pix_hess_coord, pix_grad);

	double h00_plus_1 = curr_warp(0, 0);
	double h01 = curr_warp(0, 1);
	double h10 = curr_warp(1, 0);
	double h11_plus_1 = curr_warp(1, 1);
	double h20 = curr_warp(2, 0);
	double h21 = curr_warp(2, 1);

	Matrix2d ssm_pt_jac, ssm_pt_hess_x, ssm_pt_hess_y;

	Matrix28d ssm_jacobian;
	for(int pt_id = 0; pt_id < n_pts; pt_id++) {
		hom_spi(pt_id);

		double curr_x = curr_pts(0, pt_id);
		double curr_y = curr_pts(1, pt_id);
		double D = curr_pts_hm(2, pt_id);
		double inv_det2 = 1.0 / (D*D);
		double inv_det = 1.0 / D;

		double a = (h00_plus_1 - h20*curr_x) * inv_det;
		double b = (h01 - h21*curr_x) * inv_det;
		double c = (h10 - h20*curr_y) * inv_det;
		double d = (h11_plus_1 - h21*curr_y) * inv_det;
		ssm_pt_jac << a, b, c, d;

		double ax = -h20*(h00_plus_1 + a*D - h20*curr_x)*inv_det2;
		double bx = -(h20*h01 + h21*(a*D - h20*curr_x))*inv_det2;
		double cx = -h20*(h10 + c*D - h20*curr_y)*inv_det2;
		double dx = -(h20*h11_plus_1 + h21*(c*D - h20*curr_y))*inv_det2;
		ssm_pt_hess_x << ax, bx, cx, dx;

		double ay = -(h21*h00_plus_1 + h20*(b*D - h21*curr_x))*inv_det2;
		double by = -h21*(h01 + b*D - h21*curr_x)*inv_det2;
		double cy = -(h21*h10 + h20*(d*D - h21*curr_y))*inv_det2;
		double dy = -h21*(h11_plus_1 + d*D - h21*curr_y)*inv_det2;
		ssm_pt_hess_y << ay, by, cy, dy;

		Map<Matrix8d> curr_pix_hess_ssm((double*)pix_hess_ssm.col(pt_id).data());
		Map<Matrix2d> curr_pix_hess_coord((double*)pix_hess_coord.col(pt_id).data());

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		ssm_jacobian <<
			x, y, 1, 0, 0, 0, -x*x, -y*y,
			0, 0, 0, x, y, 1, -x*x, -y*y;

		curr_pix_hess_ssm = ssm_jacobian.transpose()*(
			ssm_pt_jac.transpose() * curr_pix_hess_coord * ssm_pt_jac
			+
			pix_grad(pt_id, 0)*ssm_pt_hess_x + pix_grad(pt_id, 1)*ssm_pt_hess_y
			)*ssm_jacobian;

		double Ix = a*pix_grad(pt_id, 0) + c*pix_grad(pt_id, 1);
		double Iy = b*pix_grad(pt_id, 0) + d*pix_grad(pt_id, 1);

		double Ixx = Ix * x;
		double Ixy = Ix * y;
		double Iyy = Iy * y;
		double Iyx = Iy * x;

		double Ixxx = Ixx * x;
		double Ixxy = Ixx * y;
		double Ixyy = Ixy * y;

		double Iyyy = Iyy * y;
		double Iyyx = Iyy * x;
		double Iyxx = Iyx * x;

		curr_pix_hess_ssm(0, 6) += Ixxx; curr_pix_hess_ssm(6, 0) += Ixxx;
		curr_pix_hess_ssm(0, 7) += Ixxy; curr_pix_hess_ssm(7, 0) += Ixxy;
		curr_pix_hess_ssm(1, 6) += Ixxy; curr_pix_hess_ssm(6, 1) += Ixxy;
		curr_pix_hess_ssm(1, 7) += Ixyy; curr_pix_hess_ssm(7, 1) += Ixyy;
		curr_pix_hess_ssm(2, 6) += Ixx; curr_pix_hess_ssm(6, 2) += Ixx;
		curr_pix_hess_ssm(2, 7) += Ixy; curr_pix_hess_ssm(7, 2) += Ixy;

		curr_pix_hess_ssm(3, 6) += Iyxx; curr_pix_hess_ssm(6, 3) += Iyxx;
		curr_pix_hess_ssm(3, 7) += Iyyx; curr_pix_hess_ssm(7, 3) += Iyyx;
		curr_pix_hess_ssm(4, 6) += Iyyx; curr_pix_hess_ssm(6, 4) += Iyyx;
		curr_pix_hess_ssm(4, 7) += Iyyy; curr_pix_hess_ssm(7, 4) += Iyyy;
		curr_pix_hess_ssm(5, 6) += Iyx; curr_pix_hess_ssm(6, 5) += Iyx;
		curr_pix_hess_ssm(5, 7) += Iyy; curr_pix_hess_ssm(7, 5) += Iyy;

		curr_pix_hess_ssm(6, 6) -= Ixxx*x + Iyxx*y;
		curr_pix_hess_ssm(6, 7) -= Ixxy*x + Iyyx*y;
		curr_pix_hess_ssm(7, 6) -= Ixxy*x + Iyyx*y;
		curr_pix_hess_ssm(7, 7) -= Ixyy*x + Iyyy*y;
	}
}

void TPS::getInitPixGrad(Matrix2Xd &ssm_grad, int pix_id) {
	double x = init_pts(0, pix_id);
	double y = init_pts(1, pix_id);

	ssm_grad <<
		x, y, 1, 0, 0, 0, -x*x, -y*x,
		0, 0, 0, x, y, 1, -x*y, -y*y;
}

void TPS::getCurrPixGrad(Matrix2Xd &ssm_grad, int pix_id) {
	double x = init_pts(0, pix_id);
	double y = init_pts(1, pix_id);

	double curr_x = curr_pts(0, pix_id);
	double curr_y = curr_pts(1, pix_id);
	double inv_d = 1.0 / curr_pts_hm(2, pix_id);

	ssm_grad <<
		x, y, 1, 0, 0, 0, -x*curr_x, -y*curr_x,
		0, 0, 0, x, y, 1, -x*curr_y, -y*curr_y;
	ssm_grad *= inv_d;
}

void TPS::cmptApproxPixJacobian(MatrixXd &jacobian_prod,
	const PixGradT &pix_grad) {
	validate_ssm_jacobian(jacobian_prod, pix_grad);

	double h00_plus_1 = curr_warp(0, 0);
	double h01 = curr_warp(0, 1);
	double h10 = curr_warp(1, 0);
	double h11_plus_1 = curr_warp(1, 1);
	double h20 = curr_warp(2, 0);
	double h21 = curr_warp(2, 1);

	for(int pt_id = 0; pt_id < n_pts; pt_id++){
		hom_spi(pt_id);
		double curr_x = curr_pts(0, pt_id);
		double curr_y = curr_pts(1, pt_id);
		double D = curr_pts_hm(2, pt_id);

		double inv_det = 1.0 / D;

		double a = (h00_plus_1 - h20*curr_x) * inv_det;
		double b = (h01 - h21*curr_x) * inv_det;
		double c = (h10 - h20*curr_y) * inv_det;
		double d = (h11_plus_1 - h21*curr_y) * inv_det;

		double inv_factor = 1.0 / (a*d - b*c);
		double Ix = (d*pix_grad(pt_id, 0) - c*pix_grad(pt_id, 1))*inv_factor;
		double Iy = (a*pix_grad(pt_id, 1) - b*pix_grad(pt_id, 0))*inv_factor;

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		double Ixx = Ix * x;
		double Ixy = Ix * y;
		double Iyy = Iy * y;
		double Iyx = Iy * x;

		jacobian_prod(pt_id, 0) = Ixx * inv_det;
		jacobian_prod(pt_id, 1) = Ixy * inv_det;
		jacobian_prod(pt_id, 2) = Ix * inv_det;
		jacobian_prod(pt_id, 3) = Iyx * inv_det;
		jacobian_prod(pt_id, 4) = Iyy * inv_det;
		jacobian_prod(pt_id, 5) = Iy * inv_det;
		jacobian_prod(pt_id, 6) = (-curr_x*Ixx - curr_y*Iyx) * inv_det;
		jacobian_prod(pt_id, 7) = (-curr_x*Ixy - curr_y*Iyy) * inv_det;

		//jacobian_prod(pt_id, 0) = (Ixx*d - Iyx*c) * inv_det;
		//jacobian_prod(pt_id, 1) = (Ixy*d - Iyy*c) * inv_det;
		//jacobian_prod(pt_id, 2) = (Ix*d - Iy*c) * inv_det;
		//jacobian_prod(pt_id, 3) = (Iyx*a - Ixx*b) * inv_det;
		//jacobian_prod(pt_id, 4) = (Iyy*a - Ixy*b) * inv_det;
		//jacobian_prod(pt_id, 5) = (Iy*a - Ix*b) * inv_det;
		//jacobian_prod(pt_id, 6) = (Ix*(b*curr_y*x - d*curr_x*x) + 
		//	Iy*(c*curr_x*x - a*curr_y*x)) * inv_det;
		//jacobian_prod(pt_id, 7) = (Ix*(b*curr_y*y - d*curr_x*y) +
		//	Iy*(c*curr_x*y - a*curr_y*y)) * inv_det;
	}
}

void TPS::cmptApproxPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
	const PixGradT &pix_grad) {
	validate_ssm_hessian(pix_hess_ssm, pix_hess_coord, pix_grad);

	double h00_plus_1 = curr_warp(0, 0);
	double h01 = curr_warp(0, 1);
	double h10 = curr_warp(1, 0);
	double h11_plus_1 = curr_warp(1, 1);
	double h20 = curr_warp(2, 0);
	double h21 = curr_warp(2, 1);

	Matrix2d ssm_pt_jac, ssm_pt_jac_inv;
	Matrix2d ssm_pt_hess_x, ssm_pt_hess_y;

	Matrix28d ssm_jacobian;
	for(int pt_id = 0; pt_id < n_pts; pt_id++) {
		hom_spi(pt_id);

		double curr_x = curr_pts(0, pt_id);
		double curr_y = curr_pts(1, pt_id);
		double D = curr_pts_hm(2, pt_id);
		double inv_det2 = 1.0 / (D*D);
		double inv_det = 1.0 / D;


		double a = (h00_plus_1 - h20*curr_x) * inv_det;
		double b = (h01 - h21*curr_x) * inv_det;
		double c = (h10 - h20*curr_y) * inv_det;
		double d = (h11_plus_1 - h21*curr_y) * inv_det;
		double inv_factor = 1.0 / (a*d - b*c);

		ssm_pt_jac << a, b, c, d;
		ssm_pt_jac_inv << d, -b, -c, a;
		ssm_pt_jac_inv *= inv_factor;

		double ax = -h20*(h00_plus_1 + a*D - h20*curr_x)*inv_det2;
		double bx = -(h20*h01 + h21*(a*D - h20*curr_x))*inv_det2;
		double cx = -h20*(h10 + c*D - h20*curr_y)*inv_det2;
		double dx = -(h20*h11_plus_1 + h21*(c*D - h20*curr_y))*inv_det2;
		ssm_pt_hess_x << ax, bx, cx, dx;

		double ay = -(h21*h00_plus_1 + h20*(b*D - h21*curr_x))*inv_det2;
		double by = -h21*(h01 + b*D - h21*curr_x)*inv_det2;
		double cy = -(h21*h10 + h20*(d*D - h21*curr_y))*inv_det2;
		double dy = -h21*(h11_plus_1 + d*D - h21*curr_y)*inv_det2;
		ssm_pt_hess_y << ay, by, cy, dy;

		Map<Matrix8d> curr_pix_hess_ssm((double*)pix_hess_ssm.col(pt_id).data());
		Map<Matrix2d> curr_pix_hess_coord((double*)pix_hess_coord.col(pt_id).data());

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		double Ix = (d*pix_grad(pt_id, 0) - c*pix_grad(pt_id, 1))*inv_factor;
		double Iy = (a*pix_grad(pt_id, 1) - b*pix_grad(pt_id, 0))*inv_factor;

		ssm_jacobian <<
			x, y, 1, 0, 0, 0, -x*x, -y*y,
			0, 0, 0, x, y, 1, -x*x, -y*y;

		curr_pix_hess_ssm = ssm_jacobian.transpose()*(
			ssm_pt_jac_inv.transpose() * (
			curr_pix_hess_coord -
			(Ix*ssm_pt_hess_x + Iy*ssm_pt_hess_y)
			) * ssm_pt_jac_inv
			)*ssm_jacobian;

		double Ixx = Ix * x;
		double Ixy = Ix * y;
		double Iyy = Iy * y;
		double Iyx = Iy * x;

		double Ixxx = Ixx * x;
		double Ixxy = Ixx * y;
		double Ixyy = Ixy * y;

		double Iyyy = Iyy * y;
		double Iyyx = Iyy * x;
		double Iyxx = Iyx * x;

		curr_pix_hess_ssm(0, 6) += Ixxx; curr_pix_hess_ssm(6, 0) += Ixxx;
		curr_pix_hess_ssm(0, 7) += Ixxy; curr_pix_hess_ssm(7, 0) += Ixxy;
		curr_pix_hess_ssm(1, 6) += Ixxy; curr_pix_hess_ssm(6, 1) += Ixxy;
		curr_pix_hess_ssm(1, 7) += Ixyy; curr_pix_hess_ssm(7, 1) += Ixyy;
		curr_pix_hess_ssm(2, 6) += Ixx; curr_pix_hess_ssm(6, 2) += Ixx;
		curr_pix_hess_ssm(2, 7) += Ixy; curr_pix_hess_ssm(7, 2) += Ixy;

		curr_pix_hess_ssm(3, 6) += Iyxx; curr_pix_hess_ssm(6, 3) += Iyxx;
		curr_pix_hess_ssm(3, 7) += Iyyx; curr_pix_hess_ssm(7, 3) += Iyyx;
		curr_pix_hess_ssm(4, 6) += Iyyx; curr_pix_hess_ssm(6, 4) += Iyyx;
		curr_pix_hess_ssm(4, 7) += Iyyy; curr_pix_hess_ssm(7, 4) += Iyyy;
		curr_pix_hess_ssm(5, 6) += Iyx; curr_pix_hess_ssm(6, 5) += Iyx;
		curr_pix_hess_ssm(5, 7) += Iyy; curr_pix_hess_ssm(7, 5) += Iyy;

		curr_pix_hess_ssm(6, 6) -= Ixxx*x + Iyxx*y;
		curr_pix_hess_ssm(6, 7) -= Ixxy*x + Iyyx*y;
		curr_pix_hess_ssm(7, 6) -= Ixxy*x + Iyyx*y;
		curr_pix_hess_ssm(7, 7) -= Ixyy*x + Iyyy*y;
	}
}

void TPS::cmptInitPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
	const PixGradT &pix_grad){
	validate_ssm_hessian(pix_hess_ssm, pix_hess_coord, pix_grad);
	Matrix28d ssm_jacobian;
	for(int pt_id = 0; pt_id < n_pts; pt_id++){
		hom_spi(pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		double Ix = pix_grad(pt_id, 0);
		double Iy = pix_grad(pt_id, 1);

		double Ixx = Ix * x;
		double Ixy = Ix * y;
		double Iyy = Iy * y;
		double Iyx = Iy * x;

		double Ixxx = Ixx * x;
		double Ixxy = Ixx * y;
		double Ixyy = Ixy * y;

		double Iyyy = Iyy * y;
		double Iyyx = Iyy * x;
		double Iyxx = Iyx * x;

		Map<Matrix8d> curr_pix_hess_ssm((double*)pix_hess_ssm.col(pt_id).data());
		Map<Matrix2d> curr_pix_hess_coord((double*)pix_hess_coord.col(pt_id).data());

		ssm_jacobian << x, y, 1, 0, 0, 0, -x*x, -y*y,
			0, 0, 0, x, y, 1, -x*x, -y*y;

		curr_pix_hess_ssm = ssm_jacobian.transpose()*curr_pix_hess_coord*ssm_jacobian;

		curr_pix_hess_ssm(0, 6) += Ixxx; curr_pix_hess_ssm(6, 0) += Ixxx;
		curr_pix_hess_ssm(0, 7) += Ixxy; curr_pix_hess_ssm(7, 0) += Ixxy;
		curr_pix_hess_ssm(1, 6) += Ixxy; curr_pix_hess_ssm(6, 1) += Ixxy;
		curr_pix_hess_ssm(1, 7) += Ixyy; curr_pix_hess_ssm(7, 1) += Ixyy;
		curr_pix_hess_ssm(2, 6) += Ixx; curr_pix_hess_ssm(6, 2) += Ixx;
		curr_pix_hess_ssm(2, 7) += Ixy; curr_pix_hess_ssm(7, 2) += Ixy;

		curr_pix_hess_ssm(3, 6) += Iyxx; curr_pix_hess_ssm(6, 3) += Iyxx;
		curr_pix_hess_ssm(3, 7) += Iyyx; curr_pix_hess_ssm(7, 3) += Iyyx;
		curr_pix_hess_ssm(4, 6) += Iyyx; curr_pix_hess_ssm(6, 4) += Iyyx;
		curr_pix_hess_ssm(4, 7) += Iyyy; curr_pix_hess_ssm(7, 4) += Iyyy;
		curr_pix_hess_ssm(5, 6) += Iyx; curr_pix_hess_ssm(6, 5) += Iyx;
		curr_pix_hess_ssm(5, 7) += Iyy; curr_pix_hess_ssm(7, 5) += Iyy;

		curr_pix_hess_ssm(6, 6) -= Ixxx*x + Iyxx*y;
		curr_pix_hess_ssm(6, 7) -= Ixxy*x + Iyyx*y;
		curr_pix_hess_ssm(7, 6) -= Ixxy*x + Iyyx*y;
		curr_pix_hess_ssm(7, 7) -= Ixyy*x + Iyyy*y;
	}
}

void TPS::cmptPixHessian(MatrixXd &pix_hess_ssm, const PixHessT &pix_hess_coord,
	const PixGradT &pix_grad){
	validate_ssm_hessian(pix_hess_ssm, pix_hess_coord, pix_grad);

	Matrix28d ssm_jacobian;
	for(int pt_id = 0; pt_id < n_pts; pt_id++){
		hom_spi(pt_id);

		double x = init_pts(0, pt_id);
		double y = init_pts(1, pt_id);

		double curr_x = curr_pts(0, pt_id);
		double curr_y = curr_pts(1, pt_id);

		double Ix = pix_grad(pt_id, 0);
		double Iy = pix_grad(pt_id, 1);

		double Ixx = Ix * x;
		double Ixy = Ix * y;
		double Iyy = Iy * y;
		double Iyx = Iy * x;

		double Ixxx = Ixx * x;
		double Ixxy = Ixx * y;
		double Ixyy = Ixy * y;

		double Iyyy = Iyy * y;
		double Iyyx = Iyy * x;
		double Iyxx = Iyx * x;


		Map<Matrix8d> curr_pix_hess_ssm((double*)pix_hess_ssm.col(pt_id).data());
		Map<Matrix2d> curr_pix_hess_coord((double*)pix_hess_coord.col(pt_id).data());

		ssm_jacobian << x, y, 1, 0, 0, 0, -curr_x*x, -curr_x*y,
			0, 0, 0, x, y, 1, -curr_y*x, -curr_y*y;

		curr_pix_hess_ssm = ssm_jacobian.transpose()*curr_pix_hess_coord*ssm_jacobian;

		curr_pix_hess_ssm(0, 6) += Ixxx; curr_pix_hess_ssm(6, 0) += Ixxx;
		curr_pix_hess_ssm(0, 7) += Ixxy; curr_pix_hess_ssm(7, 0) += Ixxy;
		curr_pix_hess_ssm(1, 6) += Ixxy; curr_pix_hess_ssm(6, 1) += Ixxy;
		curr_pix_hess_ssm(1, 7) += Ixyy; curr_pix_hess_ssm(7, 1) += Ixyy;
		curr_pix_hess_ssm(2, 6) += Ixx; curr_pix_hess_ssm(6, 2) += Ixx;
		curr_pix_hess_ssm(2, 7) += Ixy; curr_pix_hess_ssm(7, 2) += Ixy;

		curr_pix_hess_ssm(3, 6) += Iyxx; curr_pix_hess_ssm(6, 3) += Iyxx;
		curr_pix_hess_ssm(3, 7) += Iyyx; curr_pix_hess_ssm(7, 3) += Iyyx;
		curr_pix_hess_ssm(4, 6) += Iyyx; curr_pix_hess_ssm(6, 4) += Iyyx;
		curr_pix_hess_ssm(4, 7) += Iyyy; curr_pix_hess_ssm(7, 4) += Iyyy;
		curr_pix_hess_ssm(5, 6) += Iyx; curr_pix_hess_ssm(6, 5) += Iyx;
		curr_pix_hess_ssm(5, 7) += Iyy; curr_pix_hess_ssm(7, 5) += Iyy;

		curr_pix_hess_ssm(6, 6) -= Ixxx*curr_x + Iyxx*curr_y;
		curr_pix_hess_ssm(6, 7) -= Ixxy*curr_x + Iyyx*curr_y;
		curr_pix_hess_ssm(7, 6) -= Ixxy*curr_x + Iyyx*curr_y;
		curr_pix_hess_ssm(7, 7) -= Ixyy*curr_x + Iyyy*curr_y;
	}
}

void TPS::updateGradPts(double grad_eps){
	Vector3d diff_vec_x_warped = curr_warp.col(0) * grad_eps;
	Vector3d diff_vec_y_warped = curr_warp.col(1) * grad_eps;

	Vector3d pt_inc_warped, pt_dec_warped;
	for(int pt_id = 0; pt_id < n_pts; pt_id++){
		hom_spi(pt_id);

		pt_inc_warped = curr_pts_hm.col(pt_id) + diff_vec_x_warped;
		grad_pts(0, pt_id) = pt_inc_warped(0) / pt_inc_warped(2);
		grad_pts(1, pt_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pt_id) - diff_vec_x_warped;
		grad_pts(2, pt_id) = pt_dec_warped(0) / pt_dec_warped(2);
		grad_pts(3, pt_id) = pt_dec_warped(1) / pt_dec_warped(2);

		pt_inc_warped = curr_pts_hm.col(pt_id) + diff_vec_y_warped;
		grad_pts(4, pt_id) = pt_inc_warped(0) / pt_inc_warped(2);
		grad_pts(5, pt_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pt_id) - diff_vec_y_warped;
		grad_pts(6, pt_id) = pt_dec_warped(0) / pt_dec_warped(2);
		grad_pts(7, pt_id) = pt_dec_warped(1) / pt_dec_warped(2);
	}
}

void TPS::updateHessPts(double hess_eps){
	double hess_eps2 = 2 * hess_eps;

	Vector3d diff_vec_xx_warped = curr_warp.col(0) * hess_eps2;
	Vector3d diff_vec_yy_warped = curr_warp.col(1) * hess_eps2;
	Vector3d diff_vec_xy_warped = (curr_warp.col(0) + curr_warp.col(1)) * hess_eps;
	Vector3d diff_vec_yx_warped = (curr_warp.col(0) - curr_warp.col(1)) * hess_eps;

	Vector3d pt_inc_warped, pt_dec_warped;

	for(int pt_id = 0; pt_id < n_pts; ++pt_id){

		hom_spi(pt_id);

		pt_inc_warped = curr_pts_hm.col(pt_id) + diff_vec_xx_warped;
		hess_pts(0, pt_id) = pt_inc_warped(0) / pt_inc_warped(2);
		hess_pts(1, pt_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pt_id) - diff_vec_xx_warped;
		hess_pts(2, pt_id) = pt_dec_warped(0) / pt_dec_warped(2);
		hess_pts(3, pt_id) = pt_dec_warped(1) / pt_dec_warped(2);

		pt_inc_warped = curr_pts_hm.col(pt_id) + diff_vec_yy_warped;
		hess_pts(4, pt_id) = pt_inc_warped(0) / pt_inc_warped(2);
		hess_pts(5, pt_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pt_id) - diff_vec_yy_warped;
		hess_pts(6, pt_id) = pt_dec_warped(0) / pt_dec_warped(2);
		hess_pts(7, pt_id) = pt_dec_warped(1) / pt_dec_warped(2);

		pt_inc_warped = curr_pts_hm.col(pt_id) + diff_vec_xy_warped;
		hess_pts(8, pt_id) = pt_inc_warped(0) / pt_inc_warped(2);
		hess_pts(9, pt_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pt_id) - diff_vec_xy_warped;
		hess_pts(10, pt_id) = pt_dec_warped(0) / pt_dec_warped(2);
		hess_pts(11, pt_id) = pt_dec_warped(1) / pt_dec_warped(2);

		pt_inc_warped = curr_pts_hm.col(pt_id) + diff_vec_yx_warped;
		hess_pts(12, pt_id) = pt_inc_warped(0) / pt_inc_warped(2);
		hess_pts(13, pt_id) = pt_inc_warped(1) / pt_inc_warped(2);

		pt_dec_warped = curr_pts_hm.col(pt_id) - diff_vec_yx_warped;
		hess_pts(14, pt_id) = pt_dec_warped(0) / pt_dec_warped(2);
		hess_pts(15, pt_id) = pt_dec_warped(1) / pt_dec_warped(2);
	}
}

void TPS::estimateWarpFromCorners(VectorXd &state_update, const Matrix24d &in_corners,
	const Matrix24d &out_corners){
	validate_ssm_state(state_update);
	warp_update_mat = utils::computeTPSDLT(in_corners, out_corners);
	getStateFromWarp(state_update, warp_update_mat);
}

void TPS::estimateWarpFromPts(VectorXd &state_update, vector<uchar> &mask,
	const vector<cv::Point2f> &in_pts, const vector<cv::Point2f> &out_pts,
	int estimation_method, double ransac_reproj_thresh){
	cv::Mat warp_mat_cv = cv::findTPS(in_pts, out_pts, estimation_method,
		ransac_reproj_thresh, mask);
	state_update(0) = warp_mat_cv.at<double>(0, 0) - 1;
	state_update(1) = warp_mat_cv.at<double>(0, 1);
	state_update(2) = warp_mat_cv.at<double>(0, 2);
	state_update(3) = warp_mat_cv.at<double>(1, 0);
	state_update(4) = warp_mat_cv.at<double>(1, 1) - 1;
	state_update(5) = warp_mat_cv.at<double>(1, 2);
	state_update(6) = warp_mat_cv.at<double>(2, 0);
	state_update(7) = warp_mat_cv.at<double>(2, 1);
}

void TPS::generatePerturbation(VectorXd &state_update){
	if(params.direct_samples){
		rand_t(0) = rand_dist[0](rand_gen[0]);
		rand_t(1) = rand_dist[0](rand_gen[0]);
		for(int corner_id = 0; corner_id < 4; corner_id++){
			rand_d(0, corner_id) = rand_dist[1](rand_gen[1]);
			rand_d(1, corner_id) = rand_dist[1](rand_gen[1]);
		}
		disturbed_corners = init_corners + rand_d;
		disturbed_corners = disturbed_corners.colwise() + rand_t;
		estimateWarpFromCorners(state_update, init_corners, disturbed_corners);
	} else{
		for(int state_id = 0; state_id < 8; state_id++){
			state_update(state_id) = rand_dist[state_id](rand_gen[state_id]);
		}
	}
}



_MTF_END_NAMESPACE

