#include "mtf/Utilities/imgUtils.h"
#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/warpUtils.h"
#include "opencv2/highgui/highgui.hpp"

#ifdef USE_TBB
#include "tbb/tbb.h" 
#endif

_MTF_BEGIN_NAMESPACE

namespace utils{
	const char* toString(InterpType interp_type){
		switch(interp_type){
		case InterpType::Nearest:
			return "Nearest";
		case InterpType::Linear:
			return "Linear";
		case InterpType::Cubic:
			return "Cubic";
		case InterpType::Cubic2:
			return "Cubic2";
		case InterpType::CubicBSpl:
			return "CubicBSpl";
		default:
			throw std::invalid_argument("Invalid interpolation type provided");
		}
	}
	const char* toString(BorderType border_type){
		switch(border_type){
		case BorderType::Constant:
			return "Constant";
		case BorderType::Replicate:
			return "Replicate";
		default:
			throw std::invalid_argument("Invalid border type provided");
		}
	}

	const char* getType(const cv::Mat& mat){
		const int mtype = mat.type();

		switch(mtype){
		case CV_8UC1:  return "CV_8UC1";
		case CV_8UC2:  return "CV_8UC2";
		case CV_8UC3:  return "CV_8UC3";
		case CV_8UC4:  return "CV_8UC4";

		case CV_8SC1:  return "CV_8SC1";
		case CV_8SC2:  return "CV_8SC2";
		case CV_8SC3:  return "CV_8SC3";
		case CV_8SC4:  return "CV_8SC4";

		case CV_16UC1: return "CV_16UC1";
		case CV_16UC2: return "CV_16UC2";
		case CV_16UC3: return "CV_16UC3";
		case CV_16UC4: return "CV_16UC4";

		case CV_16SC1: return "CV_16SC1";
		case CV_16SC2: return "CV_16SC2";
		case CV_16SC3: return "CV_16SC3";
		case CV_16SC4: return "CV_16SC4";

		case CV_32SC1: return "CV_32SC1";
		case CV_32SC2: return "CV_32SC2";
		case CV_32SC3: return "CV_32SC3";
		case CV_32SC4: return "CV_32SC4";

		case CV_32FC1: return "CV_32FC1";
		case CV_32FC2: return "CV_32FC2";
		case CV_32FC3: return "CV_32FC3";
		case CV_32FC4: return "CV_32FC4";

		case CV_64FC1: return "CV_64FC1";
		case CV_64FC2: return "CV_64FC2";
		case CV_64FC3: return "CV_64FC3";
		case CV_64FC4: return "CV_64FC4";

		default:
			return "Invalid type of matrix!";
		}
	}
	// using cubic BSpline  interpolation
	inline double cubicBSplInterpolate(double p0, double p1, double p2, double p3, double dx) {
		double dx2 = dx*dx;
		double dx3 = dx2*dx;
		double dxi = 1 - dx;
		double dxi2 = dxi*dxi;
		double dxi3 = dxi2*dxi;
		return (p0*dxi3 + p1*(4 - 6 * dx2 + 3 * dx3) + p2*(4 - 6 * dxi2 + 3 * dxi3) + p3*dx3) / 6;
	}
	template<>
	inline double getPixVal<InterpType::CubicBSpl, BorderType::Constant>(
		const EigImgT &img, double x, double y,
		int h, int w, double overflow_val){
		assert(img.rows() == h && img.cols() == w);
		//printf("----------Cubic BSpline Interpolation------------\n\n");
		if(checkOverflow(x, y, h, w)){
			return overflow_val;
		}
		int x1 = static_cast<int>(x);
		int y1 = static_cast<int>(y);

		Matrix4d neigh_pix_grid;
		if(!getNeighboringPixGrid(neigh_pix_grid, img, x1, y1, h, w))
			return overflow_val;
		double dx = x - x1;
		double dy = y - y1;
		Vector4d col_interp_pix;
		col_interp_pix(0) = cubicBSplInterpolate(neigh_pix_grid.row(0), dy);
		col_interp_pix(1) = cubicBSplInterpolate(neigh_pix_grid.row(1), dy);
		col_interp_pix(2) = cubicBSplInterpolate(neigh_pix_grid.row(2), dy);
		col_interp_pix(3) = cubicBSplInterpolate(neigh_pix_grid.row(3), dy);
		return cubicBSplInterpolate(col_interp_pix, dx);
	}

	// using bicubic interpolation
	inline double cubicInterpolate(double p0, double p1, double p2, double p3, double x) {
		return p1 + 0.5 * x*(p2 - p0 + x*(2.0*p0 - 5.0*p1 + 4.0*p2 - p3 + x*(3.0*(p1 - p2) + p3 - p0)));
	}
	template<>
	inline double getPixVal<InterpType::Cubic, BorderType::Constant>(
		const EigImgT &img, double x, double y,
		int h, int w, double overflow_val){
		assert(img.rows() == h && img.cols() == w);
		//printf("----------Bicubic Interpolation------------\n\n");
		if(checkOverflow(x, y, h, w)){
			return overflow_val;
		}
		Matrix4d neigh_pix_grid;
		if(!getNeighboringPixGrid(neigh_pix_grid, img, x, y, h, w))
			return overflow_val;

		double dx = x - static_cast<int>(x);
		double dy = y - static_cast<int>(y);

		Vector4d col_interp_pix;
		col_interp_pix(0) = cubicInterpolate(neigh_pix_grid.row(0), dy);
		col_interp_pix(1) = cubicInterpolate(neigh_pix_grid.row(1), dy);
		col_interp_pix(2) = cubicInterpolate(neigh_pix_grid.row(2), dy);
		col_interp_pix(3) = cubicInterpolate(neigh_pix_grid.row(3), dy);
		return cubicInterpolate(col_interp_pix, dx);
	}
	template<>
	inline double getPixVal<InterpType::Cubic2, BorderType::Constant>(
		const EigImgT &img, double x, double y,
		int h, int w, double overflow_val){
		assert(img.rows() == h && img.cols() == w);
		//printf("----------Bi cubic Interpolation using coefficients------------\n\n");		
		Matrix4d neigh_pix_grid, bicubic_coeff;
		if(!getNeighboringPixGrid(neigh_pix_grid, img, x, y, h, w))
			return overflow_val;
		getBiCubicCoefficients(bicubic_coeff, neigh_pix_grid);
		double dx = x - static_cast<int>(x);
		double dy = y - static_cast<int>(y);
		return  biCubic(bicubic_coeff, dx, dy);
	}	

#ifdef USE_TBB
#include "imgUtils_tbb.cc"
#else

	template<typename PtsT>
	void getPixVals(VectorXd &pix_vals,
		const EigImgT &img, const PtsT &pts, int n_pix, int h, int w,
		double norm_mult, double norm_add){
		//printf("n_pix: %d\t pix_vals.size(): %l\t pts.cols(): %l", n_pix, pix_vals.size(),  pts.cols());
		assert(pix_vals.size() == n_pix && pts.cols() == n_pix);

		for(int i = 0; i < n_pix; i++){
			pix_vals(i) = norm_mult * getPixVal<PIX_INTERP_TYPE, PIX_BORDER_TYPE>(img, pts(0, i), pts(1, i), h, w) + norm_add;
		}
	}

	/****************************************************************/
	/******************** Gradient of Warped Image********************/
	/****************************************************************/
	void getWarpedImgGrad(PixGradT &warped_img_grad,
		const EigImgT &img, const Matrix8Xd &warped_offset_pts,
		double grad_eps, int n_pix,
		int h, int w, double pix_mult_factor){
		assert(warped_img_grad.rows() == n_pix);

		double grad_mult_factor = pix_mult_factor / (2 * grad_eps);
		double pix_val_inc, pix_val_dec;

		for(int pix_id = 0; pix_id < n_pix; pix_id++){

			pix_val_inc = getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img, warped_offset_pts(0, pix_id),
				warped_offset_pts(1, pix_id), h, w);
			pix_val_dec = getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img, warped_offset_pts(2, pix_id),
				warped_offset_pts(3, pix_id), h, w);
			warped_img_grad(pix_id, 0) = (pix_val_inc - pix_val_dec)*grad_mult_factor;

			pix_val_inc = getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img, warped_offset_pts(4, pix_id),
				warped_offset_pts(5, pix_id), h, w);
			pix_val_dec = getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img, warped_offset_pts(6, pix_id),
				warped_offset_pts(7, pix_id), h, w);
			warped_img_grad(pix_id, 1) = (pix_val_inc - pix_val_dec)*grad_mult_factor;
		}
		//utils::printMatrixToFile(warped_img_grad, "warped_img_grad", "log/mtf_log.txt", "%15.9f", "a");

	}
	// mapped version
	template<InterpType mapping_type>
	void getWarpedImgGrad(PixGradT &warped_img_grad,
		const EigImgT &img, const VectorXd &intensity_map,
		const Matrix8Xd &warped_offset_pts, double grad_eps,
		int n_pix, int h, int w, double pix_mult_factor){
		assert(warped_img_grad.rows() == n_pix);

		double grad_mult_factor = pix_mult_factor / (2 * grad_eps);
		double pix_val_inc, pix_val_dec;

		for(int pix_id = 0; pix_id < n_pix; pix_id++){

			pix_val_inc = mapPixVal<mapping_type>(getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img,
				warped_offset_pts(0, pix_id), warped_offset_pts(1, pix_id), h, w), intensity_map);
			pix_val_dec = mapPixVal<mapping_type>(getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img,
				warped_offset_pts(2, pix_id), warped_offset_pts(3, pix_id), h, w), intensity_map);
			warped_img_grad(pix_id, 0) = (pix_val_inc - pix_val_dec)*grad_mult_factor;

			pix_val_inc = mapPixVal<mapping_type>(getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img,
				warped_offset_pts(4, pix_id), warped_offset_pts(5, pix_id), h, w), intensity_map);
			pix_val_dec = mapPixVal<mapping_type>(getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img,
				warped_offset_pts(6, pix_id), warped_offset_pts(7, pix_id), h, w), intensity_map);
			warped_img_grad(pix_id, 1) = (pix_val_inc - pix_val_dec)*grad_mult_factor;
		}
	}
	/***************************************************************/
	/******************** Warp of Image Gradient********************/
	/***************************************************************/

	void getImgGrad(PixGradT &img_grad,
		const EigImgT &img, const PtsT &pts,
		double grad_eps, int n_pix, int h, int w,
		double pix_mult_factor){
		assert(img_grad.rows() == n_pix && pts.cols() == n_pix);

		double grad_mult_factor = pix_mult_factor / (2 * grad_eps);
		double pix_val_inc, pix_val_dec;

		for(int pix_id = 0; pix_id < n_pix; pix_id++){
			double curr_x = pts(0, pix_id), curr_y = pts(1, pix_id);

			pix_val_inc = getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img, curr_x + grad_eps, curr_y, h, w);
			pix_val_dec = getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img, curr_x - grad_eps, curr_y, h, w);
			img_grad(pix_id, 0) = (pix_val_inc - pix_val_dec)*grad_mult_factor;

			pix_val_inc = getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img, curr_x, curr_y + grad_eps, h, w);
			pix_val_dec = getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img, curr_x, curr_y - grad_eps, h, w);

			img_grad(pix_id, 1) = (pix_val_inc - pix_val_dec)*grad_mult_factor;
		}
	}

	/***************************************************************/
	/******************** Hessian of Warped Image ******************/
	/***************************************************************/

	void getWarpedImgHess(PixHessT &warped_img_hess,
		const EigImgT &img, const PtsT &warped_pts,
		const HessPtsT &warped_offset_pts, double hess_eps, int n_pix,
		int h, int w, double pix_mult_factor){
		assert(warped_img_hess.cols() == n_pix && warped_pts.cols() == n_pix);

		double hess_eps2 = 2 * hess_eps;
		double hess_mult_factor = pix_mult_factor / (hess_eps2 * hess_eps2);

		double pix_val_inc, pix_val_dec, pix_val_inc2, pix_val_dec2;

		for(int pix_id = 0; pix_id < n_pix; pix_id++){

			double pix_val = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, warped_pts(0, pix_id), warped_pts(1, pix_id), h, w);

			pix_val_inc = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, warped_offset_pts(0, pix_id), warped_offset_pts(1, pix_id), h, w);
			pix_val_dec = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, warped_offset_pts(2, pix_id), warped_offset_pts(3, pix_id), h, w);
			warped_img_hess(0, pix_id) = (pix_val_inc + pix_val_dec - 2 * pix_val)*hess_mult_factor;

			pix_val_inc = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, warped_offset_pts(4, pix_id), warped_offset_pts(5, pix_id), h, w);
			pix_val_dec = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, warped_offset_pts(6, pix_id), warped_offset_pts(7, pix_id), h, w);
			warped_img_hess(3, pix_id) = (pix_val_inc + pix_val_dec - 2 * pix_val)*hess_mult_factor;

			pix_val_inc = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, warped_offset_pts(8, pix_id), warped_offset_pts(9, pix_id), h, w);
			pix_val_dec = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, warped_offset_pts(10, pix_id), warped_offset_pts(11, pix_id), h, w);
			pix_val_inc2 = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, warped_offset_pts(12, pix_id), warped_offset_pts(13, pix_id), h, w);
			pix_val_dec2 = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, warped_offset_pts(14, pix_id), warped_offset_pts(15, pix_id), h, w);
			warped_img_hess(1, pix_id) = warped_img_hess(2, pix_id) = ((pix_val_inc + pix_val_dec) - (pix_val_inc2 + pix_val_dec2)) * hess_mult_factor;
		}
	}

	// mapped version
	template<InterpType mapping_type>
	void getWarpedImgHess(PixHessT &warped_img_hess,
		const EigImgT &img, const VectorXd &intensity_map,
		const PtsT &warped_pts, const HessPtsT &warped_offset_pts,
		double hess_eps, int n_pix, int h, int w, double pix_mult_factor){
		assert(warped_img_hess.cols() == n_pix && warped_pts.cols() == n_pix);

		double hess_eps2 = 2 * hess_eps;
		double hess_mult_factor = pix_mult_factor / (hess_eps2 * hess_eps2);

		double pix_val_inc, pix_val_dec, pix_val_inc2, pix_val_dec2;

		for(int pix_id = 0; pix_id < n_pix; pix_id++){

			double pix_val = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, warped_pts(0, pix_id),
				warped_pts(1, pix_id), h, w), intensity_map);

			pix_val_inc = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, warped_offset_pts(0, pix_id),
				warped_offset_pts(1, pix_id), h, w), intensity_map);
			pix_val_dec = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, warped_offset_pts(2, pix_id),
				warped_offset_pts(3, pix_id), h, w), intensity_map);
			warped_img_hess(0, pix_id) = (pix_val_inc + pix_val_dec - 2 * pix_val)*hess_mult_factor;

			pix_val_inc = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, warped_offset_pts(4, pix_id),
				warped_offset_pts(5, pix_id), h, w), intensity_map);
			pix_val_dec = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, warped_offset_pts(6, pix_id),
				warped_offset_pts(7, pix_id), h, w), intensity_map);
			warped_img_hess(3, pix_id) = (pix_val_inc + pix_val_dec - 2 * pix_val)*hess_mult_factor;

			pix_val_inc = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, warped_offset_pts(8, pix_id),
				warped_offset_pts(9, pix_id), h, w), intensity_map);
			pix_val_dec = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, warped_offset_pts(10, pix_id),
				warped_offset_pts(11, pix_id), h, w), intensity_map);
			pix_val_inc2 = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, warped_offset_pts(12, pix_id),
				warped_offset_pts(13, pix_id), h, w), intensity_map);
			pix_val_dec2 = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, warped_offset_pts(14, pix_id),
				warped_offset_pts(15, pix_id), h, w), intensity_map);
			warped_img_hess(1, pix_id) = warped_img_hess(2, pix_id) = ((pix_val_inc + pix_val_dec) - (pix_val_inc2 + pix_val_dec2)) * hess_mult_factor;
		}
	}


	/***************************************************************/
	/******************** Warp of Image Hessian********************/
	/***************************************************************/

	void getImgHess(PixHessT &img_hess, const EigImgT &img,
		const PtsT &pts, double hess_eps,
		int n_pix, int h, int w, double pix_mult_factor){
		assert(img_hess.cols() == n_pix && pts.cols() == n_pix);

		double hess_eps2 = 2 * hess_eps;
		double hess_mult_factor = pix_mult_factor / (hess_eps2 * hess_eps2);

		for(int pix_id = 0; pix_id < n_pix; pix_id++){
			double curr_x = pts(0, pix_id), curr_y = pts(1, pix_id);
			double curr_pix_val = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, curr_x, curr_y, h, w);

			double ix_pix_val = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, curr_x + hess_eps2, curr_y, h, w);
			double dx_pix_val = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, curr_x - hess_eps2, curr_y, h, w);
			img_hess(0, pix_id) = (ix_pix_val + dx_pix_val - 2 * curr_pix_val) * hess_mult_factor;

			double iy_pix_val = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, curr_x, curr_y + hess_eps2, h, w);
			double dy_pix_val = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, curr_x, curr_y - hess_eps2, h, w);
			img_hess(3, pix_id) = (iy_pix_val + dy_pix_val - 2 * curr_pix_val) * hess_mult_factor;

			double inc_x = curr_x + hess_eps, dec_x = curr_x - hess_eps;
			double inc_y = curr_y + hess_eps, dec_y = curr_y - hess_eps;
			double ixiy_pix_val = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, inc_x, inc_y, h, w);
			double dxdy_pix_val = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, dec_x, dec_y, h, w);
			double ixdy_pix_val = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, inc_x, dec_y, h, w);
			double iydx_pix_val = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, dec_x, inc_y, h, w);
			img_hess(1, pix_id) = img_hess(2, pix_id) = ((ixiy_pix_val + dxdy_pix_val) - (ixdy_pix_val + iydx_pix_val)) * hess_mult_factor;
		}
	}

#endif
	// mapped version
	template<InterpType mapping_type>
	void getImgGrad(PixGradT &img_grad, const EigImgT &img,
		const VectorXd &intensity_map, const PtsT &pts,
		double grad_eps, int n_pix, int h, int w,
		double pix_mult_factor){
		assert(img_grad.rows() == n_pix && pts.cols() == n_pix);

		double grad_mult_factor = pix_mult_factor / (2 * grad_eps);
		double pix_val_inc, pix_val_dec;
		for(int pix_id = 0; pix_id < n_pix; pix_id++){
			double curr_x = pts(0, pix_id), curr_y = pts(1, pix_id);

			pix_val_inc = mapPixVal<mapping_type>(getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img, curr_x + grad_eps, curr_y, h, w), intensity_map);
			pix_val_dec = mapPixVal<mapping_type>(getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img, curr_x - grad_eps, curr_y, h, w), intensity_map);
			img_grad(pix_id, 0) = (pix_val_inc - pix_val_dec)*grad_mult_factor;

			pix_val_inc = mapPixVal<mapping_type>(getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img, curr_x, curr_y + grad_eps, h, w), intensity_map);
			pix_val_dec = mapPixVal<mapping_type>(getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img, curr_x, curr_y - grad_eps, h, w), intensity_map);

			img_grad(pix_id, 1) = (pix_val_inc - pix_val_dec)*grad_mult_factor;
		}
	}
	// mapped version
	template<InterpType mapping_type>
	void getImgHess(PixHessT &img_hess, const EigImgT &img,
		const VectorXd &intensity_map, const PtsT &pts, double hess_eps,
		int n_pix, int h, int w, double pix_mult_factor){
		assert(img_hess.cols() == n_pix && pts.cols() == n_pix);

		double hess_eps2 = 2 * hess_eps;
		double hess_mult_factor = pix_mult_factor / (hess_eps2 * hess_eps2);

		for(int pix_id = 0; pix_id < n_pix; pix_id++){
			double curr_x = pts(0, pix_id), curr_y = pts(1, pix_id);
			double curr_pix_val = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, curr_x, curr_y, h, w), intensity_map);

			double ix_pix_val = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, curr_x + hess_eps2, curr_y, h, w), intensity_map);
			double dx_pix_val = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, curr_x - hess_eps2, curr_y, h, w), intensity_map);
			img_hess(0, pix_id) = (ix_pix_val + dx_pix_val - 2 * curr_pix_val) * hess_mult_factor;

			double iy_pix_val = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, curr_x, curr_y + hess_eps2, h, w), intensity_map);
			double dy_pix_val = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, curr_x, curr_y - hess_eps2, h, w), intensity_map);
			img_hess(3, pix_id) = (iy_pix_val + dy_pix_val - 2 * curr_pix_val) * hess_mult_factor;

			double inc_x = curr_x + hess_eps, dec_x = curr_x - hess_eps;
			double inc_y = curr_y + hess_eps, dec_y = curr_y - hess_eps;
			double ixiy_pix_val = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, inc_x, inc_y, h, w), intensity_map);
			double dxdy_pix_val = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, dec_x, dec_y, h, w), intensity_map);
			double ixdy_pix_val = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, inc_x, dec_y, h, w), intensity_map);
			double iydx_pix_val = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, dec_x, inc_y, h, w), intensity_map);
			img_hess(1, pix_id) = img_hess(2, pix_id) = ((ixiy_pix_val + dxdy_pix_val) - (ixdy_pix_val + iydx_pix_val)) * hess_mult_factor;
		}
	}
	// computes image hessian analytically by fitting a cubic polynomial surface to a 4x4 grid of pixels around each location
	void getImgHess(PixHessT &img_hess,
		const EigImgT &img, const PtsT &pts,
		int n_pix, int h, int w){
		assert(img_hess.cols() == n_pix && pts.cols() == n_pix);
		Matrix4d neigh_pix_grid, bicubic_coeff;
		for(int pix_id = 0; pix_id < n_pix; pix_id++){

			double x = pts(0, pix_id), y = pts(1, pix_id);

			if(!getNeighboringPixGrid(neigh_pix_grid, img, x, y, h, w)){
				img_hess(0, pix_id) = img_hess(1, pix_id) = img_hess(2, pix_id) = img_hess(3, pix_id) = 0;
				continue;
			}
			getBiCubicCoefficients(bicubic_coeff, neigh_pix_grid);
			double dx = x - static_cast<int>(x);
			double dy = y - static_cast<int>(y);
			img_hess(0, pix_id) = biCubicHessXX(bicubic_coeff, dx, dy);
			img_hess(3, pix_id) = biCubicHessYY(bicubic_coeff, dx, dy);
			img_hess(1, pix_id) = img_hess(2, pix_id) = biCubicHessYX(bicubic_coeff, dx, dy);

			//img_hess(2, pix_id) = biCubicHessXY(bicubic_coeff, dx, dy);
			//printMatrix(neigh_pix_grid, "neigh_pix_grid");
			//printMatrix(bicubic_coeff, "bicubic_coeff");
			//printMatrix(Map<Matrix2d>((double*)img_hess.col(pix_id).data()), "img_hess");
		}
		//printScalarToFile(hess_eps, "hess_eps", "./log/mtf_log.txt", "%e", "a");
		//printMatrixToFile(img_hess, "img_hess", "./log/mtf_log.txt", "%e", "a");
	}

	template<typename PtsT>
	void getPixValsSC(VectorXd &pix_vals,
		const cv::Mat &img, const PtsT &pts, int n_pix, int h, int w,
		double norm_mult, double norm_add){
		//printf("n_pix: %d\t pix_vals.size(): %l\t pts.cols(): %l", n_pix, pix_vals.size(),  pts.cols());
		assert(pix_vals.size() == n_pix && pts.cols() == n_pix);

		for(int i = 0; i < n_pix; i++){
			pix_vals(i) = norm_mult * getPixVal<PIX_INTERP_TYPE, PIX_BORDER_TYPE>(img, pts(0, i), pts(1, i), h, w) + norm_add;
		}
	}

	//--------------------------------------------------------- //
	// ---------------- multi channel versions ---------------- //
	//--------------------------------------------------------- //	

	template<typename PtsT>
	void getPixVals(VectorXd &pix_vals,
		const cv::Mat &img, const PtsT &pts, int n_pix, int h, int w,
		double norm_mult, double norm_add){
		//printf("n_pix: %d\t pix_vals.size(): %l\t pts.cols(): %l", n_pix, pix_vals.size(),  pts.cols());
		assert(pix_vals.size() == 3 * n_pix && pts.cols() == n_pix);
		double *pix_vals_data = pix_vals.data();
		for(int pix_id = 0; pix_id < n_pix; ++pix_id){
			getPixVal<PIX_INTERP_TYPE, PIX_BORDER_TYPE>(pix_vals_data, img,
				pts(0, pix_id), pts(1, pix_id), h, w);
			pix_vals_data[0] = norm_mult*pix_vals_data[0] + norm_add;
			pix_vals_data[1] = norm_mult*pix_vals_data[1] + norm_add;
			pix_vals_data[2] = norm_mult*pix_vals_data[2] + norm_add;
			pix_vals_data += 3;
		}
	}
	void getWarpedImgGrad(PixGradT &warped_img_grad,
		const cv::Mat &img, const Matrix8Xd &warped_offset_pts,
		double grad_eps, int n_pix,
		int h, int w, double pix_mult_factor){
		assert(warped_img_grad.rows() == n_pix * 3 && warped_offset_pts.cols() == n_pix && img.type() == CV_32FC3);

		double grad_mult_factor = pix_mult_factor / (2 * grad_eps);

		int ch_pix_id = 0;
		for(int pix_id = 0; pix_id < n_pix; ++pix_id){
			double pix_val_inc_x[3], pix_val_dec_x[3];
			double pix_val_inc_y[3], pix_val_dec_y[3];

			getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_inc_x, img, warped_offset_pts(0, pix_id),
				warped_offset_pts(1, pix_id), h, w);
			getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dec_x, img, warped_offset_pts(2, pix_id),
				warped_offset_pts(3, pix_id), h, w);
			getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_inc_y, img, warped_offset_pts(4, pix_id),
				warped_offset_pts(5, pix_id), h, w);
			getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dec_y, img, warped_offset_pts(6, pix_id),
				warped_offset_pts(7, pix_id), h, w);
			for(int channel_id = 0; channel_id < 3; ++channel_id){
				warped_img_grad(ch_pix_id, 0) = (pix_val_inc_x[channel_id] - pix_val_dec_x[channel_id])*grad_mult_factor;
				warped_img_grad(ch_pix_id, 1) = (pix_val_inc_y[channel_id] - pix_val_dec_y[channel_id])*grad_mult_factor;
				++ch_pix_id;
			}
		}
		//utils::printMatrixToFile(warped_img_grad, "warped_img_grad", "log/mtf_log.txt", "%15.9f", "a");
	}
	// mapped version
	template<InterpType mapping_type>
	void getWarpedImgGrad(PixGradT &warped_img_grad,
		const cv::Mat &img, const VectorXd &intensity_map,
		const Matrix8Xd &warped_offset_pts, double grad_eps,
		int n_pix, int h, int w, double pix_mult_factor){
		assert(warped_img_grad.rows() == n_pix * 3);

		double grad_mult_factor = pix_mult_factor / (2 * grad_eps);

		int ch_pix_id = 0;
		for(int pix_id = 0; pix_id < n_pix; pix_id++){
			double pix_val_inc_x[3], pix_val_dec_x[3];
			double pix_val_inc_y[3], pix_val_dec_y[3];
			getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_inc_x, img,
				warped_offset_pts(0, pix_id), warped_offset_pts(1, pix_id), h, w);
			getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dec_x, img,
				warped_offset_pts(2, pix_id), warped_offset_pts(3, pix_id), h, w);
			getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_inc_y, img,
				warped_offset_pts(4, pix_id), warped_offset_pts(5, pix_id), h, w);
			getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dec_y, img,
				warped_offset_pts(6, pix_id), warped_offset_pts(7, pix_id), h, w);
			for(int channel_id = 0; channel_id < 3; ++channel_id){
				warped_img_grad(ch_pix_id, 0) = (mapPixVal<mapping_type>(pix_val_inc_x[channel_id], intensity_map)
					- mapPixVal<mapping_type>(pix_val_dec_x[channel_id], intensity_map))*grad_mult_factor;
				warped_img_grad(ch_pix_id, 1) = (mapPixVal<mapping_type>(pix_val_inc_y[channel_id], intensity_map)
					- mapPixVal<mapping_type>(pix_val_dec_y[channel_id], intensity_map))*grad_mult_factor;
				++ch_pix_id;
			}
		}
	}
	void getImgGrad(PixGradT &img_grad,
		const cv::Mat &img, const PtsT &pts,
		double grad_eps, int n_pix, int h, int w,
		double pix_mult_factor){
		assert(img_grad.rows() == n_pix * 3 && pts.cols() == n_pix && img.type() == CV_32FC3);

		double grad_mult_factor = pix_mult_factor / (2 * grad_eps);

		int ch_pix_id = 0;

		for(int pix_id = 0; pix_id < n_pix; pix_id++){
			double curr_x = pts(0, pix_id), curr_y = pts(1, pix_id);

			double pix_val_inc_x[3], pix_val_dec_x[3];
			double pix_val_inc_y[3], pix_val_dec_y[3];

			getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_inc_x, img, curr_x + grad_eps, curr_y, h, w);
			getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dec_x, img, curr_x - grad_eps, curr_y, h, w);

			getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_inc_y, img, curr_x, curr_y + grad_eps, h, w);
			getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dec_y, img, curr_x, curr_y - grad_eps, h, w);

			for(int channel_id = 0; channel_id < 3; ++channel_id){
				img_grad(ch_pix_id, 0) = (pix_val_inc_x[channel_id] - pix_val_dec_x[channel_id])*grad_mult_factor;
				img_grad(ch_pix_id, 1) = (pix_val_inc_y[channel_id] - pix_val_dec_y[channel_id])*grad_mult_factor;
				++ch_pix_id;
			}
		}
	}
	// mapped version
	template<InterpType mapping_type>
	void getImgGrad(PixGradT &img_grad, const cv::Mat &img,
		const VectorXd &intensity_map, const PtsT &pts,
		double grad_eps, int n_pix, int h, int w,
		double pix_mult_factor){
		assert(img_grad.rows() == n_pix * 3 && pts.cols() == n_pix);

		double grad_mult_factor = pix_mult_factor / (2 * grad_eps);

		int ch_pix_id = 0;
		for(int pix_id = 0; pix_id < n_pix; pix_id++){
			double curr_x = pts(0, pix_id), curr_y = pts(1, pix_id);
			double pix_val_inc_x[3], pix_val_dec_x[3];
			double pix_val_inc_y[3], pix_val_dec_y[3];

			getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_inc_x, img, curr_x + grad_eps, curr_y, h, w);
			getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dec_x, img, curr_x - grad_eps, curr_y, h, w);
			getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_inc_y, img, curr_x, curr_y + grad_eps, h, w);
			getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dec_y, img, curr_x, curr_y - grad_eps, h, w);

			for(int channel_id = 0; channel_id < 3; ++channel_id){
				img_grad(ch_pix_id, 0) = (mapPixVal<mapping_type>(pix_val_inc_x[channel_id], intensity_map) -
					mapPixVal<mapping_type>(pix_val_dec_x[channel_id], intensity_map))*grad_mult_factor;
				img_grad(ch_pix_id, 1) = (mapPixVal<mapping_type>(pix_val_inc_y[channel_id], intensity_map) -
					mapPixVal<mapping_type>(pix_val_dec_y[channel_id], intensity_map))*grad_mult_factor;
				++ch_pix_id;
			}
		}
	}
	void getWarpedImgHess(PixHessT &warped_img_hess,
		const cv::Mat &img, const PtsT &warped_pts,
		const HessPtsT &warped_offset_pts, double hess_eps, int n_pix,
		int h, int w, double pix_mult_factor){
		assert(warped_img_hess.cols() == n_pix * 3 && warped_pts.cols() == n_pix && img.type() == CV_32FC3);

		double hess_eps2 = 2 * hess_eps;
		double hess_mult_factor = pix_mult_factor / (hess_eps2 * hess_eps2);

		int ch_pix_id = 0;

		for(int pix_id = 0; pix_id < n_pix; pix_id++){
			double pix_val[3];
			double pix_val_inc_x[3], pix_val_dec_x[3];
			double pix_val_inc_y[3], pix_val_dec_y[3];
			double pix_val_inc_xy[3], pix_val_dec_xy[3];
			double pix_val_inc_yx[3], pix_val_dec_yx[3];

			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val, img, warped_pts(0, pix_id), warped_pts(1, pix_id), h, w);

			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_inc_x, img, warped_offset_pts(0, pix_id), warped_offset_pts(1, pix_id), h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dec_x, img, warped_offset_pts(2, pix_id), warped_offset_pts(3, pix_id), h, w);

			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_inc_y, img, warped_offset_pts(4, pix_id), warped_offset_pts(5, pix_id), h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dec_y, img, warped_offset_pts(6, pix_id), warped_offset_pts(7, pix_id), h, w);

			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_inc_xy, img, warped_offset_pts(8, pix_id), warped_offset_pts(9, pix_id), h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dec_xy, img, warped_offset_pts(10, pix_id), warped_offset_pts(11, pix_id), h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_inc_yx, img, warped_offset_pts(12, pix_id), warped_offset_pts(13, pix_id), h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dec_yx, img, warped_offset_pts(14, pix_id), warped_offset_pts(15, pix_id), h, w);

			for(int channel_id = 0; channel_id < 3; ++channel_id){
				warped_img_hess(0, ch_pix_id) = (pix_val_inc_x[channel_id] + pix_val_dec_x[channel_id] - 2 * pix_val[channel_id])*hess_mult_factor;
				warped_img_hess(3, ch_pix_id) = (pix_val_inc_y[channel_id] + pix_val_dec_y[channel_id] - 2 * pix_val[channel_id])*hess_mult_factor;
				warped_img_hess(1, ch_pix_id) = warped_img_hess(2, ch_pix_id) = ((pix_val_inc_xy[channel_id] + pix_val_dec_xy[channel_id]) -
					(pix_val_inc_yx[channel_id] + pix_val_dec_yx[channel_id])) * hess_mult_factor;
				++ch_pix_id;
			}
		}
	}
	// mapped version
	template<InterpType mapping_type>
	void getWarpedImgHess(PixHessT &warped_img_hess,
		const cv::Mat &img, const VectorXd &intensity_map,
		const PtsT &warped_pts, const HessPtsT &warped_offset_pts,
		double hess_eps, int n_pix, int h, int w, double pix_mult_factor){
		assert(warped_img_hess.cols() == n_pix && warped_pts.cols() == n_pix);

		double hess_eps2 = 2 * hess_eps;
		double hess_mult_factor = pix_mult_factor / (hess_eps2 * hess_eps2);

		int ch_pix_id = 0;
		for(int pix_id = 0; pix_id < n_pix; pix_id++){

			double pix_val[3];
			double pix_val_inc_x[3], pix_val_dec_x[3];
			double pix_val_inc_y[3], pix_val_dec_y[3];
			double pix_val_inc_xy[3], pix_val_dec_xy[3];
			double pix_val_inc_yx[3], pix_val_dec_yx[3];

			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val, img, warped_pts(0, pix_id), warped_pts(1, pix_id), h, w);

			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_inc_x, img, warped_offset_pts(0, pix_id), warped_offset_pts(1, pix_id), h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dec_x, img, warped_offset_pts(2, pix_id), warped_offset_pts(3, pix_id), h, w);

			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_inc_y, img, warped_offset_pts(4, pix_id), warped_offset_pts(5, pix_id), h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dec_y, img, warped_offset_pts(6, pix_id), warped_offset_pts(7, pix_id), h, w);

			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_inc_xy, img, warped_offset_pts(8, pix_id), warped_offset_pts(9, pix_id), h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dec_xy, img, warped_offset_pts(10, pix_id), warped_offset_pts(11, pix_id), h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_inc_yx, img, warped_offset_pts(12, pix_id), warped_offset_pts(13, pix_id), h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dec_yx, img, warped_offset_pts(14, pix_id), warped_offset_pts(15, pix_id), h, w);

			for(int channel_id = 0; channel_id < 3; ++channel_id){
				warped_img_hess(0, ch_pix_id) = (mapPixVal<mapping_type>(pix_val_inc_x[channel_id], intensity_map) +
					mapPixVal<mapping_type>(pix_val_dec_x[channel_id], intensity_map) -
					2 * mapPixVal<mapping_type>(pix_val[channel_id], intensity_map))*hess_mult_factor;
				warped_img_hess(3, ch_pix_id) = (mapPixVal<mapping_type>(pix_val_inc_y[channel_id], intensity_map) +
					mapPixVal<mapping_type>(pix_val_dec_y[channel_id], intensity_map) -
					2 * mapPixVal<mapping_type>(pix_val[channel_id], intensity_map))*hess_mult_factor;
				warped_img_hess(1, ch_pix_id) = warped_img_hess(2, ch_pix_id) = (
					(mapPixVal<mapping_type>(pix_val_inc_xy[channel_id], intensity_map) +
					mapPixVal<mapping_type>(pix_val_dec_xy[channel_id], intensity_map))
					-
					(mapPixVal<mapping_type>(pix_val_inc_yx[channel_id], intensity_map) +
					mapPixVal<mapping_type>(pix_val_dec_yx[channel_id], intensity_map))) * hess_mult_factor;
				++ch_pix_id;
			}
		}
	}
	void getImgHess(PixHessT &img_hess, const cv::Mat &img,
		const PtsT &pts, double hess_eps,
		int n_pix, int h, int w, double pix_mult_factor){
		assert(img_hess.cols() == n_pix * 3 && pts.cols() == n_pix && img.type() == CV_32FC3);

		double hess_eps2 = 2 * hess_eps;
		double hess_mult_factor = pix_mult_factor / (hess_eps2 * hess_eps2);

		int ch_pix_id = 0;
		for(int pix_id = 0; pix_id < n_pix; ++pix_id){
			double pix_val[3];
			double pix_val_inc_x[3], pix_val_dec_x[3];
			double pix_val_inc_y[3], pix_val_dec_y[3];
			double pix_val_inc_xy[3], pix_val_dec_xy[3];
			double pix_val_inc_yx[3], pix_val_dec_yx[3];

			double curr_x = pts(0, pix_id), curr_y = pts(1, pix_id);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val, img, curr_x, curr_y, h, w);

			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_inc_x, img, curr_x + hess_eps2, curr_y, h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dec_x, img, curr_x - hess_eps2, curr_y, h, w);

			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_inc_y, img, curr_x, curr_y + hess_eps2, h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dec_y, img, curr_x, curr_y - hess_eps2, h, w);

			double inc_x = curr_x + hess_eps, dec_x = curr_x - hess_eps;
			double inc_y = curr_y + hess_eps, dec_y = curr_y - hess_eps;
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_inc_xy, img, inc_x, inc_y, h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dec_xy, img, dec_x, dec_y, h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_inc_yx, img, inc_x, dec_y, h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dec_yx, img, dec_x, inc_y, h, w);

			for(int channel_id = 0; channel_id < 3; ++channel_id){
				img_hess(0, ch_pix_id) = (pix_val_inc_x[channel_id] + pix_val_dec_x[channel_id] - 2 * pix_val[channel_id]) * hess_mult_factor;
				img_hess(3, ch_pix_id) = (pix_val_inc_y[channel_id] + pix_val_dec_y[channel_id] - 2 * pix_val[channel_id]) * hess_mult_factor;
				img_hess(1, ch_pix_id) = img_hess(2, ch_pix_id) = ((pix_val_inc_xy[channel_id] + pix_val_dec_xy[channel_id]) -
					(pix_val_inc_yx[channel_id] + pix_val_dec_yx[channel_id])) * hess_mult_factor;
				++ch_pix_id;
			}
		}
	}
	// mapped version
	template<InterpType mapping_type>
	void getImgHess(PixHessT &img_hess, const cv::Mat &img,
		const VectorXd &intensity_map, const PtsT &pts, double hess_eps,
		int n_pix, int h, int w, double pix_mult_factor){
		assert(img_hess.cols() == n_pix * 3 && pts.cols() == n_pix);

		double hess_eps2 = 2 * hess_eps;
		double hess_mult_factor = pix_mult_factor / (hess_eps2 * hess_eps2);

		int ch_pix_id = 0;
		for(int pix_id = 0; pix_id < n_pix; pix_id++){
			double curr_x = pts(0, pix_id), curr_y = pts(1, pix_id);
			double inc_x = curr_x + hess_eps, dec_x = curr_x - hess_eps;
			double inc_y = curr_y + hess_eps, dec_y = curr_y - hess_eps;

			double pix_val[3];
			double pix_val_inc_x[3], pix_val_dec_x[3];
			double pix_val_inc_y[3], pix_val_dec_y[3];
			double pix_val_ixiy[3], pix_val_dxdy[3];
			double pix_val_ixdy[3], pix_val_dxiy[3];

			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val, img, curr_x, curr_y, h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_inc_x, img, curr_x + hess_eps2, curr_y, h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dec_x, img, curr_x - hess_eps2, curr_y, h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_inc_y, img, curr_x, curr_y + hess_eps2, h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dec_y, img, curr_x, curr_y - hess_eps2, h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_ixiy, img, inc_x, inc_y, h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dxdy, img, dec_x, dec_y, h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_ixdy, img, inc_x, dec_y, h, w);
			getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(pix_val_dxiy, img, dec_x, inc_y, h, w);

			for(int channel_id = 0; channel_id < 3; ++channel_id){
				img_hess(0, ch_pix_id) = (mapPixVal<mapping_type>(pix_val_inc_x[channel_id], intensity_map) +
					mapPixVal<mapping_type>(pix_val_dec_x[channel_id], intensity_map) - 2 * mapPixVal<mapping_type>(pix_val[channel_id], intensity_map)) * hess_mult_factor;

				img_hess(3, ch_pix_id) = (mapPixVal<mapping_type>(pix_val_inc_y[channel_id], intensity_map) +
					mapPixVal<mapping_type>(pix_val_dec_y[channel_id], intensity_map) -
					2 * mapPixVal<mapping_type>(pix_val[channel_id], intensity_map)) * hess_mult_factor;
				img_hess(1, ch_pix_id) = img_hess(2, ch_pix_id) = (
					(mapPixVal<mapping_type>(pix_val_ixiy[channel_id], intensity_map) +
					mapPixVal<mapping_type>(pix_val_dxdy[channel_id], intensity_map))
					-
					(mapPixVal<mapping_type>(pix_val_ixdy[channel_id], intensity_map) +
					mapPixVal<mapping_type>(pix_val_dxiy[channel_id], intensity_map))
					) * hess_mult_factor;
				++ch_pix_id;
			}
		}
	}


	// convenience functions for mapping
	void getWarpedImgGrad(PixGradT &warped_img_grad, const EigImgT &img,
		bool weighted_mapping, const VectorXd &intensity_map, const Matrix8Xd &warped_offset_pts,
		double grad_eps, int n_pix, int h, int w, double pix_mult_factor){
		if(weighted_mapping){
			utils::getWarpedImgGrad<utils::InterpType::Linear>(warped_img_grad,
				img, intensity_map, warped_offset_pts, grad_eps, n_pix, h, w,
				pix_mult_factor);
		} else{
			utils::getWarpedImgGrad<utils::InterpType::Nearest>(warped_img_grad,
				img, intensity_map, warped_offset_pts, grad_eps, n_pix, h, w,
				pix_mult_factor);
		}
	}

	void getWarpedImgHess(PixHessT &warped_img_hess, const EigImgT &img,
		bool weighted_mapping, const VectorXd &intensity_map,
		const PtsT &warped_pts, const Matrix16Xd &warped_offset_pts,
		double hess_eps, int n_pix, int h, int w, double pix_mult_factor){
		if(weighted_mapping){
			utils::getWarpedImgHess<utils::InterpType::Linear>(warped_img_hess,
				img, intensity_map, warped_pts, warped_offset_pts,
				hess_eps, n_pix, h, w, pix_mult_factor);
		} else{
			utils::getWarpedImgHess<utils::InterpType::Nearest>(warped_img_hess,
				img, intensity_map, warped_pts, warped_offset_pts,
				hess_eps, n_pix, h, w, pix_mult_factor);
		}
	}
	void getImgGrad(PixGradT &img_grad, const EigImgT &img,
		bool weighted_mapping, const VectorXd &intensity_map,
		const PtsT &pts, double grad_eps, int n_pix, int h, int w,
		double pix_mult_factor){
		if(weighted_mapping){
			utils::getImgGrad<utils::InterpType::Linear>(img_grad,
				img, intensity_map, pts, grad_eps,
				n_pix, h, w, pix_mult_factor);
		} else{
			utils::getImgGrad<utils::InterpType::Nearest>(img_grad,
				img, intensity_map, pts, grad_eps,
				n_pix, h, w, pix_mult_factor);
		}
	}
	void getImgHess(PixHessT &img_hess, const EigImgT &img, bool weighted_mapping,
		const VectorXd &intensity_map, const PtsT &pts, double hess_eps, int n_pix,
		int h, int w, double pix_mult_factor){
		if(weighted_mapping){
			utils::getImgHess<utils::InterpType::Linear>(img_hess,
				img, intensity_map, pts, hess_eps, n_pix, h, w, pix_mult_factor);
		} else{
			utils::getImgHess<utils::InterpType::Nearest>(img_hess,
				img, intensity_map, pts, hess_eps, n_pix, h, w, pix_mult_factor);
		}
	}

	void getWarpedImgGrad(PixGradT &warped_img_grad, const cv::Mat &img,
		bool weighted_mapping, const VectorXd &intensity_map,
		const Matrix8Xd &warped_offset_pts, double grad_eps,
		int n_pix, int h, int w, double pix_mult_factor){
		if(weighted_mapping){
			getWarpedImgGrad<InterpType::Linear>(warped_img_grad,
				img, intensity_map, warped_offset_pts, grad_eps,
				n_pix, h, w, pix_mult_factor);
		} else{
			getWarpedImgGrad<InterpType::Nearest>(warped_img_grad,
				img, intensity_map, warped_offset_pts, grad_eps,
				n_pix, h, w, pix_mult_factor);
		}
	}
	void getImgGrad(PixGradT &img_grad, const cv::Mat &img,
		bool weighted_mapping, const VectorXd &intensity_map,
		const PtsT &pts, double grad_eps, int n_pix, int h, int w,
		double pix_mult_factor){
		if(weighted_mapping){
			utils::getImgGrad<utils::InterpType::Linear>(img_grad,
				img, intensity_map, pts, grad_eps,
				n_pix, h, w, pix_mult_factor);
		} else{
			utils::getImgGrad<utils::InterpType::Nearest>(img_grad,
				img, intensity_map, pts, grad_eps,
				n_pix, h, w, pix_mult_factor);
		}
	}
	void getWarpedImgHess(PixHessT &warped_img_hess,
		const cv::Mat &img, bool weighted_mapping, const VectorXd &intensity_map,
		const PtsT &warped_pts, const HessPtsT &warped_offset_pts,
		double hess_eps, int n_pix, int h, int w, double pix_mult_factor){
		if(weighted_mapping){
			getWarpedImgHess<InterpType::Linear>(warped_img_hess,
				img, intensity_map,
				warped_pts, warped_offset_pts,
				hess_eps, n_pix, h, w, pix_mult_factor);
		} else{
			getWarpedImgHess<InterpType::Nearest>(warped_img_hess,
				img, intensity_map,
				warped_pts, warped_offset_pts,
				hess_eps, n_pix, h, w, pix_mult_factor);
		}
	}
	void getImgHess(PixHessT &img_hess, const cv::Mat &img, bool weighted_mapping,
		const VectorXd &intensity_map, const PtsT &pts, double hess_eps,
		int n_pix, int h, int w, double pix_mult_factor){
		if(weighted_mapping){
			getImgHess<InterpType::Linear>(img_hess, img, intensity_map, pts,
				hess_eps, n_pix, h, w, pix_mult_factor);
		} else{
			getImgHess<InterpType::Nearest>(img_hess, img, intensity_map, pts,
				hess_eps, n_pix, h, w, pix_mult_factor);
		}
	}

	// ******************************************************************************* //
	// ************ functions for cubic BSpline and bicubic interpolation ************ //
	// ******************************************************************************* //

	bool getNeighboringPixGrid(Matrix4d &pix_grid, const EigImgT &img, double x, double y,
		int h, int w){
		assert(img.rows() == h && img.cols() == w);

		if(checkOverflow(x, y, h, w)){
			return false;
		}
		int x1 = static_cast<int>(x);
		int y1 = static_cast<int>(y);

		int x2 = x1 + 1;
		int y2 = y1 + 1;

		if(checkOverflow(x1, y1, h, w) || checkOverflow(x2, y2, h, w)){
			return false;
		}
		pix_grid(1, 1) = img(y1, x1), pix_grid(1, 2) = img(y1, x2);
		pix_grid(2, 1) = img(y2, x1), pix_grid(2, 2) = img(y2, x2);

		int x0 = x1 - 1, x3 = x2 + 1;
		int y0 = y1 - 1, y3 = y2 + 1;

		bool ul = true, ur = true, lr = true, ll = true;

		if(y0 < 0){
			ul = ur = false;
			pix_grid(0, 1) = pix_grid(1, 1);
			pix_grid(0, 2) = pix_grid(1, 2);
		} else{
			pix_grid(0, 1) = img(y0, x1);
			pix_grid(0, 2) = img(y0, x2);
		}
		if(y3 >= h){
			ll = lr = false;
			pix_grid(3, 1) = pix_grid(2, 1);
			pix_grid(3, 2) = pix_grid(2, 2);
		} else{
			pix_grid(3, 1) = img(y3, x1);
			pix_grid(3, 2) = img(y3, x2);
		}
		if(x0 < 0){
			ul = ll = false;
			pix_grid(1, 0) = pix_grid(1, 1);
			pix_grid(2, 0) = pix_grid(2, 1);
		} else{
			pix_grid(1, 0) = img(y1, x0);
			pix_grid(2, 0) = img(y2, x0);
		}
		if(x3 >= w){
			ur = lr = false;
			pix_grid(1, 3) = pix_grid(1, 2);
			pix_grid(2, 3) = pix_grid(2, 2);
		} else{
			pix_grid(1, 3) = img(y1, x3);
			pix_grid(2, 3) = img(y2, x3);
		}

		pix_grid(0, 0) = ul ? img(y0, x0) : pix_grid(1, 1);
		pix_grid(0, 3) = ur ? img(y0, x3) : pix_grid(1, 2);
		pix_grid(3, 3) = lr ? img(y3, x3) : pix_grid(2, 2);
		pix_grid(3, 0) = ll ? img(y3, x0) : pix_grid(2, 1);

		return true;
	}
	void getBiCubicCoefficients(Matrix4d &bicubic_coeff, const Matrix4d &pix_grid){
		bicubic_coeff(0, 0) = pix_grid(1, 1);
		bicubic_coeff(0, 1) = -.5*pix_grid(1, 0) + .5*pix_grid(1, 2);
		bicubic_coeff(0, 2) = pix_grid(1, 0) - 2.5*pix_grid(1, 1) + 2 * pix_grid(1, 2) - .5*pix_grid(1, 3);
		bicubic_coeff(0, 3) = -.5*pix_grid(1, 0) + 1.5*pix_grid(1, 1) - 1.5*pix_grid(1, 2) + .5*pix_grid(1, 3);
		bicubic_coeff(1, 0) = -.5*pix_grid(0, 1) + .5*pix_grid(2, 1);
		bicubic_coeff(1, 1) = .25*pix_grid(0, 0) - .25*pix_grid(0, 2) - .25*pix_grid(2, 0) + .25*pix_grid(2, 2);
		bicubic_coeff(1, 2) = -.5*pix_grid(0, 0) + 1.25*pix_grid(0, 1) - pix_grid(0, 2) + .25*pix_grid(0, 3) + .5*pix_grid(2, 0) - 1.25*pix_grid(2, 1) + pix_grid(2, 2) - .25*pix_grid(2, 3);
		bicubic_coeff(1, 3) = .25*pix_grid(0, 0) - .75*pix_grid(0, 1) + .75*pix_grid(0, 2) - .25*pix_grid(0, 3) - .25*pix_grid(2, 0) + .75*pix_grid(2, 1) - .75*pix_grid(2, 2) + .25*pix_grid(2, 3);
		bicubic_coeff(2, 0) = pix_grid(0, 1) - 2.5*pix_grid(1, 1) + 2 * pix_grid(2, 1) - .5*pix_grid(3, 1);
		bicubic_coeff(2, 1) = -.5*pix_grid(0, 0) + .5*pix_grid(0, 2) + 1.25*pix_grid(1, 0) - 1.25*pix_grid(1, 2) - pix_grid(2, 0) + pix_grid(2, 2) + .25*pix_grid(3, 0) - .25*pix_grid(3, 2);
		bicubic_coeff(2, 2) = pix_grid(0, 0) - 2.5*pix_grid(0, 1) + 2 * pix_grid(0, 2) - .5*pix_grid(0, 3) - 2.5*pix_grid(1, 0) + 6.25*pix_grid(1, 1) - 5 * pix_grid(1, 2) + 1.25*pix_grid(1, 3) + 2 * pix_grid(2, 0) - 5 * pix_grid(2, 1) + 4 * pix_grid(2, 2) - pix_grid(2, 3) - .5*pix_grid(3, 0) + 1.25*pix_grid(3, 1) - pix_grid(3, 2) + .25*pix_grid(3, 3);
		bicubic_coeff(2, 3) = -.5*pix_grid(0, 0) + 1.5*pix_grid(0, 1) - 1.5*pix_grid(0, 2) + .5*pix_grid(0, 3) + 1.25*pix_grid(1, 0) - 3.75*pix_grid(1, 1) + 3.75*pix_grid(1, 2) - 1.25*pix_grid(1, 3) - pix_grid(2, 0) + 3 * pix_grid(2, 1) - 3 * pix_grid(2, 2) + pix_grid(2, 3) + .25*pix_grid(3, 0) - .75*pix_grid(3, 1) + .75*pix_grid(3, 2) - .25*pix_grid(3, 3);
		bicubic_coeff(3, 0) = -.5*pix_grid(0, 1) + 1.5*pix_grid(1, 1) - 1.5*pix_grid(2, 1) + .5*pix_grid(3, 1);
		bicubic_coeff(3, 1) = .25*pix_grid(0, 0) - .25*pix_grid(0, 2) - .75*pix_grid(1, 0) + .75*pix_grid(1, 2) + .75*pix_grid(2, 0) - .75*pix_grid(2, 2) - .25*pix_grid(3, 0) + .25*pix_grid(3, 2);
		bicubic_coeff(3, 2) = -.5*pix_grid(0, 0) + 1.25*pix_grid(0, 1) - pix_grid(0, 2) + .25*pix_grid(0, 3) + 1.5*pix_grid(1, 0) - 3.75*pix_grid(1, 1) + 3 * pix_grid(1, 2) - .75*pix_grid(1, 3) - 1.5*pix_grid(2, 0) + 3.75*pix_grid(2, 1) - 3 * pix_grid(2, 2) + .75*pix_grid(2, 3) + .5*pix_grid(3, 0) - 1.25*pix_grid(3, 1) + pix_grid(3, 2) - .25*pix_grid(3, 3);
		bicubic_coeff(3, 3) = .25*pix_grid(0, 0) - .75*pix_grid(0, 1) + .75*pix_grid(0, 2) - .25*pix_grid(0, 3) - .75*pix_grid(1, 0) + 2.25*pix_grid(1, 1) - 2.25*pix_grid(1, 2) + .75*pix_grid(1, 3) + .75*pix_grid(2, 0) - 2.25*pix_grid(2, 1) + 2.25*pix_grid(2, 2) - .75*pix_grid(2, 3) - .25*pix_grid(3, 0) + .75*pix_grid(3, 1) - .75*pix_grid(3, 2) + .25*pix_grid(3, 3);
	}
	//evaluates the cubic polynomial surface defines by the given parameters at the given location
	double biCubic(const Matrix4d &bicubic_coeff, double x, double y) {
		double x2 = x * x;
		double x3 = x2 * x;
		double y2 = y * y;
		double y3 = y2 * y;
		return (bicubic_coeff(0, 0) + bicubic_coeff(0, 1) * y + bicubic_coeff(0, 2) * y2 + bicubic_coeff(0, 3) * y3) +
			(bicubic_coeff(1, 0) + bicubic_coeff(1, 1) * y + bicubic_coeff(1, 2) * y2 + bicubic_coeff(1, 3) * y3) * x +
			(bicubic_coeff(2, 0) + bicubic_coeff(2, 1) * y + bicubic_coeff(2, 2)* y2 + bicubic_coeff(2, 3) * y3) * x2 +
			(bicubic_coeff(3, 0) + bicubic_coeff(3, 1) * y + bicubic_coeff(3, 2) * y2 + bicubic_coeff(3, 3) * y3) * x3;
	}
	//evaluates the gradient of cubic polynomial surface defines by the given parameters at the given location
	double biCubicGradX(Vector2d &grad, const Matrix4d &bicubic_coeff, double x, double y) {
		double x2 = x * x;
		double y2 = y * y;
		double y3 = y2 * y;
		return (bicubic_coeff(1, 0) + bicubic_coeff(1, 1) * y + bicubic_coeff(1, 2) * y2 + bicubic_coeff(1, 3) * y3) +
			(bicubic_coeff(2, 0) + bicubic_coeff(2, 1) * y + bicubic_coeff(2, 2)* y2 + bicubic_coeff(2, 3) * y3) * 2 * x +
			(bicubic_coeff(3, 0) + bicubic_coeff(3, 1) * y + bicubic_coeff(3, 2) * y2 + bicubic_coeff(3, 3) * y3) * 3 * x2;
	}
	double biCubicGradY(Vector2d &grad, const Matrix4d &bicubic_coeff, double x, double y) {
		double x2 = x * x;
		double x3 = x2 * x;
		double y2 = y * y;
		return (bicubic_coeff(0, 1) + bicubic_coeff(1, 1) * x + bicubic_coeff(2, 1) * x2 + bicubic_coeff(3, 1) * x3) +
			(bicubic_coeff(0, 2) + bicubic_coeff(1, 2) * x + bicubic_coeff(2, 2)* x2 + bicubic_coeff(3, 2) * x3) * 2 * y +
			(bicubic_coeff(0, 3) + bicubic_coeff(1, 3) * x + bicubic_coeff(2, 3) * x2 + bicubic_coeff(3, 3) * x3) * 3 * y2;
	}
	//evaluates the hessian of the cubic polynomial surface defines by the given parameters at the given location
	double biCubicHessXX(const Matrix4d &bicubic_coeff, double x, double y) {
		double y2 = y * y;
		double y3 = y2 * y;
		return 2 * (bicubic_coeff(2, 0) + bicubic_coeff(2, 1) * y + bicubic_coeff(2, 2)* y2 + bicubic_coeff(2, 3) * y3) +
			6 * x*(bicubic_coeff(3, 0) + bicubic_coeff(3, 1) * y + bicubic_coeff(3, 2) * y2 + bicubic_coeff(3, 3) * y3);
	}
	double biCubicHessYY(const Matrix4d &bicubic_coeff, double x, double y) {
		double x2 = x * x;
		double x3 = x2 * x;
		return 2 * (bicubic_coeff(0, 2) + bicubic_coeff(1, 2) * x + bicubic_coeff(2, 2)* x2 + bicubic_coeff(3, 2) * x3) +
			6 * y*(bicubic_coeff(0, 3) + bicubic_coeff(1, 3) * x + bicubic_coeff(2, 3) * x2 + bicubic_coeff(3, 3) * x3);
	}
	double biCubicHessYX(const Matrix4d &bicubic_coeff, double x, double y) {
		double x2 = x * x;
		double y2 = y * y;
		double y2d = 2 * y, y3d = 3 * y2;
		return (bicubic_coeff(1, 1) + bicubic_coeff(1, 2) * y2d + bicubic_coeff(1, 3) * y3d) +
			(bicubic_coeff(2, 1) + bicubic_coeff(2, 2)* y2d + bicubic_coeff(2, 3) * y3d) * 2 * x +
			(bicubic_coeff(3, 1) + bicubic_coeff(3, 2) * y2d + bicubic_coeff(3, 3) * y3d) * 3 * x2;
	}
	double biCubicHessXY(const Matrix4d &bicubic_coeff, double x, double y) {
		double y2 = y * y;
		double y3 = y * y2;
		double x2 = x * x;
		double x2d = 2 * x, x3d = 3 * x2;
		return (bicubic_coeff(1, 1) + bicubic_coeff(2, 1) * x2d + bicubic_coeff(3, 1) * x3d) +
			(bicubic_coeff(1, 2) + bicubic_coeff(2, 2)* x2d + bicubic_coeff(3, 2) * x3d) * 2 * y +
			(bicubic_coeff(1, 3) + bicubic_coeff(2, 3) * x2d + bicubic_coeff(3, 3) * x3d) * 3 * y2;
	}

	cv::Mat convertFloatImgToUchar(cv::Mat &img, int nchannels){
		cv::Mat imgU;
		double minVal;
		double maxVal;
		cv::Point minLoc;
		cv::Point maxLoc;
		if(nchannels == 1){
			cv::minMaxLoc(img, &minVal, &maxVal, &minLoc, &maxLoc);
			img -= minVal;
			img.convertTo(imgU, CV_8UC1, 255.0 / (maxVal - minVal));
		} else {
			cv::Mat planes[3];
			cv::split(img, planes);
			cv::minMaxLoc(planes[0], &minVal, &maxVal, &minLoc, &maxLoc);
			img.convertTo(imgU, CV_8UC3, 255.0 / (maxVal - minVal));
		}
		return imgU;
	}

	void generateWarpedImg(cv::Mat &warped_img, const cv::Mat &warped_corners,
		const mtf::PtsT &warped_pts, const mtf::PixValT orig_patch,
		const cv::Mat &orig_img, int img_width, int img_height,
		int n_pts, int background_type, bool show_warped_img, const char* win_name){

		double inf = std::numeric_limits<double>::infinity();
		double max_x = -inf, min_x = inf, max_y = -inf, min_y = inf;
		for(int pt_id = 0; pt_id < n_pts; ++pt_id){
			if(warped_pts(0, pt_id) > max_x){
				max_x = warped_pts(0, pt_id);
			}
			if(warped_pts(0, pt_id) < min_x){
				min_x = warped_pts(0, pt_id);
			}
			if(warped_pts(1, pt_id) > max_y){
				max_y = warped_pts(1, pt_id);
			}
			if(warped_pts(1, pt_id) < min_y){
				min_y = warped_pts(1, pt_id);
			}
		}
		printf("min_x: %f max_x: %f min_y: %f max_y: %f\n", min_x, max_x, min_y, max_y);

		for(int row_id = 0; row_id < img_height; ++row_id){
			for(int col_id = 0; col_id < img_width; ++col_id){
				if(row_id < min_y || row_id > max_y ||
					col_id < min_x || col_id > max_x ||
					!mtf::utils::isInsideRegion(warped_corners, col_id, row_id)){
					if(background_type){
						if(warped_img.type() == CV_8UC3){
							warped_img.at<cv::Vec3b>(row_id, col_id) =
								orig_img.at<cv::Vec3f>(row_id, col_id);
						} else{
							warped_img.at<uchar>(row_id, col_id) =
								static_cast<uchar>(orig_img.at<float>(row_id, col_id));
						}
					}
				} else{
					cv::Vec4i neigh_pts_id;
					std::vector<cv::Vec2d> neigh_pts_dist(4);
					mtf::utils::getBilinearPts(neigh_pts_id, neigh_pts_dist, col_id, row_id, warped_pts, n_pts);

					if(neigh_pts_id[0] < 0 || neigh_pts_id[1] < 0 || neigh_pts_id[2] < 0 || neigh_pts_id[3] < 0){
						printf("neigh_pts_id: %d %d %d %d\n", neigh_pts_id[0], neigh_pts_id[1],
							neigh_pts_id[2], neigh_pts_id[3]);
						if(warped_img.type() == CV_8UC3){
							warped_img.at<cv::Vec3b>(row_id, col_id) =
								orig_img.at<cv::Vec3f>(row_id, col_id);
						} else{
							warped_img.at<uchar>(row_id, col_id) =
								static_cast<uchar>(orig_img.at<float>(row_id, col_id));
						}
						//throw std::logic_error("Neighboring points ID are invalid");
					} else{
						//int ulx = original_pts(0, neigh_pts_id[0]), uly = original_pts(1, neigh_pts_id[0]);
						//int urx = original_pts(0, neigh_pts_id[1]), ury = original_pts(1, neigh_pts_id[1]);
						//int lrx = original_pts(0, neigh_pts_id[2]), lry = original_pts(1, neigh_pts_id[2]);
						//int llx = original_pts(0, neigh_pts_id[3]), lly = original_pts(1, neigh_pts_id[3]);

						//const cv::Vec3b& pix_ul = orig_img.at<cv::Vec3b>(uly, ulx);
						//const cv::Vec3b& pix_ur = orig_img.at<cv::Vec3b>(ury, urx);
						//const cv::Vec3b& pix_lr = orig_img.at<cv::Vec3b>(lry, lrx);
						//const cv::Vec3b& pix_ll = orig_img.at<cv::Vec3b>(lly, llx);

						double dist_sum_x = neigh_pts_dist[0][0] + neigh_pts_dist[1][0] + neigh_pts_dist[2][0] + neigh_pts_dist[3][0];
						double dist_sum_y = neigh_pts_dist[0][1] + neigh_pts_dist[1][1] + neigh_pts_dist[2][1] + neigh_pts_dist[3][1];
						double wt_ul = (dist_sum_x - neigh_pts_dist[0][0]) * (dist_sum_y - neigh_pts_dist[0][1]);
						double wt_ur = (dist_sum_x - neigh_pts_dist[1][0]) * (dist_sum_y - neigh_pts_dist[1][1]);
						double wt_lr = (dist_sum_x - neigh_pts_dist[2][0]) * (dist_sum_y - neigh_pts_dist[2][1]);
						double wt_ll = (dist_sum_x - neigh_pts_dist[3][0]) * (dist_sum_y - neigh_pts_dist[3][1]);
						double wt_sum = wt_ul + wt_ur + wt_lr + wt_ll;

						//printf("neigh_pts_id:\n");
						//for(int pt_id = 0; pt_id < 4; ++pt_id){
						//	printf("%d ", neigh_pts_id[pt_id]);
						//}
						//printf("\n");
						//printf("neigh_pts_dist:\n");
						//for(int pt_id = 0; pt_id < 4; ++pt_id){
						//	printf("%f %f\n", neigh_pts_dist[pt_id][0], neigh_pts_dist[pt_id][1]);
						//}
						if(warped_img.type() == CV_8UC3){
							cv::Vec3d orig_pix_vals[4];
							//printf("orig_pix_vals:\n");
							for(int pt_id = 0; pt_id < 4; ++pt_id){
								for(int channel_id = 0; channel_id < 3; ++channel_id){
									orig_pix_vals[pt_id][channel_id] = orig_patch[neigh_pts_id[pt_id] * 3 + channel_id];
									//printf("%f\t", orig_pix_vals[pt_id][channel_id]);
								}
								//printf("\n");
							}

							cv::Vec3b  pix_val;
							pix_val[0] = (
								orig_pix_vals[0][0] * wt_ul +
								orig_pix_vals[1][0] * wt_ur +
								orig_pix_vals[2][0] * wt_lr +
								orig_pix_vals[3][0] * wt_ll
								) / wt_sum;
							pix_val[1] = (
								orig_pix_vals[0][1] * wt_ul +
								orig_pix_vals[1][1] * wt_ur +
								orig_pix_vals[2][1] * wt_lr +
								orig_pix_vals[3][1] * wt_ll
								) / wt_sum;
							pix_val[2] = (
								orig_pix_vals[0][2] * wt_ul +
								orig_pix_vals[1][2] * wt_ur +
								orig_pix_vals[2][2] * wt_lr +
								orig_pix_vals[3][2] * wt_ll
								) / wt_sum;

							warped_img.at<cv::Vec3b>(row_id, col_id) = pix_val;
						} else{
							double orig_pix_vals[4];
							//printf("orig_pix_vals:\n");
							for(int pt_id = 0; pt_id < 4; ++pt_id){
								orig_pix_vals[pt_id] = orig_patch[neigh_pts_id[pt_id]];
							}
							double  pix_val = (
								orig_pix_vals[0] * wt_ul +
								orig_pix_vals[1] * wt_ur +
								orig_pix_vals[2] * wt_lr +
								orig_pix_vals[3] * wt_ll
								) / wt_sum;
							warped_img.at<uchar>(row_id, col_id) = pix_val;
						}
					}
				}
			}
			if(show_warped_img){
				cv::imshow("warped_img", warped_img);
				if(cv::waitKey(1) == 27){ break; }
			}
		}
	}

	void generateInverseWarpedImg(cv::Mat &warped_img, const mtf::PtsT &warped_pts,
		const cv::Mat &orig_img, const mtf::PtsT &orig_pts, int img_width, int img_height,
		int n_pts, bool show_warped_img, const char* win_name){
		assert(n_pts == img_width*img_height);

		for(int pt_id = 0; pt_id < n_pts; ++pt_id){
			int col_id = pt_id % img_width;
			int row_id = pt_id / img_width;

			//printf("col_id: %5d row_id: %5d\n", col_id, row_id);
			//printf("orig_x: %5.1f orig_y: %5.1f\n", orig_pts(0, pt_id), 
			//	orig_pts(1, pt_id));

			if(warped_img.type() == CV_8UC3){
				double pix_vals[3];
				getPixVal<PIX_INTERP_TYPE, PIX_BORDER_TYPE>(pix_vals, orig_img,
					warped_pts(0, pt_id), warped_pts(1, pt_id), img_height, img_width);
				for(int channel_id = 0; channel_id < 3; ++channel_id){
					pix_vals[channel_id] = min(255.0, pix_vals[channel_id]);
					pix_vals[channel_id] = max(0.0, pix_vals[channel_id]);
				}
				warped_img.at<cv::Vec3b>(row_id, col_id) = cv::Vec3b(pix_vals[0], pix_vals[1], pix_vals[2]);
			} else{
				double pix_val = getPixVal<PIX_INTERP_TYPE, PIX_BORDER_TYPE>(orig_img,
					warped_pts(0, pt_id), warped_pts(1, pt_id), img_height, img_width);
				pix_val = min(255.0, pix_val);
				pix_val = max(0.0, pix_val);
				warped_img.at<uchar>(row_id, col_id) = static_cast<uchar>(pix_val);
			}

			//const cv::Vec3b &dst_pix_vals = warped_img.at<cv::Vec3b>(orig_pts(1, pt_id), orig_pts(0, pt_id));
			//printf("pix_vals %d: %f, %f, %f\n", pt_id, pix_vals[0], pix_vals[1], pix_vals[2]);
			//printf("warped_img %d: %d, %d, %d\n", pt_id, 
			//	dst_pix_vals(0), dst_pix_vals(1), dst_pix_vals(2));

			if(show_warped_img && pt_id % img_width == 0){
				cv::imshow(win_name, warped_img);
				if(cv::waitKey(1) == 27){ break; }
			}
		}
	}


	void generateInverseWarpedImg(cv::Mat &warped_img,
		const PixValT &pix_vals, int img_width, int img_height,
		int n_pts, bool show_warped_img, const char* win_name){
		assert(n_pts == img_width*img_height);
		assert((warped_img.type() == CV_8UC3 && pix_vals.size() == n_pts * 3) ||
			(warped_img.type() == CV_8UC1 && pix_vals.size() == n_pts));

		for(int pt_id = 0; pt_id < n_pts; ++pt_id){
			int col_id = pt_id % img_width;
			int row_id = pt_id / img_width;

			if(warped_img.type() == CV_8UC3){
				double pix_val[3];
				for(int channel_id = 0; channel_id < 3; ++channel_id){
					pix_val[channel_id] = pix_vals[pt_id * 3 + channel_id];
					pix_val[channel_id] = min(255.0, pix_val[channel_id]);
					pix_val[channel_id] = max(0.0, pix_val[channel_id]);
				}
				warped_img.at<cv::Vec3b>(row_id, col_id) =
					cv::Vec3b(pix_val[0], pix_val[1], pix_val[2]);
			} else{
				double pix_val = pix_vals[pt_id];
				pix_val = min(255.0, pix_val);
				pix_val = max(0.0, pix_val);
				warped_img.at<uchar>(row_id, col_id) = static_cast<uchar>(pix_val);
			}

			//const cv::Vec3b &dst_pix_vals = warped_img.at<cv::Vec3b>(orig_pts(1, pt_id), orig_pts(0, pt_id));
			//printf("pix_vals %d: %f, %f, %f\n", pt_id, pix_vals[0], pix_vals[1], pix_vals[2]);
			//printf("warped_img %d: %d, %d, %d\n", pt_id, 
			//	dst_pix_vals(0), dst_pix_vals(1), dst_pix_vals(2));

			if(show_warped_img && pt_id % img_width == 0){
				cv::imshow(win_name, warped_img);
				if(cv::waitKey(1) == 27){ break; }
			}
		}
	}
	template<class ScalarT>
	inline uchar clamp(ScalarT pix){
		return pix < 0 ? 0 : pix > 255 ? 255 : static_cast<uchar>(pix);
	}

	void writePixelsToImage(cv::Mat &img, 
		const PixValT &pix_vals, const mtf::PtsT &pts, 
		int n_channels, cv::Mat &mask){
		int n_pts = pts.cols();
		assert(pix_vals.size() == n_pts*n_channels);

		bool use_mask = !mask.empty();

		for(int pt_id = 0; pt_id < n_pts; ++pt_id){
			int x = round(pts(0, pt_id));
			int y = round(pts(1, pt_id));

			if(checkOverflow(x, y, img.rows, img.cols)){ continue; }

			if(use_mask && mask.at<uchar>(y, x)){ continue; }

			switch(n_channels){
			case 1:
				img.at<uchar>(y, x) = clamp(pix_vals(pt_id));
				break;
			case 3:
				img.at<cv::Vec3b>(y, x) = cv::Vec3b(pix_vals(pt_id*n_channels),
					pix_vals(pt_id*n_channels + 1), pix_vals(pt_id*n_channels + 2));
				break;
			default:
				throw std::invalid_argument(
					cv::format("Invalid channel count provided: %d", n_channels));			
			}
			if(use_mask){ mask.at<uchar>(y, x) = 255; }

		}
	}
	//! adapted from http://answers.opencv.org/question/68589/adding-noise-to-image-opencv/
	bool addGaussianNoise(const cv::Mat mSrc, cv::Mat &mDst,
		int n_channels,	double Mean, double StdDev){
		if(mSrc.empty()){
			printf("addGaussianNoise:: Error: Input Image is empty!");
			return false;
		}
		cv::Mat mSrc_16SC;
		cv::Mat mGaussian_noise(mSrc.size(), n_channels == 1 ? CV_16SC1 : CV_16SC3);
		cv::randn(mGaussian_noise, cv::Scalar::all(Mean), cv::Scalar::all(StdDev));

		mSrc.convertTo(mSrc_16SC, n_channels == 1 ? CV_16SC1 : CV_16SC3);
		addWeighted(mSrc_16SC, 1.0, mGaussian_noise, 1.0, 0.0, mSrc_16SC);
		mSrc_16SC.convertTo(mDst, mSrc.type());
		return true;
	}


	/******************** Explicit Template Specializations ******************/

	template
		void getPixVals<PtsT>(VectorXd &pix_vals,
		const EigImgT &img, const PtsT &pts, int n_pix, int h, int w,
		double norm_mult, double norm_add);
	template
		void getPixValsSC<PtsT>(VectorXd &pix_vals,
		const cv::Mat &img, const PtsT &pts, int n_pix, int h, int w,
		double norm_mult, double norm_add);
	template
		void getPixVals<PtsT>(VectorXd &pix_vals,
		const cv::Mat &img, const PtsT &pts, int n_pix, int h, int w,
		double norm_mult, double norm_add);

	template
		void getWarpedImgGrad<InterpType::Nearest>(PixGradT &warped_img_grad,
		const EigImgT &img, const VectorXd &intensity_map,
		const Matrix8Xd &warped_offset_pts,
		double grad_eps, int n_pix, int h, int w, double pix_mult_factor);
	template
		void getWarpedImgGrad<InterpType::Linear>(PixGradT &warped_img_grad,
		const EigImgT &img, const VectorXd &intensity_map,
		const Matrix8Xd &warped_offset_pts,
		double grad_eps, int n_pix, int h, int w, double pix_mult_factor);

	template
		void getImgGrad<InterpType::Nearest>(PixGradT &img_grad, const EigImgT &img,
		const VectorXd &intensity_map, const PtsT &pts,
		double grad_eps, int n_pix, int h, int w,
		double pix_mult_factor);
	template
		void getImgGrad<InterpType::Linear>(PixGradT &img_grad, const EigImgT &img,
		const VectorXd &intensity_map, const PtsT &pts,
		double grad_eps, int n_pix, int h, int w, double pix_mult_factor);

	template
		void getWarpedImgHess<InterpType::Nearest>(PixHessT &warped_img_hess,
		const EigImgT &img, const VectorXd &intensity_map, const PtsT &warped_pts,
		const HessPtsT &warped_offset_pts, double hess_eps, int n_pix,
		int h, int w, double pix_mult_factor);
	template
		void getWarpedImgHess<InterpType::Linear>(PixHessT &warped_img_hess,
		const EigImgT &img, const VectorXd &intensity_map, const PtsT &warped_pts,
		const HessPtsT &warped_offset_pts, double hess_eps, int n_pix,
		int h, int w, double pix_mult_factor);

	template
		void getImgHess<InterpType::Nearest>(PixHessT &img_hess, const EigImgT &img, const VectorXd &intensity_map,
		const PtsT &pts, double hess_eps, int n_pix, int h, int w,
		double pix_mult_factor);
	template
		void getImgHess<InterpType::Linear>(PixHessT &img_hess, const EigImgT &img, const VectorXd &intensity_map,
		const PtsT &pts, double hess_eps, int n_pix, int h, int w,
		double pix_mult_factor);
	template
		void getWarpedImgGrad<InterpType::Nearest>(PixGradT &warped_img_grad,
		const cv::Mat &img, const VectorXd &intensity_map,
		const Matrix8Xd &warped_offset_pts,
		double grad_eps, int n_pix, int h, int w, double pix_mult_factor);
	template
		void getWarpedImgGrad<InterpType::Linear>(PixGradT &warped_img_grad,
		const cv::Mat &img, const VectorXd &intensity_map,
		const Matrix8Xd &warped_offset_pts,
		double grad_eps, int n_pix, int h, int w, double pix_mult_factor);

	template
		void getImgGrad<InterpType::Nearest>(PixGradT &img_grad, const cv::Mat &img,
		const VectorXd &intensity_map, const PtsT &pts,
		double grad_eps, int n_pix, int h, int w,
		double pix_mult_factor);
	template
		void getImgGrad<InterpType::Linear>(PixGradT &img_grad, const cv::Mat &img,
		const VectorXd &intensity_map, const PtsT &pts,
		double grad_eps, int n_pix, int h, int w, double pix_mult_factor);

	template
		void getWarpedImgHess<InterpType::Nearest>(PixHessT &warped_img_hess,
		const cv::Mat &img, const VectorXd &intensity_map, const PtsT &warped_pts,
		const HessPtsT &warped_offset_pts, double hess_eps, int n_pix,
		int h, int w, double pix_mult_factor);
	template
		void getWarpedImgHess<InterpType::Linear>(PixHessT &warped_img_hess,
		const cv::Mat &img, const VectorXd &intensity_map, const PtsT &warped_pts,
		const HessPtsT &warped_offset_pts, double hess_eps, int n_pix,
		int h, int w, double pix_mult_factor);

	template
		void getImgHess<InterpType::Nearest>(PixHessT &img_hess, const cv::Mat &img, const VectorXd &intensity_map,
		const PtsT &pts, double hess_eps, int n_pix, int h, int w,
		double pix_mult_factor);
	template
		void getImgHess<InterpType::Linear>(PixHessT &img_hess, const cv::Mat &img, const VectorXd &intensity_map,
		const PtsT &pts, double hess_eps, int n_pix, int h, int w,
		double pix_mult_factor);

#ifdef ENABLE_OLD_IMG_GRAD
	void getWarpedImgGrad(PixGradT &warped_img_grad,
		const EigImgT &img, const HomPtsT &warped_pts,
		const ProjWarpT &warp, double grad_eps, int n_pix,
		int h, int w, double pix_mult_factor){
		assert(warped_img_grad.rows() == n_pix);

		double grad_mult_factor = pix_mult_factor / (2 * grad_eps);

		Vector3d diff_vec_x_warped = warp.col(0) * grad_eps;
		Vector3d diff_vec_y_warped = warp.col(1) * grad_eps;

		//utils::printMatrixToFile(warped_pts, "warped_pts", "log/mtf_log.txt", "%15.9f", "a");
		//utils::printMatrixToFile(warp, "warp", "log/mtf_log.txt", "%15.9f", "a");
		//utils::printMatrixToFile(diff_vec_x_warped, "diff_vec_x_warped", "log/mtf_log.txt", "%15.9f", "a");
		//utils::printMatrixToFile(diff_vec_y_warped, "diff_vec_y_warped", "log/mtf_log.txt", "%15.9f", "a");

		Vector3d pt_inc_warped, pt_dec_warped;
		for(int pix_id = 0; pix_id < n_pix; pix_id++){
			pt_inc_warped = warped_pts.col(pix_id) + diff_vec_x_warped;
			pt_dec_warped = warped_pts.col(pix_id) - diff_vec_x_warped;
			warped_img_grad(pix_id, 0) = getPixGrad(img, pt_inc_warped, pt_dec_warped,
				grad_mult_factor, h, w);

			pt_inc_warped = warped_pts.col(pix_id) + diff_vec_y_warped;
			pt_dec_warped = warped_pts.col(pix_id) - diff_vec_y_warped;
			warped_img_grad(pix_id, 1) = getPixGrad(img, pt_inc_warped, pt_dec_warped,
				grad_mult_factor, h, w);
		}
		//utils::printMatrixToFile(warped_img_grad, "warped_img_grad", "log/mtf_log.txt", "%15.9f", "a");

	}

	void getWarpedImgHess(PixHessT &warped_img_hess,
		const EigImgT &img, const HomPtsT &warped_pts_hm,
		const ProjWarpT &warp, double hess_eps, int n_pix,
		int h, int w, double pix_mult_factor){
		assert(warped_img_hess.cols() == n_pix && warped_pts_hm.cols() == n_pix);

		double hess_eps2 = 2 * hess_eps;
		double hess_mult_factor = pix_mult_factor / (hess_eps2 * hess_eps2);

		Vector3d diff_vec_xx_warped = warp.col(0) * hess_eps2;
		Vector3d diff_vec_yy_warped = warp.col(1) * hess_eps2;
		Vector3d diff_vec_xy_warped = (warp.col(0) + warp.col(1)) * hess_eps;
		Vector3d diff_vec_yx_warped = (warp.col(0) - warp.col(1)) * hess_eps;

		Vector3d pt_warped, pt_inc_warped, pt_dec_warped, pt_inc_warped2, pt_dec_warped2;

		for(int pix_id = 0; pix_id < n_pix; pix_id++){

			pt_warped = warped_pts_hm.col(pix_id);

			double pix_val = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, pt_warped(0) / pt_warped(2),
				pt_warped(1) / pt_warped(2), h, w);

			pt_inc_warped = pt_warped + diff_vec_xx_warped;
			pt_dec_warped = pt_warped - diff_vec_xx_warped;
			warped_img_hess(0, pix_id) = getPixHess(img, pt_inc_warped, pt_dec_warped,
				pix_val, hess_mult_factor, h, w);

			pt_inc_warped = pt_warped + diff_vec_yy_warped;
			pt_dec_warped = pt_warped - diff_vec_yy_warped;
			warped_img_hess(3, pix_id) = getPixHess(img, pt_inc_warped, pt_dec_warped,
				pix_val, hess_mult_factor, h, w);

			pt_inc_warped = pt_warped + diff_vec_xy_warped;
			pt_dec_warped = pt_warped - diff_vec_xy_warped;
			pt_inc_warped2 = pt_warped + diff_vec_yx_warped;
			pt_dec_warped2 = pt_warped - diff_vec_yx_warped;
			warped_img_hess(1, pix_id) = warped_img_hess(2, pix_id) = getPixHess(img, pt_inc_warped, pt_dec_warped,
				pt_inc_warped2, pt_dec_warped2, hess_mult_factor, h, w);
		}
	}
	// mapped version
	template<InterpType mapping_type>
	void getWarpedImgGrad(PixGradT &warped_img_grad,
		const EigImgT &img, const VectorXd &intensity_map,
		const HomPtsT &warped_pts, const ProjWarpT &warp,
		double grad_eps, int n_pix, int h, int w,
		double pix_mult_factor){
		assert(warped_img_grad.rows() == n_pix);

		double grad_mult_factor = pix_mult_factor / (2 * grad_eps);

		Vector3d diff_vec_x_warped = warp.col(0) * grad_eps;
		Vector3d diff_vec_y_warped = warp.col(1) * grad_eps;

		Vector3d pt_inc_warped, pt_dec_warped;
		for(int pix_id = 0; pix_id < n_pix; pix_id++){
			pt_inc_warped = warped_pts.col(pix_id) + diff_vec_x_warped;
			pt_dec_warped = warped_pts.col(pix_id) - diff_vec_x_warped;
			warped_img_grad(pix_id, 0) = getPixGrad<mapping_type>(img, intensity_map,
				pt_inc_warped, pt_dec_warped, grad_mult_factor, h, w);

			pt_inc_warped = warped_pts.col(pix_id) + diff_vec_y_warped;
			pt_dec_warped = warped_pts.col(pix_id) - diff_vec_y_warped;
			warped_img_grad(pix_id, 1) = getPixGrad<mapping_type>(img, intensity_map,
				pt_inc_warped, pt_dec_warped, grad_mult_factor, h, w);
		}
	}
	// mapped version
	template<InterpType mapping_type>
	void getWarpedImgHess(PixHessT &warped_img_hess, const EigImgT &img,
		const VectorXd &intensity_map, const HomPtsT &warped_pts_hm,
		const ProjWarpT &warp, double hess_eps, int n_pix,
		int h, int w, double pix_mult_factor){
		assert(warped_img_hess.cols() == n_pix && warped_pts_hm.cols() == n_pix);

		double hess_eps2 = 2 * hess_eps;
		double hess_mult_factor = pix_mult_factor / (hess_eps2 * hess_eps2);

		Vector3d diff_vec_xx_warped = warp.col(0) * hess_eps2;
		Vector3d diff_vec_yy_warped = warp.col(1) * hess_eps2;
		Vector3d diff_vec_xy_warped = (warp.col(0) + warp.col(1)) * hess_eps;
		Vector3d diff_vec_yx_warped = (warp.col(0) - warp.col(1)) * hess_eps;

		Vector3d pt_warped, pt_inc_warped, pt_dec_warped, pt_inc_warped2, pt_dec_warped2;

		for(int pix_id = 0; pix_id < n_pix; pix_id++){

			pt_warped = warped_pts_hm.col(pix_id);

			double pix_val = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, pt_warped(0) / pt_warped(2),
				pt_warped(1) / pt_warped(2), h, w);

			pt_inc_warped = pt_warped + diff_vec_xx_warped;
			pt_dec_warped = pt_warped - diff_vec_xx_warped;
			warped_img_hess(0, pix_id) = getPixHess<mapping_type>(img, intensity_map, pt_inc_warped, pt_dec_warped,
				pix_val, hess_mult_factor, h, w);

			pt_inc_warped = pt_warped + diff_vec_yy_warped;
			pt_dec_warped = pt_warped - diff_vec_yy_warped;
			warped_img_hess(3, pix_id) = getPixHess<mapping_type>(img, intensity_map, pt_inc_warped, pt_dec_warped,
				pix_val, hess_mult_factor, h, w);

			pt_inc_warped = pt_warped + diff_vec_xy_warped;
			pt_dec_warped = pt_warped - diff_vec_xy_warped;
			pt_inc_warped2 = pt_warped + diff_vec_yx_warped;
			pt_dec_warped2 = pt_warped - diff_vec_yx_warped;
			warped_img_hess(1, pix_id) = warped_img_hess(2, pix_id) = getPixHess<mapping_type>(img, intensity_map, pt_inc_warped, pt_dec_warped,
				pt_inc_warped2, pt_dec_warped2, hess_mult_factor, h, w);
		}
	}

	template
		void getWarpedImgGrad<InterpType::Nearest>(PixGradT &warped_img_grad,
		const EigImgT &img, const VectorXd &intensity_map,
		const HomPtsT &warped_pts, const ProjWarpT &warp,
		double grad_eps, int n_pix, int h, int w, double pix_mult_factor);
	template
		void getWarpedImgGrad<InterpType::Linear>(PixGradT &warped_img_grad,
		const EigImgT &img, const VectorXd &intensity_map,
		const HomPtsT &warped_pts, const ProjWarpT &warp,
		double grad_eps, int n_pix, int h, int w, double pix_mult_factor);

	template
		void getWarpedImgHess<InterpType::Nearest>(PixHessT &warped_img_hess,
		const EigImgT &img, const VectorXd &intensity_map, const HomPtsT &warped_pts_hm,
		const ProjWarpT &warp, double hess_eps, int n_pix,
		int h, int w, double pix_mult_factor);
	template
		void getWarpedImgHess<InterpType::Linear>(PixHessT &warped_img_hess,
		const EigImgT &img, const VectorXd &intensity_map, const HomPtsT &warped_pts_hm,
		const ProjWarpT &warp, double hess_eps, int n_pix,
		int h, int w, double pix_mult_factor);
#endif

}

_MTF_END_NAMESPACE
