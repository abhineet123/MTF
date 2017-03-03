#ifndef MTF_IMG_UTILS_H
#define MTF_IMG_UTILS_H

#include "mtf/Macros/common.h"

#ifndef PIX_INTERP_TYPE
#define PIX_INTERP_TYPE utils::InterpType::Linear
#define DEFAULT_PIX_INTERP_TYPE
#endif

#ifndef GRAD_INTERP_TYPE
#define GRAD_INTERP_TYPE utils::InterpType::Linear
#define DEFAULT_GRAD_INTERP_TYPE
#endif

#ifndef HESS_INTERP_TYPE
#define HESS_INTERP_TYPE utils::InterpType::Linear
#define DEFAULT_HESS_INTERP_TYPE
#endif

#ifndef PIX_BORDER_TYPE
#define PIX_BORDER_TYPE utils::BorderType::Constant
#define DEFAULT_PIX_BORDER_TYPE
#endif

#ifndef GRAD_EPS
#define GRAD_EPS 1e-8
#endif

#ifndef HESS_EPS
#define HESS_EPS 1
#endif

_MTF_BEGIN_NAMESPACE

namespace utils{
	// types of interpolation
	enum class InterpType { Nearest, Linear, Cubic, Cubic2, CubicBSpl };
	enum class BorderType{ Constant, Replicate };
	const char* toString(InterpType interp_type);
	const char* toString(BorderType border_type);
	const char* getType(const cv::Mat& mat);
	/******* functions for extracting pixel values from image ***********/

	// check if the given x, y coordinates are outside the extents of an image with the given height and width
	inline bool checkOverflow(double x, double y, int h, int w){
		return ((x < 0) || (x >= w) || (y < 0) || (y >= h));
	}
	// computes the pixel value at the given location expressed in (real) x, y, coordinates
	template<InterpType interp_type, BorderType border_type>
	inline double getPixVal(const EigImgT &img, double x, double y,
		int h, int w, double overflow_val = 128.0){
		throw std::invalid_argument(
			cv::format("getPixVal :: Invalid interpolation type specified: %d",
			interp_type));
	}
	// using nearest neighbor interpolation with constant border
	template<>
	inline double getPixVal<InterpType::Nearest, BorderType::Constant>(
		const EigImgT &img, double x, double y,
		int h, int w, double overflow_val){
		assert(img.rows() == h && img.cols() == w);

		if(checkOverflow(x, y, h, w)){
			return overflow_val;
		}
		int nx = static_cast<int>(rint(x));
		int ny = static_cast<int>(rint(y));
		return checkOverflow(nx, ny, h, w) ? overflow_val : img(ny, nx);
	}
	// using nearest neighbor interpolation with replicated border
	template<>
	inline double getPixVal<InterpType::Nearest, BorderType::Replicate>(
		const EigImgT &img, double x, double y,
		int h, int w, double overflow_val){
		assert(img.rows() == h && img.cols() == w);

		if(x > w - 1){ x = w - 1; } else if(x < 0){ x = 0; }
		if(y > h - 1){ y = h - 1; } else if(y < 0){ y = 0; }

		int nx = static_cast<int>(rint(x));
		int ny = static_cast<int>(rint(y));
		return checkOverflow(nx, ny, h, w) ? overflow_val : img(ny, nx);
	}
	// using bilinear interpolation with constant border
	template<>
	inline double getPixVal<InterpType::Linear, BorderType::Constant>(
		const EigImgT &img, double x, double y,
		int h, int w, double overflow_val){
		assert(img.rows() == h && img.cols() == w);
		if(checkOverflow(x, y, h, w)){
			return overflow_val;
		}
		int lx = static_cast<int>(x);
		int ly = static_cast<int>(y);
		double dx = x - lx;
		double dy = y - ly;
		int ux = dx == 0 ? lx : lx + 1;
		int uy = dy == 0 ? ly : ly + 1;

		if(checkOverflow(lx, ly, h, w) || checkOverflow(ux, uy, h, w)){
			return overflow_val;
		}
		return 	img(ly, lx) * (1 - dx)*(1 - dy) +
			img(ly, ux) * dx*(1 - dy) +
			img(uy, lx) * (1 - dx)*dy +
			img(uy, ux) * dx*dy;
	}
	// using bilinear interpolation with replicated border
	template<>
	inline double getPixVal<InterpType::Linear, BorderType::Replicate>(
		const EigImgT &img, double x, double y,
		int h, int w, double overflow_val){
		assert(img.rows() == h && img.cols() == w);

		if(x > w - 1){ x = w - 1; } else if(x < 0){ x = 0; }
		if(y > h - 1){ y = h - 1; } else if(y < 0){ y = 0; }

		int lx = static_cast<int>(x);
		int ly = static_cast<int>(y);
		double dx = x - lx;
		double dy = y - ly;
		int ux = dx == 0 ? lx : lx + 1;
		int uy = dy == 0 ? ly : ly + 1;

		return 	img(ly, lx) * (1 - dx)*(1 - dy) +
			img(ly, ux) * dx*(1 - dy) +
			img(uy, lx) * (1 - dx)*dy +
			img(uy, ux) * dx*dy;
	}
	// using bi cubic interpolation
	template<>
	double getPixVal<InterpType::Cubic, BorderType::Constant>(
		const EigImgT &img, double x, double y,
		int h, int w, double overflow_val);
	// using cubic BSpline  interpolation - an alternate formulation that uses polynomial coefficients
	template<>
	double getPixVal<InterpType::Cubic2, BorderType::Constant>(
		const EigImgT &img, double x, double y,
		int h, int w, double overflow_val);
	// using cubic BSpline  interpolation
	template<>
	double getPixVal<InterpType::CubicBSpl, BorderType::Constant>(
		const EigImgT &img, double x, double y,
		int h, int w, double overflow_val);

	template<typename PtsT>
	void getPixVals(VectorXd &pix_vals,
		const EigImgT &img, const PtsT &pts, int n_pix, int h, int w,
		double norm_mult = 1, double norm_add = 0);

	//! get weighted pixel values using alpha as weighting factor 
	//! between existing and new pixel values
	void getWeightedPixVals(VectorXd &pix_vals, const EigImgT &img, const PtsT &pts,
		int frame_count, double alpha, bool use_running_avg, int n_pix,
		int h, int w, double norm_mult, double norm_add);

	/************ functions for image gradient and Hessian ************/
	void getWarpedImgGrad(PixGradT &warped_img_grad,
		const EigImgT &img, const Matrix8Xd &warped_offset_pts,
		double grad_eps, int n_pix,
		int h, int w, double pix_mult_factor = 1.0);
	template<InterpType mapping_type>
	void getWarpedImgGrad(PixGradT &warped_img_grad,
		const EigImgT &img, const VectorXd &intensity_map,
		const Matrix8Xd &warped_offset_pts,
		double grad_eps, int n_pix, int h, int w,
		double pix_mult_factor = 1.0);
	void getWarpedImgGrad(PixGradT &warped_img_grad, const EigImgT &img, bool weighted_mapping,
		const VectorXd &intensity_map, const Matrix8Xd &warped_offset_pts,
		double grad_eps, int n_pix, int h, int w, double pix_mult_factor = 1.0);
	void getImgGrad(PixGradT &img_grad,
		const EigImgT &img, const PtsT &pts,
		double grad_eps, int n_pix, int h, int w,
		double pix_mult_factor = 1.0);
	// mapping enabled version
	template<InterpType mapping_type>
	void getImgGrad(PixGradT &img_grad, const EigImgT &img,
		const VectorXd &intensity_map, const PtsT &pts,
		double grad_eps, int n_pix, int h, int w,
		double pix_mult_factor = 1.0);
	void getImgGrad(PixGradT &img_grad, const EigImgT &img,
		bool weighted_mapping, const VectorXd &intensity_map,
		const PtsT &pts, double grad_eps, int n_pix, int h, int w,
		double pix_mult_factor = 1.0);
	void getWarpedImgHess(PixHessT &warped_img_hess,
		const EigImgT &img, const PtsT &warped_pts,
		const Matrix16Xd &warped_offset_pts, double hess_eps, int n_pix,
		int h, int w, double pix_mult_factor = 1.0);
	// mapped version
	template<InterpType mapping_type>
	void getWarpedImgHess(PixHessT &warped_img_hess,
		const EigImgT &img, const VectorXd &intensity_map,
		const PtsT &warped_pts, const Matrix16Xd &warped_offset_pts,
		double hess_eps, int n_pix, int h, int w, double pix_mult_factor = 1.0);
	void getWarpedImgHess(PixHessT &warped_img_hess, const EigImgT &img,
		bool weighted_mapping, const VectorXd &intensity_map,
		const PtsT &warped_pts, const Matrix16Xd &warped_offset_pts,
		double hess_eps, int n_pix, int h, int w, double pix_mult_factor = 1.0);
	void getImgHess(PixHessT &img_hess, const EigImgT &img,
		const PtsT &pts, double hess_eps,
		int n_pix, int h, int w, double pix_mult_factor = 1.0);
	template<InterpType mapping_type>
	void getImgHess(PixHessT &img_hess, const EigImgT &img, const VectorXd &intensity_map,
		const PtsT &pts, double hess_eps, int n_pix, int h, int w,
		double pix_mult_factor = 1.0);
	void getImgHess(PixHessT &img_hess, const EigImgT &img, bool weighted_mapping,
		const VectorXd &intensity_map, const PtsT &pts, double hess_eps, int n_pix,
		int h, int w, double pix_mult_factor = 1.0);
	// fits a bi cubic polynomial surface at each pixel location and computes the analytical hessian
	// of this surface thus eliminating the need for finite difference approximations
	void getImgHess(PixHessT &img_hess, const EigImgT &img,
		const PtsT &pts, int n_pix, int h, int w);

	namespace sc{

		// *************************************************************************************** //
		// ***** functions for single channel images that work directly on OpenCV Mat images ***** //
		// *************************************************************************************** //

		template<typename ScalarType, InterpType interp_type, BorderType border_type>
		struct PixVal{
			static inline double get(const cv::Mat &img, double x, double y,
				int h, int w, double overflow_val = 128.0){
				throw std::invalid_argument(
					cv::format("get :: Invalid interpolation type specified: %d",
					interp_type));
			}
		};
		// using nearest neighbor interpolation with constant border
		template<typename ScalarType>
		struct PixVal < ScalarType, InterpType::Nearest, BorderType::Constant > {
			static inline double get(
				const cv::Mat &img, double x, double y,
				int h, int w, double overflow_val = 128.0){
				assert(img.rows == h && img.cols == w);

				if(checkOverflow(x, y, h, w)){
					return overflow_val;
				}
				int nx = static_cast<int>(rint(x));
				int ny = static_cast<int>(rint(y));
				return checkOverflow(nx, ny, h, w) ? overflow_val : img.at<ScalarType>(ny, nx);
			}
		};
		// using nearest neighbor interpolation with replicated border
		template<typename ScalarType>
		struct PixVal < ScalarType, InterpType::Nearest, BorderType::Replicate > {
			static inline double get(
				const cv::Mat &img, double x, double y,
				int h, int w, double overflow_val = 128.0){
				assert(img.rows == h && img.cols == w);

				if(x > w - 1){ x = w - 1; } else if(x < 0){ x = 0; }
				if(y > h - 1){ y = h - 1; } else if(y < 0){ y = 0; }

				int nx = static_cast<int>(rint(x));
				int ny = static_cast<int>(rint(y));
				return checkOverflow(nx, ny, h, w) ? overflow_val : img.at<ScalarType>(ny, nx);
			}
		};
		// using bilinear interpolation with constant border
		template<typename ScalarType>
		struct PixVal < ScalarType, InterpType::Linear, BorderType::Constant > {
			static inline double get(
				const cv::Mat &img, double x, double y,
				int h, int w, double overflow_val = 128.0){
				assert(img.rows == h && img.cols == w);
				if(checkOverflow(x, y, h, w)){
					return overflow_val;
				}
				int lx = static_cast<int>(x);
				int ly = static_cast<int>(y);
				double dx = x - lx;
				double dy = y - ly;
				int ux = dx == 0 ? lx : lx + 1;
				int uy = dy == 0 ? ly : ly + 1;

				if(checkOverflow(lx, ly, h, w) || checkOverflow(ux, uy, h, w)){
					return overflow_val;
				}
				return 	img.at<ScalarType>(ly, lx) * (1 - dx)*(1 - dy) +
					img.at<ScalarType>(ly, ux) * dx*(1 - dy) +
					img.at<ScalarType>(uy, lx) * (1 - dx)*dy +
					img.at<ScalarType>(uy, ux) * dx*dy;
			}
		};
		// using bilinear interpolation with replicated border
		template<typename ScalarType>
		struct PixVal < ScalarType, InterpType::Linear, BorderType::Replicate > {
			static inline double get(
				const cv::Mat &img, double x, double y,
				int h, int w, double overflow_val = 128.0){
				assert(img.rows == h && img.cols == w);

				if(x > w - 1){ x = w - 1; } else if(x < 0){ x = 0; }
				if(y > h - 1){ y = h - 1; } else if(y < 0){ y = 0; }

				int lx = static_cast<int>(x);
				int ly = static_cast<int>(y);
				double dx = x - lx;
				double dy = y - ly;
				int ux = dx == 0 ? lx : lx + 1;
				int uy = dy == 0 ? ly : ly + 1;

				return 	img.at<ScalarType>(ly, lx) * (1 - dx)*(1 - dy) +
					img.at<ScalarType>(ly, ux) * dx*(1 - dy) +
					img.at<ScalarType>(uy, lx) * (1 - dx)*dy +
					img.at<ScalarType>(uy, ux) * dx*dy;
			}
		};
		template<typename ScalarType, typename PtsT>
		void getPixVals(VectorXd &pix_vals,
			const cv::Mat &img, const PtsT &pts, int n_pix, int h, int w,
			double norm_mult = 1, double norm_add = 0);

		/************ functions for image gradient and Hessian ************/

		template<typename ScalarType = float>
		void getWarpedImgGrad(PixGradT &warped_img_grad,
			const cv::Mat &img, const Matrix8Xd &warped_offset_pts,
			double grad_eps, int n_pix,
			int h, int w, double pix_mult_factor = 1.0);
		template<typename ScalarType = float>
		void getImgGrad(PixGradT &img_grad,
			const  cv::Mat &img, const PtsT &pts,
			double grad_eps, int n_pix, int h, int w,
			double pix_mult_factor = 1.0);
		template<typename ScalarType = float>
		void getWarpedImgHess(PixHessT &warped_img_hess,
			const cv::Mat &img, const PtsT &warped_pts,
			const Matrix16Xd &warped_offset_pts, double hess_eps, int n_pix,
			int h, int w, double pix_mult_factor = 1.0);
		template<typename ScalarType = float>
		void getImgHess(PixHessT &img_hess, const cv::Mat &img,
			const PtsT &pts, double hess_eps,
			int n_pix, int h, int w, double pix_mult_factor = 1.0);
		// mapped versions
		template<typename ScalarType = float, InterpType mapping_type>
		void getWarpedImgGrad(PixGradT &warped_img_grad,
			const cv::Mat &img, const VectorXd &intensity_map,
			const Matrix8Xd &warped_offset_pts, double grad_eps,
			int n_pix, int h, int w, double pix_mult_factor = 1.0);
		template<typename ScalarType = float>
		void getWarpedImgGrad(PixGradT &warped_img_grad, const cv::Mat &img,
			bool weighted_mapping, const VectorXd &intensity_map,
			const Matrix8Xd &warped_offset_pts, double grad_eps,
			int n_pix, int h, int w, double pix_mult_factor = 1.0);
		template<typename ScalarType = float, InterpType mapping_type>
		void getImgGrad(PixGradT &img_grad, const cv::Mat &img,
			const VectorXd &intensity_map, const PtsT &pts,
			double grad_eps, int n_pix, int h, int w,
			double pix_mult_factor = 1.0);
		template<typename ScalarType = float>
		void getImgGrad(PixGradT &img_grad, const cv::Mat &img,
			bool weighted_mapping, const VectorXd &intensity_map,
			const PtsT &pts, double grad_eps, int n_pix, int h, int w,
			double pix_mult_factor = 1.0);
		template<typename ScalarType = float, InterpType mapping_type>
		void getWarpedImgHess(PixHessT &warped_img_hess,
			const cv::Mat &img, const VectorXd &intensity_map,
			const PtsT &warped_pts, const HessPtsT &warped_offset_pts,
			double hess_eps, int n_pix, int h, int w, double pix_mult_factor = 1.0);
		template<typename ScalarType = float>
		void getWarpedImgHess(PixHessT &warped_img_hess,
			const cv::Mat &img, bool weighted_mapping, const VectorXd &intensity_map,
			const PtsT &warped_pts, const HessPtsT &warped_offset_pts,
			double hess_eps, int n_pix, int h, int w, double pix_mult_factor = 1.0);
		template<typename ScalarType = float, InterpType mapping_type>
		void getImgHess(PixHessT &img_hess, const cv::Mat &img,
			const VectorXd &intensity_map, const PtsT &pts, double hess_eps,
			int n_pix, int h, int w, double pix_mult_factor = 1.0);
		template<typename ScalarType = float>
		void getImgHess(PixHessT &img_hess, const cv::Mat &img, bool weighted_mapping,
			const VectorXd &intensity_map, const PtsT &pts, double hess_eps,
			int n_pix, int h, int w, double pix_mult_factor = 1.0);
	}

	namespace mc{

		// ************************************************************************************** //
		// ***** functions for multi channel images that work directly on OpenCV Mat images ***** //
		// ************************************************************************************** //

		template<typename ScalarType, InterpType interp_type, BorderType border_type>
		struct PixVal{
			static inline void get(double *pix_val, const cv::Mat &img, double x, double y,
				int h, int w, double overflow_val = 128.0){
				throw std::invalid_argument(
					cv::format("get :: Invalid interpolation type specified: %d",
					interp_type));
			}
		};
		// using nearest neighbor interpolation with constant border
		template<typename ScalarType>
		struct PixVal < ScalarType, InterpType::Nearest, BorderType::Constant > {
			static inline void get(
				double *pix_val, const cv::Mat &img, double x, double y,
				int h, int w, double overflow_val = 128.0){
				assert(img.rows == h && img.cols == w && img.type() == CV_32FC3);

				if(checkOverflow(x, y, h, w)){
					pix_val[0] = pix_val[1] = pix_val[2] = overflow_val;
					return;
				}
				int nx = static_cast<int>(rint(x));
				int ny = static_cast<int>(rint(y));
				if(checkOverflow(nx, ny, h, w)){
					pix_val[0] = pix_val[1] = pix_val[2] = overflow_val;
				} else{
					const ScalarType *pix_data = img.ptr<ScalarType>(ny);
					pix_val[0] = pix_data[nx];
					pix_val[1] = pix_data[nx + 1];
					pix_val[2] = pix_data[nx + 2];
				}
			}
		};
		// using nearest neighbor interpolation with replicated border
		template<typename ScalarType>
		struct PixVal < ScalarType, InterpType::Nearest, BorderType::Replicate > {
			static inline void get(
				double *pix_val, const cv::Mat &img, double x, double y,
				int h, int w, double overflow_val = 128.0){
				assert(img.rows == h && img.cols == w && img.type() == CV_32FC3);

				if(x > w - 1){ x = w - 1; } else if(x < 0){ x = 0; }
				if(y > h - 1){ y = h - 1; } else if(y < 0){ y = 0; }

				int nx = static_cast<int>(rint(x));
				int ny = static_cast<int>(rint(y));

				if(checkOverflow(nx, ny, h, w)){
					pix_val[0] = pix_val[1] = pix_val[2] = overflow_val;
				} else{
					const ScalarType *pix_data = img.ptr<ScalarType>(ny);
					pix_val[0] = pix_data[nx];
					pix_val[1] = pix_data[nx + 1];
					pix_val[2] = pix_data[nx + 2];
				}
			}
		};
		// using bilinear interpolation with constant border
		template<typename ScalarType>
		struct PixVal < ScalarType, InterpType::Linear, BorderType::Constant > {
			static inline void get(
				double *pix_val, const cv::Mat &img, double x, double y,
				int h, int w, double overflow_val = 128.0){
				assert(img.rows == h && img.cols == w && img.type() == CV_32FC3);

				typedef cv::Vec<ScalarType, 3> Vec;

				if(checkOverflow(x, y, h, w)){
					pix_val[0] = pix_val[1] = pix_val[2] = overflow_val;
					return;
				}
				int lx = static_cast<int>(x);
				int ly = static_cast<int>(y);
				double dx = x - lx;
				double dy = y - ly;
				int ux = dx == 0 ? lx : lx + 1;
				int uy = dy == 0 ? ly : ly + 1;

				if(checkOverflow(lx, ly, h, w) || checkOverflow(ux, uy, h, w)){
					pix_val[0] = pix_val[1] = pix_val[2] = overflow_val;
				} else{
					double ly_lx = (1 - dx)*(1 - dy), ly_ux = dx*(1 - dy), uy_lx = (1 - dx)*dy, uy_ux = dx*dy;
					const Vec& pix_ly_lx = img.at<Vec>(ly, lx);
					const Vec& pix_ly_ux = img.at<Vec>(ly, ux);
					const Vec& pix_uy_lx = img.at<Vec>(uy, lx);
					const Vec& pix_uy_ux = img.at<Vec>(uy, ux);
					pix_val[0] =
						pix_ly_lx[0] * ly_lx +
						pix_ly_ux[0] * ly_ux +
						pix_uy_lx[0] * uy_lx +
						pix_uy_ux[0] * uy_ux;
					pix_val[1] =
						pix_ly_lx[1] * ly_lx +
						pix_ly_ux[1] * ly_ux +
						pix_uy_lx[1] * uy_lx +
						pix_uy_ux[1] * uy_ux;
					pix_val[2] =
						pix_ly_lx[2] * ly_lx +
						pix_ly_ux[2] * ly_ux +
						pix_uy_lx[2] * uy_lx +
						pix_uy_ux[2] * uy_ux;
				}
			}
		};
		// using bilinear interpolation with replicated border
		template<typename ScalarType>
		struct PixVal < ScalarType, InterpType::Linear, BorderType::Replicate > {
			static inline void get(
				double *pix_val, const cv::Mat &img, double x, double y,
				int h, int w, double overflow_val = 128.0){
				assert(img.rows == h && img.cols == w && img.type() == CV_32FC3);

				typedef cv::Vec<ScalarType, 3> Vec;

				if(x > w - 1){ x = w - 1; } else if(x < 0){ x = 0; }
				if(y > h - 1){ y = h - 1; } else if(y < 0){ y = 0; }

				int lx = static_cast<int>(x);
				int ly = static_cast<int>(y);
				double dx = x - lx;
				double dy = y - ly;
				int ux = dx == 0 ? lx : lx + 1;
				int uy = dy == 0 ? ly : ly + 1;

				if(checkOverflow(lx, ly, h, w) || checkOverflow(ux, uy, h, w)){
					pix_val[0] = pix_val[1] = pix_val[2] = overflow_val;
					return;
				} else{
					const Vec& pix_ly_lx = img.at<Vec>(ly, lx);
					const Vec& pix_ly_ux = img.at<Vec>(ly, ux);
					const Vec& pix_uy_lx = img.at<Vec>(uy, lx);
					const Vec& pix_uy_ux = img.at<Vec>(uy, ux);
					double ly_lx = (1 - dx)*(1 - dy), ly_ux = dx*(1 - dy), uy_lx = (1 - dx)*dy, uy_ux = dx*dy;

					pix_val[0] =
						pix_ly_lx[0] * ly_lx +
						pix_ly_ux[0] * ly_ux +
						pix_uy_lx[0] * uy_lx +
						pix_uy_ux[0] * uy_ux;
					pix_val[1] =
						pix_ly_lx[1] * ly_lx +
						pix_ly_ux[1] * ly_ux +
						pix_uy_lx[1] * uy_lx +
						pix_uy_ux[1] * uy_ux;
					pix_val[2] =
						pix_ly_lx[2] * ly_lx +
						pix_ly_ux[2] * ly_ux +
						pix_uy_lx[2] * uy_lx +
						pix_uy_ux[2] * uy_ux;
				}
			}
		};
		template<typename ScalarType = float, typename PtsT>
		void getPixVals(VectorXd &pix_vals,
			const cv::Mat &img, const PtsT &pts, int n_pix, int h, int w,
			double norm_mult = 1, double norm_add = 0);

		//! get weighted pixel values using alpha as weighting factor 
		//! between existing and new pixel values
		template<typename ScalarType = float>
		void getWeightedPixVals(VectorXd &pix_vals, const cv::Mat &img, const PtsT &pts,
			int frame_count, double alpha, bool use_running_avg, int n_pix,
			int h, int w, double pix_norm_mult, double pix_norm_add);
		
		/************ functions for image gradient and Hessian ************/

		template<typename ScalarType = float>
		void getWarpedImgGrad(PixGradT &warped_img_grad,
			const cv::Mat &img, const Matrix8Xd &warped_offset_pts,
			double grad_eps, int n_pix,
			int h, int w, double pix_mult_factor = 1.0);
		template<typename ScalarType = float>
		void getImgGrad(PixGradT &img_grad,
			const  cv::Mat &img, const PtsT &pts,
			double grad_eps, int n_pix, int h, int w,
			double pix_mult_factor = 1.0);
		template<typename ScalarType = float>
		void getWarpedImgHess(PixHessT &warped_img_hess,
			const cv::Mat &img, const PtsT &warped_pts,
			const Matrix16Xd &warped_offset_pts, double hess_eps, int n_pix,
			int h, int w, double pix_mult_factor = 1.0);
		template<typename ScalarType = float>
		void getImgHess(PixHessT &img_hess, const cv::Mat &img,
			const PtsT &pts, double hess_eps,
			int n_pix, int h, int w, double pix_mult_factor = 1.0);
		// mapped versions
		template<typename ScalarType = float, InterpType mapping_type>
		void getWarpedImgGrad(PixGradT &warped_img_grad,
			const cv::Mat &img, const VectorXd &intensity_map,
			const Matrix8Xd &warped_offset_pts, double grad_eps,
			int n_pix, int h, int w, double pix_mult_factor = 1.0);
		template<typename ScalarType = float>
		void getWarpedImgGrad(PixGradT &warped_img_grad, const cv::Mat &img,
			bool weighted_mapping, const VectorXd &intensity_map,
			const Matrix8Xd &warped_offset_pts, double grad_eps,
			int n_pix, int h, int w, double pix_mult_factor = 1.0);
		template<typename ScalarType = float, InterpType mapping_type>
		void getImgGrad(PixGradT &img_grad, const cv::Mat &img,
			const VectorXd &intensity_map, const PtsT &pts,
			double grad_eps, int n_pix, int h, int w,
			double pix_mult_factor = 1.0);
		template<typename ScalarType = float>
		void getImgGrad(PixGradT &img_grad, const cv::Mat &img,
			bool weighted_mapping, const VectorXd &intensity_map,
			const PtsT &pts, double grad_eps, int n_pix, int h, int w,
			double pix_mult_factor = 1.0);
		template<typename ScalarType = float, InterpType mapping_type>
		void getWarpedImgHess(PixHessT &warped_img_hess,
			const cv::Mat &img, const VectorXd &intensity_map,
			const PtsT &warped_pts, const HessPtsT &warped_offset_pts,
			double hess_eps, int n_pix, int h, int w, double pix_mult_factor = 1.0);
		template<typename ScalarType = float>
		void getWarpedImgHess(PixHessT &warped_img_hess,
			const cv::Mat &img, bool weighted_mapping, const VectorXd &intensity_map,
			const PtsT &warped_pts, const HessPtsT &warped_offset_pts,
			double hess_eps, int n_pix, int h, int w, double pix_mult_factor = 1.0);
		template<typename ScalarType = float, InterpType mapping_type>
		void getImgHess(PixHessT &img_hess, const cv::Mat &img,
			const VectorXd &intensity_map, const PtsT &pts, double hess_eps,
			int n_pix, int h, int w, double pix_mult_factor = 1.0);
		template<typename ScalarType = float>
		void getImgHess(PixHessT &img_hess, const cv::Mat &img, bool weighted_mapping,
			const VectorXd &intensity_map, const PtsT &pts, double hess_eps,
			int n_pix, int h, int w, double pix_mult_factor = 1.0);
	}
	/*********** functions for mapping pixel values *******************/

	//! map pixel values using the given intensity map
	template<InterpType interp_type>
	inline double mapPixVal(double x, const VectorXd &intensity_map){
		printf("mapPixVal:: invalid interpolation type specified: %d\n", interp_type);
		return 0;
	}
	//using nearest neighbor interpolation
	template<>
	inline double mapPixVal<InterpType::Nearest>(double x, const VectorXd &intensity_map){
		return intensity_map(static_cast<int>(rint(x)));
	}
	//using linear interpolation
	template<>
	inline double mapPixVal<InterpType::Linear>(double x, const VectorXd &intensity_map){
		int lx = static_cast<int>(x);
		double dx = x - lx;
		double mapped_x = dx == 0 ? intensity_map(lx) : (1 - dx)*intensity_map(lx) + dx*intensity_map(lx + 1);;
		//printf("Weighted mapping: x=%f lx=%d mapped x=%f\n", x, lx, mapped_x);
		//printMatrix(intensity_map, "intensity_map");
		return mapped_x;
	}
	// maps a vector of pixel values using the given intensity map; templated on the type of mapping to be done
	template<InterpType interp_type>
	inline void mapPixVals(VectorXd &dst_pix_vals, const VectorXd &src_pix_vals,
		const VectorXd &intensity_map, int n_pix){
		for(int i = 0; i < n_pix; i++){
			dst_pix_vals(i) = mapPixVal<interp_type>(src_pix_vals(i), intensity_map);
		}
	}

	/*********** functions for bicubic interpolation *******************/

	// extracts pixel values from a 4x4 grid of integral locations around the given location;
	// this grid extends in the range [x1-1, x1+2] and [y1-1, y1+2]; 
	// uses pixel duplication for points in the grid that lie outside the image extents
	// these pixels can be used for bicubic interpolation
	template<typename PixVecT>
	inline double cubicBSplInterpolate(const PixVecT &pix_vec, double dx) {
		double dx2 = dx*dx;
		double dx3 = dx2*dx;
		double dxi = 1 - dx;
		double dxi2 = dxi*dxi;
		double dxi3 = dxi2*dxi;
		return (pix_vec(0)*dxi3 + pix_vec(1)*(4 - 6 * dx2 + 3 * dx3) + pix_vec(2)*(4 - 6 * dxi2 + 3 * dxi3) + pix_vec(3)*dx3) / 6;
	}
	template<typename PixVecT>
	inline double cubicInterpolate(const PixVecT &pix_vec, double x) {
		return pix_vec(1) + 0.5 * x*(pix_vec(2) - pix_vec(0) + x*(2.0*pix_vec(0) - 5.0*pix_vec(1) + 4.0*pix_vec(2) - pix_vec(3) + x*(3.0*(pix_vec(1) - pix_vec(2)) + pix_vec(3) - pix_vec(0))));
	}
	bool getNeighboringPixGrid(Matrix4d &pix_grid, const EigImgT &img, double x, double y,
		int h, int w);
	void getBiCubicCoefficients(Matrix4d &bicubic_coeff, const Matrix4d &pix_grid);
	//evaluates the cubic polynomial surface defines by the given parameters at the given location
	double biCubic(const Matrix4d &bicubic_coeff, double x, double y);
	//evaluates the gradient of cubic polynomial surface defines by the given parameters at the given location
	double biCubicGradX(Vector2d &grad, const Matrix4d &bicubic_coeff, double x, double y);
	double biCubicGradY(Vector2d &grad, const Matrix4d &bicubic_coeff, double x, double y);
	//evaluates the hessian of the cubic polynomial surface defines by the given parameters at the given location
	double biCubicHessXX(const Matrix4d &bicubic_coeff, double x, double y);
	double biCubicHessYY(const Matrix4d &bicubic_coeff, double x, double y);
	double biCubicHessYX(const Matrix4d &bicubic_coeff, double x, double y);
	double biCubicHessXY(const Matrix4d &bicubic_coeff, double x, double y);

	//! Miscellaneous image related utility functions
	cv::Mat convertFloatImgToUchar(cv::Mat &img, int nchannels);
	void generateWarpedImg(cv::Mat &warped_img, const cv::Mat &warped_corners,
		const mtf::PtsT &warped_pts, const mtf::PixValT orig_patch,
		const cv::Mat &orig_img, int img_width, int img_height,
		int n_pts, int background_type, bool show_warped_img = false,
		const char* win_name = "warped_img");
	template<typename ScalarType = float>
	void generateInverseWarpedImg(cv::Mat &warped_img, const mtf::PtsT &warped_pts,
		const cv::Mat &orig_img, const mtf::PtsT &orig_pts,
		int img_width, int img_height, int n_pts, bool show_warped_img = false,
		const char* win_name = "warped_img");
	void generateInverseWarpedImg(cv::Mat &warped_img, const PixValT &pix_vals,
		int img_width, int img_height, int n_pts, bool show_warped_img,
		const char* win_name = "warped_img");
	//! write pixel values in the given image at the given locations
	void writePixelsToImage(cv::Mat &img, const PixValT &pix_vals,
		const mtf::PtsT &pts, int n_channels, cv::Mat &mask);
	void writePixelsToImage(cv::Mat &img, const cv::Mat &pix_vals,
		const mtf::PtsT &pts, int n_channels, cv::Mat &mask);
	void writePixelsToImage(cv::Mat &img, const cv::Mat &pix_vals,
		const mtf::CornersT &corners, int n_channels, cv::Mat &mask);
	void writePixelsToImage(cv::Mat &img, const cv::Mat &pix_vals,
		const cv::Mat &corners, int n_channels, cv::Mat &mask);
	//! add Gaussian distributed random noise to the image
	bool addGaussianNoise(const cv::Mat mSrc, cv::Mat &mDst,
		int n_channels, double Mean = 0.0, double StdDev = 10.0);
	cv::Mat reshapePatch(const VectorXd &curr_patch,
		int img_height, int img_width, int n_channels = 1);
	VectorXd reshapePatch(const cv::Mat &cv_patch,
		int start_x = 0, int start_y = 0,
		int end_x = -1, int end_y = -1);


#ifdef ENABLE_OLD_IMG_GRAD

	/*********** compute image gradient at a given pixel location *******************/

	// takes homogeneous points
	inline double getPixGrad(const EigImgT &img,
		const Vector3d &pt_inc, const Vector3d &pt_dec,
		double grad_mult_factor, int h, int w){
		double pix_val_inc = getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img, pt_inc(0) / pt_inc(2),
			pt_inc(1) / pt_inc(2), h, w);
		double pix_val_dec = getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img, pt_dec(0) / pt_dec(2),
			pt_dec(1) / pt_dec(2), h, w);
		return (pix_val_inc - pix_val_dec)*grad_mult_factor;
	}
	// takes non-homogeneous points
	inline double getPixGrad(const EigImgT &img,
		const Vector2d &pt_inc, const Vector2d &pt_dec,
		double grad_mult_factor, int h, int w){
		double pix_val_inc = getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img, pt_inc(0), pt_inc(1), h, w);
		double pix_val_dec = getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img, pt_dec(0), pt_dec(1), h, w);
		return (pix_val_inc - pix_val_dec)*grad_mult_factor;
	}
	// uses an intensity map to remap the pixel values before computing their difference; 
	// templated on the type of mapping to be done: either floor or linear weighting
	// takes homogeneous points
	template<InterpType mapping_type>
	inline double getPixGrad(const EigImgT &img, const VectorXd &intensity_map,
		const Vector3d &pt_inc, const Vector3d &pt_dec,
		double grad_mult_factor, int h, int w){
		double pix_val_inc = mapPixVal<mapping_type>(getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img, pt_inc(0) / pt_inc(2),
			pt_inc(1) / pt_inc(2), h, w), intensity_map);
		double pix_val_dec = mapPixVal<mapping_type>(getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img, pt_dec(0) / pt_dec(2),
			pt_dec(1) / pt_dec(2), h, w), intensity_map);
		return (pix_val_inc - pix_val_dec)*grad_mult_factor;
	}
	// takes non-homogeneous points
	template<InterpType mapping_type>
	inline double getPixGrad(const EigImgT &img, const VectorXd &intensity_map,
		const Vector2d &pt_inc, const Vector2d &pt_dec,
		double grad_mult_factor, int h, int w){
		double pix_val_inc = mapPixVal<mapping_type>(getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img, pt_inc(0), pt_inc(1), h, w), intensity_map);
		double pix_val_dec = mapPixVal<mapping_type>(getPixVal<GRAD_INTERP_TYPE, PIX_BORDER_TYPE>(img, pt_dec(0), pt_dec(1), h, w), intensity_map);
		return (pix_val_inc - pix_val_dec)*grad_mult_factor;
	}

	/*********** compute image Hessian at a given pixel location ***********/

	// compute diagonal entries of the pixel Hessian matrix
	// takes homogeneous points
	inline double getPixHess(const EigImgT &img,
		const Vector3d &pt_inc, const Vector3d &pt_dec,
		double pix_val, double hess_mult_factor, int h, int w){
		double pix_val_inc = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, pt_inc(0) / pt_inc(2),
			pt_inc(1) / pt_inc(2), h, w);
		double pix_val_dec = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, pt_dec(0) / pt_dec(2),
			pt_dec(1) / pt_dec(2), h, w);
		return (pix_val_inc + pix_val_dec - 2 * pix_val)*hess_mult_factor;
	}
	// takes non-homogeneous points
	inline double getPixHess(const EigImgT &img,
		const Vector2d &pt_inc, const Vector2d &pt_dec,
		double pix_val, double hess_mult_factor, int h, int w){
		double pix_val_inc = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, pt_inc(0), pt_inc(1), h, w);
		double pix_val_dec = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, pt_dec(0), pt_dec(1), h, w);
		return (pix_val_inc + pix_val_dec - 2 * pix_val)*hess_mult_factor;
	}
	// compute off diagonal entries of the pixel Hessian matrix
	// takes homogeneous points
	inline double getPixHess(const EigImgT &img,
		const Vector3d &pt_inc1, const Vector3d &pt_dec1,
		const Vector3d &pt_inc2, const Vector3d &pt_dec2,
		double hess_mult_factor, int h, int w){
		double pix_val_inc1 = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, pt_inc1(0) / pt_inc1(2),
			pt_inc1(1) / pt_inc1(2), h, w);
		double pix_val_dec1 = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, pt_dec1(0) / pt_dec1(2),
			pt_dec1(1) / pt_dec1(2), h, w);
		double pix_val_inc2 = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, pt_inc2(0) / pt_inc2(2),
			pt_inc2(1) / pt_inc2(2), h, w);
		double pix_val_dec2 = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, pt_dec2(0) / pt_dec2(2),
			pt_dec2(1) / pt_dec2(2), h, w);
		return ((pix_val_inc1 + pix_val_dec1) - (pix_val_inc2 + pix_val_dec2)) * hess_mult_factor;
	}
	// takes non-homogeneous points
	inline double getPixHess(const EigImgT &img,
		const Vector2d &pt_inc1, const Vector2d &pt_dec1,
		const Vector2d &pt_inc2, const Vector2d &pt_dec2,
		double hess_mult_factor, int h, int w){
		double pix_val_inc1 = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, pt_inc1(0), pt_inc1(1), h, w);
		double pix_val_dec1 = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, pt_dec1(0), pt_dec1(1), h, w);
		double pix_val_inc2 = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, pt_inc2(0), pt_inc2(1), h, w);
		double pix_val_dec2 = getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(img, pt_dec2(0), pt_dec2(1), h, w);
		return ((pix_val_inc1 + pix_val_dec1) - (pix_val_inc2 + pix_val_dec2)) * hess_mult_factor;
	}
	// mapped versions for homogeneous points
	template<InterpType mapping_type>
	inline double getPixHess(const EigImgT &img, const VectorXd &intensity_map,
		const Vector3d &pt_inc, const Vector3d &pt_dec,
		double pix_val, double hess_mult_factor, int h, int w){
		double pix_val_inc = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(
			img, pt_inc(0) / pt_inc(2),
			pt_inc(1) / pt_inc(2), h, w), intensity_map);
		double pix_val_dec = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(
			img, pt_dec(0) / pt_dec(2),
			pt_dec(1) / pt_dec(2), h, w), intensity_map);
		return (pix_val_inc + pix_val_dec - 2 * pix_val)*hess_mult_factor;
	}
	template<InterpType mapping_type>
	inline double getPixHess(const EigImgT &img, const VectorXd &intensity_map,
		const Vector3d &pt_inc1, const Vector3d &pt_dec1,
		const Vector3d &pt_inc2, const Vector3d &pt_dec2,
		double hess_mult_factor, int h, int w){
		double pix_val_inc1 = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(
			img, pt_inc1(0) / pt_inc1(2),
			pt_inc1(1) / pt_inc1(2), h, w), intensity_map);
		double pix_val_dec1 = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(
			img, pt_dec1(0) / pt_dec1(2),
			pt_dec1(1) / pt_dec1(2), h, w), intensity_map);
		double pix_val_inc2 = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(
			img, pt_inc2(0) / pt_inc2(2),
			pt_inc2(1) / pt_inc2(2), h, w), intensity_map);
		double pix_val_dec2 = mapPixVal<mapping_type>(getPixVal<HESS_INTERP_TYPE, PIX_BORDER_TYPE>(
			img, pt_dec2(0) / pt_dec2(2),
			pt_dec2(1) / pt_dec2(2), h, w), intensity_map);
		return ((pix_val_inc1 + pix_val_dec1) - (pix_val_inc2 + pix_val_dec2)) * hess_mult_factor;
	}
	void getWarpedImgGrad(PixGradT &warped_img_grad,
		const EigImgT &img, const Matrix3Xd &warped_pts,
		const ProjWarpT &warp, double grad_eps,
		int n_pix, int h, int w, double pix_mult_factor = 1.0);
	void getWarpedImgGrad(PixGradT &warped_img_grad,
		const EigImgT &img, const Matrix8Xd &warped_offset_pts,
		double grad_eps, int n_pix,
		int h, int w, double pix_mult_factor = 1.0);
	template<InterpType mapping_type>
	void getWarpedImgGrad(PixGradT &warped_img_grad,
		const EigImgT &img, const VectorXd &intensity_map,
		const Matrix3Xd &warped_pts, const ProjWarpT &warp,
		double grad_eps, int n_pix, int h, int w,
		double pix_mult_factor = 1.0);

	void getWarpedImgHess(PixHessT &warped_img_hess,
		const EigImgT &img, const Matrix3Xd &warped_pts_hm,
		const ProjWarpT &warp, double hess_eps,
		int n_pix, int h, int w, double pix_mult_factor = 1.0);
	template<InterpType mapping_type>
	void getWarpedImgHess(PixHessT &warped_img_hess,
		const EigImgT &img, const VectorXd &intensity_map, const Matrix3Xd &warped_pts_hm,
		const ProjWarpT &warp, double hess_eps, int n_pix,
		int h, int w, double pix_mult_factor = 1.0);

	void getImgHess(PixHessT &img_hess, const EigImgT &img,
		const PtsT &pts, double hess_eps,
		int n_pix, int h, int w, double pix_mult_factor = 1.0);
	template<InterpType mapping_type>
	void getImgHess(PixHessT &img_hess, const EigImgT &img, const VectorXd &intensity_map,
		const PtsT &pts, double hess_eps,int n_pix, int h, int w, 
		double pix_mult_factor = 1.0);

	// fits a bi cubic polynomial surface at each pixel location and computes the analytical hessian
	// of this surface thus eliminating the need for finite difference approximations
	void getImgHess(PixHessT &img_hess, const EigImgT &img,
		const PtsT &pts, int n_pix, int h, int w);

#endif
}
_MTF_END_NAMESPACE
#endif
