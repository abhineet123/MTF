/****************************************************************/
/********************Image Pixel Vakues********************/
/****************************************************************/

template<typename PtsT>
class PixVals {
	VectorXd &pix_vals;
	const EigImgType &img;
	const PtsT &pts;
	int h, w;
public:
	void operator()(const tbb::blocked_range<size_t>& r) const {
		double pix_val_inc, pix_val_dec;
		for(size_t pix_id = r.begin(); pix_id != r.end(); ++pix_id){
			pix_vals(pix_id) = getPixVal<PIX_INTERP_TYPE>(img, pts(0, pix_id), pts(1, pix_id), h, w);
		}
	}
	PixVals(VectorXd &pix_vals,
		const EigImgType &img, const PtsT &pts,
		int h, int w) : pix_vals(pix_vals),
		img(img), pts(pts), h(h), w(w){}
};
template<typename PtsT>
void getPixVals(VectorXd &pix_vals,
	const EigImgType &img, const PtsT &pts, int n_pix, int h, int w){
	assert(pix_vals.size() == n_pix && pts.cols() == n_pix);

	parallel_for(tbb::blocked_range<size_t>(0, n_pix),
		PixVals<PtsT>(pix_vals, img, pts, h, w));
}
template<typename PtsT>
class NormPixVals {
	VectorXd &pix_vals;
	const EigImgType &img;
	const PtsT &pts;
	double norm_mult, norm_add;
	int h, w;
public:
	void operator()(const tbb::blocked_range<size_t>& r) const {
		double pix_val_inc, pix_val_dec;
		for(size_t pix_id = r.begin(); pix_id != r.end(); ++pix_id){
			pix_vals(pix_id) = norm_mult * getPixVal<PIX_INTERP_TYPE>(img, pts(0, pix_id), pts(1, pix_id), h, w) + norm_add;
		}
	}
	NormPixVals(VectorXd &pix_vals,
		const EigImgType &img, const PtsT &pts,
		int h, int w, double norm_mult, double norm_add) : pix_vals(pix_vals),
		img(img), pts(pts), h(h), w(w), norm_mult(norm_mult), norm_add(norm_add){}
};
template<typename PtsT>
inline void getNormPixVals(VectorXd &pix_vals,
	const EigImgType &img, const PtsT &pts,
	int n_pix, int h, int w,
	double norm_mult, double norm_add){
	assert(pix_vals.size() == n_pix && pts.cols() == n_pix);

	parallel_for(tbb::blocked_range<size_t>(0, n_pix),
		NormPixVals<PtsT>(pix_vals, img, pts,
		h, w, norm_mult, norm_add));
}

/****************************************************************/
/******************** Gradient of Warped Image********************/
/****************************************************************/

class WarpedImgGrad {
	MatrixX3d &warped_img_grad;
	const EigImgType &img;
	const Matrix3Xd &warped_pts;
	const Matrix3d &warp;
	const Vector3d &diff_vec_x_warped;
	const Vector3d &diff_vec_y_warped;
	int n_pix, h, w;
	double grad_mult_factor;
public:
	void operator()(const tbb::blocked_range<size_t>& r) const {
		Vector3d pt_inc_warped, pt_dec_warped;
		for(size_t pix_id = r.begin(); pix_id != r.end(); ++pix_id){
			//printf("pix_id=%ld\n", pix_id);
			//printf("warped_img_grad.rows()=%ld\n", this->warped_img_grad.rows());
			//printf("img.rows()=%ld\n", this->img.rows());
			//printf("img.cols()=%ld\n", this->img.cols());
			//printf("warped_pts.cols()=%ld\n", this->warped_pts.cols());

			//printMatrix(diff_vec_x_warped, "diff_vec_x_warped");
			//printMatrix(diff_vec_y_warped, "diff_vec_y_warped");


			pt_inc_warped = warped_pts.col(pix_id) + diff_vec_x_warped;
			pt_dec_warped = warped_pts.col(pix_id) - diff_vec_x_warped;
			warped_img_grad(pix_id, 0) = getPixGrad(img, pt_inc_warped, pt_dec_warped,
				grad_mult_factor, h, w);

			pt_inc_warped = warped_pts.col(pix_id) + diff_vec_y_warped;
			pt_dec_warped = warped_pts.col(pix_id) - diff_vec_y_warped;
			warped_img_grad(pix_id, 1) = getPixGrad(img, pt_inc_warped, pt_dec_warped,
				grad_mult_factor, h, w);
		}
	}
	WarpedImgGrad(MatrixX3d &warped_img_grad,
		const EigImgType &img, const Matrix3Xd &warped_pts,
		const Matrix3d &warp, int n_pix,
		int h, int w, double grad_mult_factor,
		const Vector3d &diff_vec_x_warped,
		const Vector3d &diff_vec_y_warped) : warped_img_grad(warped_img_grad),
		img(img), warped_pts(warped_pts), warp(warp), diff_vec_x_warped(diff_vec_x_warped),
		diff_vec_y_warped(diff_vec_y_warped),
		n_pix(n_pix), h(h), w(w), grad_mult_factor(grad_mult_factor){
		assert(warped_img_grad.rows() == n_pix);
		//printf("warped_img_grad.rows()=%ld\n", this->warped_img_grad.rows());
		//printf("img.rows()=%ld\n", this->img.rows());
		//printf("img.cols()=%ld\n", this->img.cols());
		//printf("warped_pts.cols()=%ld\n", this->warped_pts.cols());
	}
};
void getWarpedImgGrad(MatrixX3d &warped_img_grad,
	const EigImgType &img, const Matrix3Xd &warped_pts,
	const Matrix3d &warp, double grad_eps, int n_pix,
	int h, int w, double pix_mult_factor){
	assert(warped_img_grad.rows() == n_pix);

	double grad_mult_factor = pix_mult_factor / (2 * grad_eps);

	Vector3d diff_vec_x_warped = warp.col(0) * grad_eps;
	Vector3d diff_vec_y_warped = warp.col(1) * grad_eps;
	parallel_for(tbb::blocked_range<size_t>(0, n_pix), WarpedImgGrad(warped_img_grad, img, warped_pts,
		warp, n_pix, h, w, grad_mult_factor, diff_vec_x_warped, diff_vec_y_warped));
}

/***************************************************************/
/******************** Warp of Image Gradient********************/
/***************************************************************/

class ImgGrad {
	MatrixX3d &img_grad;
	const EigImgType &img;
	const Matrix2Xd &pts;
	int h, w;
	double grad_eps;
	double grad_mult_factor;
public:
	void operator()(const tbb::blocked_range<size_t>& r) const {
		double pix_val_inc, pix_val_dec;
		for(size_t pix_id = r.begin(); pix_id != r.end(); ++pix_id){

			double curr_x = pts(0, pix_id), curr_y = pts(1, pix_id);

			pix_val_inc = getPixVal<GRAD_INTERP_TYPE>(img, curr_x + grad_eps, curr_y, h, w);
			pix_val_dec = getPixVal<GRAD_INTERP_TYPE>(img, curr_x - grad_eps, curr_y, h, w);
			img_grad(pix_id, 0) = (pix_val_inc - pix_val_dec)*grad_mult_factor;

			pix_val_inc = getPixVal<GRAD_INTERP_TYPE>(img, curr_x, curr_y + grad_eps, h, w);
			pix_val_dec = getPixVal<GRAD_INTERP_TYPE>(img, curr_x, curr_y - grad_eps, h, w);
			img_grad(pix_id, 1) = (pix_val_inc - pix_val_dec)*grad_mult_factor;
		}
	}
	ImgGrad(MatrixX3d &img_grad,
		const EigImgType &img, const Matrix2Xd &pts,
		int h, int w, double grad_eps, double grad_mult_factor) : img_grad(img_grad),
		img(img), pts(pts), h(h), w(w), 
		grad_eps(grad_eps), grad_mult_factor(grad_mult_factor){}
};
void getImgGrad(MatrixX3d &img_grad,
	const EigImgType &img, const Matrix2Xd &pts,
	double grad_eps, int n_pix, int h, int w,
	double pix_mult_factor){
	assert(img_grad.rows() == n_pix && pts.cols() == n_pix);

	double grad_mult_factor = pix_mult_factor / (2 * grad_eps);
	parallel_for(tbb::blocked_range<size_t>(0, n_pix), ImgGrad(img_grad, img, pts, h, w, grad_eps, grad_mult_factor));
}


/***************************************************************/
/******************** Hessian of Warped Image ******************/
/***************************************************************/

class WarpedImgHess {
	Matrix4Xd &warped_img_hess;
	const EigImgType &img;
	const Matrix3Xd &warped_pts_hm;
	const Matrix3d &warp;
	const Vector3d &diff_vec_xx_warped;
	const Vector3d &diff_vec_yy_warped;
	const Vector3d &diff_vec_xy_warped;
	const Vector3d &diff_vec_yx_warped;

	int n_pix, h, w;
	double hess_mult_factor;
public:
	void operator()(const tbb::blocked_range<size_t>& r) const {

		Vector3d pt_warped, pt_inc_warped, pt_dec_warped, pt_inc_warped2, pt_dec_warped2;

		for(size_t pix_id = r.begin(); pix_id != r.end(); ++pix_id){
			pt_warped = warped_pts_hm.col(pix_id);

			double pix_val = getPixVal<HESS_INTERP_TYPE>(img, pt_warped(0) / pt_warped(2),
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
	WarpedImgHess(Matrix4Xd &warped_img_hess,
		const EigImgType &img, const Matrix3Xd &warped_pts_hm,
		const Matrix3d &warp, int n_pix,
		int h, int w, 
		double hess_eps, double hess_eps2,
		double hess_mult_factor,
		const Vector3d &diff_vec_xx_warped,
		const Vector3d &diff_vec_yy_warped,
		const Vector3d &diff_vec_xy_warped,
		const Vector3d &diff_vec_yx_warped) : warped_img_hess(warped_img_hess),
		img(img), warped_pts_hm(warped_pts_hm), warp(warp),
		diff_vec_xx_warped(diff_vec_xx_warped),
		diff_vec_yy_warped(diff_vec_yy_warped),
		diff_vec_xy_warped(diff_vec_xy_warped),
		diff_vec_yx_warped(diff_vec_yx_warped),
		n_pix(n_pix), h(h), w(w), hess_mult_factor(hess_mult_factor){}
};
void getWarpedImgHess(Matrix4Xd &warped_img_hess,
	const EigImgType &img, const Matrix3Xd &warped_pts_hm,
	const Matrix3d &warp, double hess_eps, int n_pix,
	int h, int w, double pix_mult_factor){
	assert(warped_img_hess.cols() == n_pix && warped_pts_hm.cols() == n_pix);

	double hess_eps2 = 2 * hess_eps;
	double hess_mult_factor = pix_mult_factor / (hess_eps2 * hess_eps2);

	Vector3d diff_vec_xx_warped = warp.col(0) * hess_eps2;
	Vector3d diff_vec_yy_warped = warp.col(1) * hess_eps2;
	Vector3d diff_vec_xy_warped = (warp.col(0) + warp.col(1)) * hess_eps;
	Vector3d diff_vec_yx_warped = (warp.col(0) - warp.col(1)) * hess_eps;

	parallel_for(tbb::blocked_range<size_t>(0, n_pix), WarpedImgHess(warped_img_hess, img, warped_pts_hm,
		warp, n_pix, h, w, hess_eps, hess_eps2, hess_mult_factor,
		diff_vec_xx_warped, diff_vec_yy_warped,
		diff_vec_xy_warped, diff_vec_yx_warped));
}

/***************************************************************/
/******************** Warp of Image Hessian********************/
/***************************************************************/

class ImgHess {
	Matrix4Xd &img_hess;
	const EigImgType &img;
	const Matrix2Xd &pts;
	int h, w;
	double hess_eps, hess_eps2;
	double hess_mult_factor;
public:
	void operator()(const tbb::blocked_range<size_t>& r) const {
		double pix_val_inc, pix_val_dec;
		for(size_t pix_id = r.begin(); pix_id != r.end(); ++pix_id){

			double curr_x = pts(0, pix_id), curr_y = pts(1, pix_id);
			double curr_pix_val = getPixVal<HESS_INTERP_TYPE>(img, curr_x, curr_y, h, w);

			double ix_pix_val = getPixVal<HESS_INTERP_TYPE>(img, curr_x + hess_eps2, curr_y, h, w);
			double dx_pix_val = getPixVal<HESS_INTERP_TYPE>(img, curr_x - hess_eps2, curr_y, h, w);
			img_hess(0, pix_id) = (ix_pix_val + dx_pix_val - 2 * curr_pix_val) * hess_mult_factor;

			double iy_pix_val = getPixVal<HESS_INTERP_TYPE>(img, curr_x, curr_y + hess_eps2, h, w);
			double dy_pix_val = getPixVal<HESS_INTERP_TYPE>(img, curr_x, curr_y - hess_eps2, h, w);
			img_hess(3, pix_id) = (iy_pix_val + dy_pix_val - 2 * curr_pix_val) * hess_mult_factor;

			double inc_x = curr_x + hess_eps, dec_x = curr_x - hess_eps;
			double inc_y = curr_y + hess_eps, dec_y = curr_y - hess_eps;
			double ixiy_pix_val = getPixVal<HESS_INTERP_TYPE>(img, inc_x, inc_y, h, w);
			double dxdy_pix_val = getPixVal<HESS_INTERP_TYPE>(img, dec_x, dec_y, h, w);
			double ixdy_pix_val = getPixVal<HESS_INTERP_TYPE>(img, inc_x, dec_y, h, w);
			double iydx_pix_val = getPixVal<HESS_INTERP_TYPE>(img, dec_x, inc_y, h, w);
			img_hess(1, pix_id) = img_hess(2, pix_id) = ((ixiy_pix_val + dxdy_pix_val) - (ixdy_pix_val + iydx_pix_val)) * hess_mult_factor;
		}
	}
	ImgHess(Matrix4Xd &img_hess,
		const EigImgType &img, const Matrix2Xd &pts,
		int h, int w, double hess_eps, double hess_eps2,
		double hess_mult_factor) : img_hess(img_hess),
		img(img), pts(pts), h(h), w(w), 
		hess_eps(hess_eps), hess_eps2(hess_eps2),
		hess_mult_factor(hess_mult_factor){}
};
void getImgHess(Matrix4Xd &img_hess, const EigImgType &img,
	const Matrix2Xd &pts, double hess_eps,
	int n_pix, int h, int w, double pix_mult_factor){
	assert(img_hess.cols() == n_pix && pts.cols() == n_pix);

	double hess_eps2 = 2 * hess_eps;
	double hess_mult_factor = pix_mult_factor / (hess_eps2 * hess_eps2);

	parallel_for(tbb::blocked_range<size_t>(0, n_pix), ImgHess(img_hess, img, pts, h, w, 
		hess_eps, hess_eps2, hess_mult_factor));
}