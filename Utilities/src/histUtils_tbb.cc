class DiracHist {
	VectorXd &hist;
	VectorXi &pix_vals_int;
	const VectorXd &pix_vals;
public:
	void operator()(const tbb::blocked_range<size_t>& r) const {
		for(size_t pix_id = r.begin(); pix_id != r.end(); ++pix_id){
			int pix_int = pix_vals_int(pix_id) = static_cast<int>(pix_vals(pix_id));
			hist(pix_int)++;
		}
	}
	DiracHist(VectorXd &hist, VectorXi &pix_vals_int,
		const VectorXd &pix_vals) : hist(hist),
		pix_vals_int(pix_vals_int), pix_vals(pix_vals){}
};
void getDiracHist(VectorXd &hist, VectorXi &pix_vals_int,
	const VectorXd &pix_vals, double hist_pre_seed, int n_pix){
	assert(pix_vals_int.size() == n_pix && pix_vals.size() == n_pix);

	hist.fill(hist_pre_seed);
	parallel_for(tbb::blocked_range<size_t>(0, n_pix), DiracHist(hist, pix_vals_int, pix_vals));
}

//! computes both the histogram and its gradient simultaneously to take advantage 
//! of the common computations involved thus speeding up the process
class BSplHistWithGrad {
	VectorXd &hist;
	MatrixXd &hist_mat;
	MatrixXd &hist_grad;
	MatrixX2i &bspline_ids;
	const VectorXd &pix_vals;
	const MatrixX2i &std_bspline_ids;
public:
	void operator()(const tbb::blocked_range<size_t>& r) const {
		for(size_t pix = r.begin(); pix != r.end(); ++pix){
			bspline_ids.row(pix) = std_bspline_ids.row(static_cast<int>(pix_vals(pix)));
			double curr_diff = bspline_ids(pix, 0) - pix_vals(pix);
			for(int id = bspline_ids(pix, 0); id <= bspline_ids(pix, 1); id++) {
				// since the ids of all bins affected by a pixel are sequential, repeated computation
				// of curr_diff can be avoided by simply incrementing it by 1 which is (hopefully) faster
				bSpl3WithGrad(hist_mat(id, pix), hist_grad(id, pix), curr_diff++);
				hist(id) += hist_mat(id, pix);
			}
		}
	}
	BSplHistWithGrad(VectorXd &hist, MatrixXd &hist_mat, MatrixXd &hist_grad,
		MatrixX2i &bspline_ids, const VectorXd &pix_vals, const MatrixX2i &std_bspline_ids) : hist(hist),
		hist_mat(hist_mat), hist_grad(hist_grad), bspline_ids(bspline_ids), 
		pix_vals(pix_vals), std_bspline_ids(std_bspline_ids){}
};
void getBSplHistWithGrad(VectorXd &hist, MatrixXd &hist_mat, MatrixXd &hist_grad,
	MatrixX2i &bspline_ids, const VectorXd &pix_vals, const MatrixX2i &std_bspline_ids,
	double pre_seed, int n_pix) {
	assert(hist.size() == std_bspline_ids.rows());
	assert(hist_grad.cols() == n_pix);
	assert(hist_mat.cols() == n_pix);
	assert(bspline_ids.rows() == n_pix);

	hist.fill(pre_seed);
	hist_mat.fill(0);
	hist_grad.fill(0);

	parallel_for(tbb::blocked_range<size_t>(0, n_pix), BSplHistWithGrad(hist, hist_mat, hist_grad,
		bspline_ids, pix_vals, std_bspline_ids));

	hist_grad = -hist_grad;
}

//! computes both the histograms and their gradients simultaneously along with the joint histogram 
//! to take advantage  of the common computations involved to speed up the process
class BSplJointHistWithGrad {
	MatrixXd &joint_hist;
	VectorXd &hist1;
	MatrixXd &hist1_mat;
	MatrixXd &hist1_grad;
	MatrixXd &joint_hist_grad;
	MatrixX2i &bspline_ids1;
	const VectorXd &pix_vals1;
	const MatrixX2i &bspline_ids2;
	const MatrixXd &hist2_mat;
	const MatrixX2i &std_bspline_ids;
	const MatrixXi &linear_idx;
public:
	void operator()(const tbb::blocked_range<size_t>& r) const {
		for(size_t pix = r.begin(); pix != r.end(); ++pix){
			bspline_ids1.row(pix) = std_bspline_ids.row(static_cast<int>(pix_vals1(pix)));
			double curr_diff = bspline_ids1(pix, 0) - pix_vals1(pix);
			for(int id1 = bspline_ids1(pix, 0); id1 <= bspline_ids1(pix, 1); id1++) {
				bSpl3WithGrad(hist1_mat(id1, pix), hist1_grad(id1, pix), curr_diff++);
				hist1(id1) += hist1_mat(id1, pix);
				for(int id2 = bspline_ids2(pix, 0); id2 <= bspline_ids2(pix, 1); id2++) {
					joint_hist(id1, id2) += hist1_mat(id1, pix) * hist2_mat(id2, pix);
					joint_hist_grad(linear_idx(id1, id2), pix) = hist1_grad(id1, pix) * hist2_mat(id2, pix);
				}
			}
		}
	}
	BSplJointHistWithGrad(MatrixXd &joint_hist, VectorXd &hist1,
		MatrixXd &hist1_mat, MatrixXd &hist1_grad, MatrixXd &joint_hist_grad, 
		MatrixX2i &bspline_ids1,
		const VectorXd &pix_vals1, const MatrixX2i &bspline_ids2,
		const MatrixXd &hist2_mat, const MatrixX2i &std_bspline_ids, const MatrixXi &linear_idx) : joint_hist(joint_hist),
		hist1(hist1), hist1_mat(hist1_mat), hist1_grad(hist1_grad), joint_hist_grad(joint_hist_grad), 
		bspline_ids1(bspline_ids1),	pix_vals1(pix_vals1), bspline_ids2(bspline_ids2), hist2_mat(hist2_mat), 
		std_bspline_ids(std_bspline_ids), linear_idx(linear_idx){}
};
void getBSplJointHistWithGrad(MatrixXd &joint_hist, VectorXd &hist1,
	MatrixXd &hist1_mat, MatrixXd &hist1_grad, MatrixXd &joint_hist_grad, MatrixX2i &bspline_ids1,
	const VectorXd &pix_vals1, const MatrixX2i &bspline_ids2,
	const MatrixXd &hist2_mat, const MatrixX2i &std_bspline_ids, const MatrixXi &linear_idx,
	double hist_pre_seed, double joint_hist_pre_seed, int n_pix) {
	assert(joint_hist.rows() == joint_hist.cols());
	assert(hist1.size() == std_bspline_ids.rows());
	assert(pix_vals1.size() == n_pix);
	assert(joint_hist_grad.cols() == n_pix);
	assert(hist1_grad.cols() == n_pix);
	assert(hist1_mat.cols() == n_pix && hist2_mat.cols() == n_pix);
	assert(bspline_ids1.rows() == n_pix && bspline_ids2.rows() == n_pix);
	assert(linear_idx.rows() == linear_idx.cols());

	hist1.fill(hist_pre_seed);
	joint_hist.fill(joint_hist_pre_seed);
	hist1_mat.fill(0);
	hist1_grad.fill(0);
	joint_hist_grad.fill(0);

	parallel_for(tbb::blocked_range<size_t>(0, n_pix), BSplJointHistWithGrad(joint_hist, hist1,
		hist1_mat, hist1_grad, joint_hist_grad,	bspline_ids1, pix_vals1, bspline_ids2,
		hist2_mat, std_bspline_ids, linear_idx));

	hist1_grad = -hist1_grad;
	joint_hist_grad = -joint_hist_grad;
}

// computes a matrix with (n_bins*n_bins) rows and n_pix columns that contains the gradient/derivative
// of each entry of the B Spline joint histogram w.r.t. each pixel value in the first image; 
// assuming the second image to be constant
// it is formed by flattening the first two dimensions of the n_bins x n_bins x n_pix 3D tensor;
class BSplJointHistGrad {
	MatrixXd &joint_hist_grad;
	MatrixXd &hist1_grad;
	const VectorXd &pix_vals1;
	const MatrixXd &hist2_mat;
	const MatrixX2i &bspline_ids1;
	const MatrixX2i &bspline_ids2;
	const MatrixXi &linear_idx;
public:
	void operator()(const tbb::blocked_range<size_t>& r) const {
		for(size_t pix = r.begin(); pix != r.end(); ++pix){
			double curr_diff = bspline_ids1(pix, 0) - pix_vals1(pix);
			for(int id1 = bspline_ids1(pix, 0); id1 <= bspline_ids1(pix, 1); id1++) {
				hist1_grad(id1, pix) = bSpl3Grad(curr_diff++);
				for(int id2 = bspline_ids2(pix, 0); id2 <= bspline_ids2(pix, 1); id2++) {
					joint_hist_grad(linear_idx(id1, id2), pix) = hist1_grad(id1, pix) * hist2_mat(id2, pix);
				}
			}
		}
	}
	BSplJointHistGrad(MatrixXd &joint_hist_grad, MatrixXd &hist1_grad,
		const VectorXd &pix_vals1, const MatrixXd &hist2_mat,
		const MatrixX2i &bspline_ids1, const MatrixX2i &bspline_ids2,
		const MatrixXi &linear_idx) : joint_hist_grad(joint_hist_grad),
		hist1_grad(hist1_grad), pix_vals1(pix_vals1), 
		hist2_mat(hist2_mat), bspline_ids1(bspline_ids1),
		bspline_ids2(bspline_ids2), linear_idx(linear_idx){}
};
void getBSplJointHistGrad(MatrixXd &joint_hist_grad, MatrixXd &hist1_grad,
	const VectorXd &pix_vals1, const MatrixXd &hist2_mat,
	const MatrixX2i &bspline_ids1, const MatrixX2i &bspline_ids2,
	const MatrixXi &linear_idx, int n_pix) {
	assert(joint_hist_grad.cols() == n_pix);
	assert(hist1_grad.cols() == n_pix);
	assert(pix_vals1.size() == n_pix);
	assert(hist2_mat.cols() == n_pix);
	assert(bspline_ids1.rows() == n_pix && bspline_ids2.rows() == n_pix);
	assert(linear_idx.rows() == linear_idx.cols());
	//assert((hist2_mat.array() > 0).all() && (hist2_mat.array() <= 1).all());

	hist1_grad.fill(0);
	joint_hist_grad.fill(0);

	parallel_for(tbb::blocked_range<size_t>(0, n_pix), BSplJointHistGrad(joint_hist_grad,
		hist1_grad, pix_vals1, hist2_mat, bspline_ids1, bspline_ids2, linear_idx));

	hist1_grad = -hist1_grad;
	joint_hist_grad = -joint_hist_grad;
}

//! assumes that the gradient of the histogram has already been computed; 
//! simply multiplies it with the also precomputed histogram matrix and arranges the result correctly;
//! usually used in conjunction with 'getBSplHistWithGrad' for maximum speed;
class BSplJointHistGradFast {
	MatrixXd &joint_hist_grad;
	const MatrixXd &hist1_grad;
	const MatrixXd &hist2_mat;
	const MatrixX2i &bspline_ids1;
	const MatrixX2i &bspline_ids2;
	const MatrixXi &linear_idx;
public:
	void operator()(const tbb::blocked_range<size_t>& r) const {
		for(size_t pix = r.begin(); pix != r.end(); ++pix){
			for(int id1 = bspline_ids1(pix, 0); id1 <= bspline_ids1(pix, 1); id1++) {
				for(int id2 = bspline_ids2(pix, 0); id2 <= bspline_ids2(pix, 1); id2++) {
					joint_hist_grad(linear_idx(id1, id2), pix) = hist1_grad(id1, pix) * hist2_mat(id2, pix);
				}
			}
		}
	}
	BSplJointHistGradFast(MatrixXd &joint_hist_grad,
		const MatrixXd &hist1_grad, const MatrixXd &hist2_mat,
		const MatrixX2i &bspline_ids1, const MatrixX2i &bspline_ids2,
		const MatrixXi &linear_idx) : joint_hist_grad(joint_hist_grad),
		hist1_grad(hist1_grad), hist2_mat(hist2_mat), bspline_ids1(bspline_ids1),
		bspline_ids2(bspline_ids2), linear_idx(linear_idx){}
};
void getBSplJointHistGrad(MatrixXd &joint_hist_grad,
	const MatrixXd &hist1_grad, const MatrixXd &hist2_mat,
	const MatrixX2i &bspline_ids1, const MatrixX2i &bspline_ids2,
	const MatrixXi &linear_idx, int n_pix) {

	assert(joint_hist_grad.cols() == n_pix);
	assert(hist1_grad.cols() == n_pix);
	assert(hist2_mat.cols() == n_pix);
	assert(bspline_ids1.rows() == n_pix && bspline_ids2.rows() == n_pix);
	assert(linear_idx.rows() == linear_idx.cols());

	joint_hist_grad.fill(0);

	parallel_for(tbb::blocked_range<size_t>(0, n_pix), BSplJointHistGradFast(joint_hist_grad,
		hist1_grad, hist2_mat, bspline_ids1, bspline_ids2, linear_idx));
}

//! compute the gradient of the MI vector
class MIVecGrad {
	MatrixXd &mi_vec_grad;
	const MatrixXd &joint_hist_grad;
	const VectorXd &log_hist_ratio;
	const MatrixXd &hist1_grad_ratio;
	const MatrixXd &joint_hist;
	const MatrixX2i &bspline_ids;
	const MatrixXi &linear_idx;
	const int n_bins;
public:
	void operator()(const tbb::blocked_range<size_t>& r) const {
		for(size_t pix = r.begin(); pix != r.end(); ++pix){
			for(int id1 = bspline_ids(pix, 0); id1 <= bspline_ids(pix, 1); id1++) {
				for(int id2 = 0; id2 < n_bins; id2++){
					int idx = linear_idx(id1, id2);
					mi_vec_grad(idx, pix) = joint_hist_grad(idx, pix) * (1 + log_hist_ratio(idx)) -
						hist1_grad_ratio(id1, pix) * joint_hist(id1, id2);
					//mi_vec_grad(idx, pix) -=  hist1_grad_ratio(id1, pix) * joint_hist(id1, id2);
				}
			}
		}
	}
	MIVecGrad(MatrixXd &mi_vec_grad,const MatrixXd &joint_hist_grad, 
		const VectorXd &log_hist_ratio,	const MatrixXd &hist1_grad_ratio, 
		const MatrixXd &joint_hist, const MatrixX2i &bspline_ids,
		const MatrixXi &linear_idx, int n_bins) : mi_vec_grad(mi_vec_grad),
		joint_hist_grad(joint_hist_grad), log_hist_ratio(log_hist_ratio), hist1_grad_ratio(hist1_grad_ratio),
		joint_hist(joint_hist), bspline_ids(bspline_ids),
		linear_idx(linear_idx), n_bins(n_bins){}
};
void getMIVecGrad(MatrixXd &mi_vec_grad,
	const MatrixXd &joint_hist_grad, const VectorXd &log_hist_ratio,
	const MatrixXd &hist1_grad_ratio, const MatrixXd &joint_hist, const MatrixX2i &bspline_ids,
	const MatrixXi &linear_idx, int n_bins, int n_pix){

	assert(mi_vec_grad.rows() == n_bins*n_bins && mi_vec_grad.cols() == n_pix);
	assert(joint_hist_grad.cols() == n_pix);
	assert(log_hist_ratio.size() == n_bins*n_bins);
	assert(hist1_grad_ratio.rows() == n_bins && hist1_grad_ratio.cols() == n_pix);
	assert(joint_hist.rows() == n_bins && joint_hist.cols() == n_bins);
	assert(bspline_ids.rows() == n_pix);
	assert(linear_idx.rows() == linear_idx.cols());

	parallel_for(tbb::blocked_range<size_t>(0, n_pix), MIVecGrad(mi_vec_grad, joint_hist_grad,
		log_hist_ratio, hist1_grad_ratio, joint_hist, bspline_ids, linear_idx, n_bins));
}

// computes a matrix with 'n_bins' rows and 'n_pix' columns that contains the hessian/second order derivative
// of each bin of the B Spline histogram w.r.t. each pixel value in the image
// here 'n_bins' is specified implicitly by the contents of pix_vals and pix_vals_int 
// which are required to be between 0 and n_bins-1
// assumes that the integral parts of all pixel values have been precomputed and provided
// along with the original floating point values in separate vectors
class BSplHistHess {
	MatrixXd &hist_hess;
	const VectorXd &pix_vals;
	const MatrixX2i &bspline_ids;
	double hist_norm_mult;
public:
	void operator()(const tbb::blocked_range<size_t>& r) const {
		for(size_t pix = r.begin(); pix != r.end(); ++pix){
			double curr_diff = bspline_ids(pix, 0) - pix_vals(pix);
			for(int id = bspline_ids(pix, 0); id <= bspline_ids(pix, 1); id++) {
				// since the ids of all bins affected by a pixel are sequential, repeated computation
				// of curr_diff can be avoided by simply incrementing it by 1 which is (hopefully) faster
				hist_hess(id, pix) = hist_norm_mult*bSpl3Hess(curr_diff++);
			}
		}
	}
	BSplHistHess(MatrixXd &_hist_hess,
		const VectorXd &_pix_vals, const MatrixX2i &_bspline_ids,
		double _hist_norm_mult) : hist_hess(_hist_hess),
		pix_vals(_pix_vals), bspline_ids(_bspline_ids),
		hist_norm_mult(_hist_norm_mult){}
};
void getBSplHistHess(
	MatrixXd &hist_hess,
	// input arguments
	const VectorXd &pix_vals, const MatrixX2i &bspline_ids, 
	int n_pix, double hist_norm_mult
	) {
	assert(hist_hess.cols() == n_pix);
	assert(pix_vals.size() == n_pix);
	assert(bspline_ids.rows() == n_pix);

	hist_hess.fill(0);

	parallel_for(tbb::blocked_range<size_t>(0, n_pix), 
		BSplHistHess(hist_hess, pix_vals, bspline_ids, hist_norm_mult));
	// unlike the gradient, the hessian does not need to be inverted since two negatives make a positive
	//hist_hess = -hist_hess;
}


