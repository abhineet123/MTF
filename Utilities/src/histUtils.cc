#include "mtf/Utilities/histUtils.h"
#include "mtf/Utilities/miscUtils.h"

#ifdef ENABLE_HIST_TBB
#include "tbb/tbb.h" 
#endif

_MTF_BEGIN_NAMESPACE

namespace utils{
	void getDiracHist(
		//! output arguments
		VectorXi &hist,
		//! input arguments
		const VectorXi &vals, int n_vals
		) {
		assert(vals.size() == n_vals);
		hist.fill(0);
		for(int val_id = 0; val_id < n_vals; val_id++) {
			hist(vals[val_id]) += 1;
		}
	}
#ifdef ENABLE_HIST_TBB
#include "histUtilsTBB.cc"
#else
	void getDiracHist(
		// output arguments
		VectorXd &hist, VectorXi &pix_vals_int,
		// input arguments
		const VectorXd &pix_vals, double hist_pre_seed, int n_pix
		) {
		assert(pix_vals_int.size() == n_pix && pix_vals.size() == n_pix);

		hist.fill(hist_pre_seed);
		for(int pix_id = 0; pix_id < n_pix; pix_id++) {
			int pix_int = pix_vals_int(pix_id) = static_cast<int>(pix_vals(pix_id));
			hist(pix_int) += 1;
		}
	}
	void getBilinearHist(
		// output arguments
		VectorXd &hist, VectorXi &pix_vals_int,
		// input arguments
		const VectorXd &pix_vals, double hist_pre_seed, int n_pix
		) {
		assert(pix_vals_int.size() == n_pix && pix_vals.size() == n_pix);

		hist.fill(hist_pre_seed);
		for(int pix_id = 0; pix_id < n_pix; ++pix_id) {
			int pix_int = pix_vals_int(pix_id) = static_cast<int>(pix_vals(pix_id));
			double pix_frac = pix_vals(pix_id) - pix_int;
			hist(pix_int) += 1 - pix_frac;
			if(pix_frac != 0){
				hist(pix_int + 1) = pix_frac;
			}
		}
	}
	//! computes both the histograms and their gradients simultaneously to take advantage 
	//! of the common computations involved to speed up th process
	void getBSplJointHistWithGrad(
		// output arguments
		MatrixXd &joint_hist, VectorXd &hist1,
		MatrixXd &hist1_mat, MatrixXd &hist1_grad, 
		MatrixXd &joint_hist_grad, MatrixX2i &bspl_ids1, 
		// input arguments
		const VectorXd &pix_vals1, const MatrixX2i &bspl_ids2, 
		const MatrixXd &hist2_mat, const MatrixX2i &std_bspl_ids, 
		const MatrixXi &linear_idx,	double hist_pre_seed, 
		double joint_hist_pre_seed, int n_pix, double hist_norm_mult
		) {
		assert(joint_hist.rows() == joint_hist.cols());
		assert(hist1.size() == std_bspl_ids.rows());
		assert(pix_vals1.size() == n_pix);
		assert(joint_hist_grad.cols() == n_pix);
		assert(hist1_grad.cols() == n_pix);
		assert(hist1_mat.cols() == n_pix && hist2_mat.cols() == n_pix);
		assert(bspl_ids1.rows() == n_pix && bspl_ids2.rows() == n_pix);
		assert(linear_idx.rows() == linear_idx.cols());

		hist1.fill(hist_pre_seed);
		joint_hist.fill(joint_hist_pre_seed);

		_eig_set_zero(hist1_mat, double);
		_eig_set_zero(hist1_grad, double);
		_eig_set_zero(joint_hist_grad, double);

		for(int pix_id = 0; pix_id < n_pix; pix_id++) {
			bspl_ids1.row(pix_id) = std_bspl_ids.row(static_cast<int>(pix_vals1(pix_id)));
			double curr_diff = bspl_ids1(pix_id, 0) - pix_vals1(pix_id);
			for(int id1 = bspl_ids1(pix_id, 0); id1 <= bspl_ids1(pix_id, 1); id1++) {
				bSpl3WithGrad(hist1_mat(id1, pix_id), hist1_grad(id1, pix_id), curr_diff++);
				hist1_grad(id1, pix_id) *= -hist_norm_mult;
				hist1(id1) += hist1_mat(id1, pix_id);
				for(int id2 = bspl_ids2(pix_id, 0); id2 <= bspl_ids2(pix_id, 1); id2++) {
					joint_hist(id1, id2) += hist1_mat(id1, pix_id) * hist2_mat(id2, pix_id);
					joint_hist_grad(linear_idx(id1, id2), pix_id) = hist1_grad(id1, pix_id) * hist2_mat(id2, pix_id);
				}
			}
		}
		///* normalize the histograms*/
		hist1 *= hist_norm_mult;
		joint_hist *= hist_norm_mult;
	}


	inline void bSplJointHistWithGradFast(MatrixXd &joint_hist, MatrixXd &joint_hist_grad,
		const MatrixXd &hist1_mat, const MatrixXd &hist1_grad, const MatrixXd &hist2_mat,
		const MatrixXi &linear_idx, int pix, int id1, int id2){

		joint_hist(id1, id2) += hist1_mat(id1, pix) * hist2_mat(id2, pix);
		joint_hist_grad(linear_idx(id1, id2), pix) = hist1_grad(id1, pix) * hist2_mat(id2, pix);
		++id2;

		joint_hist(id1, id2) += hist1_mat(id1, pix) * hist2_mat(id2, pix);
		joint_hist_grad(linear_idx(id1, id2), pix) = hist1_grad(id1, pix) * hist2_mat(id2, pix);
		++id2;

		joint_hist(id1, id2) += hist1_mat(id1, pix) * hist2_mat(id2, pix);
		joint_hist_grad(linear_idx(id1, id2), pix) = hist1_grad(id1, pix) * hist2_mat(id2, pix);
		++id2;

		joint_hist(id1, id2) += hist1_mat(id1, pix) * hist2_mat(id2, pix);
		joint_hist_grad(linear_idx(id1, id2), pix) = hist1_grad(id1, pix) * hist2_mat(id2, pix);
	}

	void getBSplJointHistWithGradFast(
		// output arguments
		MatrixXd &joint_hist, VectorXd &hist1,
		MatrixXd &hist1_mat, MatrixXd &hist1_grad,
		MatrixXd &joint_hist_grad, MatrixX2i &bspl_ids1,
		// input arguments
		const VectorXd &pix_vals1, const MatrixX2i &bspl_ids2,
		const MatrixXd &hist2_mat, const MatrixX2i &std_bspl_ids,
		const MatrixXi &linear_idx, double hist_pre_seed,
		double joint_hist_pre_seed, int n_pix, double hist_norm_mult
		) {
		assert(joint_hist.rows() == joint_hist.cols());
		assert(hist1.size() == std_bspl_ids.rows());
		assert(pix_vals1.size() == n_pix);
		assert(joint_hist_grad.cols() == n_pix);
		assert(hist1_grad.cols() == n_pix);
		assert(hist1_mat.cols() == n_pix && hist2_mat.cols() == n_pix);
		assert(bspl_ids1.rows() == n_pix && bspl_ids2.rows() == n_pix);
		assert(linear_idx.rows() == linear_idx.cols());

		hist1.fill(hist_pre_seed);
		joint_hist.fill(joint_hist_pre_seed);

		_eig_set_zero(hist1_mat, double);
		_eig_set_zero(hist1_grad, double);
		_eig_set_zero(joint_hist_grad, double);

		for(int pix = 0; pix < n_pix; pix++) {
			bspl_ids1.row(pix) = std_bspl_ids.row(static_cast<int>(pix_vals1(pix)));

			int hist1_id0 = bspl_ids1(pix, 0);
			int hist2_id0 = bspl_ids2(pix, 0);
			//int hist2_id1 = hist2_id0 + 1, hist2_id2 = hist2_id0 + 2, hist2_id3 = hist2_id0 + 3;

			double curr_diff = hist1_id0 - pix_vals1(pix);

			bSpl3WithGradFast<0>(hist1_mat(hist1_id0, pix), hist1_grad(hist1_id0, pix), curr_diff);
			hist1_grad(hist1_id0, pix) *= -hist_norm_mult;
			hist1(hist1_id0) += hist1_mat(hist1_id0, pix);
			bSplJointHistWithGradFast(joint_hist, joint_hist_grad,
				hist1_mat, hist1_grad, hist2_mat, linear_idx, pix, hist1_id0, hist2_id0);
			++hist1_id0;

			bSpl3WithGradFast<1>(hist1_mat(hist1_id0, pix), hist1_grad(hist1_id0, pix), ++curr_diff);
			hist1_grad(hist1_id0, pix) *= -hist_norm_mult;
			hist1(hist1_id0) += hist1_mat(hist1_id0, pix);
			bSplJointHistWithGradFast(joint_hist, joint_hist_grad,
				hist1_mat, hist1_grad, hist2_mat, linear_idx, pix, hist1_id0, hist2_id0);
			++hist1_id0;

			bSpl3WithGradFast<2>(hist1_mat(hist1_id0, pix), hist1_grad(hist1_id0, pix), ++curr_diff);
			hist1_grad(hist1_id0, pix) *= -hist_norm_mult;
			hist1(hist1_id0) += hist1_mat(hist1_id0, pix);
			bSplJointHistWithGradFast(joint_hist, joint_hist_grad,
				hist1_mat, hist1_grad, hist2_mat, linear_idx, pix, hist1_id0, hist2_id0);
			++hist1_id0;

			bSpl3WithGradFast<3>(hist1_mat(hist1_id0, pix), hist1_grad(hist1_id0, pix), ++curr_diff);
			hist1_grad(hist1_id0, pix) *= -hist_norm_mult;
			hist1(hist1_id0) += hist1_mat(hist1_id0, pix);
			bSplJointHistWithGradFast(joint_hist, joint_hist_grad,
				hist1_mat, hist1_grad, hist2_mat, linear_idx, pix, hist1_id0, hist2_id0);
		}
		hist1 *= hist_norm_mult;
		joint_hist *= hist_norm_mult;
	}

	// computes a matrix with (n_bins*n_bins) rows and n_pix columns that contains the gradient/derivative
	// of each entry of the B Spline joint histogram w.r.t. each pixel value in the first image; 
	// assuming the second image to be constant
	// it is formed by flattening the first two dimensions of the n_bins x n_bins x n_pix 3D tensor;
	void getBSplJointHistGrad(
		// output arguments
		MatrixXd &joint_hist_grad, MatrixXd &hist1_grad,
		// input arguments
		const VectorXd &pix_vals1, const MatrixXd &hist2_mat,
		const MatrixX2i &bspl_ids1, const MatrixX2i &bspl_ids2,
		const MatrixXi &linear_idx, int n_pix
		) {
		assert(joint_hist_grad.cols() == n_pix);
		assert(hist1_grad.cols() == n_pix);
		assert(pix_vals1.size() == n_pix);
		assert(hist2_mat.cols() == n_pix);
		assert(bspl_ids1.rows() == n_pix && bspl_ids2.rows() == n_pix);
		assert(linear_idx.rows() == linear_idx.cols());
		//assert((hist2_mat.array() > 0).all() && (hist2_mat.array() <= 1).all());

		hist1_grad.setZero();
		joint_hist_grad.setZero();
		for(int pix = 0; pix < n_pix; pix++) {
			double curr_diff = bspl_ids1(pix, 0) - pix_vals1(pix);
			for(int id1 = bspl_ids1(pix, 0); id1 <= bspl_ids1(pix, 1); id1++) {
				hist1_grad(id1, pix) = -bSpl3Grad(curr_diff++);
				for(int id2 = bspl_ids2(pix, 0); id2 <= bspl_ids2(pix, 1); id2++) {
					joint_hist_grad(linear_idx(id1, id2), pix) = hist1_grad(id1, pix) * hist2_mat(id2, pix);
				}
			}
		}
		//hist1_grad = -hist1_grad;
		//joint_hist_grad = -joint_hist_grad;
	}

	// computes a matrix with 'n_bins' rows and 'n_pix' columns that contains the hessian/second order derivative
	// of each bin of the B Spline histogram w.r.t. each pixel value in the image
	// here 'n_bins' is specified implicitly by the contents of pix_vals and pix_vals_int 
	// which are required to be between 0 and n_bins-1
	// assumes that the integral parts of all pixel values have been precomputed and provided
	// along with the original floating point values in separate vectors
	void getBSplHistHess(
		// output arguments
		MatrixXd &hist_hess,
		// input arguments
		const VectorXd &pix_vals, const MatrixX2i &bspl_ids, 
		int n_pix, double hist_norm_mult
		) {
		assert(hist_hess.cols() == n_pix);
		assert(pix_vals.size() == n_pix);
		assert(bspl_ids.rows() == n_pix);

		hist_hess.setZero();
		for(int pix = 0; pix < n_pix; pix++) {
			double curr_diff = bspl_ids(pix, 0) - pix_vals(pix);
			for(int id = bspl_ids(pix, 0); id <= bspl_ids(pix, 1); id++) {
				// since the ids of all bins affected by a pixel are sequential, repeated computation
				// of curr_diff can be avoided by simply incrementing it by 1 which is (hopefully) faster
				hist_hess(id, pix) = hist_norm_mult*bSpl3Hess(curr_diff++);
			}
		}
		// unlike the gradient, the hessian does not need to be inverted since two negatives make a positive
		//hist_hess = -hist_hess;
	}


	//! assumes that the gradient of the histogram has already been computed; 
	//! simply multiplies it with the also precomputed histogram matrix and arranges the result correctly;
	//! usually used in conjunction with 'getBSplHistWithGrad' for maximum speed;
	void getBSplJointHistGrad(
		// output arguments
		MatrixXd &joint_hist_grad,
		// input arguments
		const MatrixXd &hist1_grad, const MatrixXd &hist2_mat,
		const MatrixX2i &bspl_ids1, const MatrixX2i &bspl_ids2,
		const MatrixXi &linear_idx, int n_pix
		) {
		assert(joint_hist_grad.cols() == n_pix);
		assert(hist1_grad.cols() == n_pix);
		assert(hist2_mat.cols() == n_pix);
		assert(bspl_ids1.rows() == n_pix && bspl_ids2.rows() == n_pix);
		assert(linear_idx.rows() == linear_idx.cols());

		joint_hist_grad.setZero();
		for(int pix = 0; pix < n_pix; pix++) {
			for(int id1 = bspl_ids1(pix, 0); id1 <= bspl_ids1(pix, 1); id1++) {
				for(int id2 = bspl_ids2(pix, 0); id2 <= bspl_ids2(pix, 1); id2++) {
					joint_hist_grad(linear_idx(id1, id2), pix) = hist1_grad(id1, pix) * hist2_mat(id2, pix);
				}
			}
		}
	}
	//! computes both the histogram and its gradient simultaneously to take advantage 
	//! of the common computations involved thus speeding up the process
	void getBSplHistWithGrad(
		// output arguments
		VectorXd &hist, MatrixXd &hist_mat, 
		MatrixXd &hist_grad,MatrixX2i &bspl_ids, 
		// input arguments
		const VectorXd &pix_vals, const MatrixX2i &std_bspl_ids,
		double pre_seed, int n_pix, double hist_norm_mult
		) {
		assert(hist.size() == std_bspl_ids.rows());
		assert(hist_grad.cols() == n_pix);
		assert(hist_mat.cols() == n_pix);
		assert(bspl_ids.rows() == n_pix);

		hist.fill(pre_seed);
		_eig_set_zero(hist_mat, double);
		_eig_set_zero(hist_grad, double);

		for(int pix = 0; pix < n_pix; pix++) {
			bspl_ids.row(pix) = std_bspl_ids.row(static_cast<int>(pix_vals(pix)));
			double curr_diff = bspl_ids(pix, 0) - pix_vals(pix);
			for(int id = bspl_ids(pix, 0); id <= bspl_ids(pix, 1); id++) {
				bSpl3WithGrad(hist_mat(id, pix), hist_grad(id, pix), curr_diff);
				hist_grad(id, pix) *= -hist_norm_mult;
				hist(id) += hist_mat(id, pix);
				// since the ids of all bins affected by a pixel are sequential, repeated computation
				// of curr_diff can be avoided by simply incrementing it by 1 which is (hopefully) faster
				++curr_diff;
			}
		}
		hist *= hist_norm_mult;
	}

	void getMIVecGrad(
		// output arguments
		MatrixXd &mi_vec_grad,
		// input arguments
		const MatrixXd &joint_hist_grad, const VectorXd &log_hist_ratio,
		const MatrixXd &hist1_grad_ratio, const MatrixXd &joint_hist, const MatrixX2i &bspl_ids,
		const MatrixXi &linear_idx, int n_bins, int n_pix
		){
		assert(mi_vec_grad.rows() == n_bins*n_bins && mi_vec_grad.cols() == n_pix);
		assert(joint_hist_grad.cols() == n_pix);
		assert(log_hist_ratio.size() == n_bins*n_bins);
		assert(hist1_grad_ratio.rows() == n_bins && hist1_grad_ratio.cols() == n_pix);
		assert(joint_hist.rows() == n_bins && joint_hist.cols() == n_bins);
		assert(bspl_ids.rows() == n_pix);
		assert(linear_idx.rows() == linear_idx.cols());
		//mi_vec_grad = joint_hist_grad.array().colwise() * (log_hist_ratio.array() + 1);
		for(int pix = 0; pix < n_pix; pix++){
			for(int id1 = bspl_ids(pix, 0); id1 <= bspl_ids(pix, 1); id1++) {
				for(int id2 = 0; id2 < n_bins; id2++){
					int idx = linear_idx(id1, id2);
					mi_vec_grad(idx, pix) = joint_hist_grad(idx, pix) * (1 + log_hist_ratio(idx)) -
						hist1_grad_ratio(id1, pix) * joint_hist(id1, id2);
					//mi_vec_grad(idx, pix) -=  hist1_grad_ratio(id1, pix) * joint_hist(id1, id2);
				}
			}
		}
	}
#endif
	// assumes that the histogram for the second image has already been computed 
	// along with the floors of its pixel values
	void getDiracJointHist(
		// output arguments
		MatrixXd &joint_hist, VectorXd &hist1,
		// input arguments
		const VectorXd &pix_vals1, const VectorXi &pix_vals2_int,
		double hist_pre_seed, double joint_hist_pre_seed, int n_pix, int n_bins
		) {
		assert(joint_hist.rows() == n_bins && joint_hist.cols() == n_bins);
		assert(hist1.size() == n_bins && hist1.size() == n_bins);
		assert(pix_vals1.size() == n_pix && pix_vals2_int.size() == n_pix);

		hist1.fill(hist_pre_seed);
		joint_hist.fill(joint_hist_pre_seed);
		for(int pix = 0; pix < n_pix; pix++) {
			int pix1_int = static_cast<int>(pix_vals1(pix));
			int pix2_int = pix_vals2_int(pix);

			hist1(pix1_int) += 1;
			joint_hist(pix1_int, pix2_int) += 1;
		}
	}
	void getDiracJointHist(
		// output arguments
		MatrixXd &joint_hist, VectorXd &hist1, VectorXd &hist2,
		// input arguments
		const VectorXd &pix_vals1, const VectorXd &pix_vals2,
		double hist_pre_seed, double joint_hist_pre_seed, int n_pix, int n_bins
		) {
		assert(joint_hist.rows() == n_bins && joint_hist.cols() == n_bins);
		assert(hist1.size() == n_bins && hist1.size() == n_bins);
		assert(pix_vals1.size() == n_pix && pix_vals2.size() == n_pix);

		hist1.fill(hist_pre_seed);
		hist2.fill(hist_pre_seed);
		joint_hist.fill(joint_hist_pre_seed);
		for(int pix = 0; pix < n_pix; pix++) {
			int pix1_int = static_cast<int>(pix_vals1(pix));
			int pix2_int = static_cast<int>(pix_vals2(pix));

			//printf("pix1_int: %d pix2_int: %d\n", pix1_int, pix2_int);

			hist1(pix1_int) += 1;
			hist2(pix2_int) += 1;
			joint_hist(pix1_int, pix2_int) += 1;
		}
	}
	// consider only corresponding sub regions in two image patches
	void getDiracJointHist(
		// output arguments
		MatrixXd &joint_hist, VectorXd &hist1, VectorXd &hist2,
		// input arguments
		const MatrixXdMr &patch1, const MatrixXdMr &patch2,
		int start_x, int end_x, int start_y, int end_y,
		double hist_pre_seed, double joint_hist_pre_seed, int n_pix, int n_bins
		) {
		assert(joint_hist.rows() == n_bins && joint_hist.cols() == n_bins);
		assert(hist1.size() == n_bins && hist1.size() == n_bins);

		hist1.fill(hist_pre_seed);
		hist2.fill(hist_pre_seed);
		joint_hist.fill(joint_hist_pre_seed);
		for(int y = start_y; y <= end_y; y++) {
			for(int x = start_x; x <= end_x; x++) {
				int pix1_int = static_cast<int>(patch1(y, x));
				int pix2_int = static_cast<int>(patch2(y, x));
				hist1(pix1_int) += 1;
				hist2(pix2_int) += 1;
				joint_hist(pix1_int, pix2_int) += 1;
			}
		}
	}

	void getBilinearJointHist(
		// output arguments
		MatrixXd &joint_hist, VectorXd &hist1, VectorXd &hist2,
		// input arguments
		const VectorXd &pix_vals1, const VectorXd &pix_vals2,
		double hist_pre_seed, double joint_hist_pre_seed, int n_pix, int n_bins
		) {
		assert(joint_hist.rows() == n_bins && joint_hist.cols() == n_bins);
		assert(hist1.size() == n_bins && hist1.size() == n_bins);
		assert(pix_vals1.size() == n_pix && pix_vals2.size() == n_pix);

		hist1.fill(hist_pre_seed);
		hist2.fill(hist_pre_seed);
		joint_hist.fill(joint_hist_pre_seed);
		for(int pix = 0; pix < n_pix; pix++) {
			int pix1_int = static_cast<int>(pix_vals1(pix));
			int pix2_int = static_cast<int>(pix_vals2(pix));

			double r_wt = pix_vals1(pix) - pix1_int;
			double l_wt = 1.0 - r_wt;

			double b_wt = pix_vals2(pix) - pix2_int;
			double t_wt = 1 - b_wt;

			hist1(pix1_int) += l_wt;
			hist2(pix2_int) += t_wt;
			joint_hist(pix1_int, pix2_int) += l_wt*t_wt;

			if(r_wt != 0){
				hist1(pix1_int + 1) += r_wt;
				joint_hist(pix1_int + 1, pix2_int) += r_wt*t_wt;
				if(b_wt != 0){
					joint_hist(pix1_int + 1, pix2_int + 1) += r_wt*b_wt;
				}
			}
			if(b_wt != 0){
				hist2(pix2_int + 1) += b_wt;
				joint_hist(pix1_int, pix2_int + 1) += l_wt*b_wt;
			}
		}
	}

	// computes the histogram for the given image specified as a vector of pixel values.
	// Since the pixel values are allowed to be real numbers, each pixel contributes
	// to multiple bins in the histogram according to a B Spline function of order 3
	// that approximates a Gaussian distribution clipped to be non zero between +2 and -2
	// see also: bSpl3
	// also stores the floors (or integral parts) of all pixel values in a separate array since these 
	// may be needed again for computing the joint histogram of this image with another.
	void getBSplHist(
		// output arguments
		VectorXd &hist, MatrixXd &hist_mat, MatrixX2i &bspl_ids,
		// input arguments
		const VectorXd &pix_vals, const MatrixX2i &std_bspl_ids,
		double pre_seed, int n_pix
		) {
		assert(hist.size() == std_bspl_ids.rows());
		assert(hist_mat.cols() == n_pix);
		assert(pix_vals.size() == n_pix);

		hist.fill(pre_seed);
		hist_mat.setZero();
		for(int pix = 0; pix < n_pix; pix++) {
			bspl_ids.row(pix) = std_bspl_ids.row(static_cast<int>(pix_vals(pix)));
			double curr_diff = bspl_ids(pix, 0) - pix_vals(pix);
			for(int id = bspl_ids(pix, 0); id <= bspl_ids(pix, 1); id++) {
				// since the ids of all bins affected by a pixel are sequential, repeated computation
				// of curr_diff can be avoided by simply incrementing it by 1 which is (hopefully) faster
				hist(id) += hist_mat(id, pix) = bSpl3(curr_diff++);
			}
		}
	}
	void getBSplJointHist(
		// output arguments
		MatrixXd &joint_hist, VectorXd &hist1, VectorXd &hist2,
		// input arguments
		const VectorXd &pix_vals1, const VectorXd &pix_vals2,
		double hist_pre_seed, double joint_hist_pre_seed, int n_pix, int n_bins
		) {
		assert(joint_hist.rows() == n_bins && joint_hist.cols() == n_bins);
		assert(hist1.size() == n_bins && hist1.size() == n_bins);
		assert(pix_vals1.size() == n_pix && pix_vals2.size() == n_pix);

		hist1.fill(hist_pre_seed);
		hist2.fill(hist_pre_seed);
		joint_hist.fill(joint_hist_pre_seed);

		MatrixXd hist1_mat(n_bins, n_pix), hist2_mat(n_bins, n_pix);

		for(int pix = 0; pix < n_pix; pix++) {

			for(int id2 = 0; id2 < n_bins; id2++) {
				double curr_diff = id2 - pix_vals2(pix);
				hist2(id2) += hist2_mat(id2, pix) = bSpl3(curr_diff);
			}
			for(int id1 = 0; id1 < n_bins; id1++) {
				double curr_diff = id1 - pix_vals1(pix);
				hist1(id1) += hist1_mat(id1, pix) = bSpl3(curr_diff);
				for(int id2 = 0; id2 < n_bins; id2++) {
					joint_hist(id1, id2) += hist1_mat(id1, pix) * hist2_mat(id2, pix);
				}
			}
		}
	}
	void getBSplJointHist(
		// output arguments
		MatrixXd &joint_hist, VectorXd &hist1, VectorXd &hist2,
		MatrixXd &hist1_mat, MatrixXd &hist2_mat,
		MatrixX2i &bspl_ids1, MatrixX2i &bspl_ids2,
		// input arguments
		const VectorXd &pix_vals1, const VectorXd &pix_vals2,
		const MatrixX2i &std_bspl_ids, double hist_pre_seed,
		double joint_hist_pre_seed, int n_pix
		) {

		assert(pix_vals1.size() == n_pix && pix_vals2.size() == n_pix);

		hist1.fill(hist_pre_seed);
		hist2.fill(hist_pre_seed);
		joint_hist.fill(joint_hist_pre_seed);
		for(int pix = 0; pix < n_pix; pix++) {
			bspl_ids2.row(pix) = std_bspl_ids.row(static_cast<int>(pix_vals2(pix)));
			double curr_diff2 = bspl_ids2(pix, 0) - pix_vals2(pix);
			for(int id2 = bspl_ids2(pix, 0); id2 <= bspl_ids2(pix, 1); id2++) {
				hist2(id2) += hist2_mat(id2, pix) = bSpl3(curr_diff2++);
			}
			bspl_ids1.row(pix) = std_bspl_ids.row(static_cast<int>(pix_vals1(pix)));
			double curr_diff1 = bspl_ids1(pix, 0) - pix_vals1(pix);
			for(int id1 = bspl_ids1(pix, 0); id1 <= bspl_ids1(pix, 1); id1++) {
				hist1(id1) += hist1_mat(id1, pix) = bSpl3(curr_diff1++);
				for(int id2 = bspl_ids2(pix, 0); id2 <= bspl_ids2(pix, 1); id2++) {
					joint_hist(id1, id2) += hist1_mat(id1, pix) * hist2_mat(id2, pix);
				}
			}
		}
	}
	// consider only corresponding sub regions in two image patches
	void getBSplJointHist(
		// output arguments
		MatrixXd &joint_hist, VectorXd &hist1, VectorXd &hist2,
		MatrixXd &hist1_mat, MatrixXd &hist2_mat,
		// input arguments
		const MatrixXdMr &patch1, const MatrixXdMr &patch2,
		int start_x, int end_x, int start_y, int end_y,
		const MatrixX2i &std_bspl_ids, double hist_pre_seed,
		double joint_hist_pre_seed, int n_pix
		) {
		hist1.fill(hist_pre_seed);
		hist2.fill(hist_pre_seed);
		joint_hist.fill(joint_hist_pre_seed);
		int pix_id = 0;
		for(int y = start_y; y <= end_y; y++) {
			for(int x = start_x; x <= end_x; x++) {
				int pix2_int = static_cast<int>(patch2(y, x));
				double curr_diff2 = std_bspl_ids(pix2_int, 0) - patch2(y, x);
				for(int id2 = std_bspl_ids(pix2_int, 0); id2 <= std_bspl_ids(pix2_int, 1); id2++) {
					hist2(id2) += hist2_mat(id2, pix_id) = bSpl3(curr_diff2++);
				}
				int pix1_int = static_cast<int>(patch1(y, x));
				double curr_diff1 = std_bspl_ids(pix1_int, 0) - patch1(y, x);
				for(int id1 = std_bspl_ids(pix1_int, 0); id1 <= std_bspl_ids(pix1_int, 1); id1++) {
					hist1(id1) += hist1_mat(id1, pix_id) = bSpl3(curr_diff1++);
					for(int id2 = std_bspl_ids(pix2_int, 0); id2 <= std_bspl_ids(pix2_int, 1); id2++) {
						joint_hist(id1, id2) += hist1_mat(id1, pix_id) * hist2_mat(id2, pix_id);
					}
				}
				pix_id++;
			}
		}
	}


	// this assumes that the histogram of the second image (typically the reference or initial image)
	// has already been computed, also assumes that the 'floor' or integral parts of the pixels values 
	// of this image are available (computed with its histogram) to avoid repeated computation;
	// computes the histogram for the first image and uses the two histograms to compute the joint histogram
	void getBSplJointHist(
		// output arguments
		MatrixXd &joint_hist, VectorXd &hist1,
		MatrixXd &hist1_mat, MatrixX2i &bspl_ids1,
		// input arguments
		const VectorXd &pix_vals1, const MatrixX2i &bspl_ids2,
		const MatrixXd &hist2_mat, const MatrixX2i &std_bspl_ids,
		double hist_pre_seed, double joint_hist_pre_seed, int n_pix
		) {
		assert(hist1.size() == std_bspl_ids.rows());
		assert(bspl_ids1.rows() == n_pix && bspl_ids2.rows() == n_pix);
		assert(hist1_mat.cols() == n_pix && hist2_mat.cols() == n_pix);
		assert(pix_vals1.size() == n_pix);

		hist1.fill(hist_pre_seed);
		joint_hist.fill(joint_hist_pre_seed);
		hist1_mat.setZero();
		for(int pix = 0; pix < n_pix; pix++) {
			bspl_ids1.row(pix) = std_bspl_ids.row(static_cast<int>(pix_vals1(pix)));
			double curr_diff = bspl_ids1(pix, 0) - pix_vals1(pix);
			for(int id1 = bspl_ids1(pix, 0); id1 <= bspl_ids1(pix, 1); id1++) {
				hist1(id1) += hist1_mat(id1, pix) = bSpl3(curr_diff++);
				for(int id2 = bspl_ids2(pix, 0); id2 <= bspl_ids2(pix, 1); id2++) {
					joint_hist(id1, id2) += hist1_mat(id1, pix) * hist2_mat(id2, pix);
				}
			}
		}
	}
	// computes the joint histogram of an image with itself
	void getBSplJointHist(
		// output arguments
		MatrixXd &joint_hist,
		// input arguments
		const MatrixXd &hist_mat, const MatrixX2i &bspl_ids,
		double pre_seed, int n_pix
		) {
		assert(joint_hist.rows() == joint_hist.cols());
		assert(hist_mat.cols() == n_pix);
		assert(bspl_ids.rows() == n_pix);

		joint_hist.fill(pre_seed);
		for(int pix = 0; pix < n_pix; pix++) {
			for(int id1 = bspl_ids(pix, 0); id1 <= bspl_ids(pix, 1); id1++) {
				for(int id2 = bspl_ids(pix, 0); id2 <= bspl_ids(pix, 1); id2++) {
					joint_hist(id1, id2) += hist_mat(id1, pix) * hist_mat(id2, pix);
				}
			}
		}
	}

	// computes a matrix with 'n_bins' rows and 'n_pix' columns that contains the gradient/derivative of each bin of the 
	// B Spline histogram w.r.t. each pixel value in the image
	// here 'n_bins' is specified implicitly by the contents of pix_vals and pix_vals_int 
	// which are required to be between 0 and n_bins-1
	// assumes that the integral parts of all pixel values have been precomputed and provided
	// along with the original floating point values in separate vectors
	void getBSplHistGrad(
		// output arguments
		MatrixXd &hist_grad,
		// input arguments
		const VectorXd &pix_vals, const MatrixX2i &bspl_ids, int n_pix
		) {
		assert(hist_grad.cols() == n_pix);
		assert(pix_vals.size() == n_pix);
		assert(bspl_ids.rows() == n_pix);

		hist_grad.setZero();
		for(int pix = 0; pix < n_pix; pix++) {
			double curr_diff = bspl_ids(pix, 0) - pix_vals(pix);
			for(int id = bspl_ids(pix, 0); id <= bspl_ids(pix, 1); id++) {
				// since the ids of all bins affected by a pixel are sequential, repeated computation
				// of curr_diff can be avoided by simply incrementing it by 1 which is (hopefully) faster
				hist_grad(id, pix) = -bSpl3Grad(curr_diff++);
			}
		}
		//since the BSpline is treated as a function of negative of the pixel value, its gradient needs to be inverted too
		//hist_grad = -hist_grad;
	}

	//---------------------------------------------------------------------------------------------------//
	//------------------------ Functions for Cumulative Cubic BSpline Histograms ------------------------//
	//---------------------------------------------------------------------------------------------------//

	void getCumBSplHistWithGrad(
		// output arguments
		VectorXd &cum_hist, MatrixXd &cum_hist_mat,
		MatrixXd &cum_hist_grad, MatrixX2i &cum_bspl_ids,
		// input arguments
		const VectorXd &pix_vals, const MatrixX2i &std_bspl_ids,
		double pre_seed, int n_bins, int n_pix, double hist_norm_mult
		) {
		assert(cum_hist.size() == std_bspl_ids.rows());
		assert(cum_hist_grad.cols() == n_pix);
		assert(cum_hist_mat.cols() == n_pix);
		assert(cum_bspl_ids.rows() == n_pix);

		cum_hist.fill(pre_seed);

		for(int pix = 0; pix < n_pix; pix++) {
			cum_bspl_ids.row(pix) = std_bspl_ids.row(static_cast<int>(pix_vals(pix)));
			int cum_hist_id = 0;
			while(cum_hist_id < cum_bspl_ids(pix, 0)){
				cum_hist_grad(cum_hist_id, pix) = 0;
				cum_hist(cum_hist_id) += cum_hist_mat(cum_hist_id, pix) = 1;
				++cum_hist_id;
			}
			double curr_diff = cum_hist_id - pix_vals(pix);
			while(cum_hist_id <= cum_bspl_ids(pix, 1)){
				// since the ids of all bins affected by a pixel are sequential, repeated computation
				// of curr_diff can be avoided by simply incrementing it by 1 which is (hopefully) faster
				cumBSpl3WithGrad(cum_hist_mat(cum_hist_id, pix), cum_hist_grad(cum_hist_id, pix), curr_diff);
				cum_hist_grad(cum_hist_id, pix) *= -hist_norm_mult;
				cum_hist(cum_hist_id) += cum_hist_mat(cum_hist_id, pix);
				++curr_diff;
				++cum_hist_id;
			}
			while(cum_hist_id < n_bins){
				cum_hist_mat(cum_hist_id, pix) = cum_hist_grad(cum_hist_id, pix) = 0;
				++cum_hist_id;
			}
		}
		cum_hist *= hist_norm_mult;
	}

	// computes the cumulative joint histogram and its gradient from the 
	// precomputed histogram and cumulative histogram matrices; 
	// the gradient is computed w.r.t. the pixels used for computing the cumulative histogram
	void getCumBSplJointHistWithGrad(
		// output arguments
		MatrixXd &cum_joint_hist, MatrixXd &cum_joint_hist_grad,
		// input arguments
		const MatrixXd &cum_hist_mat, const MatrixXd &hist_mat,
		const MatrixXd &cum_hist_grad, const MatrixX2i &cum_bspl_ids,
		const MatrixX2i &bspl_ids, const MatrixXi &linear_idx,	
		double pre_seed, int n_bins, int n_pix
		) {
		assert(cum_joint_hist.rows() == cum_joint_hist.cols());
		assert(hist_mat.rows() == n_bins && hist_mat.cols() == n_pix);
		assert(bspl_ids.rows() == n_pix);

		cum_joint_hist.fill(pre_seed);
		_eig_set_zero(cum_joint_hist_grad, double);

		for(int pix_id = 0; pix_id < n_pix; pix_id++) {
			for(int hist_id = bspl_ids(pix_id, 0); hist_id <= bspl_ids(pix_id, 1); ++hist_id) {
				int cum_hist_id = 0;
				while(cum_hist_id < cum_bspl_ids(pix_id, 0)){
					cum_joint_hist(cum_hist_id, hist_id) += hist_mat(hist_id, pix_id);
					++cum_hist_id;
				}
				while(cum_hist_id <= cum_bspl_ids(pix_id, 1)){
					cum_joint_hist(cum_hist_id, hist_id) += cum_hist_mat(cum_hist_id, pix_id) * hist_mat(hist_id, pix_id);
					cum_joint_hist_grad(linear_idx(cum_hist_id, hist_id), pix_id) = cum_hist_grad(cum_hist_id, pix_id) * hist_mat(hist_id, pix_id);
					++cum_hist_id;
				}
			}
		}
	}
	// computes the marginal histogram and its matrix too
	void getCumBSplJointHistWithGrad(
		MatrixXd &cum_joint_hist, MatrixXd &cum_joint_hist_grad,
		VectorXd &hist, MatrixXd &hist_mat,
		// input arguments
		const VectorXd &pix_vals, const MatrixXd &cum_hist_mat,
		const MatrixXd &cum_hist_grad, const MatrixX2i &cum_bspl_ids, 
		const MatrixX2i &bspl_ids, const MatrixXi &linear_idx, 
		double pre_seed, double joint_pre_seed,	int n_bins, int n_pix
		) {
		assert(cum_joint_hist.rows() == cum_joint_hist.cols());
		assert(hist_mat.rows() == n_bins && hist_mat.cols() == n_pix);
		assert(bspl_ids.rows() == n_pix);

		hist.fill(pre_seed);
		cum_joint_hist.fill(joint_pre_seed);

		_eig_set_zero(hist_mat, double);
		_eig_set_zero(cum_joint_hist_grad, double);

		for(int pix_id = 0; pix_id < n_pix; pix_id++) {
			double curr_diff = bspl_ids(pix_id, 0) - pix_vals(pix_id);
			for(int hist_id = bspl_ids(pix_id, 0); hist_id <= bspl_ids(pix_id, 1); ++hist_id) {
				// since the ids of all bins affected by a pixel are sequential, repeated computation
				// of curr_diff can be avoided by simply incrementing it by 1 which is (hopefully) faster
				hist_mat(hist_id, pix_id) = utils::bSpl3(curr_diff++);
				hist(hist_id) += hist_mat(hist_id, pix_id);
				int cum_hist_id = 0;
				while(cum_hist_id < cum_bspl_ids(pix_id, 0)){
					cum_joint_hist(cum_hist_id, hist_id) += hist_mat(hist_id, pix_id);
					++cum_hist_id;
				}
				while(cum_hist_id <= cum_bspl_ids(pix_id, 1)){
					cum_joint_hist(cum_hist_id, hist_id) += cum_hist_mat(cum_hist_id, pix_id) * hist_mat(hist_id, pix_id);
					cum_joint_hist_grad(linear_idx(cum_hist_id, hist_id), pix_id) = cum_hist_grad(cum_hist_id, pix_id) * hist_mat(hist_id, pix_id);
					++cum_hist_id;
				}
			}	
		}
	}

	void getCumBSplJointHistWithGrad(
		// output arguments
		MatrixXd &cum_joint_hist, VectorXd &cum_hist,
		MatrixXd &cum_hist_mat, MatrixXd &cum_hist_grad,
		MatrixXd &cum_joint_hist_grad, MatrixX2i &cum_bspl_ids,
		// input arguments
		const VectorXd &pix_vals, const MatrixX2i &bspl_ids,
		const MatrixXd &hist_mat, const MatrixX2i &std_bspl_ids,
		const MatrixXi &linear_idx, double hist_pre_seed,
		double joint_hist_pre_seed, int n_bins, int n_pix,
		double hist_norm_mult
		) {
		assert(cum_joint_hist.rows() == cum_joint_hist.cols());
		assert(cum_hist.size() == std_bspl_ids.rows());
		assert(pix_vals.size() == n_pix);
		assert(cum_joint_hist_grad.cols() == n_pix);
		assert(cum_hist_grad.cols() == n_pix);
		assert(cum_hist_mat.cols() == n_pix && hist_mat.cols() == n_pix);
		assert(cum_bspl_ids.rows() == n_pix && bspl_ids.rows() == n_pix);
		assert(linear_idx.rows() == linear_idx.cols());

		cum_hist.fill(hist_pre_seed);
		cum_joint_hist.fill(joint_hist_pre_seed);

		_eig_set_zero(cum_hist_grad, double);
		_eig_set_zero(cum_joint_hist_grad, double);

		//cum_joint_hist_grad.setZero();
		for(int pix_id = 0; pix_id < n_pix; pix_id++) {
			cum_bspl_ids.row(pix_id) = std_bspl_ids.row(static_cast<int>(pix_vals(pix_id)));
			int cum_hist_id = 0;
			while(cum_hist_id < cum_bspl_ids(pix_id, 0)){// cum_hist_mat is 1 and cum_hist_grad is zero but the latter has already been zeroed
				cum_hist(cum_hist_id) += cum_hist_mat(cum_hist_id, pix_id) = 1;
				for(int hist_id = bspl_ids(pix_id, 0); hist_id <= bspl_ids(pix_id, 1); hist_id++) {
					cum_joint_hist(cum_hist_id, hist_id) +=  hist_mat(hist_id, pix_id);
				}
				++cum_hist_id;
			}
			double curr_diff = cum_hist_id - pix_vals(pix_id);
			while(cum_hist_id <= cum_bspl_ids(pix_id, 1)){
				// since the ids of all bins affected by a pix_idel are sequential, repeated computation
				// of curr_diff can be avoided by simply incrementing it by 1 which is (hopefully) faster
				cumBSpl3WithGrad(cum_hist_mat(cum_hist_id, pix_id), cum_hist_grad(cum_hist_id, pix_id), curr_diff);
				cum_hist_grad(cum_hist_id, pix_id) = -hist_norm_mult*cum_hist_grad(cum_hist_id, pix_id);
				cum_hist(cum_hist_id) += cum_hist_mat(cum_hist_id, pix_id);
				for(int hist_id = bspl_ids(pix_id, 0); hist_id <= bspl_ids(pix_id, 1); hist_id++) {
					cum_joint_hist(cum_hist_id, hist_id) += cum_hist_mat(cum_hist_id, pix_id) * hist_mat(hist_id, pix_id);
					cum_joint_hist_grad(linear_idx(cum_hist_id, hist_id), pix_id) = 
						cum_hist_grad(cum_hist_id, pix_id) * hist_mat(hist_id, pix_id);
				}
				++curr_diff;
				++cum_hist_id;
			}
		}
		cum_hist *= hist_norm_mult;
		cum_joint_hist *= hist_norm_mult;
	}

	// gradient of cumulative joint histogram w.r.t. pixels used for computing the marginal histogram
	void getInvCumBSplJointHistGrad(
		// output arguments
		MatrixXd &cum_joint_hist_grad,
		// input arguments
		const MatrixXd &hist_grad, const MatrixXd &cum_hist_mat,
		const MatrixX2i &cum_bspl_ids, const MatrixX2i &bspl_ids,
		const MatrixXi &linear_idx, int n_bins, int n_pix
		) {
		assert(cum_joint_hist_grad.cols() == n_pix);
		assert(hist_grad.cols() == n_pix);
		assert(cum_hist_mat.cols() == n_pix);
		assert(bspl_ids.rows() == n_pix);
		assert(linear_idx.rows() == linear_idx.cols());

		_eig_set_zero(cum_joint_hist_grad, double);
		for(int pix_id = 0; pix_id < n_pix; pix_id++) {
			for(int hist_id = bspl_ids(pix_id, 0); hist_id <= bspl_ids(pix_id, 1); ++hist_id) {
				int cum_hist_id = 0;
				while(cum_hist_id < cum_bspl_ids(pix_id, 0)){
					cum_joint_hist_grad(linear_idx(cum_hist_id, hist_id), pix_id) = hist_grad(hist_id, pix_id);
					++cum_hist_id;
				}
				while(cum_hist_id <= cum_bspl_ids(pix_id, 1)){
					cum_joint_hist_grad(linear_idx(cum_hist_id, hist_id), pix_id) = 
						cum_hist_mat(cum_hist_id, pix_id) * hist_grad(hist_id, pix_id);
					++cum_hist_id;
				}
			}
		}


	}

	// computes a matrix with 'n_bins' rows and 'n_pix' columns that contains the hessian/second order derivative
	// of each bin of the B Spline histogram w.r.t. each pixel value in the image
	// here 'n_bins' is specified implicitly by the contents of pix_vals and pix_vals_int 
	// which are required to be between 0 and n_bins-1
	// assumes that the integral parts of all pixel values have been precomputed and provided
	// along with the original floating point values in separate vectors
	void getCumBSplHistHess(
		// output arguments
		MatrixXd &cum_hist_hess,
		// input arguments
		const VectorXd &pix_vals, const MatrixX2i &bspl_ids,
		int n_pix, double hist_norm_mult
		) {
		assert(cum_hist_hess.cols() == n_pix);
		assert(pix_vals.size() == n_pix);
		assert(bspl_ids.rows() == n_pix);

		_eig_set_zero(cum_hist_hess, double);

		for(int pix = 0; pix < n_pix; pix++) {
			double curr_diff = bspl_ids(pix, 0) - pix_vals(pix);
			for(int id = bspl_ids(pix, 0); id <= bspl_ids(pix, 1); id++) {
				// since the ids of all bins affected by a pixel are sequential, repeated computation
				// of curr_diff can be avoided by simply incrementing it by 1 which is (hopefully) faster
				cum_hist_hess(id, pix) = hist_norm_mult*cumBSpl3Hess(curr_diff++);
			}
		}
		// unlike the gradient, the hessian does not need to be inverted since two negatives make a positive
		//hist_hess = -hist_hess;
	}

	// misc

	// computes a vector of size n_bins^2 whose sum gives the MI between the two images
	// whose normalized histograms and joint histogram are provided
	void getMIVec(
		// output arguments
		VectorXd &mi_vec, VectorXd &log_hist_ratio,
		// input arguments
		const MatrixXd &joint_hist, const VectorXd &norm_hist1,
		const VectorXd &norm_hist2, const MatrixXi &linear_idx, int n_bins
		){
		assert(mi_vec.size() == n_bins*n_bins);
		assert(log_hist_ratio.size() == n_bins*n_bins);
		assert(joint_hist.rows() == joint_hist.cols());
		assert(norm_hist1.size() == n_bins && norm_hist2.size() == n_bins);
		assert(linear_idx.rows() == linear_idx.cols());

		assert((joint_hist.array() > 0).all());
		assert((norm_hist1.array() > 0).all() && (norm_hist1.array() <= 1).all());
		assert((norm_hist2.array() > 0).all() && (norm_hist2.array() <= 1).all());

		for(int id1 = 0; id1 < n_bins; id1++){
			for(int id2 = 0; id2 < n_bins; id2++){
				int idx = linear_idx(id1, id2);
				log_hist_ratio(idx) = log(joint_hist(id1, id2) / (norm_hist1(id1) * norm_hist2(id2)));
				mi_vec(idx) = joint_hist(id1, id2) * log_hist_ratio(idx);
			}
		}
	}
	// uses precomputed log of the histograms to avoid repeated and costly log computations
	void getMIVec(
		// output arguments
		VectorXd &mi_vec, VectorXd &log_hist_ratio,
		// input arguments
		const MatrixXd &joint_hist, const MatrixXd &joint_hist_log,
		const VectorXd &hist1_log, const VectorXd &hist2_log,
		const MatrixXi &linear_idx, int n_bins
		){
		assert(mi_vec.size() == n_bins*n_bins);
		assert(log_hist_ratio.size() == n_bins*n_bins);
		assert(joint_hist.rows() == n_bins && joint_hist.cols() == n_bins);
		assert(joint_hist_log.rows() == n_bins && joint_hist_log.cols() == n_bins);
		assert(hist1_log.size() == n_bins && hist2_log.size() == n_bins);
		assert(linear_idx.rows() == linear_idx.cols());

		for(int id1 = 0; id1 < n_bins; id1++){
			for(int id2 = 0; id2 < n_bins; id2++){
				int idx = linear_idx(id1, id2);
				log_hist_ratio(idx) = joint_hist_log(id1, id2) - hist1_log(id1) - hist2_log(id2);
				mi_vec(idx) = joint_hist(id1, id2) * log_hist_ratio(idx);
			}
		}
	}

	// computes a vector of size 'n_bins' whose sum is equal to the entropy of the image
	// whose normalized histogram is provided
	void getEntropyVec(
		// output arguments
		VectorXd &entropy_vec,
		// input arguments
		const VectorXd &norm_hist, int n_bins
		){
		assert((norm_hist.array() > 0).all());
		entropy_vec.setZero();
		for(int i = 0; i < n_bins; i++){
			entropy_vec(i) = norm_hist(i) * log(1.0 / norm_hist(i));
		}
	}
	// use precomputed log of the histogram
	void getEntropyVec(
		// output arguments
		VectorXd &entropy_vec,
		// output arguments
		const VectorXd &norm_hist, const VectorXd &norm_hist_log,
		int n_bins
		){
		assert((norm_hist.array() > 0).all());
		for(int i = 0; i < n_bins; i++){
			entropy_vec(i) = -norm_hist(i) * norm_hist_log(i);
		}
	}


#define VALIDATE_PREC 1e-6

	void validateJointHist(const MatrixXd &joint_hist,
		const VectorXd &hist1, const VectorXd &hist2){

		VectorXd rw_sum = joint_hist.rowwise().sum();
		RowVectorXd cw_sum = joint_hist.colwise().sum();
		RowVectorXd hist2_t = hist2.transpose();
		//if(!((rw_sum - hist1).array() == 0).any()){
		//	printf("Sum of rows of joint histogram is not same as hist1\n");
		//	printMatrix(rw_sum, "rw_sum");
		//	printMatrix(hist1, "hist1");
		//}
		if(!rw_sum.isApprox(hist1, VALIDATE_PREC)){
			printf("Sum of rows of joint histogram is not same as hist1\n");
			printMatrix(rw_sum, "rw_sum", "%15.12f");
			printMatrix(hist1, "hist1", "%15.12f");
		}
		if(!cw_sum.isApprox(hist2_t, VALIDATE_PREC)){
			printf("Sum of columns of joint histogram is not same as hist2\n");
			printMatrix(cw_sum, "cw_sum", "%15.12f");
			printMatrix(hist2_t, "hist2_t", "%15.12f");
		}
		//assert((joint_hist.array().rowwise().sum() == hist1.array()).any());
		//assert((joint_hist.array().colwise().sum() == hist2.transpose().array()).any());
	}

	void validateJointHistGrad(const MatrixXd &joint_hist_grad,
		const MatrixXd &hist1_grad, const MatrixXd &hist2_grad,
		const MatrixXi &linear_idx, int n_bins, int n_pix){

		MatrixXd rw_sum(n_bins, n_pix);
		MatrixXd cw_sum(n_bins, n_pix);
		rw_sum.setZero();
		cw_sum.setZero();

		RowVectorXd hist1_grad_sum = hist1_grad.colwise().sum();
		RowVectorXd hist2_grad_sum = hist2_grad.colwise().sum();

		for(int i = 0; i < n_bins; i++){
			for(int j = 0; j < n_bins; j++){
				int idx1 = linear_idx(i, j);
				int idx2 = linear_idx(i, j);
				rw_sum.row(i) += joint_hist_grad.row(idx1);
				cw_sum.row(j) += joint_hist_grad.row(idx2);
			}
		}
		if(!rw_sum.isApprox(hist1_grad, VALIDATE_PREC)){
			printf("Sum of rows of joint_hist_grad is not same as hist1_grad\n");
			printMatrix(rw_sum, "rw_sum", "%15.12f");
			printMatrix(hist1_grad, "hist1_grad", "%15.12f");
		}
		if(!cw_sum.isZero(VALIDATE_PREC)){
			printf("Sum of columns of joint_hist_grad is not zero\n");
			printMatrix(cw_sum, "cw_sum", "%15.12f");
		}
		if(!hist1_grad_sum.isZero(VALIDATE_PREC)){
			printf("Sum of columns of hist1_grad is not zero\n");
			printMatrix(hist1_grad_sum, "cw_sum", "%15.12f");
		}
		if(!hist2_grad_sum.isZero(VALIDATE_PREC)){
			printf("Sum of columns of hist2_grad is not zero\n");
			printMatrix(hist2_grad_sum, "cw_sum", "%15.12f");
		}
	}
}

_MTF_END_NAMESPACE
//#endif