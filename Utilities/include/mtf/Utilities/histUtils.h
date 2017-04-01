#ifndef MTF_HIST_UTILS_H
#define MTF_HIST_UTILS_H

//------ Functions for computing histograms, joint histograms and their derivatives------//

#include "mtf/Macros/common.h"
// precomputed constants for BSpl function
#define _1_BY_3 0.33333333333
#define _2_BY_3 0.66666666666
#define _1_BY_6 0.16666666666
#define _eig_set_zero(eig_mat, scalar_type) eig_mat.setZero()
//#define _eig_set_zero(eig_mat, scalar_type) \
//	memset(eig_mat.data(), 0, eig_mat.size()*sizeof(scalar_type))
	

_MTF_BEGIN_NAMESPACE

namespace utils{

	inline void getLog(MatrixXd &eig_log,
		const MatrixXd &eig_mat, int n_rows, int n_cols){
			for(int i = 0; i < n_rows; i++) {
				for(int j = 0; j < n_cols; j++) {
					if(eig_mat(i, j)<=0){
						eig_log(i, j) = 0;
					}else{
						eig_log(i, j) = log(eig_mat(i, j));
					}
				}
			}
	}

	inline void getLog(VectorXd &eig_log,
		const VectorXd &eig_vec, int vec_size){
			for(int i = 0; i < vec_size; i++) {
				if(eig_vec(i)<=0){
					eig_log(i) = 0;
				}else{
					eig_log(i) = log(eig_vec(i));
				}
			}
	}

	inline double cumBSpl3(double x){
		//fprintf(stdout, "x=%f\t", x);
		if(x<=-2){
			return 1;
		} else if((x>-2) && (x <= -1)){
			double temp = 2 + x;
			double temp2 = temp*temp;
			return 1.0 - (temp2*temp2) / 24;
		} else if((x>-1) && (x <= 0)){
			return 0.5 + x*(x*x*(1.0/3.0 + x / 8) - 2.0 / 3.0);
		} else if((x>0) && (x <= 1)){
			return 0.5 + x*(x*x*(1.0 / 3.0 - x / 8) - 2.0 / 3.0);
		} else if((x>1) && (x<2)){
			double temp = 2-x;
			double temp2 = temp*temp;
			return (temp2*temp2) / 24;
		}
		return 0;
	}

	template<int bspl_id>
	inline void cumBSpl3WithGradFast(double &val, double &diff, double x){
		printf("bspl_id: %d\n", bspl_id);
		throw std::invalid_argument("cumBSpl3WithGradFast :: Invalid bspl_id specified");
	}
	template<>
	inline void cumBSpl3WithGradFast<0>(double &val, double &diff, double x){
		double temp = 2 + x;
		diff = -(temp*temp*temp) / 6;
		val = 1 + (diff * temp) / 4;
	}
	template<>
	inline void cumBSpl3WithGradFast<1>(double &val, double &diff, double x){
		double x2 = x*x;
		val = 0.5 + x*(x2*(_1_BY_3 + x / 8) - _2_BY_3);
		diff = x2*(1 + x / 2) - _2_BY_3;
	}
	template<>
	inline void cumBSpl3WithGradFast<2>(double &val, double &diff, double x){
		double x2 = x*x;
		val = 0.5 + x*(x2*(_1_BY_3 - x / 8) - _2_BY_3);
		diff = x2*(1 - x / 2) - _2_BY_3;
	}
	template<>
	inline void cumBSpl3WithGradFast<3>(double &val, double &diff, double x){
		double temp = x - 2;
		diff = (temp*temp*temp) / 6;
		val = (diff * temp) / 4;
	}

	inline void cumBSpl3WithGrad(double &val, double &diff, double x){
		if(x <= -2){
			diff = 0;
			val = 1;
		} else if((x>-2) && (x <= -1)){
			double temp = 2 + x;
			diff = -(temp*temp*temp) / 6;
			val = 1 + (diff * temp) / 4;
		} else if((x>-1) && (x <= 0)){
			double x2 = x*x;
			val = 0.5 + x*(x2*(_1_BY_3 + x / 8) - _2_BY_3);
			diff = x2*(1 + x / 2) - _2_BY_3;
		} else if((x>0) && (x <= 1)){
			double x2 = x*x;
			val = 0.5 + x*(x2*(_1_BY_3 - x / 8) - _2_BY_3);
			diff = x2*(1 - x/2) - _2_BY_3;
		} else if((x>1) && (x<2)){
			double temp = x-2;
			diff = (temp*temp*temp) / 6;
			val = (diff * temp) / 4;
		} else{
			val = diff = 0;
		}
	}

	template<int bspl_id>
	inline double cumBSpl3HessFast(double x){
		printf("bspl_id: %d\n", bspl_id);
		throw std::invalid_argument("cumBSpl3HessFast :: Invalid bspl_id specified");
	}
	template<>
	inline double cumBSpl3HessFast<0>(double x){
		double temp = 2 + x;
		return -(temp*temp) / 2;
	}
	template<>
	inline double cumBSpl3HessFast<1>(double x){
		return x*(3 * x + 4) / 2;
	}
	template<>
	inline double cumBSpl3HessFast<2>(double x){
		return -x*(3 * x - 4) / 2;
	}
	template<>
	inline double cumBSpl3HessFast<3>(double x){
		double temp = 2 - x;
		return (temp * temp) / 2;
	}

	inline double cumBSpl3Hess(double x){
		//fprintf(stdout, "x=%f\t", x);
		if((x>-2) && (x <= -1)){
			double temp = 2 + x;
			return -(temp*temp) / 2;
		} else if((x>-1) && (x <= 0)){
			return x*(3 * x + 4) / 2;
		} else if((x>0) && (x <= 1)){
			return -x*(3 * x - 4) / 2;
		} else if((x>1) && (x<2)){
			double temp = 2 - x;
			return (temp * temp) / 2;
		}
		return 0;
	}

	inline double bSpl3(double x){
		//fprintf(stdout, "x=%f\t", x);
		if((x>-2) && (x<=-1)){
			double temp = 2 + x;
			return (temp*temp*temp) / 6;
		} else if((x>-1) && (x<=0)){
			return (4 - 3*x*x*(2+x)) / 6;
		} else if((x>0) && (x<=1)){
			return (4 - 3*x*x*(2-x)) / 6;
		} else if((x>1) && (x<2)){
			double temp = 2-x;
			return (temp*temp*temp) / 6;
		}
		return 0;
	}
	template<int bspl_id>
	void bSpl3WithGradFast(double &val, double &diff, double x){	
		printf("bspl_id: %d\n", bspl_id);
		throw std::invalid_argument("bSpl3WithGradFast :: Invalid bspl_id specified");
	}
	template<>
	inline void bSpl3WithGradFast<0>(double &val, double &diff, double x){
		double temp = 2 + x;
		diff = (temp * temp) / 2;
		val = (diff * temp) / 3;
	}
	template<>
	inline void bSpl3WithGradFast<1>(double &val, double &diff, double x){
		double temp = x / 2;
		val = _2_BY_3 - x*x*(1 + temp);
		diff = -x * (temp + x + 2);
	}
	template<>
	inline void bSpl3WithGradFast<2>(double &val, double &diff, double x){
		double temp = x / 2;
		val = _2_BY_3 - x*x*(1 - temp);
		diff = x * (temp + x - 2);
	}
	template<>
	inline void bSpl3WithGradFast<3>(double &val, double &diff, double x){
		double temp = 2 - x;
		diff = -(temp * temp) / 2;
		val = -(diff * temp) / 3;
	}

	inline void bSpl3WithGrad(double &val, double &diff, double x){
		if((x>-2) && (x<=-1)){
			double temp = 2 + x;			
			diff = (temp * temp) / 2;
			val = (diff * temp) / 3;
		} else if((x>-1) && (x<=0)){
			double temp = x / 2;
			val =_2_BY_3 - x*x*(1 + temp);
			diff = -x * (temp + x + 2);
		} else if((x>0) && (x<=1)){
			double temp = x / 2;
			val =_2_BY_3 - x*x*(1 - temp);
			diff = x * (temp + x -2);
		} else if((x>1) && (x<2)){
			double temp = 2 - x;
			diff = - (temp * temp) / 2;
			val = - (diff * temp) / 3 ;
		}
	}
	/** first order derivative of the
	* BSpline function of order 3
	*/
	inline double bSpl3Grad(double x){
		//fprintf(stdout, "x=%f\t", x);
		if((x>-2) && (x<=-1)){
			double temp = 2 + x;
			return (temp*temp) / 2;
		} else if((x>-1) && (x<=0)){
			return -x*(3*x + 4) / 2;
		} else if((x>0) && (x<=1)){
			return x*(3*x - 4) / 2;
		} else if((x>1) && (x<2)){
			double temp = 2 - x;
			return -(temp * temp) / 2;
		}
		return 0;
	}

	template<int bspl_id>
	inline double bSpl3HessFast(double x){
		printf("bspl_id: %d\n", bspl_id);
		throw std::invalid_argument("bSpl3HessFast :: Invalid bspl_id specified");
	}
	template<>
	inline double bSpl3HessFast<0>(double x){
		return 2 + x;
	}
	template<>
	inline double bSpl3HessFast<1>(double x){
		return -(3 * x + 2);
	}
	template<>
	inline double bSpl3HessFast<2>(double x){
		return 3 * x - 2;
	}
	template<>
	inline double bSpl3HessFast<3>(double x){
		return 2 - x;
	}

	/** second order derivative of the
	  * BSpline function of order 3
	  */
	inline double bSpl3Hess(double x){
		//fprintf(stdout, "x=%f\t", x);
		if((x>-2) && (x <= -1)){
			return 2 + x;
		} else if((x > -1) && (x <= 0)){
			return -(3 * x + 2);
		} else if((x > 0) && (x <= 1)){
			return 3 * x - 2;
		} else if((x > 1) && (x < 2)){
			return 2 - x;
		}
		return 0;
	}


	typedef void(*BSpl3WithGradPtr)(double &, double &, double);
	inline void bSpl3WithGrad0(double &val, double &diff, double x){
		double temp = 2 + x;
		diff = (temp * temp) / 2;
		val = (diff * temp) / 3;
	}
	inline void bSpl3WithGrad1(double &val, double &diff, double x){
		double temp = 2 + x;
		diff = (temp * temp) / 2;
		val = (diff * temp) / 3;
	}
	inline void bSpl3WithGrad2(double &val, double &diff, double x){
		double temp = x / 2;
		val = _2_BY_3 - x*x*(1 + temp);
		diff = -x * (temp + x + 2);
	}
	inline void bSpl3WithGrad3(double &val, double &diff, double x){
		double temp = 2 - x;
		diff = -(temp * temp) / 2;
		val = -(diff * temp) / 3;
	}
	/**
	computes histogram using the Dirac delta function to determine the bins to which each pixel contributes,
	i.e. each pixel has a unit contribution to the bin corresponding to the floor (or nearest integer) of its value;
	this method of computing histograms is fast but not differentiable
	*/
	void getDiracHist(VectorXd &hist, VectorXi &pix_vals_int,
		const VectorXd &pix_vals, double hist_pre_seed, int n_pix);
	/**
	simpler version that assumes that both input and output vectors are integral
	*/
	void getDiracHist(VectorXi &hist, const VectorXi &vals, int n_vals);
	/**
	assumes that the histogram for the second image has already been computed
	along with the floors of its pixel values
	*/
	void getDiracJointHist(MatrixXd &joint_hist, VectorXd &hist1,
		const VectorXd &pix_vals1, const VectorXi &pix_vals2_int,
		double hist_pre_seed, double joint_hist_pre_seed, int n_pix, int n_bins);
	void getDiracJointHist(MatrixXd &joint_hist, VectorXd &hist1, VectorXd &hist2,
		const VectorXd &pix_vals1, const VectorXd &pix_vals2,
		double hist_pre_seed, double joint_hist_pre_seed, int n_pix, int n_bins);
	//! consider only corresponding sub regions in two image patches
	void getDiracJointHist(MatrixXd &joint_hist, VectorXd &hist1, VectorXd &hist2,
		const MatrixXdMr &patch1, const MatrixXdMr &patch2,
		int start_x, int end_x, int start_y, int end_y,
		double hist_pre_seed, double joint_hist_pre_seed, int n_pix, int n_bins);
	/**
	computes histogram using the bilinear interpolation to determine contributions
	for the bins corresponding to the floor and ceil of each pixel
	*/
	void getBilinearHist(
		// output arguments
		VectorXd &hist, VectorXi &pix_vals_int,
		// input arguments
		const VectorXd &pix_vals, double hist_pre_seed, int n_pix
		);
	/**
	computes joint histogram using the bilinear interpolation to determine contributions 
	for the bins corresponding to the floor and ceil of each pixel
	*/
	void getBilinearJointHist(
		// output arguments
		MatrixXd &joint_hist, VectorXd &hist1, VectorXd &hist2,
		// input arguments
		const VectorXd &pix_vals1, const VectorXd &pix_vals2,
		double hist_pre_seed, double joint_hist_pre_seed, int n_pix, int n_bins
		);
	/**
	computes the histogram for the given image specified as a vector of pixel values.
	Since the pixel values are allowed to be real numbers, each pixel contributes
	to multiple bins in the histogram according to a B Spline function of order 3
	that approximates a Gaussian distribution clipped to be non zero between +2 and -2
	see also: bSpl3
	also stores the floors (or integral parts) of all pixel values in a separate array since these 
	may be needed again for computing the joint histogram of this image with another.
	*/
	void getBSplHist(VectorXd &hist, MatrixXd &hist_mat, MatrixX2i &bspl_ids, 
		const VectorXd &pix_vals, const MatrixX2i &std_bspl_ids, 
		double pre_seed, int n_pix);

	void getBSplJointHist(MatrixXd &joint_hist, VectorXd &hist1, VectorXd &hist2,
		const VectorXd &pix_vals1, const VectorXd &pix_vals2,
		double hist_pre_seed, double joint_hist_pre_seed, int n_pix, int n_bins);
	void getBSplJointHist(MatrixXd &joint_hist, VectorXd &hist1, VectorXd &hist2,
		MatrixXd &hist1_mat, MatrixXd &hist2_mat,
		MatrixX2i &bspl_ids1, MatrixX2i &bspl_ids2,
		const VectorXd &pix_vals1, const VectorXd &pix_vals2,
		const MatrixX2i &std_bspl_ids, double hist_pre_seed,
		double joint_hist_pre_seed, int n_pix);

	//! this assumes that the histogram of the second image (typically the reference or initial image)
	//! has already been computed, also assumes that the 'floor' or integral parts of the pixels values 
	//! of this image are available (computed with its histogram) to avoid repeated computation;
	//! computes the histogram for the first image and uses the two histograms to compute the joint histogram
	//!
	void getBSplJointHist(MatrixXd &joint_hist, VectorXd &hist1, 
		MatrixXd &hist1_mat, MatrixX2i &bspl_ids1, 
		const VectorXd &pix_vals1, const MatrixX2i &bspl_ids2, 
		const MatrixXd &hist2_mat, const MatrixX2i &std_bspl_ids,
		double hist_pre_seed, double joint_hist_pre_seed, int n_pix);

	// computes the joint histogram of an image with itself
	void  getBSplJointHist(MatrixXd &joint_hist, 
		const MatrixXd &hist_mat, const MatrixX2i &bspl_ids, 
		double pre_seed, int n_pix);
	
	//! computes a matrix with 'n_bins' rows and 'n_pix' columns that contains the gradient of each bin of the 
	//! B Spline histogram w.r.t. each pixel value in the image
	//! here 'n_bins' is specified implicitly by the contents of pix_vals and pix_vals_int 
	// which are therefore required to be between 0 and n_bins-1
	//! assumes that the integral parts of all pixel values have been precomputed and provided
	//! along with the original floating point values in a separate vector
	//! -----output params----- //!
	//!	@param hist_grad: preallocated matrix of size [n_bins X n_pix] that will hold the gradient of the histogram
	//!	@param	pix_vals_int: preallocated vector of size 'n_pix' that will hold the integral parts of the entries in 'pix_vals'
	//! -----input params----- //!
	//!	@param	pix_vals: vector of size 'n_pix' that contains the image's pixel values
	//!	@param	bspl_ids: n_bins X n_bins matrix of integers such that that its row 'i' contains the indices of all bins 
	//!		that a pixel whose integral part is 'i' contributes to; this matrix along with 'bspl_id_count' is used to reduce the runtime cost
	//!		by pre-computing and storing these indices since they do not change
	//!	@param	bspl_id_count: vector of size 'n_bins' that contains the number of bins that each integral pixel value contributes to; 
	//!		this is related to 'bspl_ids' since only the first bspl_id_count(i) elements of row i of bspl_ids contains valid indices
	//!
	void getBSplHistGrad(
		MatrixXd &hist_grad,
		// input arguments
		const VectorXd &pix_vals, const MatrixX2i &bspl_ids, int n_pix
		);
	void getBSplHistHess(
		MatrixXd &hist_hess,
		// input arguments
		const VectorXd &pix_vals, const MatrixX2i &bspline_ids,
		int n_pix, double hist_norm_mult
		);

	//! computes a matrix with (n_bins*n_bins) rows and n_pix columns that contains the gradient
	//! of each entry of the B Spline joint histogram w.r.t. each pixel value in the first image; 
	//! assuming the second image to be constant; simulataneusly computes the gradient of the histogram of the first image 
	//! since the latter is required to compute the former.
	//! it is formed by flattening the first two dimensions of the n_bins x n_bins x n_pix 3D tensor;
	//! -----output params----- //!
	//!	@param joint_hist_grad: preallocated matrix of size [(n_bins*n_bins) X n_pix] that is filled by this function; 
	//!		this represents a 3D tenor of size [n_bins X n_bins X n_pix] whose first two dimensions have been flattened into a vector.
	//!	@param	hist1_grad: preallocated n_bins X n_pix matrix that is filled by this function
	//! -----input params----- //!
	//!	@param	pix_vals1: column vector of size 'n_pix' that contains the first image's pixel values
	//!	@param	pix_vals2_int: column vector of size 'n_pix' that contains the integral parts of the second image's pixel values
	//!	@param	hist2_mat: n_bins X n_pix matrix that contains the contribution of each pixel of image 2 to its histogram such that
	//!		the sum of each row of this matrix gives the value of the corresponding bin in its histogram
	//!	@param	bspl_ids: n_bins X n_bins matrix of integers such that that its row 'i' contains the indices of all bins 
	//!		that a pixel whose integral part is 'i' contributes to; this matrix along with 'bspl_id_count' is used to reduce the runtime cost
	//!		by pre-computing and storing these indices since they do not change
	//!	@param	bspl_id_count: vector of size 'n_bins' that contains the number of bins that each integral pixel value contributes to; 
	//!		this is related to 'bspl_ids' since only the first bspl_id_count(i) elements of row i of bspl_ids contains valid indices
	//!	@param	linear_idx: n_bins X n_bins matrix of integers such that linear_idx(i, j) contains the index that the element at row i and column j
	//!		of a matrix of dimension n_bins X n_bins goes to when this matrix is flattened into a vector; 
	//!		this is used for determining the row indices of joint_hist_grad where each element goes;
	//!
	void getBSplJointHistGrad(MatrixXd &joint_hist_grad, MatrixXd &hist1_grad,
		const VectorXd &pix_vals1, const MatrixXd &hist2_mat, 
		const MatrixX2i &bspl_ids1, const MatrixX2i &bspl_ids2, 
		const MatrixXi &linear_idx, int n_pix);
	void getBSplJointHistGrad(MatrixXd &joint_hist_grad, 
		const MatrixXd &hist1_grad, const MatrixXd &hist2_mat, 
		const MatrixX2i &bspl_ids1, const MatrixX2i &bspl_ids2, 
		const MatrixXi &linear_idx, int n_pix);

	//! computes both the histogram and its gradient simultaneously to take advantage of the common computations involved
	void getBSplHistWithGrad(

		VectorXd &hist, MatrixXd &hist_mat, 
		MatrixXd &hist_grad, MatrixX2i &bspl_ids, 

		const VectorXd &pix_vals, const MatrixX2i &std_bspl_ids,
		double pre_seed, int n_pix, double hist_norm_mult
		);
	//! computes both the histogram and gradient of the first image as well as the joint histogram and its gradient
	//! w.r.t. the first image assuming that the histogram of the second image has already been computed
	void getBSplJointHistWithGrad(
		
		MatrixXd &joint_hist, VectorXd &hist1, 
		MatrixXd &hist1_mat, MatrixXd &hist1_grad, 
		MatrixXd &joint_hist_grad, MatrixX2i &bspl_ids1, 

		const VectorXd &pix_vals1, const MatrixX2i &bspl_ids2, 
		const MatrixXd &hist2_mat, const MatrixX2i &std_bspl_ids, const MatrixXi &linear_idx, 
		double hist_pre_seed, double joint_hist_pre_seed, int n_pix, 
		double hist_norm_mult=1
		);

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
		);


	//! computes a vector of size 'n_bins' whose sum is equal to the entropy of the image
	//! whose normalized histogram is provided
	//!
	void getEntropyVec(VectorXd &entropy_vec, const VectorXd &norm_hist, int n_bins);

	//! same as above except it uses precomputed log of the histogram to reduce computation
	//!
	void getEntropyVec(	VectorXd &entropy_vec, 
		const VectorXd &norm_hist, const VectorXd &norm_hist_log, int n_bins
		);

	//! computes a vector of size n_bins x n_bins whose sum gives the MI of the two images
	//! whose normalized histograms and joint histogram are provided
	//!
	void getMIVec(VectorXd &mi_vec, VectorXd &log_hist_ratio,
		const MatrixXd &joint_hist, const VectorXd &norm_hist1, const VectorXd &norm_hist2,
		const MatrixXi &linear_idx, int n_bins);

	//! uses precomputed log of the histograms to avoid repeated costly log computations
	//!
	void getMIVec(VectorXd &mi_vec, VectorXd &log_hist_ratio, 
		const MatrixXd &joint_hist, const MatrixXd &joint_hist_log, 
		const VectorXd &hist1_log, const VectorXd &hist2_log, const MatrixXi &linear_idx, int n_bins);

	//! computes the gradient of each entry of the MI vector w.r.t. each pixel value in the first image; 
	//! assuming the second image to be constant; simulataneusly computes the gradient of the histogram of the first image 
	//! since the latter is required to compute the former.
	//! it is formed by flattening the first two dimensions of the n_bins x n_bins x n_pix 3D tensor;
	//! -----output params----- //!
	//!	@param mi_vec_grad: preallocated matrix of size [(n_bins*n_bins) X n_pix] that will hold the gradient computed by this function
	//!		this represents a 3D tenor of size [n_bins X n_bins X n_pix] whose first two dimensions have been flattened into a vector.
	//! -----input params----- //!
	//!	@param	joint_hist_grad: column vector of size 'n_pix' that contains the first image's pixel values
	//!	@param	log_hist_ratio: column vector of size 'n_pix' that contains the integral parts of the second image's pixel values
	//!	@param	hist1_grad_ratio: n_bins X n_pix matrix that contains the contribution of each pixel of image 2 to its histogram such that
	//!		the sum of each row of this matrix gives the value of the corresponding bin in its histogram
	//!	@param	joint_hist: n_bins X n_bins matrix of integers such that that its row 'i' contains the indices of all bins 
	//!		that a pixel whose integral part is 'i' contributes to; this matrix along with 'bspl_id_count' is used to reduce the runtime cost
	//!		by pre-computing and storing these indices since they do not change
	//!	@param	bspl_id_count: vector of size 'n_bins' that contains the number of bins that each integral pixel value contributes to; 
	//!		this is related to 'bspl_ids' since only the first bspl_id_count(i) elements of row i of bspl_ids contains valid indices
	//!	@param	linear_idx: n_bins X n_bins matrix of integers such that linear_idx(i, j) contains the index that the element at row i and column j
	//!		of a matrix of dimension n_bins X n_bins goes to when this matrix is flattened into a vector; 
	//!		this is used for determining the row indices of joint_hist_grad where each element goes;
	//!
	void getMIVecGrad(MatrixXd &mi_vec_grad,
		const MatrixXd &joint_hist_grad, const VectorXd &log_hist_ratio, 
		const MatrixXd &hist1_grad_ratio, const MatrixXd &joint_hist, const MatrixX2i &bspl_ids, 
		const MatrixXi &linear_idx, int n_bins, int n_pix);


	//---------------------------------------------------------------------------------------------------//
	//------------------------ Functions for Cumulative Cubic BSpline Histograms ------------------------//
	//---------------------------------------------------------------------------------------------------//

	//! computes the cumulative cubic BSpline histogram and its gradient 
	void getCumBSplHistWithGrad(
		// output arguments
		VectorXd &cum_hist, MatrixXd &cum_hist_mat, 
		MatrixXd &cum_hist_grad, MatrixX2i &bspline_ids, 
		// input arguments
		const VectorXd &pix_vals, const MatrixX2i &std_bspline_ids,
		double pre_seed, int n_bins, 
		int n_pix, double hist_norm_mult
		);
	// cumulative joint histogram from precomputed cumulative and marginal histogram matrices 
	// which may be either from the same image or from two different images
	void getCumBSplJointHistWithGrad(
		// output arguments
		MatrixXd &cum_joint_hist, MatrixXd &cum_joint_hist_grad,
		// input arguments
		const MatrixXd &cum_hist_mat, const MatrixXd &hist_mat,
		const MatrixXd &cum_hist_grad, const MatrixX2i &cum_bspl_ids,
		const MatrixX2i &bspl_ids, const MatrixXi &linear_idx,
		double pre_seed, int n_bins, int n_pix
		);
	// computes the marginal histogram and its matrix too
	void getCumBSplJointHistWithGrad(
		// output arguments
		MatrixXd &cum_joint_hist, MatrixXd &cum_joint_hist_grad,
		VectorXd &hist, MatrixXd &hist_mat,
		// input arguments
		const VectorXd &pix_vals, const MatrixXd &cum_hist_mat,
		const MatrixXd &cum_hist_grad, const MatrixX2i &cum_bspl_ids, 
		const MatrixX2i &bspline_ids, const MatrixXi &linear_idx, 
		double pre_seed, double joint_pre_seed, int n_bins, int n_pix
		);
	void getCumBSplJointHistWithGrad(
		MatrixXd &cum_joint_hist, VectorXd &cum_hist,
		MatrixXd &cum_hist_mat, MatrixXd &cum_hist_grad,
		MatrixXd &cum_joint_hist_grad, MatrixX2i &bspline_ids1,
		// input arguments
		const VectorXd &pix_vals1, const MatrixX2i &bspline_ids2,
		const MatrixXd &hist2_mat, const MatrixX2i &std_bspline_ids, const MatrixXi &linear_idx,
		double hist_pre_seed, double joint_hist_pre_seed,
		int n_bins, int n_pix, double hist_norm_mult
		);
	// derivative of the cumulative joint histogram w.r.t. 
	// pixels used in the non cumulative component
	void getInvCumBSplJointHistGrad(
		// output arguments
		MatrixXd &cum_joint_hist_grad,
		// input arguments
		const MatrixXd &hist_grad, const MatrixXd &cum_hist_mat,
		const MatrixX2i &cum_bspl_ids, const MatrixX2i &bspl_ids, 
		const MatrixXi &linear_idx, int n_bins, int n_pix
		);
	void getCumBSplHistHess(
		// output arguments
		MatrixXd &cum_hist_hess,
		// input arguments
		const VectorXd &pix_vals,
		const MatrixX2i &bspline_ids,
		int n_pix, double hist_norm_mult
		);

	// histogram validation functions for debugging

	void validateJointHist(const MatrixXd &joint_hist, 
		const VectorXd &hist1, const VectorXd &hist2);

	void validateJointHistGrad(const MatrixXd &joint_hist_grad, 
		const MatrixXd &hist1_grad, const MatrixXd &hist2_grad, 
		const MatrixXi &linear_idx, int n_bins, int n_pix);
}
_MTF_END_NAMESPACE
#endif
