#include <fstream>      // std::ifstream
#include <iostream>     // std::cout
#include <time.h>
#include <assert.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "mtf/AM/PCA.h"
#include "mtf/Utilities/imgUtils.h"
// #define DEBUG

// Default parameters
#define PCA_N_EIGENVEC 16 // same as in ivt paper
#define PCA_BATCHSIZE 5
#define PCA_F_FACTOR 0.95
#define PCA_SHOW_BASIS 0

_MTF_BEGIN_NAMESPACE

//! value constructor
PCAParams::PCAParams(const AMParams *am_params,
int _n_eigenvec,
int _batch_size,
double _f_factor,
bool _show_basis) :
AMParams(am_params),
n_eigenvec(_n_eigenvec),
batch_size(_batch_size),
f_factor(_f_factor),
show_basis(_show_basis){}

//! default/copy constructor
PCAParams::PCAParams(const PCAParams *params) :
AMParams(params),
n_eigenvec(PCA_N_EIGENVEC),
batch_size(PCA_BATCHSIZE),
f_factor(PCA_F_FACTOR),
show_basis(PCA_SHOW_BASIS){
	if(params){
		n_eigenvec = params->n_eigenvec;
		batch_size = params->batch_size;
		f_factor = params->f_factor;
		show_basis = params->show_basis;
	}
}

PCA::PCA(const ParamType *pca_params, const int _n_channels) :
SSDBase(pca_params, _n_channels), params(pca_params){
	printf("\nInitializing PCA AM with...\n");
	printf("n_eigenvec: %d\n", params.n_eigenvec);
	printf("batch_size: %d\n", params.batch_size);
	printf("f_factor: %f\n", params.f_factor);
	printf("show_basis: %d\n", params.show_basis);
	printf("likelihood_alpha: %f\n", params.likelihood_alpha);
	name = "pca";
	batch_size = params.batch_size;
	switch(n_channels){
	case 1:
		cv_img_type_uchar = CV_8UC1;
		cv_img_type_float = CV_64FC1;
		break;
	case 3:
		cv_img_type_uchar = CV_8UC3;
		cv_img_type_float = CV_64FC3;
		break;
	default:
		throw std::domain_error(cv::format("PCA :: %d channel images are not supported yet", n_channels));
	}
}

double PCA::getLikelihood() const{
	// Convert to likelihood for particle filters.
	// Normalize over the number of pixels
	double result = exp(-params.likelihood_alpha * sqrt(-f / static_cast<double>(patch_size)));
#ifdef DEBUG
	std::cout << "The likelihood at frame " << frame_count << " is " << result << std::endl;
#endif
	return result;
}

void PCA::initializeSimilarity() {
	SSDBase::initializeSimilarity();
	max_patch_eachframe.resize(patch_size);
	//printf("Initialize PCA ");
	//printf("at frame %d.\n", frame_count);
	addi_patches = getInitPixVals();
	U_available = false;
	if(params.show_basis){
		cv::Scalar gt_color(0, 255, 0);
		VectorXd init_patch = addi_patches.col(0);
		cv::Mat template_cv, template_cv_uchar, template_cv_uchar_bigger;
		template_cv = cv::Mat(getResY(), getResX(), cv_img_type_float, init_patch.data());
		// Construct Mat object for the uint8 gray scale patch
		template_cv_uchar.create(getResY(), getResX(), cv_img_type_uchar);
		template_cv_uchar_bigger.create(5 * getResY(), 5 * getResX(), cv_img_type_uchar);
		// Convert the RGB patch to grayscale
		double min_val = init_patch.minCoeff();
		double max_val = init_patch.maxCoeff();
		double range = max_val - min_val;
		template_cv.convertTo(template_cv_uchar, template_cv_uchar.type(), 255. / range, -255.*min_val / range);
		// Make a bigger image		
		cv::resize(template_cv_uchar, template_cv_uchar_bigger, template_cv_uchar_bigger.size());
		cv::putText(template_cv_uchar_bigger, "Template", cv::Point2d(5, 15),
			cv::FONT_HERSHEY_SIMPLEX, 0.5, gt_color);
		imshow("Template Image", template_cv_uchar_bigger);
	}
}

//! Similarity: negative number, the higher the score, the more similar it is
void PCA::updateSimilarity(bool prereq_only/*=true*/) {
#ifdef DEBUG
	cout << "updateSimilarity at frame " <<  frame_count << endl;
#endif

	// update similarity of each particle
	if(U_available) {
		// if having U, update similarity of each frame using newest eigenbasis
		// error = |I_t - U U^T I_t| in an optimized way
		//    refer to https://eigen.tuxfamily.org/dox/group__TopicAliasing.html
		//             for how to use noalias() to make efficient computation
		VectorXd current_patch = getCurrPixVals();
		current_patch -= mean_prev_patches;
		I_diff = current_patch;
		I_diff.noalias() -= U * (U.transpose() * current_patch);
	} else {
		// if not having U, using SSD
		I_diff = getCurrPixVals() - getInitPixVals();
	}

	// if(prereq_only){ return; }
	f = -I_diff.squaredNorm() / 2;
#ifdef DEBUG
	std::cout << "The similarity is " << f << std::endl;
#endif

}

void PCA::updateModel(const Matrix2Xd& curr_pts){
	//display_images(getCurrPixVals(), I_diff);
	switch(input_type){
	case InputType::MTF_8UC1:
		utils::sc::getPixVals<uchar>(max_patch_eachframe, curr_img_cv, curr_pts, n_pix, img_height, img_width,
			pix_norm_mult, pix_norm_add);
		break;
	case InputType::MTF_8UC3:
		utils::mc::getPixVals<uchar>(max_patch_eachframe, curr_img_cv, curr_pts, n_pix, img_height, img_width,
			pix_norm_mult, pix_norm_add);
		break;
	case InputType::MTF_32FC1:
		utils::getPixVals(max_patch_eachframe, curr_img, curr_pts, n_pix, img_height, img_width,
			pix_norm_mult, pix_norm_add);
		break;
	case InputType::MTF_32FC3:
		utils::mc::getPixVals<float>(max_patch_eachframe, curr_img_cv, curr_pts, n_pix, img_height, img_width,
			pix_norm_mult, pix_norm_add);
		break;
	default:
		throw std::domain_error("PCA :: updateModel::Invalid input type found");
	}
	//! update the basis
	updateBasis();
}


void PCA::setFirstIter() {
	first_iter = true;
	++frame_count;
#ifdef DEBUG
	printf("It's the first particle at frame %d\n", frame_count);
#endif
}

void PCA::clearFirstIter() {
	first_iter = false;
}

void PCA::updateBasis() {
	// update addi_patches before batch_size number of frames
	if(frame_count < batch_size) {
		addi_patches.conservativeResize(NoChange, addi_patches.cols() + 1);
		addi_patches.col(addi_patches.cols() - 1) = max_patch_eachframe;
	} else if(frame_count == batch_size){
		// if just had enough images to compute the initial SVD
		n_prev_patches = batch_size;
		// append current patch to addi_patches
		addi_patches.conservativeResize(NoChange, addi_patches.cols() + 1);
		addi_patches.col(addi_patches.cols() - 1) = max_patch_eachframe;
		mean_prev_patches = addi_patches.rowwise().mean();
		// computer SVD
		JacobiSVD < MatrixXd > svd(addi_patches.colwise() - mean_prev_patches, ComputeThinU | ComputeThinV);
		U = svd.matrixU();
		sigma = svd.singularValues();
		U_available = true;
		if(params.show_basis){
			display_basis();
		}
	} else {
		if(frame_count % batch_size == 1) {
			// only have one additional patch at this point
			addi_patches = max_patch_eachframe;
		} else {
			// first update addi_patches
			addi_patches.conservativeResize(NoChange, addi_patches.cols() + 1);
			addi_patches.col(addi_patches.cols() - 1) = max_patch_eachframe;
			if(0 == (frame_count % batch_size)) {
				// if received enough frames, update new eigenbasis each batch_size of frames
				incrementalPCA();
				if(params.show_basis){
					display_basis();
				}
			}
		}

	}
}

// the core algorithm in ivt tracker: 
// sklm(Sequential Karhunen-Loeve Transform with Mean update)
// Notation follows the ivt paper Figure 1.
void PCA::sklm(MatrixXd &U, VectorXd &sigma, VectorXd &mu_A, MatrixXd &B, int &n, double ff, int max_n_eig){
	// step 1
	// mean of additional patches
	int m = B.cols();
	VectorXd mu_B = B.rowwise().mean();

	// step 2
	// Form matrix B hat
	B = B.colwise() - mu_B;
	// append additional column
	B.conservativeResize(NoChange, B.cols() + 1);
	B.col(B.cols() - 1) = sqrt(n*m / (n + m)) * (mu_B - mu_A);

	// step 3
	// compute B tilde
	MatrixXd B_proj = U.transpose() * B;
	MatrixXd B_res = B - U * B_proj;
	HouseholderQR<MatrixXd> qr(B_res);
	MatrixXd thinQ(MatrixXd::Identity(B_res.rows(), B_res.cols()));
	MatrixXd B_tilde = qr.householderQ() * thinQ;

	// R
	MatrixXd R(sigma.size() + B_tilde.cols(), sigma.size() + B.cols());
	MatrixXd sigma_m = sigma.asDiagonal();
	R << ff * sigma_m, B_proj,
		MatrixXd::Zero(B_tilde.cols(), sigma.size()), B_tilde.transpose() * B_res;

	// step 4
	// SVD of R
	JacobiSVD < MatrixXd > svd(R, ComputeThinU | ComputeThinV);
	MatrixXd Q(U.rows(), U.cols() + B_tilde.cols());
	Q << U, B_tilde;
	U = svd.matrixU();
	sigma = svd.singularValues();


	// step 5
	// Remove small singular values
	double cutoff = sigma.norm() * 1e-3; // same as ivt implementation
	int keep; // the number of columns to keep in U
	double val;
	// Note that sigma is in descending order
	int i;
	for(i = 0; i < sigma.size(); i++){
		val = *(sigma.data() + i);
		if(val <= cutoff) break;
	}

	// check U and sigma
	keep = i > max_n_eig ? max_n_eig : i;// at most max_n_eig singular values
	sigma.conservativeResize(keep);
	U = Q * U.leftCols(keep);

	// update the mean image
	mu_A = (ff * n * mu_A + m * mu_B) / (ff*n + m);

	// update n <- fn + m
	n = static_cast<int>(ff*n + m);

}

/*! entry condition: addi_patches is updated
   It updates:
   - mean_prev_patches
   - n_prev_patches
   - U
   - sigma
   */
void PCA::incrementalPCA() {
	sklm(U, sigma, mean_prev_patches, addi_patches, n_prev_patches, params.f_factor, params.n_eigenvec);
}

void PCA::display_basis() {
	// The size of U is (resX * resY) x cols(16 at most).
	// to display, first reshape the matrix U to (resY * 4) by (resX * 4)
	// pad zeros if no data
	int m = getResY();
	int n = static_cast<int>(sqrt(params.n_eigenvec));
	int row = m *  n;
	int col = m *  n;
	double min_val; // for computing the normalization parameters
	double max_val;
	double range;

	MatrixXd mean_tmp = mean_prev_patches;
	cv::Mat mean_img, mean_uchar, mean_uchar_bigger, dst, dst_uchar, dst_bigger;
	mean_img = cv::Mat(m, m, cv_img_type_float, mean_tmp.data());
	mean_uchar.create(m, m, cv_img_type_uchar);
	mean_uchar_bigger.create(5 * m, 5 * m, cv_img_type_uchar);
	dst = cv::Mat(row, col, cv_img_type_float, cv::Scalar(0));
	dst_uchar.create(row, col, cv_img_type_uchar);
	dst_bigger.create(5 * row, 5 * col, cv_img_type_uchar);

	min_val = mean_tmp.colwise().minCoeff().minCoeff();
	max_val = mean_tmp.colwise().maxCoeff().maxCoeff();
	range = max_val - min_val;
	mean_img.convertTo(mean_uchar, mean_uchar.type(), 255. / range, -255.*min_val / range);
	cv::resize(mean_uchar, mean_uchar_bigger, mean_uchar_bigger.size());
	imshow("mean image", mean_uchar_bigger);
	// Reshape the basis images into a 'row' by 'col' grid
	for(int i = 0; i < U.cols(); i++){
		cv::Mat sub_img(m, m, cv_img_type_float, U.col(i).data());
		sub_img.copyTo(dst(cv::Rect((i%n)*m, (i / n)*m, m, m)));
	}

	minMaxLoc(dst, &min_val, &max_val);
	range = max_val - min_val;
	// Construct Mat object for the uint8 gray scale patch
	
	// Convert the RGB patch to grayscale
	dst.convertTo(dst_uchar, dst_uchar.type(), 255. / range, -255.*min_val / range);
	// set the empty basis back to zeros after the conversion
	if(U.cols() < params.n_eigenvec) {
		for(int i = U.cols(); i < params.n_eigenvec; i++){
			dst_uchar(cv::Rect((i%n)*m, (i / n)*m, m, m)).setTo(0);
		}
	}
	cv::resize(dst_uchar, dst_bigger, dst_bigger.size());
	imshow("Basis Images", dst_bigger);
}

/*!
 *  Display the intermediate results
 *  1, current image
 *  1, template
 *  2, reconstructed image
 *
 *  	@param curr_image mean-subtracted image
 *		@param error_image difference between the current image and the reconstructed
 *
 */
void PCA::display_images(const VectorXd &curr_image,
	const VectorXdM &error_image) {
	double min_val, max_val, range;
	// Visualize the current image
	cv::Scalar gt_color(0, 255, 0);
	VectorXd curr_image_full = curr_image + mean_prev_patches;
	cv::Mat curr_cv = cv::Mat(getResY(), getResX(), cv_img_type_float, curr_image_full.data());
	// Construct Mat object for the uint8 gray scale patch
	cv::Mat curr_unchar(getResY(), getResX(), cv_img_type_uchar);
	min_val = curr_image_full.colwise().minCoeff().minCoeff();
	max_val = curr_image_full.colwise().maxCoeff().maxCoeff();
	range = max_val - min_val;
	// Convert the RGB patch to grayscale
	curr_cv.convertTo(curr_unchar, curr_unchar.type(), 255. / range, -255.*min_val / range); // normalize the data to [0,255]
	//minMaxLoc(curr_unchar, &min_val, &max_val);
	//range = max_val - min_val;
	cv::Mat curr_unchar_bigger(5 * getResY(), 5 * getResX(), cv_img_type_uchar); // why 5?
	cv::resize(curr_unchar, curr_unchar_bigger, curr_unchar_bigger.size());
	cv::putText(curr_unchar_bigger, "Current Image", cv::Point2d(5, 15),
		cv::FONT_HERSHEY_SIMPLEX, 0.5, gt_color);
	imshow("Current Image", curr_unchar_bigger);

	// Visualize the reconstructed image
	VectorXd recon_image = U*(U.transpose()*curr_image) + mean_prev_patches;
	cv::Mat curr_recon_cv = cv::Mat(getResY(), getResX(), cv_img_type_float, recon_image.data());
	// Construct Mat object for the uint8 gray scale patch
	cv::Mat curr_recon_cv_uchar(getResY(), getResX(), cv_img_type_uchar);
	cv::Mat curr_recon_cv_uchar_bigger(5 * getResY(), 5 * getResX(), cv_img_type_uchar);
	// Convert the RGB patch to grayscale
	min_val = curr_image_full.colwise().minCoeff().minCoeff();
	max_val = curr_image_full.colwise().maxCoeff().maxCoeff();
	range = max_val - min_val;
	curr_recon_cv.convertTo(curr_recon_cv_uchar, curr_recon_cv_uchar.type(), 255. / range, -255.*min_val / range);
	cv::resize(curr_recon_cv_uchar, curr_recon_cv_uchar_bigger, curr_recon_cv_uchar_bigger.size());
	cv::putText(curr_recon_cv_uchar_bigger, "Reconstructed Image", cv::Point2d(5, 15),
		cv::FONT_HERSHEY_SIMPLEX, 0.5, gt_color);
	imshow("Reconstructed Image", curr_recon_cv_uchar_bigger);

	// Visualize the error image
	VectorXd err_img = error_image;
	cv::Mat err_cv = cv::Mat(getResY(), getResX(), cv_img_type_float, err_img.data());
	// Construct Mat object for the uint8 gray scale patch
	cv::Mat err_cv_uchar(getResY(), getResX(), cv_img_type_uchar);
	cv::Mat err_cv_uchar_bigger(5 * getResY(), 5 * getResX(), cv_img_type_uchar);
	// Convert the RGB patch to grayscale
	// normalize to [0,255]
	min_val = err_img.colwise().minCoeff().minCoeff();
	max_val = err_img.colwise().maxCoeff().maxCoeff();
	range = max_val - min_val;
	err_cv.convertTo(err_cv_uchar, err_cv_uchar.type(), 255. / range, -255.*min_val / range);
	cv::resize(err_cv_uchar, err_cv_uchar_bigger, err_cv_uchar_bigger.size());
	cv::putText(err_cv_uchar_bigger, "Error Image", cv::Point2d(5, 15),
		cv::FONT_HERSHEY_SIMPLEX, 0.5, gt_color);
	imshow("Error Image", err_cv_uchar_bigger);
}

_MTF_END_NAMESPACE

