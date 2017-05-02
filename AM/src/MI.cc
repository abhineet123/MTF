#include "mtf/AM/MI.h"
#include "mtf/Utilities/histUtils.h"
#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/imgUtils.h"

#define MI_N_BINS 8
#define MI_PRE_SEED 10
#define MI_POU false
#define MI_PIX_MAPPER nullptr
#define MI_DEBUG_MODE false

_MTF_BEGIN_NAMESPACE

//! value constructor
MIParams::MIParams(const AMParams *am_params,
int _n_bins, double _pre_seed,
bool _partition_of_unity,

ImageBase *_pix_mapper,
bool _debug_mode) :
AMParams(am_params),
n_bins(_n_bins),
pre_seed(_pre_seed),
partition_of_unity(_partition_of_unity),
pix_mapper(_pix_mapper),
debug_mode(_debug_mode){}

//! default/copy constructor
MIParams::MIParams(const MIParams *params) :
AMParams(params),
n_bins(MI_N_BINS),
pre_seed(MI_PRE_SEED),
partition_of_unity(MI_POU),
pix_mapper(MI_PIX_MAPPER),
debug_mode(MI_DEBUG_MODE){
	if(params){
		n_bins = params->n_bins;
		pre_seed = params->pre_seed;
		partition_of_unity = params->partition_of_unity;
		pix_mapper = params->pix_mapper;		
		debug_mode = params->debug_mode;
	}
}

MIDist::MIDist(const string &_name, const int _n_bins,
	const unsigned int _feat_size, const unsigned int _patch_size,
	const double _hist_pre_seed, const double _pre_seed,
	const MatrixX2i *_std_bspl_ids,
	const double _log_hist_norm_mult) :
	AMDist(_name), n_bins(_n_bins), feat_size(_feat_size),
	patch_size(_patch_size), pre_seed(_pre_seed),
	hist_pre_seed(_hist_pre_seed), std_bspl_ids(_std_bspl_ids),
	log_hist_norm_mult(_log_hist_norm_mult){}

MI::MI(const ParamType *mi_params, const int _n_channels) :
AppearanceModel(mi_params, _n_channels), params(mi_params){
	printf("\n");
	printf("Using Mutual Information AM with:\n");
	printf("n_bins: %d\n", params.n_bins);
	printf("pre_seed: %f\n", params.pre_seed);
	printf("partition_of_unity: %d\n", params.partition_of_unity);
	printf("likelihood_alpha: %f\n", params.likelihood_alpha);
	printf("debug_mode: %d\n", params.debug_mode);

	name = "mi";
	log_fname = "log/mtf_mi_log.txt";
	time_fname = "log/mtf_mi_times.txt";

	if(params.pix_mapper){
		if(params.pix_mapper->getNChannels() != n_channels){
			throw utils::InvalidArgument(
				cv::format("MI::No. of channels in the pixel mapper %d is incorrect", 
				params.pix_mapper->getNChannels()));
		}		
	} else{
		printf("Pixel mapping is disabled\n");
	}

	double norm_pix_min = 0, norm_pix_max = params.n_bins - 1;
	if(params.partition_of_unity){
		if(params.n_bins < 4){
			throw utils::InvalidArgument(
				cv::format("MI::Too few bins %d specified to enforce partition of unity constraint", params.n_bins));
		}
		norm_pix_min = 1;
		norm_pix_max = params.n_bins - 2;
	}
	printf("norm_pix_min: %f\n", norm_pix_min);
	printf("norm_pix_max: %f\n", norm_pix_max);

	pix_norm_mult = (norm_pix_max - norm_pix_min) / (PIX_MAX - PIX_MIN + 1);
	// extra 1 in the denominator to avoid the extremely rare case when pix val = PIX_MAX 
	// causing problems with the bspline id system of indexing the histogram
	pix_norm_add = norm_pix_min;

	joint_hist_size = params.n_bins * params.n_bins; // size of the flattened MI matrix
	hist_pre_seed = params.n_bins * params.pre_seed;// preseeding the joint histogram by 's' is equivalent to 
	// preseeding individual histograms by s * n_bins

	//hist_mat_pre_seed = hist_pre_seed / static_cast<double>(n_pix); 
	/*since each element of hist is the sum of the corresponding row in hist_mat (that has n_pix elements),
	preseeding each element of hist with 's' is equivalent to preseeding each element of hist_mat with s/n_pix */

	hist_norm_mult = 1.0 / (static_cast<double>(patch_size)+hist_pre_seed * params.n_bins);
	log_hist_norm_mult = log(hist_norm_mult);
	/* denominator of this factor is equal to the sum of all entries in the individual histograms,
	so multiplying hist with this factor will give the normalized hist whose entries sum to 1*/

	std_bspl_ids.resize(params.n_bins, Eigen::NoChange);
	// _linear_idx(i, j) stores the linear index of element (i,j) of a matrix
	// of dimensions params.n_bins x params.n_bins if it is to be flattened into a vector
	// in row major order; 
	linear_idx.resize(params.n_bins, params.n_bins);
	for(int i = 0; i < params.n_bins; i++) {
		std_bspl_ids(i, 0) = max(0, i - 1);
		std_bspl_ids(i, 1) = min(params.n_bins - 1, i + 2);
		for(int j = 0; j < params.n_bins; j++) {
			linear_idx(i, j) = i * params.n_bins + j;
		}
	}
	//for functor support
	feat_size = 5 * patch_size;
}

void MI::initializePixVals(const Matrix2Xd& init_pts){
	if(!is_initialized.pix_vals){
		I0.resize(patch_size);
		It.resize(patch_size);
	}
	if(params.pix_mapper){
		params.pix_mapper->initializePixVals(init_pts);
		I0 = pix_norm_mult*(params.pix_mapper->getInitPixVals()).array() + pix_norm_add;
	} else{	
		switch(input_type){
		case InputType::MTF_8UC1:
			utils::sc::getPixVals<uchar>(I0, curr_img_cv, init_pts, n_pix,
				img_height, img_width, pix_norm_mult, pix_norm_add);
			break;
		case InputType::MTF_8UC3:
			utils::mc::getPixVals<uchar>(I0, curr_img_cv, init_pts, n_pix,
				img_height, img_width, pix_norm_mult, pix_norm_add);
			break;
		case InputType::MTF_32FC1:
			utils::getPixVals(I0, curr_img, init_pts, n_pix,
				img_height, img_width, pix_norm_mult, pix_norm_add);
			break;
		case InputType::MTF_32FC3:
			utils::mc::getPixVals<float>(I0, curr_img_cv, init_pts, n_pix,
				img_height, img_width, pix_norm_mult, pix_norm_add);
			break;
		default:
			throw utils::InvalidArgument("MI::Invalid input type found");
		}
	}
	if(!is_initialized.pix_vals){
		It = I0;
		is_initialized.pix_vals = true;
	}
}
/**
* Prerequisites :: Computed in:
*		None
* Computes :: Description
*		curr_pix_vals
*/
void MI::updatePixVals(const Matrix2Xd& curr_pts){
	if(params.pix_mapper){
		params.pix_mapper->updatePixVals(curr_pts);
		It = pix_norm_mult*(params.pix_mapper->getCurrPixVals()).array() + pix_norm_add;
	} else{
		switch(input_type){
		case InputType::MTF_8UC1:
			utils::sc::getPixVals<uchar>(It, curr_img_cv, curr_pts, n_pix, img_height, img_width,
				pix_norm_mult, pix_norm_add);
			break;
		case InputType::MTF_8UC3:
			utils::mc::getPixVals<uchar>(It, curr_img_cv, curr_pts, n_pix, img_height, img_width,
				pix_norm_mult, pix_norm_add);
			break;
		case InputType::MTF_32FC1:
			utils::getPixVals(It, curr_img, curr_pts, n_pix, img_height, img_width,
				pix_norm_mult, pix_norm_add);
			break;
		case InputType::MTF_32FC3:
			utils::mc::getPixVals<float>(It, curr_img_cv, curr_pts, n_pix, img_height, img_width,
				pix_norm_mult, pix_norm_add);
			break;
		default:
			throw utils::InvalidArgument("MI::Invalid input type found");
		}
	}
}

/**
* initialize
* Prerequisites :: Computed in:
*	init_pix_vals :: initializePixVals
* Computes :: Description
*	_init_bspl_ids :: stores the indices of the first and last bins contributed to by each pixel in the initial template
*	init_hist :: histogram of initial template
*	init_hist_mat :: stores the contribution of each pixel to each histogram bin
*	init_hist_log :: entry wise logarithm of init_hist
*	joint_hist :: joint histogram of the initial template wrt itself
*	joint_hist_log :: entry wise logarithm of joint_hist
*	init_hist_grad :: gradient of the initial histogram w.r.t. pixel values
*/
void MI::initializeSimilarity(){
	if(!is_initialized.similarity){
		init_hist.resize(params.n_bins);
		init_hist_mat.resize(params.n_bins, patch_size);
		joint_hist.resize(params.n_bins, params.n_bins);
		init_hist_log.resize(params.n_bins);
		joint_hist_log.resize(params.n_bins, params.n_bins);

		curr_hist.resize(params.n_bins);
		curr_hist_mat.resize(params.n_bins, patch_size);
		joint_hist.resize(params.n_bins, params.n_bins);
		curr_hist_log.resize(params.n_bins);
		joint_hist_log.resize(params.n_bins, params.n_bins);

		init_bspl_ids.resize(patch_size, Eigen::NoChange);
		curr_bspl_ids.resize(patch_size, Eigen::NoChange);

		// since histogram and its gradients are computed simultaneously, both must be resized here too
		init_hist_grad.resize(params.n_bins, patch_size);
		curr_hist_grad.resize(params.n_bins, patch_size);
	}

	//! initial histogram and its gradient
	init_hist.fill(hist_pre_seed);
	init_hist_mat.setZero();
	init_hist_grad.setZero();
	for(unsigned int pix_id = 0; pix_id < patch_size; pix_id++) {
		init_bspl_ids.row(pix_id) = std_bspl_ids.row(static_cast<int>(I0(pix_id)));
		double curr_diff = init_bspl_ids(pix_id, 0) - I0(pix_id);
		for(int id1 = init_bspl_ids(pix_id, 0); id1 <= init_bspl_ids(pix_id, 1); id1++) {
			utils::bSpl3WithGrad(init_hist_mat(id1, pix_id), init_hist_grad(id1, pix_id), curr_diff);
			init_hist_grad(id1, pix_id) *= -hist_norm_mult;
			init_hist(id1) += init_hist_mat(id1, pix_id);
			// since the ids of all bins affected by a pixel are sequential, repeated computation
			// of curr_diff can be avoided by simply incrementing it by 1 which is (hopefully) faster
			++curr_diff;
		}

	}
	/* normalize the histograms and compute their log*/
	init_hist *= hist_norm_mult;
	init_hist_log = init_hist.array().log();

	//utils::printMatrix(joint_hist, "joint_hist");
	//utils::printMatrix(init_hist, "init_hist");
	//utils::printMatrix(joint_hist - joint_hist.transpose(), "joint_hist diff");

	if(!is_initialized.similarity){
		joint_hist.fill(params.pre_seed);
		for(unsigned int pix_id = 0; pix_id < patch_size; pix_id++) {
			for(int id1 = init_bspl_ids(pix_id, 0); id1 <= init_bspl_ids(pix_id, 1); id1++) {
				for(int id2 = init_bspl_ids(pix_id, 0); id2 <= init_bspl_ids(pix_id, 1); id2++) {
					joint_hist(id1, id2) += init_hist_mat(id1, pix_id) * init_hist_mat(id2, pix_id);
				}
			}
		}
		joint_hist *= hist_norm_mult;
		joint_hist_log = joint_hist.array().log();

		f = 0;
		for(int curr_id = 0; curr_id < params.n_bins; curr_id++){
			for(int init_id = 0; init_id < params.n_bins; init_id++){
				f += joint_hist(curr_id, init_id) * (joint_hist_log(curr_id, init_id) -
					init_hist_log(curr_id) - init_hist_log(init_id));
			}
		}
		max_similarity = f;

		curr_bspl_ids = init_bspl_ids;
		curr_hist = init_hist;
		curr_hist_mat = init_hist_mat;
		curr_hist_log = init_hist_log;
		curr_hist_grad = init_hist_grad;

		is_initialized.similarity = true;
	}
}

/**
* initializeGrad
* Prerequisites :: Computed in:
*	init_hist_mat, _init_bspl_ids, init_hist_grad,
init_hist_log:: initialize
* Computes :: Description
*	init_grad_factor :: 1 + log(joint_hist/init_hist) - useful for computing the MI gradient
*	init_grad: gradient of MI w.r.t. initial pixel values
*/

void MI::initializeGrad(){
	if(!is_initialized.grad){
		init_joint_hist_grad.resize(joint_hist_size, patch_size);
		curr_joint_hist_grad.resize(joint_hist_size, patch_size);

		init_grad_factor.resize(params.n_bins, params.n_bins);
		curr_grad_factor.resize(params.n_bins, params.n_bins);

		df_dIt.resize(patch_size);
		df_dI0.resize(patch_size);

		for(int curr_id = 0; curr_id < params.n_bins; curr_id++){
			for(int init_id = 0; init_id < params.n_bins; init_id++){
				init_grad_factor(curr_id, init_id) = 1 + joint_hist_log(curr_id, init_id) - init_hist_log(curr_id);
			}
		}
		// gradient of the initial joint histogram and MI  w.r.t. initial pixel values; the latter does not need to be normalized 
		// since init_hist_grad has already been normalized and the computed differential will thus be implicitly normalized
		init_joint_hist_grad.setZero();
		df_dI0.setZero();
		for(unsigned int pix_id = 0; pix_id < patch_size; pix_id++) {
			for(int curr_id = init_bspl_ids(pix_id, 0); curr_id <= init_bspl_ids(pix_id, 1); curr_id++) {
				for(int init_id = init_bspl_ids(pix_id, 0); init_id <= init_bspl_ids(pix_id, 1); init_id++) {
					int joint_id = linear_idx(curr_id, init_id);
					init_joint_hist_grad(joint_id, pix_id) = init_hist_grad(curr_id, pix_id) * init_hist_mat(init_id, pix_id);
					df_dI0(pix_id) += init_joint_hist_grad(joint_id, pix_id) * init_grad_factor(curr_id, init_id);
				}
			}
		}
		curr_grad_factor = init_grad_factor;
		curr_joint_hist_grad = init_joint_hist_grad;
		df_dIt = df_dI0;
		is_initialized.grad = true;
	}
}
/**
* update
* Prerequisites :: Computed in:
*	curr_pix_vals :: updatePixVals
*	_init_bspl_ids, init_hist_mat :: initialize
* Computes :: Description
*	_curr_bspl_ids :: stores the indices of the first and last bins contributed to by each pixel in the initial template
*	curr_hist :: histogram of cuurent pixel values
*	curr_hist_mat :: stores the contribution of each pixel to each histogram bin
*	joint_hist :: joint histogram between the current and initial pixel values
*	curr_hist_log :: entry wise logarithm of curr_hist
*	joint_hist_log :: entry wise logarithm of joint_hist
*	curr_hist_grad :: gradient of the current histogram w.r.t. current pixel values
*	curr_joint_hist_grad :: gradient of the current joint histogram w.r.t. current pixel values
*	similarity :: MI between the current and initial pixel values (optional)
*/
void MI::updateSimilarity(bool prereq_only){

	// compute both the histogram and its differential simultaneously
	// to take advantage of the many common computations involved
	curr_hist.fill(hist_pre_seed);
	joint_hist.fill(params.pre_seed);
	curr_hist_mat.setZero();
	curr_hist_grad.setZero();
	for(unsigned int pix_id = 0; pix_id < patch_size; pix_id++) {
		curr_bspl_ids.row(pix_id) = std_bspl_ids.row(static_cast<int>(It(pix_id)));
		double curr_diff = curr_bspl_ids(pix_id, 0) - It(pix_id);
		for(int curr_id = curr_bspl_ids(pix_id, 0); curr_id <= curr_bspl_ids(pix_id, 1); curr_id++) {
			utils::bSpl3WithGrad(curr_hist_mat(curr_id, pix_id), curr_hist_grad(curr_id, pix_id), curr_diff);
			++curr_diff;
			curr_hist_grad(curr_id, pix_id) *= -hist_norm_mult;
			curr_hist(curr_id) += curr_hist_mat(curr_id, pix_id);
			for(int init_id = init_bspl_ids(pix_id, 0); init_id <= init_bspl_ids(pix_id, 1); init_id++) {
				joint_hist(curr_id, init_id) += curr_hist_mat(curr_id, pix_id) * init_hist_mat(init_id, pix_id);
			}
		}
	}

	// normalize the histograms and compute their log
	curr_hist *= hist_norm_mult;
	joint_hist *= hist_norm_mult;
	curr_hist_log = curr_hist.array().log();
	joint_hist_log = joint_hist.array().log();

	if(prereq_only){ return; }

	f = 0;
	for(int curr_id = 0; curr_id < params.n_bins; curr_id++){
		for(int init_id = 0; init_id < params.n_bins; init_id++){
			f += joint_hist(curr_id, init_id) * (joint_hist_log(curr_id, init_id) - curr_hist_log(curr_id) - init_hist_log(init_id));
		}
	}
}

double MI::getLikelihood() const{
	double d = (1.0 / f) - 1;
	return exp(-params.likelihood_alpha * d*d);
}
/**
* updateInitGrad
* Prerequisites :: Computed in:
*	init_hist_grad, _init_bspl_ids, init_hist_log :: initialize
*	curr_hist_mat, _curr_bspl_ids, joint_hist_log :: update
* Computes :: Description
*	init_joint_hist_grad :: gradient of the current joint histogram w.r.t. initial pixel values
*	init_grad_factor :: 1 + log(joint_hist/init_hist) - useful for computing the MI gradient too
*	init_grad: gradient of current MI w.r.t. initial pixel values
*/
void MI::updateInitGrad(){
	for(int init_id = 0; init_id < params.n_bins; init_id++){
		for(int curr_id = 0; curr_id < params.n_bins; curr_id++){
			init_grad_factor(init_id, curr_id) = 1 + joint_hist_log(curr_id, init_id) - init_hist_log(init_id);
		}
	}
	// differential of the current joint histogram w.r.t. initial pixel values; this does not need to be normalized 
	// since init_hist_grad has already been normalized and the computed differential will thus be implicitly normalized
	init_joint_hist_grad.setZero();
	for(unsigned int pix_id = 0; pix_id < patch_size; pix_id++) {
		df_dI0(pix_id) = 0;
		for(int init_id = init_bspl_ids(pix_id, 0); init_id <= init_bspl_ids(pix_id, 1); init_id++) {
			for(int curr_id = curr_bspl_ids(pix_id, 0); curr_id <= curr_bspl_ids(pix_id, 1); curr_id++) {
				int joint_id = linear_idx(init_id, curr_id);
				init_joint_hist_grad(joint_id, pix_id) = init_hist_grad(init_id, pix_id) * curr_hist_mat(curr_id, pix_id);
				df_dI0(pix_id) += init_joint_hist_grad(joint_id, pix_id) * init_grad_factor(init_id, curr_id);
			}
		}
	}
}
/**
* updateCurrGrad
* Prerequisites :: Computed in:
*	joint_hist_log, curr_hist_log, curr_joint_hist_grad :: update
* Computes :: Description
*	curr_grad_factor :: 1 + log(joint_hist/curr_hist) - useful for computing the MI gradient too
*	curr_grad: gradient of current MI w.r.t. current pixel values
*/
void MI::updateCurrGrad(){
	for(int curr_id = 0; curr_id < params.n_bins; curr_id++){
		for(int init_id = 0; init_id < params.n_bins; init_id++){
			curr_grad_factor(curr_id, init_id) = 1 + joint_hist_log(curr_id, init_id) - curr_hist_log(curr_id);
		}
	}
	curr_joint_hist_grad.setZero();
	for(unsigned int pix_id = 0; pix_id < patch_size; pix_id++){
		df_dIt(pix_id) = 0;
		for(int curr_id = curr_bspl_ids(pix_id, 0); curr_id <= curr_bspl_ids(pix_id, 1); curr_id++) {
			for(int init_id = init_bspl_ids(pix_id, 0); init_id <= init_bspl_ids(pix_id, 1); init_id++) {
				int joint_id = linear_idx(curr_id, init_id);
				curr_joint_hist_grad(joint_id, pix_id) = curr_hist_grad(curr_id, pix_id) * init_hist_mat(init_id, pix_id);
				df_dIt(pix_id) += curr_joint_hist_grad(joint_id, pix_id) * curr_grad_factor(curr_id, init_id);
			}
		}
	}
}
void MI::initializeHess(){
	if(!is_initialized.hess){
		init_hist_hess.resize(params.n_bins, patch_size);
		curr_hist_hess.resize(params.n_bins, patch_size);

		self_joint_hist.resize(params.n_bins, params.n_bins);
		self_joint_hist_log.resize(params.n_bins, params.n_bins);
		self_grad_factor.resize(params.n_bins, params.n_bins);
	}

	utils::getBSplHistHess(init_hist_hess, I0, init_bspl_ids, patch_size, hist_norm_mult);

	if(!is_initialized.hess){
		curr_hist_hess = init_hist_hess;
		is_initialized.hess = true;
	}
}
void  MI::cmptInitHessian(MatrixXd &hessian, const MatrixXd &init_pix_jacobian){
	int ssm_state_size = init_pix_jacobian.cols();
	assert(hessian.rows() == ssm_state_size && hessian.cols() == ssm_state_size);
	assert(init_pix_jacobian.rows() == patch_size);

	MatrixXd joint_hist_jacobian(joint_hist_size, ssm_state_size);
	hessian.setZero();
	joint_hist_jacobian.setZero();

	//utils::printMatrix(_curr_bspl_ids - _init_bspl_ids, "_init_bspl_ids diff", "%d");
	//utils::printMatrix(joint_hist_log - self_joint_hist_log, "joint_hist_log diff", "%e");
	//utils::printMatrix(init_hist_log - curr_hist_log, "init_hist_log diff", "%e");
	//utils::printMatrix(init_hist_hess - curr_hist_hess, "init_hist_hess diff", "%e");
	//utils::printMatrix(_curr_bspl_ids, "_curr_bspl_ids", "%d");
	//utils::printMatrix(_init_bspl_ids, "_init_bspl_ids", "%d");

	//utils::printMatrix(hessian, "init: hessian start", "%e");

	for(unsigned int pix_id = 0; pix_id < patch_size; pix_id++){
		double hist_hess_term = 0;
		for(int init_id = init_bspl_ids(pix_id, 0); init_id <= init_bspl_ids(pix_id, 1); init_id++) {
			double inner_term = 0;
			for(int curr_id = curr_bspl_ids(pix_id, 0); curr_id <= curr_bspl_ids(pix_id, 1); curr_id++) {
				int joint_id = linear_idx(curr_id, init_id);
				//init_joint_hist_grad(joint_id, pix_id) = init_hist_grad(curr_id, pix_id) * init_hist_mat(init_id, pix_id);
				//init_grad_factor(curr_id, init_id) = 1 + joint_hist_log(curr_id, init_id) - init_hist_log(curr_id);				
				joint_hist_jacobian.row(joint_id) += init_joint_hist_grad(linear_idx(init_id, curr_id), pix_id)*init_pix_jacobian.row(pix_id);
				inner_term += curr_hist_mat(curr_id, pix_id) * init_grad_factor(init_id, curr_id);

				//joint_hist_jacobian.row(joint_id) += init_hist_grad(init_id, pix_id)*curr_hist_mat(curr_id, pix_id)*init_pix_jacobian.row(pix_id);
				//inner_term += curr_hist_mat(curr_id, pix_id) * (1 + joint_hist_log(curr_id, init_id) - init_hist_log(init_id));
			}
			hist_hess_term += init_hist_hess(init_id, pix_id)*inner_term;
		}
		//utils::printScalar(hist_hess_term, "init: hist_hess_term", "%20.16f");

		hessian += hist_hess_term * init_pix_jacobian.row(pix_id).transpose() * init_pix_jacobian.row(pix_id);
		//utils::printMatrix(init_pix_jacobian.row(pix_id).transpose() * init_pix_jacobian.row(pix_id), "init: init_pix_jacobian term", "%20.16f");
		//utils::printMatrix(hessian, "init: hessian", "%20.16f");

	}
	//utils::printMatrix(init_pix_jacobian, "init: init_pix_jacobian", "%e");
	//utils::printMatrix(joint_hist, "init: joint_hist", "%e");
	//utils::printMatrix(joint_hist_jacobian, "init: joint_hist_jacobian", "%e");
	//utils::printMatrix(hessian, "init: hessian 1", "%e");

	for(int curr_id = 0; curr_id < params.n_bins; curr_id++){
		for(int init_id = 0; init_id < params.n_bins; init_id++){
			int joint_id = linear_idx(curr_id, init_id);
			double hist_factor = (1.0 / joint_hist(curr_id, init_id)) - (1.0 / init_hist(init_id));
			hessian += joint_hist_jacobian.row(joint_id).transpose() * joint_hist_jacobian.row(joint_id) * hist_factor;
		}
	}
}
void MI::cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian){
	int ssm_state_size = curr_pix_jacobian.cols();

	assert(self_hessian.rows() == ssm_state_size && self_hessian.cols() == ssm_state_size);
	assert(curr_pix_jacobian.rows() == patch_size);

	cmptSelfHist();

	MatrixXd self_grad_factor2(params.n_bins, params.n_bins);
	for(int curr_id = 0; curr_id < params.n_bins; curr_id++){
		for(int init_id = 0; init_id < params.n_bins; init_id++){
			self_grad_factor2(curr_id, init_id) = 1 + self_joint_hist_log(curr_id, init_id) - curr_hist_log(init_id);
		}
	}
	MatrixXd joint_hist_jacobian2(joint_hist_size, ssm_state_size);
	MatrixXd self_hessian2(ssm_state_size, ssm_state_size);
	self_hessian2.setZero();
	joint_hist_jacobian2.setZero();
	//utils::printMatrix(self_hessian2, "self: self_hessian2 start", "%e");
	for(unsigned int pix_id = 0; pix_id < patch_size; pix_id++){
		double hist_hess_term2 = 0;
		for(int init_id = curr_bspl_ids(pix_id, 0); init_id <= curr_bspl_ids(pix_id, 1); init_id++) {
			double inner_term2 = 0;
			for(int curr_id = curr_bspl_ids(pix_id, 0); curr_id <= curr_bspl_ids(pix_id, 1); curr_id++) {
				int joint_id = linear_idx(curr_id, init_id);
				joint_hist_jacobian2.row(joint_id) += curr_hist_grad(init_id, pix_id)*curr_hist_mat(curr_id, pix_id)*curr_pix_jacobian.row(pix_id);
				inner_term2 += curr_hist_mat(curr_id, pix_id) * self_grad_factor2(curr_id, init_id);
			}
			hist_hess_term2 += curr_hist_hess(init_id, pix_id)*inner_term2;
		}
		self_hessian2 += hist_hess_term2 * curr_pix_jacobian.row(pix_id).transpose() * curr_pix_jacobian.row(pix_id);
		//utils::printScalar(hist_hess_term2, "self: hist_hess_term2", "%20.16f");
		//utils::printMatrix(curr_pix_jacobian.row(pix_id).transpose() * curr_pix_jacobian.row(pix_id), "self: curr_pix_jacobian term", "%20.16f");
		//utils::printMatrix(self_hessian2, "self: self_hessian2", "%20.16f");

	}
	//utils::printMatrix(curr_pix_jacobian, "self: curr_pix_jacobian", "%e");
	//utils::printMatrix(self_joint_hist, "self: self_joint_hist", "%e");
	//utils::printMatrix(joint_hist_jacobian2, "self: joint_hist_jacobian2", "%e");
	//utils::printMatrix(self_hessian2, "self: self_hessian2 1", "%e");

	for(int curr_id = 0; curr_id < params.n_bins; curr_id++){
		for(int init_id = 0; init_id < params.n_bins; init_id++){
			int joint_id = linear_idx(curr_id, init_id);
			double hist_factor = (1.0 / self_joint_hist(curr_id, init_id)) - (1.0 / curr_hist(init_id));
			self_hessian2 += joint_hist_jacobian2.row(joint_id).transpose() * joint_hist_jacobian2.row(joint_id) * hist_factor;
		}
	}


	MatrixXd joint_hist_jacobian(joint_hist_size, ssm_state_size);
	self_hessian.setZero();
	joint_hist_jacobian.setZero();
	for(unsigned int pix_id = 0; pix_id < patch_size; pix_id++){
		double curr_diff = curr_bspl_ids(pix_id, 0) - It(pix_id);
		double hist_hess_term = 0;
		for(int curr_id = curr_bspl_ids(pix_id, 0); curr_id <= curr_bspl_ids(pix_id, 1); curr_id++) {
			curr_hist_hess(curr_id, pix_id) = hist_norm_mult*utils::bSpl3Hess(curr_diff);
			++curr_diff;
			double inner_term = 0;
			for(int init_id = curr_bspl_ids(pix_id, 0); init_id <= curr_bspl_ids(pix_id, 1); init_id++) {
				int idx = linear_idx(curr_id, init_id);
				joint_hist_jacobian.row(idx) += curr_hist_grad(curr_id, pix_id)*curr_hist_mat(init_id, pix_id)*curr_pix_jacobian.row(pix_id);
				inner_term += curr_hist_mat(init_id, pix_id) * self_grad_factor(curr_id, init_id);
			}
			hist_hess_term += curr_hist_hess(curr_id, pix_id)*inner_term;
		}
		self_hessian += hist_hess_term * curr_pix_jacobian.row(pix_id).transpose() * curr_pix_jacobian.row(pix_id);
	}
	for(int curr_id = 0; curr_id < params.n_bins; curr_id++){
		for(int init_id = 0; init_id < params.n_bins; init_id++){
			int joint_id = linear_idx(curr_id, init_id);
			double hist_factor = (1.0 / self_joint_hist(curr_id, init_id)) - (1.0 / curr_hist(curr_id));
			self_hessian += joint_hist_jacobian.row(joint_id).transpose() * joint_hist_jacobian.row(joint_id) * hist_factor;
		}
	}
	//utils::printMatrix(self_grad_factor - self_grad_factor2, "self_grad_factor diff", "%e");
	//utils::printMatrix(joint_hist_jacobian - joint_hist_jacobian2, "joint_hist_jacobian diff", "%e");
	//utils::printMatrix(self_hessian - self_hessian2, "self_hessian diff", "%e");
}
/**
cmptCurrHessian
* Prerequisites :: Computed in:
*	curr_joint_hist_grad, curr_hist_grad :: updateCurrErrVec if fast joint histogram is enabled; updateCurrErrVecGrad otherwise
*	joint_hist, curr_hist :: updateCurrErrVec
* Computes::Description
*	hessian::first order Hessian of current MI w.r.t. current pixel values
*/
void  MI::cmptCurrHessian(MatrixXd &hessian, const MatrixXd &curr_pix_jacobian){
	int ssm_state_size = curr_pix_jacobian.cols();
	assert(hessian.rows() == ssm_state_size && hessian.cols() == ssm_state_size);
	assert(curr_pix_jacobian.rows() == patch_size);

	// jacobians of the joint and current histograms w.r.t. SSM parameters
	//MatrixXd joint_hist_jacobian = curr_joint_hist_grad * curr_pix_jacobian;

	MatrixXd joint_hist_jacobian(joint_hist_size, ssm_state_size);
	hessian.setZero();
	joint_hist_jacobian.setZero();
	for(unsigned int pix_id = 0; pix_id < patch_size; pix_id++){
		double curr_diff = curr_bspl_ids(pix_id, 0) - It(pix_id);
		double hist_hess_term = 0;
		for(int curr_id = curr_bspl_ids(pix_id, 0); curr_id <= curr_bspl_ids(pix_id, 1); curr_id++) {
			curr_hist_hess(curr_id, pix_id) = hist_norm_mult*utils::bSpl3Hess(curr_diff);
			++curr_diff;
			double inner_term = 0;
			for(int init_id = init_bspl_ids(pix_id, 0); init_id <= init_bspl_ids(pix_id, 1); init_id++) {
				int idx = linear_idx(curr_id, init_id);
				joint_hist_jacobian.row(idx) += curr_joint_hist_grad(idx, pix_id)*curr_pix_jacobian.row(pix_id);
				inner_term += init_hist_mat(init_id, pix_id) * curr_grad_factor(curr_id, init_id);
			}
			hist_hess_term += curr_hist_hess(curr_id, pix_id)*inner_term;
		}
		hessian += hist_hess_term * curr_pix_jacobian.row(pix_id).transpose() * curr_pix_jacobian.row(pix_id);
	}
	for(int curr_id = 0; curr_id < params.n_bins; curr_id++){
		for(int init_id = 0; init_id < params.n_bins; init_id++){
			int idx = linear_idx(curr_id, init_id);
			double hist_factor = (1.0 / joint_hist(curr_id, init_id)) - (1.0 / curr_hist(curr_id));
			hessian += joint_hist_jacobian.row(idx).transpose() * joint_hist_jacobian.row(idx) * hist_factor;
		}
	}
}
// compute self histogram and its derivarives
void MI::cmptSelfHist(){
	//! current joint histogram computed wrt itself
	self_joint_hist.fill(params.pre_seed);
	for(unsigned int pix_id = 0; pix_id < patch_size; pix_id++) {
		for(int id1 = curr_bspl_ids(pix_id, 0); id1 <= curr_bspl_ids(pix_id, 1); id1++) {
			for(int id2 = curr_bspl_ids(pix_id, 0); id2 <= curr_bspl_ids(pix_id, 1); id2++) {
				self_joint_hist(id1, id2) += curr_hist_mat(id1, pix_id) * curr_hist_mat(id2, pix_id);
			}
		}
	}
	/* normalize the self joint histogram and compute its log*/
	self_joint_hist *= hist_norm_mult;
	self_joint_hist_log = self_joint_hist.array().log();

	for(int curr_id = 0; curr_id < params.n_bins; curr_id++){
		for(int init_id = 0; init_id < params.n_bins; init_id++){
			self_grad_factor(curr_id, init_id) = 1 + self_joint_hist_log(curr_id, init_id) - curr_hist_log(curr_id);
		}
	}
}
void MI::cmptInitHessian(MatrixXd &init_hessian, const MatrixXd &init_pix_jacobian,
	const MatrixXd &init_pix_hessian){

	int ssm_state_size = init_pix_jacobian.cols();

	assert(init_hessian.rows() == ssm_state_size && init_hessian.cols() == ssm_state_size);
	assert(init_pix_jacobian.rows() == patch_size);
	assert(init_pix_hessian.rows() == ssm_state_size*ssm_state_size);

	cmptInitHessian(init_hessian, init_pix_jacobian);

	for(unsigned int pix_id = 0; pix_id < patch_size; pix_id++){
		init_hessian += df_dI0(pix_id) * Map<const MatrixXd>(init_pix_hessian.col(pix_id).data(), ssm_state_size, ssm_state_size);
	}
}
/**
* Prerequisites :: Computed in:
*		[joint_hist_log, curr_hist_log] :: updateCurrGrad
* Computes :: Description:
*		curr_hessian :: hessian of the error norm w.r.t. current SSM parameters
*/
void MI::cmptCurrHessian(MatrixXd &curr_hessian, const MatrixXd &curr_pix_jacobian,
	const MatrixXd &curr_pix_hessian){
	int ssm_state_size = curr_pix_jacobian.cols();

	assert(curr_hessian.rows() == ssm_state_size && curr_hessian.cols() == ssm_state_size);
	assert(curr_pix_jacobian.rows() == patch_size);
	assert(curr_pix_hessian.rows() == ssm_state_size*ssm_state_size);

	// get first order hessian
	cmptCurrHessian(curr_hessian, curr_pix_jacobian);

	for(unsigned int pix_id = 0; pix_id < patch_size; pix_id++){
		curr_hessian += df_dIt(pix_id) * Map<const MatrixXd>(curr_pix_hessian.col(pix_id).data(), ssm_state_size, ssm_state_size);
	}
}

void MI::cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian,
	const MatrixXd &curr_pix_hessian){
	int ssm_state_size = curr_pix_jacobian.cols();

	assert(self_hessian.rows() == ssm_state_size && self_hessian.cols() == ssm_state_size);
	assert(curr_pix_jacobian.rows() == patch_size);
	assert(curr_pix_hessian.rows() == ssm_state_size*ssm_state_size);

	cmptSelfHist();
	MatrixXd joint_hist_jacobian(joint_hist_size, ssm_state_size);
	self_hessian.setZero();
	joint_hist_jacobian.setZero();
	for(unsigned int pix_id = 0; pix_id < patch_size; pix_id++){
		double curr_diff = curr_bspl_ids(pix_id, 0) - It(pix_id);
		double hist_hess_term = 0, hist_grad_term = 0;
		for(int r = curr_bspl_ids(pix_id, 0); r <= curr_bspl_ids(pix_id, 1); r++) {
			curr_hist_hess(r, pix_id) = hist_norm_mult*utils::bSpl3Hess(curr_diff++);
			double inner_term = 0;
			for(int t = curr_bspl_ids(pix_id, 0); t <= curr_bspl_ids(pix_id, 1); t++) {
				int idx = linear_idx(r, t);
				joint_hist_jacobian.row(idx) += curr_hist_grad(r, pix_id)*curr_hist_mat(t, pix_id)*curr_pix_jacobian.row(pix_id);
				inner_term += curr_hist_mat(t, pix_id) * self_grad_factor(r, t);
			}
			hist_hess_term += curr_hist_hess(r, pix_id)*inner_term;
			hist_grad_term += curr_hist_grad(r, pix_id)*inner_term;
		}
		self_hessian += hist_hess_term * curr_pix_jacobian.row(pix_id).transpose() * curr_pix_jacobian.row(pix_id)
			+ hist_grad_term * Map<const MatrixXd>(curr_pix_hessian.col(pix_id).data(), ssm_state_size, ssm_state_size);
	}
	for(int r = 0; r < params.n_bins; r++){
		for(int t = 0; t < params.n_bins; t++){
			int idx = linear_idx(r, t);
			double hist_factor = (1.0 / self_joint_hist(r, t)) - (1.0 / curr_hist(r));
			self_hessian += joint_hist_jacobian.row(idx).transpose() * joint_hist_jacobian.row(idx) * hist_factor;
		}
	}
}

//-----------------------------------functor support-----------------------------------//

void MI::updateDistFeat(double* feat_addr){
	Map<MatrixXdr> hist_mat(feat_addr, 5, patch_size);
	for(size_t pix_id = 0; pix_id < patch_size; pix_id++) {
		int pix_val_floor = static_cast<int>(It(pix_id));
		double pix_diff = std_bspl_ids(pix_val_floor, 0) - It(pix_id);
		hist_mat(0, pix_id) = pix_val_floor;
		hist_mat(1, pix_id) = utils::bSpl3(pix_diff);
		hist_mat(2, pix_id) = utils::bSpl3(++pix_diff);
		hist_mat(3, pix_id) = utils::bSpl3(++pix_diff);
		hist_mat(4, pix_id) = utils::bSpl3(++pix_diff);
	}
}

double MIDist::operator()(const double* hist1_mat_addr, const double* hist2_mat_addr,
	size_t hist_mat_size, double worst_dist) const{

	//printf("hist_mat_size: %ld\n", hist_mat_size);
	//printf("feat_size: %d\n", feat_size);

	assert(hist_mat_size == feat_size);

	VectorXd hist1(n_bins);
	VectorXd hist2(n_bins);
	MatrixXd joint_hist(n_bins, n_bins);

	hist1.fill(hist_pre_seed);
	hist2.fill(hist_pre_seed);
	joint_hist.fill(pre_seed);

	Map<const MatrixXdr> hist1_mat(hist1_mat_addr, 5, patch_size);
	Map<const MatrixXdr> hist2_mat(hist2_mat_addr, 5, patch_size);

	//utils::printMatrixToFile(hist1_mat, "hist1_mat", "log/mi_log.txt");
	//utils::printMatrixToFile(hist2_mat, "hist2_mat", "log/mi_log.txt");


	for(size_t pix_id = 0; pix_id < patch_size; ++pix_id) {
		int pix1_floor = static_cast<int>(hist1_mat(0, pix_id));
		int pix2_floor = static_cast<int>(hist2_mat(0, pix_id));

		//printf("pix1_floor: %d\n", pix1_floor);
		//if(pix2_floor >= static_params.n_bins){
		//	utils::printMatrixToFile(hist2_mat, "hist2_mat", "log/mi_log.txt");
		//	printf("pix2_floor: %d\n", pix2_floor);
		//}


		int bspl_id11 = (*std_bspl_ids)(pix1_floor, 0);
		int bspl_id12 = bspl_id11 + 1, bspl_id13 = bspl_id11 + 2, bspl_id14 = bspl_id11 + 3;
		int bspl_id21 = (*std_bspl_ids)(pix2_floor, 0);
		int bspl_id22 = bspl_id21 + 1, bspl_id23 = bspl_id21 + 2, bspl_id24 = bspl_id21 + 3;

		hist1(bspl_id11) += hist1_mat(1, pix_id);
		hist1(bspl_id12) += hist1_mat(2, pix_id);
		hist1(bspl_id13) += hist1_mat(3, pix_id);
		hist1(bspl_id14) += hist1_mat(4, pix_id);

		hist2(bspl_id21) += hist2_mat(1, pix_id);
		hist2(bspl_id22) += hist2_mat(2, pix_id);
		hist2(bspl_id23) += hist2_mat(3, pix_id);
		hist2(bspl_id24) += hist2_mat(4, pix_id);

		joint_hist(bspl_id11, bspl_id21) += hist1_mat(1, pix_id) * hist2_mat(1, pix_id);
		joint_hist(bspl_id12, bspl_id21) += hist1_mat(2, pix_id) * hist2_mat(1, pix_id);
		joint_hist(bspl_id13, bspl_id21) += hist1_mat(3, pix_id) * hist2_mat(1, pix_id);
		joint_hist(bspl_id14, bspl_id21) += hist1_mat(4, pix_id) * hist2_mat(1, pix_id);

		joint_hist(bspl_id11, bspl_id22) += hist1_mat(1, pix_id) * hist2_mat(2, pix_id);
		joint_hist(bspl_id12, bspl_id22) += hist1_mat(2, pix_id) * hist2_mat(2, pix_id);
		joint_hist(bspl_id13, bspl_id22) += hist1_mat(3, pix_id) * hist2_mat(2, pix_id);
		joint_hist(bspl_id14, bspl_id22) += hist1_mat(4, pix_id) * hist2_mat(2, pix_id);

		joint_hist(bspl_id11, bspl_id23) += hist1_mat(1, pix_id) * hist2_mat(3, pix_id);
		joint_hist(bspl_id12, bspl_id23) += hist1_mat(2, pix_id) * hist2_mat(3, pix_id);
		joint_hist(bspl_id13, bspl_id23) += hist1_mat(3, pix_id) * hist2_mat(3, pix_id);
		joint_hist(bspl_id14, bspl_id23) += hist1_mat(4, pix_id) * hist2_mat(3, pix_id);

		joint_hist(bspl_id11, bspl_id24) += hist1_mat(1, pix_id) * hist2_mat(4, pix_id);
		joint_hist(bspl_id12, bspl_id24) += hist1_mat(2, pix_id) * hist2_mat(4, pix_id);
		joint_hist(bspl_id13, bspl_id24) += hist1_mat(3, pix_id) * hist2_mat(4, pix_id);
		joint_hist(bspl_id14, bspl_id24) += hist1_mat(4, pix_id) * hist2_mat(4, pix_id);
	}
	//curr_hist *= hist_norm_mult;
	//init_hist *= hist_norm_mult;
	//joint_hist *= hist_norm_mult;

	//utils::printMatrixToFile(joint_hist, "joint_hist", "log/mi_log.txt");
	//utils::printMatrixToFile(hist1, "hist1", "log/mi_log.txt");
	//utils::printMatrixToFile(hist2, "hist2", "log/mi_log.txt");

	ResultType result = 0;
	for(int id1 = 0; id1 < n_bins; ++id1){
		for(int id2 = 0; id2 < n_bins; ++id2){
			result -= joint_hist(id1, id2) * (log(joint_hist(id1, id2) / (hist1(id1) * hist2(id2))) - log_hist_norm_mult);
		}
	}
	//result *= hist_norm_mult;
	return result;
}

//void  MI::cmptInitHessian(MatrixXd &hessian, const MatrixXd &init_pix_jacobian){
//	int ssm_state_size = init_pix_jacobian.cols();
//	assert(hessian.rows() == ssm_state_size && hessian.cols() == ssm_state_size);
//
//
//	clock_t start_time, end_time;
//
//	start_time = clock();
//	MatrixXd joint_hist_jacobian = init_joint_hist_grad*init_pix_jacobian;
//	MatrixXd joint_hist_hessian(ssm_state_size, ssm_state_size);
//	hessian.setZero();
//	for(int curr_id = 0; curr_id < params.n_bins; curr_id++){
//		for(int init_id = 0; init_id < params.n_bins; init_id++){
//			joint_hist_hessian.setZero();
//			for(int pix_id = 0; pix_id < ch_n_pix; pix_id++){
//				joint_hist_hessian += curr_hist_mat(curr_id, pix_id) * init_hist_hess(init_id, pix_id) * init_pix_jacobian.row(pix_id).transpose() * init_pix_jacobian.row(pix_id);
//			}
//			int joint_id = _linear_idx(curr_id, init_id);
//			hessian += joint_hist_jacobian.row(joint_id).transpose() * joint_hist_jacobian.row(joint_id) *  ((1.0 / joint_hist(curr_id, init_id)) - (1.0 / init_hist(init_id)))
//				+ joint_hist_hessian*init_grad_factor(curr_id, init_id);
//		}
//	}
//	end_time = clock();
//	printf("Naive time: %ld\n", end_time - start_time);
//
//	MatrixXd naive_hessian = hessian;
//
//	start_time = clock();
//	hessian.setZero();
//	joint_hist_jacobian.setZero();
//	for(int pix_id = 0; pix_id < ch_n_pix; pix_id++){
//		double hist_hess_term = 0;
//		for(int init_id = _init_bspl_ids(pix_id, 0); init_id <= _init_bspl_ids(pix_id, 1); init_id++) {
//			double inner_term = 0;
//			for(int curr_id = _curr_bspl_ids(pix_id, 0); curr_id <= _curr_bspl_ids(pix_id, 1); curr_id++) {
//				int joint_id = _linear_idx(curr_id, init_id);
//				joint_hist_jacobian.row(joint_id) += init_joint_hist_grad(joint_id, pix_id)*init_pix_jacobian.row(pix_id);
//				inner_term += curr_hist_mat(curr_id, pix_id) * init_grad_factor(curr_id, init_id);
//			}
//			hist_hess_term += init_hist_hess(init_id, pix_id)*inner_term;
//		}
//		hessian += hist_hess_term * init_pix_jacobian.row(pix_id).transpose() * init_pix_jacobian.row(pix_id);
//	}
//	for(int curr_id = 0; curr_id < params.n_bins; curr_id++){
//		for(int init_id = 0; init_id < params.n_bins; init_id++){
//			int joint_id = _linear_idx(curr_id, init_id);
//			double hist_factor = (1.0 / joint_hist(curr_id, init_id)) - (1.0 / init_hist(init_id));
//			hessian += joint_hist_jacobian.row(joint_id).transpose() * joint_hist_jacobian.row(joint_id) * hist_factor;
//		}
//	}
//	end_time = clock();
//	printf("Efficient time: %ld\n", end_time-start_time);
//	utils::printMatrix(naive_hessian - hessian, "hessian_diff");
//}


//void cmptSelfHessianSlow(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian){
//	MatrixXd joint_hist_jacobian(joint_hist_size, ssm_state_size);
//	MatrixXd joint_hist_hess(ssm_state_size*ssm_state_size, joint_hist_size);
//	MatrixXd pix_jacobian_sqr(ssm_state_size, ssm_state_size);
//
//	joint_hist_jacobian.setZero();
//	joint_hist_hess.setZero();
//	joint_hist.fill(params.pre_seed);
//
//	for(int pix_id = 0; pix_id < ch_n_pix; pix_id++){
//		double curr_diff = _curr_bspl_ids(pix_id, 0) - curr_pix_vals(pix_id);
//		pix_jacobian_sqr.noalias() = curr_pix_jacobian.row(pix_id).transpose() * curr_pix_jacobian.row(pix_id);
//		for(int r = _curr_bspl_ids(pix_id, 0); r <= _curr_bspl_ids(pix_id, 1); r++) {
//			curr_hist_hess(r, pix_id) = hist_norm_mult*utils::bSpl3Hess(curr_diff++);
//			for(int t = _curr_bspl_ids(pix_id, 0); t <= _curr_bspl_ids(pix_id, 1); t++) {
//				joint_hist(r, t) += curr_hist_mat(r, pix_id) * curr_hist_mat(t, pix_id);
//				int idx = _linear_idx(r, t);
//				joint_hist_jacobian.row(idx) += 
//					curr_hist_grad(r, pix_id)*curr_hist_mat(t, pix_id)*curr_pix_jacobian.row(pix_id);
//				Map<MatrixXd>(joint_hist_hess.col(idx).data(), ssm_state_size, ssm_state_size) +=
//					curr_hist_hess(r, pix_id)*curr_hist_mat(t, pix_id)*pix_jacobian_sqr;
//			}
//		}
//	}
//	joint_hist *= hist_norm_mult;
//	joint_hist_log = joint_hist.array().log();
//
//	self_hessian.setZero();
//	for(int r = 0; r < params.n_bins; r++){
//		for(int t = 0; t < params.n_bins; t++){
//			int idx = _linear_idx(r, t);
//			double factor_1 = (1.0 / joint_hist(r, t)) - (1.0 / curr_hist(r));
//			double factor_2 = 1 + joint_hist_log(r, t) - curr_hist_log(r);
//			self_hessian += joint_hist_jacobian.row(idx).transpose() * joint_hist_jacobian.row(idx) * factor_1
//				+ Map<MatrixXd>(joint_hist_hess.col(idx).data(), ssm_state_size, ssm_state_size)*factor_2;
//		}
//	}
//
//}

_MTF_END_NAMESPACE

