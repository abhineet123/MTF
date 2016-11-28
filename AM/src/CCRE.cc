#include "mtf/AM/CCRE.h"
#include "mtf/Utilities/histUtils.h"
#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/imgUtils.h"
#include "mtf/Utilities/graphUtils.h"
#include "opencv2/highgui/highgui.hpp"

_MTF_BEGIN_NAMESPACE

CCREParams::CCREParams(const AMParams *am_params,
int _n_bins, bool _partition_of_unity,
double _pre_seed, bool _symmetrical_grad,
int _n_blocks, 
bool _debug_mode) :
AMParams(am_params),
n_bins(_n_bins),
partition_of_unity(_partition_of_unity),
pre_seed(_pre_seed),
symmetrical_grad(_symmetrical_grad),
n_blocks(_n_blocks),
debug_mode(_debug_mode){}

//! default and copy constructor
CCREParams::CCREParams(const CCREParams *params) :
AMParams(params),
n_bins(CCRE_N_BINS),
partition_of_unity(CCRE_POU),
pre_seed(CCRE_PRE_SEED),
symmetrical_grad(CCRE_SYMMETRICAL_GRAD),
n_blocks(CCRE_N_BLOCKS),
debug_mode(CCRE_DEBUG_MODE){
	if(params){
		n_bins = params->n_bins;
		partition_of_unity = params->partition_of_unity;
		pre_seed = params->pre_seed;
		symmetrical_grad = params->symmetrical_grad;
		n_blocks = params->n_blocks;		
		debug_mode = params->debug_mode;
	}
}

CCRE::CCRE(const ParamType *ccre_params, int _n_channels) :
AppearanceModel(ccre_params, _n_channels),
params(ccre_params){
	printf("\n");
	printf("Using Cross Cumulative Residual Entropy AM with:\n");
	printf("n_bins: %d\n", params.n_bins);
	printf("pre_seed: %f\n", params.pre_seed);
	printf("partition_of_unity: %d\n", params.partition_of_unity);
	printf("symmetrical_grad: %d\n", params.symmetrical_grad);
	if(params.n_blocks <= 0){
		params.n_blocks = patch_size;
	}
	printf("n_blocks: %d\n", params.n_blocks);
	printf("likelihood_alpha: %f\n", params.likelihood_alpha);
	printf("likelihood_beta: %f\n", params.likelihood_beta);
	printf("debug_mode: %d\n", params.debug_mode);

	name = "ccre";
	log_fname = "log/mtf_ccre_log.txt";
	time_fname = "log/mtf_ccre_times.txt";

	double norm_pix_min = 0, norm_pix_max = params.n_bins - 1;
	if(params.partition_of_unity){
		if(params.n_bins < 4){
			throw std::invalid_argument(
				cv::format("CCRE::Too few bins %d specified to enforce partition of unity constraint", params.n_bins));
		}
		norm_pix_min = 1;
		norm_pix_max = params.n_bins - 2;
	}
	printf("norm_pix_min: %f\n", norm_pix_min);
	printf("norm_pix_max: %f\n", norm_pix_max);

	pix_norm_mult = (norm_pix_max - norm_pix_min) / (PIX_MAX - PIX_MIN);
	// there is an additional 1 in the denominator to ensure that the 
	// normalized pixel values is always < norm_pix_max;

	pix_norm_add = norm_pix_min;

	joint_hist_size = params.n_bins * params.n_bins; // size of the flattened joint histogram
	hist_pre_seed = params.n_bins * params.pre_seed;// preseeding the joint histogram by 's' is equivalent to 
	// preseeding individual histograms by s * n_bins

	//hist_mat_pre_seed = hist_pre_seed / static_cast<double>(patch_size); 
	/*since each element of hist is the sum of the corresponding row in hist_mat (that has patch_size elements),
	preseeding each element of hist with 's' is equivalent to preseeding each element of hist_mat with s/patch_size */

	hist_norm_mult = 1.0 / (static_cast<double>(patch_size)+hist_pre_seed*params.n_bins);
	log_hist_norm_mult = log(hist_norm_mult);
	/* denominator of this factor is equal to the sum of all entries in the individual histograms,
	so multiplying hist with this factor will give the normalized hist whose entries sum to 1*/

	_std_bspl_ids.resize(params.n_bins, Eigen::NoChange);
	// _linear_idx(i, j) stores the linear index of element (i,j) of a matrix
	// of dimensions params.n_bins x params.n_bins if it is to be flattened into a vector
	// in row major order; _linear_idx2 stores these for column major order flattening;
	_linear_idx.resize(params.n_bins, params.n_bins);
	_linear_idx2.resize(params.n_bins, params.n_bins);

	for(int bin_id = 0; bin_id < params.n_bins; bin_id++) {
		_std_bspl_ids(bin_id, 0) = max(0, bin_id - 1);
		_std_bspl_ids(bin_id, 1) = min(params.n_bins - 1, bin_id + 2);
		for(int j = 0; j < params.n_bins; j++) {
			_linear_idx2(j, bin_id) = _linear_idx(bin_id, j) = bin_id * params.n_bins + j;
		}
	}
	int pix_per_block = patch_size / params.n_blocks;
	printf("pix_per_block: %d\n", pix_per_block);
	block_extents.resize(params.n_blocks, Eigen::NoChange);
	for(int block_id = 0; block_id < params.n_blocks; block_id++) {
		block_extents(block_id, 0) = block_id*pix_per_block;
		block_extents(block_id, 1) = (block_id + 1)*pix_per_block - 1;
	}
	block_extents(params.n_blocks - 1, 1) = patch_size - 1;

#ifdef ENABLE_TBB
	printf(" ******* Parallelization enabled using TBB ******* \n");
#elif defined ENABLE_OMP
	printf(" ******* Parallelization enabled using OpenMP ******* \n");
#endif

	//for functor support
	feat_size = 9 * patch_size;
}

//-----------------------------------functor support-----------------------------------//

void CCRE::initializeSimilarity(){
	if(!is_initialized.similarity){
		init_hist.resize(params.n_bins);
		init_hist_mat.resize(params.n_bins, patch_size);
		init_hist_log.resize(params.n_bins);

		_init_bspl_ids.resize(patch_size, Eigen::NoChange);
		init_hist_grad.resize(params.n_bins, patch_size);

		init_cum_hist.resize(params.n_bins);
		init_cum_hist_mat.resize(params.n_bins, patch_size);
		init_cum_hist_grad.resize(params.n_bins, patch_size);
		init_cum_hist_log.resize(params.n_bins);
	}

	//! initial histogram and its gradient
	init_hist.fill(hist_pre_seed);
	init_hist_mat.setZero();
	init_hist_grad.setZero();
	for(int pix_id = 0; pix_id < patch_size; pix_id++) {
		_init_bspl_ids.row(pix_id) = _std_bspl_ids.row(static_cast<int>(I0(pix_id)));
		double curr_diff = _init_bspl_ids(pix_id, 0) - I0(pix_id);
		for(int hist_id = _init_bspl_ids(pix_id, 0); hist_id <= _init_bspl_ids(pix_id, 1); hist_id++) {
			utils::bSpl3WithGrad(init_hist_mat(hist_id, pix_id), init_hist_grad(hist_id, pix_id), curr_diff);
			init_hist_grad(hist_id, pix_id) *= -hist_norm_mult;
			init_hist(hist_id) += init_hist_mat(hist_id, pix_id);
			++curr_diff;
		}
	}
	init_hist *= hist_norm_mult;
	init_hist_log = init_hist.array().log();

	if(!is_initialized.similarity || params.symmetrical_grad){
		//! initial cumulative histogram and its gradient
		init_cum_hist.fill(hist_pre_seed);
		for(int pix_id = 0; pix_id < patch_size; pix_id++) {
			int cum_hist_id = 0;
			while(cum_hist_id < _init_bspl_ids(pix_id, 0)){
				init_cum_hist_grad(cum_hist_id, pix_id) = 0;
				init_cum_hist(cum_hist_id) += init_cum_hist_mat(cum_hist_id, pix_id) = 1;
				++cum_hist_id;
			}
			double curr_diff = cum_hist_id - I0(pix_id);
			while(cum_hist_id <= _init_bspl_ids(pix_id, 1)){
				utils::cumBSpl3WithGrad(init_cum_hist_mat(cum_hist_id, pix_id), init_cum_hist_grad(cum_hist_id, pix_id), curr_diff);
				init_cum_hist_grad(cum_hist_id, pix_id) *= -hist_norm_mult;
				init_cum_hist(cum_hist_id) += init_cum_hist_mat(cum_hist_id, pix_id);
				++curr_diff;
				++cum_hist_id;
			}
			while(cum_hist_id < params.n_bins){
				init_cum_hist_mat(cum_hist_id, pix_id) = init_cum_hist_grad(cum_hist_id, pix_id) = 0;
				++cum_hist_id;
			}
		}
		init_cum_hist *= hist_norm_mult;
		init_cum_hist_log = init_cum_hist.array().log();
	}

	//VectorXf init_hist_float = init_hist.cast<float>();
	//cv::imshow("CCRE :: init_hist", cv::Mat(utils::drawFloatGraph(init_hist_float.data(), params.n_bins)));

	if(!is_initialized.similarity){

		cum_joint_hist.resize(params.n_bins, params.n_bins);
		init_cum_joint_hist_grad.resize(joint_hist_size, patch_size);
		cum_joint_hist_log.resize(params.n_bins, params.n_bins);
		ccre_log_term.resize(params.n_bins, params.n_bins);

		//! initial cumulative joint histogram and its gradient
		cum_joint_hist.fill(params.pre_seed);
		init_cum_joint_hist_grad.setZero();
		for(int pix_id = 0; pix_id < patch_size; pix_id++) {
			for(int hist_id = _init_bspl_ids(pix_id, 0); hist_id <= _init_bspl_ids(pix_id, 1); ++hist_id) {
				int cum_hist_id = 0;
				while(cum_hist_id < _init_bspl_ids(pix_id, 0)){
					cum_joint_hist(cum_hist_id, hist_id) += init_hist_mat(hist_id, pix_id);
					++cum_hist_id;
				}
				while(cum_hist_id <= _init_bspl_ids(pix_id, 1)){
					cum_joint_hist(cum_hist_id, hist_id) += init_cum_hist_mat(cum_hist_id, pix_id) * init_hist_mat(hist_id, pix_id);
					init_cum_joint_hist_grad(_linear_idx(cum_hist_id, hist_id), pix_id) =
						init_cum_hist_grad(cum_hist_id, pix_id) * init_hist_mat(hist_id, pix_id);
					++cum_hist_id;
				}
			}
		}
		cum_joint_hist *= hist_norm_mult;
		cum_joint_hist_log = cum_joint_hist.array().log();

		for(int id1 = 0; id1 < params.n_bins; id1++){
			for(int id2 = 0; id2 < params.n_bins; id2++){
				ccre_log_term(id1, id2) = cum_joint_hist_log(id1, id2) - init_cum_hist_log(id1) - init_hist_log(id2);
			}
		}
		f = (cum_joint_hist.array()*ccre_log_term.array()).sum();
		max_similarity = f;
		likelihood_numr = max_similarity + params.likelihood_beta;

		curr_hist = init_hist;
		curr_hist_mat = init_hist_mat;
		curr_hist_grad = init_hist_grad;
		_curr_bspl_ids = _init_bspl_ids;
		curr_hist_log = init_hist_log;

		curr_cum_hist = init_cum_hist;
		curr_cum_hist_mat = init_cum_hist_mat;
		curr_cum_hist_grad = init_cum_hist_grad;
		curr_cum_hist_log = init_cum_hist_log;
		curr_cum_joint_hist_grad = init_cum_joint_hist_grad;

		is_initialized.similarity = true;
	}
}
/**
* initializeGrad
* Prerequisites :: Computed in:
*	init_hist_grad :: initialize
* Computes :: Description
*	init_grad :: 1 + log(curr_joint_hist/curr_hist) - useful for computing the MI gradient too
*	init_cum_joint_hist_grad: gradient of current MI w.r.t. current pixel values
*/
void CCRE::initializeGrad(){
	if(!is_initialized.grad){
		init_hist_grad_ratio.resize(params.n_bins, patch_size);

		df_dI0.resize(patch_size);
		df_dIt.resize(patch_size);

		for(int patch_id = 0; patch_id < patch_size; patch_id++){
			df_dI0(patch_id) = 0;
			for(int id1 = _init_bspl_ids(patch_id, 0); id1 <= _init_bspl_ids(patch_id, 1); id1++) {
				for(int id2 = _init_bspl_ids(patch_id, 0); id2 <= _init_bspl_ids(patch_id, 1); id2++) {
					df_dI0(patch_id) += init_cum_joint_hist_grad(_linear_idx(id1, id2), patch_id) * ccre_log_term(id1, id2);
				}
			}
		}
		df_dIt = df_dI0;
		is_initialized.grad = true;
	}
	if(!params.symmetrical_grad){
		init_hist_grad_ratio = init_hist_grad.array().colwise() / init_hist.array();
	}
}
/**
* update
* Prerequisites :: Computed in:
curr_pix_vals, _init_bspl_ids,
init_hist_mat, _std_bspl_ids, _linear_idx63
* Computes :: Description
cum_joint_hist, curr_cum_hist,
curr_cum_hist_mat, curr_cum_hist_grad,
_curr_bspl_ids,
*/
void CCRE::updateSimilarity(bool prereq_only){

	if(params.symmetrical_grad){
		updateSymSimilarity(prereq_only);
		return;
	}
	curr_cum_hist.fill(hist_pre_seed);
	cum_joint_hist.fill(params.pre_seed);

	if(is_initialized.grad){
		/**
		compute both the histogram and its differential simultaneously
		to take advantage of the many common computations involved
		*/
		curr_cum_hist_grad.setZero();
#pragma omp parallel for schedule(CCRE_OMP_SCHD)
		for(int pix_id = 0; pix_id < patch_size; pix_id++){
			_curr_bspl_ids.row(pix_id) = _std_bspl_ids.row(static_cast<int>(It(pix_id)));
			int curr_id = 0;
			while(curr_id < _curr_bspl_ids(pix_id, 0)){
				// curr_cum_hist_mat is unity and curr_cum_hist_grad is zero but the latter has already been zeroed
				curr_cum_hist(curr_id) += curr_cum_hist_mat(curr_id, pix_id) = 1;
				for(int init_id = _init_bspl_ids(pix_id, 0); init_id <= _init_bspl_ids(pix_id, 1); init_id++) {
					cum_joint_hist(curr_id, init_id) += init_hist_mat(init_id, pix_id);
				}
				++curr_id;
			}
			double curr_diff = curr_id - It(pix_id);
			while(curr_id <= _curr_bspl_ids(pix_id, 1)){
				utils::cumBSpl3WithGrad(curr_cum_hist_mat(curr_id, pix_id), curr_cum_hist_grad(curr_id, pix_id), curr_diff);
				curr_cum_hist_grad(curr_id, pix_id) *= -hist_norm_mult;

				curr_cum_hist(curr_id) += curr_cum_hist_mat(curr_id, pix_id);
				for(int init_id = _init_bspl_ids(pix_id, 0); init_id <= _init_bspl_ids(pix_id, 1); init_id++) {
					cum_joint_hist(curr_id, init_id) += curr_cum_hist_mat(curr_id, pix_id) * init_hist_mat(init_id, pix_id);
				}
				++curr_diff;
				++curr_id;
			}
		}
	} else{
#pragma omp parallel for schedule(CCRE_OMP_SCHD)
		for(int pix_id = 0; pix_id < patch_size; pix_id++){
			_curr_bspl_ids.row(pix_id) = _std_bspl_ids.row(static_cast<int>(It(pix_id)));
			int curr_id = 0;
			while(curr_id < _curr_bspl_ids(pix_id, 0)){
				//! curr_cum_hist_mat is unity
				curr_cum_hist(curr_id) += curr_cum_hist_mat(curr_id, pix_id) = 1;
				for(int init_id = _init_bspl_ids(pix_id, 0); init_id <= _init_bspl_ids(pix_id, 1); init_id++) {
					cum_joint_hist(curr_id, init_id) += init_hist_mat(init_id, pix_id);
				}
				++curr_id;
			}
			double curr_diff = curr_id - It(pix_id);
			while(curr_id <= _curr_bspl_ids(pix_id, 1)){
				curr_cum_hist_mat(curr_id, pix_id) = utils::cumBSpl3(curr_diff);
				curr_cum_hist(curr_id) += curr_cum_hist_mat(curr_id, pix_id);
				for(int init_id = _init_bspl_ids(pix_id, 0); init_id <= _init_bspl_ids(pix_id, 1); init_id++) {
					cum_joint_hist(curr_id, init_id) += curr_cum_hist_mat(curr_id, pix_id) * init_hist_mat(init_id, pix_id);
				}
				++curr_diff;
				++curr_id;
			}
		}
	}
	curr_cum_hist *= hist_norm_mult;
	cum_joint_hist *= hist_norm_mult;
	curr_cum_hist_log = curr_cum_hist.array().log();
	cum_joint_hist_log = cum_joint_hist.array().log();

	//VectorXf curr_cum_hist_float = curr_cum_hist.cast<float>();
	//cv::imshow("CCRE :: curr_cum_hist", cv::Mat(utils::drawFloatGraph(curr_cum_hist_float.data(), params.n_bins)));

	for(int curr_id = 0; curr_id < params.n_bins; curr_id++){
		for(int init_id = 0; init_id < params.n_bins; init_id++){
			ccre_log_term(curr_id, init_id) = cum_joint_hist_log(curr_id, init_id) - curr_cum_hist_log(curr_id) - init_hist_log(init_id);
		}
	}
	if(prereq_only){ return; }

	f = (cum_joint_hist.array()*ccre_log_term.array()).sum();
}
/**
* updateInitGrad
* Prerequisites :: Computed in:
*	init_hist_grad :: initialize
*	init_hist_grad_ratio :: initializeGrad
*	curr_cum_hist_mat, cum_joint_hist :: update
* Computes :: Description
*	init_grad ::
*	init_cum_joint_hist_grad: gradient of current MI w.r.t. current pixel values
*/
void CCRE::updateInitGrad(){
	if(params.symmetrical_grad){
		updateSymInitGrad();
		return;
	}
	// compute differential of the current joint histogram w.r.t. initial pixel values simultaneously with init_grad;
	// this does not need to be normalized since init_hist_grad has already been normalized
	init_cum_joint_hist_grad.setZero();
#pragma omp parallel for schedule(CCRE_OMP_SCHD)
	for(int pix_id = 0; pix_id < patch_size; pix_id++){
		df_dI0(pix_id) = 0;
		for(int init_id = _init_bspl_ids(pix_id, 0); init_id <= _init_bspl_ids(pix_id, 1); init_id++) {
			int curr_id = 0;
			while(curr_id < _curr_bspl_ids(pix_id, 0)){
				int joint_id = _linear_idx(curr_id, init_id);
				init_cum_joint_hist_grad(joint_id, pix_id) = init_hist_grad(init_id, pix_id);
				df_dI0(pix_id) += init_cum_joint_hist_grad(joint_id, pix_id) * (1 + ccre_log_term(curr_id, init_id))
					- cum_joint_hist(curr_id, init_id) * init_hist_grad_ratio(init_id, pix_id);
				++curr_id;
			}
			while(curr_id <= _curr_bspl_ids(pix_id, 1)){
				int joint_id = _linear_idx(curr_id, init_id);
				init_cum_joint_hist_grad(joint_id, pix_id) = curr_cum_hist_mat(curr_id, pix_id) * init_hist_grad(init_id, pix_id);
				df_dI0(pix_id) += init_cum_joint_hist_grad(joint_id, pix_id) * (1 + ccre_log_term(curr_id, init_id))
					- cum_joint_hist(curr_id, init_id) * init_hist_grad_ratio(init_id, pix_id);
				++curr_id;
			}
			while(curr_id < params.n_bins){
				df_dI0(pix_id) -= cum_joint_hist(curr_id, init_id) * init_hist_grad_ratio(init_id, pix_id);
				++curr_id;
			}
		}
	}
}
void CCRE::updateCurrGrad(){
	curr_cum_joint_hist_grad.setZero();
	for(int pix_id = 0; pix_id < patch_size; pix_id++){
		df_dIt(pix_id) = 0;
		for(int curr_id = _curr_bspl_ids(pix_id, 0); curr_id <= _curr_bspl_ids(pix_id, 1); curr_id++) {
			for(int init_id = _init_bspl_ids(pix_id, 0); init_id <= _init_bspl_ids(pix_id, 1); init_id++) {
				int joint_id = _linear_idx(curr_id, init_id);
				curr_cum_joint_hist_grad(joint_id, pix_id) = curr_cum_hist_grad(curr_id, pix_id) * init_hist_mat(init_id, pix_id);
				df_dIt(pix_id) += curr_cum_joint_hist_grad(joint_id, pix_id) * ccre_log_term(curr_id, init_id);
			}
		}
	}
	//if(params.debug_mode){
	//	utils::printMatrixToFile(_curr_bspl_ids.transpose(), "_curr_bspl_ids", log_fname, "%d");
	//	utils::printMatrixToFile(_init_bspl_ids.transpose(), "_init_bspl_ids", log_fname, "%d");
	//	utils::printMatrixToFile(curr_cum_hist_mat, "curr_cum_hist_mat", log_fname, "%e");
	//	utils::printMatrixToFile(ccre_log_term, "ccre_log_term", log_fname, "%e");
	//	utils::printMatrixToFile(curr_cum_hist_grad, "curr_cum_hist_grad", log_fname, "%e");
	//	utils::printMatrixToFile(init_hist_mat, "init_hist_mat", log_fname, "%e");
	//	utils::printMatrixToFile(curr_cum_joint_hist_grad, "curr_cum_joint_hist_grad", log_fname, "%e");
	//	utils::printMatrixToFile(curr_grad, "curr_grad", log_fname, "%e");
	//}
}
void CCRE::initializeHess(){
	if(!is_initialized.hess){
		curr_cum_hist_hess.resize(params.n_bins, patch_size);
		init_hess_factor.resize(params.n_bins, params.n_bins);
		cum_hess_factor.resize(params.n_bins, params.n_bins);

		self_cum_joint_hist.resize(params.n_bins, params.n_bins);
		self_cum_joint_hist_log.resize(params.n_bins, params.n_bins);
		self_ccre_log_term.resize(params.n_bins, params.n_bins);

		if(params.symmetrical_grad){
			init_cum_hist_hess.resize(params.n_bins, patch_size);
		} else{
			init_hist_hess.resize(params.n_bins, patch_size);
			init_hist_hess_ratio.resize(params.n_bins, patch_size);
			cum_joint_hist_sum.resize(params.n_bins);
		}
	}
	if(params.symmetrical_grad){
		utils::getCumBSplHistHess(
			// output arguments
			init_cum_hist_hess,
			// input arguments
			I0, _init_bspl_ids,
			patch_size, hist_norm_mult
			);
	} else{
		utils::getBSplHistHess(init_hist_hess, I0, _init_bspl_ids, patch_size, hist_norm_mult);
		init_hist_hess_ratio = init_hist_hess.array().colwise() / init_hist.array();
	}

	is_initialized.hess = true;
}
void  CCRE::cmptInitHessian(MatrixXd &hessian, const MatrixXd &init_pix_jacobian){
	if(params.symmetrical_grad){
		cmptSymInitHessian(hessian, init_pix_jacobian);
		return;
	}
	assert(hessian.rows() == hessian.cols() && hessian.cols() == init_pix_jacobian.cols());
	assert(init_pix_jacobian.rows() == patch_size);

	MatrixXd cum_joint_hist_jac = init_cum_joint_hist_grad*init_pix_jacobian;
	MatrixXd init_hist_grad_ratio_jac = init_hist_grad_ratio*init_pix_jacobian;
	hessian.setZero();
	for(int init_id = 0; init_id < params.n_bins; init_id++) {
		cum_joint_hist_sum(init_id) = 0;
		for(int curr_id = 0; curr_id < params.n_bins; curr_id++) {
			double joint_hist = cum_joint_hist(curr_id, init_id);
			int joint_id = _linear_idx(curr_id, init_id);

			hessian += cum_joint_hist_jac.row(joint_id).transpose()*
				(cum_joint_hist_jac.row(joint_id) / joint_hist
				-
				2 * init_hist_grad_ratio_jac.row(init_id));
			cum_joint_hist_sum(init_id) += joint_hist;
		}
		hessian += cum_joint_hist_sum(init_id)*init_hist_grad_ratio_jac.row(init_id).transpose()*init_hist_grad_ratio_jac.row(init_id);
	}
#pragma omp parallel for schedule(CCRE_OMP_SCHD)
	for(int pix_id = 0; pix_id < patch_size; pix_id++){
		double scalar_term = 0;
		for(int init_id = _init_bspl_ids(pix_id, 0); init_id <= _init_bspl_ids(pix_id, 1); init_id++) {
			for(int curr_id = 0; curr_id < params.n_bins; curr_id++) {
				double joint_hist_hess = init_hist_hess(init_id, pix_id) * curr_cum_hist_mat(curr_id, pix_id);
				scalar_term += joint_hist_hess*(ccre_log_term(curr_id, init_id) + 1);
			}
			scalar_term -= init_hist_hess_ratio(init_id, pix_id)*cum_joint_hist_sum(init_id);
		}
		hessian += scalar_term * init_pix_jacobian.row(pix_id).transpose() * init_pix_jacobian.row(pix_id);
	}
}
/*
Prerequisites: curr_cum_joint_hist_grad, ccre_log_term, init_hist_mat, cum_joint_hist, curr_cum_hist
*/
void  CCRE::cmptCurrHessian(MatrixXd &hessian, const MatrixXd &curr_pix_jacobian){
	int ssm_state_size = curr_pix_jacobian.cols();
	assert(hessian.rows() == ssm_state_size && hessian.cols() == ssm_state_size);
	assert(curr_pix_jacobian.rows() == patch_size);

	// compute curr_cum_hist_hess simultaneously with the hessian
	MatrixXd joint_hist_jacobian(joint_hist_size, ssm_state_size);
	hessian.setZero();
	joint_hist_jacobian.setZero();
	curr_cum_hist_hess.setZero();

#pragma omp parallel for schedule(CCRE_OMP_SCHD)
	for(int pix_id = 0; pix_id < patch_size; pix_id++){
		double curr_diff = _curr_bspl_ids(pix_id, 0) - It(pix_id);
		double hist_hess_term = 0;
		for(int curr_id = _curr_bspl_ids(pix_id, 0); curr_id <= _curr_bspl_ids(pix_id, 1); curr_id++) {
			double inner_term = 0;
			for(int init_id = _init_bspl_ids(pix_id, 0); init_id <= _init_bspl_ids(pix_id, 1); init_id++) {
				int joint_id = _linear_idx(curr_id, init_id);
				joint_hist_jacobian.row(joint_id) += curr_cum_joint_hist_grad(joint_id, pix_id)*curr_pix_jacobian.row(pix_id);
				inner_term += init_hist_mat(init_id, pix_id) * ccre_log_term(curr_id, init_id);
			}
			curr_cum_hist_hess(curr_id, pix_id) = hist_norm_mult*utils::cumBSpl3Hess(curr_diff);
			++curr_diff;
			hist_hess_term += curr_cum_hist_hess(curr_id, pix_id)*inner_term;
		}
		hessian += hist_hess_term * curr_pix_jacobian.row(pix_id).transpose() * curr_pix_jacobian.row(pix_id);
	}

	for(int curr_id = 0; curr_id < params.n_bins; curr_id++){
		for(int init_id = 0; init_id < params.n_bins; init_id++){
			int joint_id = _linear_idx(curr_id, init_id);
			double hist_factor = (1.0 / cum_joint_hist(curr_id, init_id)) - (1.0 / curr_cum_hist(curr_id));
			hessian += joint_hist_jacobian.row(joint_id).transpose() * joint_hist_jacobian.row(joint_id) * hist_factor;
		}
	}
}

void CCRE::cmptCumSelfHist(){
	// compute curr_hist, curr_hist_mat, curr_cum_hist_hess and self_cum_joint_hist in a single pass over the pixels;
	curr_hist.fill(hist_pre_seed);
	self_cum_joint_hist.fill(params.pre_seed);
	curr_hist_mat.setZero();
	curr_cum_hist_hess.setZero();

#pragma omp parallel for schedule(CCRE_OMP_SCHD)
	for(int pix_id = 0; pix_id < patch_size; pix_id++) {
		double curr_diff = _curr_bspl_ids(pix_id, 0) - It(pix_id);
		for(int hist_id = _curr_bspl_ids(pix_id, 0); hist_id <= _curr_bspl_ids(pix_id, 1); ++hist_id) {
			curr_hist_mat(hist_id, pix_id) = utils::bSpl3(curr_diff);
			curr_cum_hist_hess(hist_id, pix_id) = hist_norm_mult*utils::cumBSpl3Hess(curr_diff);
			++curr_diff;
			curr_hist(hist_id) += curr_hist_mat(hist_id, pix_id);

			int cum_hist_id = 0;
			while(cum_hist_id < _curr_bspl_ids(pix_id, 0)){
				self_cum_joint_hist(cum_hist_id, hist_id) += curr_hist_mat(hist_id, pix_id);
				++cum_hist_id;
			}
			while(cum_hist_id <= _curr_bspl_ids(pix_id, 1)){
				self_cum_joint_hist(cum_hist_id, hist_id) += curr_cum_hist_mat(cum_hist_id, pix_id) * curr_hist_mat(hist_id, pix_id);
				++cum_hist_id;
			}
		}
	}
	curr_hist *= hist_norm_mult;
	self_cum_joint_hist *= hist_norm_mult;
	curr_hist_log = curr_hist.array().log();
	self_cum_joint_hist_log = self_cum_joint_hist.array().log();

	for(int i = 0; i < params.n_bins; i++){
		for(int j = 0; j < params.n_bins; j++){
			self_ccre_log_term(i, j) = self_cum_joint_hist_log(i, j) - curr_cum_hist_log(i) - curr_hist_log(j);
		}
	}
}
void CCRE::cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian){
	int ssm_state_size = curr_pix_jacobian.cols();

	assert(self_hessian.rows() == ssm_state_size && self_hessian.cols() == ssm_state_size);
	assert(curr_pix_jacobian.rows() == patch_size);

	cmptCumSelfHist();
	MatrixXd joint_hist_jacobian(joint_hist_size, ssm_state_size);
	self_hessian.setZero();
	joint_hist_jacobian.setZero();
#pragma omp parallel for schedule(CCRE_OMP_SCHD)
	for(int pix_id = 0; pix_id < patch_size; pix_id++){
		double hist_hess_term = 0;
		for(int cum_hist_id = _curr_bspl_ids(pix_id, 0); cum_hist_id <= _curr_bspl_ids(pix_id, 1); ++cum_hist_id) {
			double inner_term = 0;
			for(int hist_id = _curr_bspl_ids(pix_id, 0); hist_id <= _curr_bspl_ids(pix_id, 1); ++hist_id) {
				int joint_id = _linear_idx(cum_hist_id, hist_id);
				joint_hist_jacobian.row(joint_id) += curr_cum_hist_grad(cum_hist_id, pix_id) * curr_hist_mat(hist_id, pix_id) *
					curr_pix_jacobian.row(pix_id);
				inner_term += curr_hist_mat(hist_id, pix_id) * self_ccre_log_term(cum_hist_id, hist_id);
			}
			hist_hess_term += curr_cum_hist_hess(cum_hist_id, pix_id)*inner_term;
		}
		self_hessian += hist_hess_term * curr_pix_jacobian.row(pix_id).transpose() * curr_pix_jacobian.row(pix_id);
	}
	for(int r = 0; r < params.n_bins; ++r){
		for(int t = 0; t < params.n_bins; ++t){
			int idx = _linear_idx(r, t);
			double hist_factor = (1.0 / self_cum_joint_hist(r, t)) - (1.0 / curr_cum_hist(r));
			self_hessian += joint_hist_jacobian.row(idx).transpose() * joint_hist_jacobian.row(idx) * hist_factor;
		}
	}
}

//-----------------------------------functions for symmetrical init_grad-----------------------------------//


void CCRE::updateSymSimilarity(bool prereq_only){
	curr_hist.fill(hist_pre_seed);
	curr_hist_mat.setZero();
	cum_joint_hist.fill(params.pre_seed);
	for(int pix_id = 0; pix_id < patch_size; pix_id++) {
		double curr_diff = _curr_bspl_ids(pix_id, 0) - It(pix_id);
		for(int curr_id = _curr_bspl_ids(pix_id, 0); curr_id <= _curr_bspl_ids(pix_id, 1); ++curr_id) {
			curr_hist_mat(curr_id, pix_id) = utils::bSpl3(curr_diff);
			++curr_diff;
			curr_hist(curr_id) += curr_hist_mat(curr_id, pix_id);
			int init_id = 0;
			while(init_id < _init_bspl_ids(pix_id, 0)){
				// init_cum_hist_mat is unity
				cum_joint_hist(init_id, curr_id) += curr_hist_mat(curr_id, pix_id);
				++init_id;
			}
			while(init_id <= _init_bspl_ids(pix_id, 1)){
				cum_joint_hist(init_id, curr_id) += init_cum_hist_mat(init_id, pix_id) * curr_hist_mat(curr_id, pix_id);
				++init_id;
			}
		}
	}
	cum_joint_hist *= hist_norm_mult;
	curr_hist *= hist_norm_mult;
	cum_joint_hist_log = cum_joint_hist.array().log();
	curr_hist_log = curr_hist.array().log();



	for(int init_id = 0; init_id < params.n_bins; init_id++){
		for(int curr_id = 0; curr_id < params.n_bins; curr_id++){
			ccre_log_term(init_id, curr_id) = cum_joint_hist_log(init_id, curr_id) - init_cum_hist_log(init_id) - curr_hist_log(curr_id);
		}
	}

	if(prereq_only){ return; }

	f = (cum_joint_hist.array()*ccre_log_term.array()).sum();

}
void CCRE::updateSymInitGrad(){
	init_cum_joint_hist_grad.setZero();
	for(int pix_id = 0; pix_id < patch_size; pix_id++) {
		df_dI0(pix_id) = 0;
		for(int curr_id = _curr_bspl_ids(pix_id, 0); curr_id <= _curr_bspl_ids(pix_id, 1); curr_id++) {
			for(int init_id = _init_bspl_ids(pix_id, 0); init_id <= _init_bspl_ids(pix_id, 1); init_id++) {
				int joint_id = _linear_idx(init_id, curr_id);
				init_cum_joint_hist_grad(joint_id, pix_id) = init_cum_hist_grad(init_id, pix_id) * curr_hist_mat(curr_id, pix_id);
				df_dI0(pix_id) += init_cum_joint_hist_grad(joint_id, pix_id) * ccre_log_term(init_id, curr_id);
			}
		}
	}
}
void  CCRE::cmptSymInitHessian(MatrixXd &hessian, const MatrixXd &init_pix_jacobian){
	assert(hessian.rows() == hessian.cols() && hessian.cols() == init_pix_jacobian.cols());
	assert(init_pix_jacobian.rows() == patch_size);

	int ssm_state_size = init_pix_jacobian.cols();
	MatrixXd joint_hist_jacobian(joint_hist_size, ssm_state_size);
	joint_hist_jacobian.setZero();
	hessian.setZero();


#pragma omp parallel for schedule(CCRE_OMP_SCHD)
	for(int pix_id = 0; pix_id < patch_size; pix_id++){
		double hist_hess_term = 0;
		for(int init_id = _init_bspl_ids(pix_id, 0); init_id <= _init_bspl_ids(pix_id, 1); init_id++) {
			double inner_term = 0;
			for(int curr_id = _curr_bspl_ids(pix_id, 0); curr_id <= _curr_bspl_ids(pix_id, 1); curr_id++) {
				int joint_id = _linear_idx(init_id, curr_id);
				joint_hist_jacobian.row(joint_id) += init_cum_joint_hist_grad(joint_id, pix_id)*init_pix_jacobian.row(pix_id);
				inner_term += curr_hist_mat(curr_id, pix_id) * ccre_log_term(init_id, curr_id);
			}
			hist_hess_term += init_cum_hist_hess(init_id, pix_id)*inner_term;
		}
		hessian += hist_hess_term * init_pix_jacobian.row(pix_id).transpose() * init_pix_jacobian.row(pix_id);
	}
	for(int init_id = 0; init_id < params.n_bins; init_id++){
		for(int curr_id = 0; curr_id < params.n_bins; curr_id++){
			int joint_id = _linear_idx(init_id, curr_id);
			double hist_factor = (1.0 / cum_joint_hist(init_id, curr_id)) - (1.0 / init_cum_hist(init_id));
			hessian += joint_hist_jacobian.row(joint_id).transpose() * joint_hist_jacobian.row(joint_id) * hist_factor;
		}
	}
}

//-----------------------------------Distance Feature-----------------------------------//

void CCRE::updateDistFeat(double* feat_addr){
	MatrixXdMr cum_hist_mat(feat_addr, 9, patch_size);

#pragma omp parallel for schedule(CCRE_OMP_SCHD)
	for(size_t patch_id = 0; patch_id < patch_size; patch_id++) {
		int pix_val_floor = static_cast<int>(It(patch_id));
		double pix_diff = _std_bspl_ids(pix_val_floor, 0) - It(patch_id);
		cum_hist_mat(0, patch_id) = pix_val_floor;
		cum_hist_mat(1, patch_id) = utils::cumBSpl3(pix_diff);
		cum_hist_mat(5, patch_id) = utils::bSpl3(pix_diff++);

		cum_hist_mat(2, patch_id) = utils::cumBSpl3(pix_diff);
		cum_hist_mat(6, patch_id) = utils::bSpl3(pix_diff++);

		cum_hist_mat(3, patch_id) = utils::cumBSpl3(pix_diff);
		cum_hist_mat(7, patch_id) = utils::bSpl3(pix_diff++);

		cum_hist_mat(4, patch_id) = utils::cumBSpl3(pix_diff);
		cum_hist_mat(8, patch_id) = utils::bSpl3(pix_diff);
	}
}
double CCRE::operator()(const double* hist1_mat_addr, const double* hist2_mat_addr,
	size_t hist_mat_size, double worst_dist) const{

	//printf("hist_mat_size: %ld\n", hist_mat_size);
	//printf("feat_size: %d\n", feat_size);

	assert(hist_mat_size == feat_size);

	VectorXd cum_hist(params.n_bins);
	VectorXd hist(params.n_bins);
	MatrixXd cum_joint_hist(params.n_bins, patch_size);

	cum_hist.fill(hist_pre_seed);
	hist.fill(hist_pre_seed);
	cum_joint_hist.fill(params.pre_seed);

	Map<const MatrixXdr> cum_hist_mat(hist1_mat_addr, 9, patch_size);
	Map<const MatrixXdr> hist_mat(hist2_mat_addr, 9, patch_size);

	//utils::printMatrixToFile(hist1_mat, "hist1_mat", "log/ccre_log.txt");
	//utils::printMatrixToFile(hist2_mat, "hist2_mat", "log/ccre_log.txt");


#pragma omp parallel for schedule(CCRE_OMP_SCHD)
	for(size_t patch_id = 0; patch_id < patch_size; patch_id++) {
		int pix1_floor = static_cast<int>(cum_hist_mat(0, patch_id));
		int pix2_floor = static_cast<int>(hist_mat(0, patch_id));

		//printf("pix1_floor: %d\n", pix1_floor);
		//if(pix2_floor >= static_params.n_bins){
		//	utils::printMatrixToFile(hist2_mat, "hist2_mat", "log/ccre_log.txt");
		//	printf("pix2_floor: %d\n", pix2_floor);
		//}
		int bspl_id11 = _std_bspl_ids(pix1_floor, 0);
		int bspl_id12 = bspl_id11 + 1, bspl_id13 = bspl_id11 + 2, bspl_id14 = bspl_id11 + 3;
		int bspl_id21 = _std_bspl_ids(pix2_floor, 0);
		int bspl_id22 = bspl_id21 + 1, bspl_id23 = bspl_id21 + 2, bspl_id24 = bspl_id21 + 3;

		cum_hist(bspl_id11) += cum_hist_mat(1, patch_id);
		cum_hist(bspl_id12) += cum_hist_mat(2, patch_id);
		cum_hist(bspl_id13) += cum_hist_mat(3, patch_id);
		cum_hist(bspl_id14) += cum_hist_mat(4, patch_id);

		hist(bspl_id21) += hist_mat(5, patch_id);
		hist(bspl_id22) += hist_mat(6, patch_id);
		hist(bspl_id23) += hist_mat(7, patch_id);
		hist(bspl_id24) += hist_mat(8, patch_id);

		cum_joint_hist(bspl_id11, bspl_id21) += cum_hist_mat(1, patch_id) * hist_mat(5, patch_id);
		cum_joint_hist(bspl_id12, bspl_id21) += cum_hist_mat(2, patch_id) * hist_mat(5, patch_id);
		cum_joint_hist(bspl_id13, bspl_id21) += cum_hist_mat(3, patch_id) * hist_mat(5, patch_id);
		cum_joint_hist(bspl_id14, bspl_id21) += cum_hist_mat(4, patch_id) * hist_mat(5, patch_id);

		cum_joint_hist(bspl_id11, bspl_id22) += cum_hist_mat(1, patch_id) * hist_mat(6, patch_id);
		cum_joint_hist(bspl_id12, bspl_id22) += cum_hist_mat(2, patch_id) * hist_mat(6, patch_id);
		cum_joint_hist(bspl_id13, bspl_id22) += cum_hist_mat(3, patch_id) * hist_mat(6, patch_id);
		cum_joint_hist(bspl_id14, bspl_id22) += cum_hist_mat(4, patch_id) * hist_mat(6, patch_id);

		cum_joint_hist(bspl_id11, bspl_id23) += cum_hist_mat(1, patch_id) * hist_mat(7, patch_id);
		cum_joint_hist(bspl_id12, bspl_id23) += cum_hist_mat(2, patch_id) * hist_mat(7, patch_id);
		cum_joint_hist(bspl_id13, bspl_id23) += cum_hist_mat(3, patch_id) * hist_mat(7, patch_id);
		cum_joint_hist(bspl_id14, bspl_id23) += cum_hist_mat(4, patch_id) * hist_mat(7, patch_id);

		cum_joint_hist(bspl_id11, bspl_id24) += cum_hist_mat(1, patch_id) * hist_mat(8, patch_id);
		cum_joint_hist(bspl_id12, bspl_id24) += cum_hist_mat(2, patch_id) * hist_mat(8, patch_id);
		cum_joint_hist(bspl_id13, bspl_id24) += cum_hist_mat(3, patch_id) * hist_mat(8, patch_id);
		cum_joint_hist(bspl_id14, bspl_id24) += cum_hist_mat(4, patch_id) * hist_mat(8, patch_id);
	}

	ResultType result = 0;
	for(int id1 = 0; id1 < params.n_bins; id1++){
		for(int id2 = 0; id2 < params.n_bins; id2++){
			result -= cum_joint_hist(id1, id2) * (log(cum_joint_hist(id1, id2) / (cum_hist(id1) * hist(id2))) - log_hist_norm_mult);
		}
	}
	return result;
}

//----------------------------------- Second order Hessians -----------------------------------//

void CCRE::cmptCurrHessian(MatrixXd &curr_hessian, const MatrixXd &curr_pix_jacobian,
	const MatrixXd &curr_pix_hessian){
	int ssm_state_size = curr_pix_jacobian.cols();

	assert(curr_hessian.rows() == ssm_state_size && curr_hessian.cols() == ssm_state_size);
	assert(curr_pix_jacobian.rows() == patch_size);
	assert(curr_pix_hessian.rows() == ssm_state_size*ssm_state_size);

	// get first order hessian
	cmptCurrHessian(curr_hessian, curr_pix_jacobian);

	for(int pix_id = 0; pix_id < patch_size; pix_id++){
		curr_hessian += df_dIt(pix_id) * Map<const MatrixXd>(curr_pix_hessian.col(pix_id).data(), ssm_state_size, ssm_state_size);
	}
}
void CCRE::cmptInitHessian(MatrixXd &init_hessian, const MatrixXd &init_pix_jacobian,
	const MatrixXd &init_pix_hessian){

	int ssm_state_size = init_pix_jacobian.cols();

	assert(init_hessian.rows() == ssm_state_size && init_hessian.cols() == ssm_state_size);
	assert(init_pix_jacobian.rows() == patch_size);
	assert(init_pix_hessian.rows() == ssm_state_size*ssm_state_size);

	cmptInitHessian(init_hessian, init_pix_jacobian);

	for(int pix_id = 0; pix_id < patch_size; pix_id++){
		init_hessian += df_dI0(pix_id) * Map<const MatrixXd>(init_pix_hessian.col(pix_id).data(), ssm_state_size, ssm_state_size);
	}
}
void CCRE::cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian,
	const MatrixXd &curr_pix_hessian){
	int ssm_state_size = curr_pix_jacobian.cols();

	assert(self_hessian.rows() == ssm_state_size && self_hessian.cols() == ssm_state_size);
	assert(curr_pix_jacobian.rows() == patch_size);
	assert(curr_pix_hessian.rows() == ssm_state_size*ssm_state_size);

	cmptCumSelfHist();

	MatrixXd joint_hist_jacobian(joint_hist_size, ssm_state_size);
	self_hessian.setZero();
	joint_hist_jacobian.setZero();


#pragma omp parallel for schedule(CCRE_OMP_SCHD)
	for(int pix_id = 0; pix_id < patch_size; pix_id++){
		double hist_hess_term = 0, hist_grad_term = 0;
		for(int r = _curr_bspl_ids(pix_id, 0); r <= _curr_bspl_ids(pix_id, 1); r++) {
			double inner_term = 0;
			for(int t = _curr_bspl_ids(pix_id, 0); t <= _curr_bspl_ids(pix_id, 1); t++) {
				int idx = _linear_idx(r, t);
				joint_hist_jacobian.row(idx) += curr_cum_hist_grad(r, pix_id) * curr_hist_mat(t, pix_id)*curr_pix_jacobian.row(pix_id);
				inner_term += curr_hist_mat(t, pix_id) * self_ccre_log_term(r, t);
			}
			hist_hess_term += curr_cum_hist_hess(r, pix_id)*inner_term;
			hist_grad_term += curr_cum_hist_grad(r, pix_id)*inner_term;
		}
		self_hessian += hist_hess_term * curr_pix_jacobian.row(pix_id).transpose() * curr_pix_jacobian.row(pix_id)
			+ hist_grad_term * Map<const MatrixXd>(curr_pix_hessian.col(pix_id).data(), ssm_state_size, ssm_state_size);
	}
	for(int r = 0; r < params.n_bins; r++){
		for(int t = 0; t < params.n_bins; t++){
			int idx = _linear_idx(r, t);
			double hist_factor = (1.0 / self_cum_joint_hist(r, t)) - (1.0 / curr_cum_hist(r));
			self_hessian += joint_hist_jacobian.row(idx).transpose() * joint_hist_jacobian.row(idx) * hist_factor;
		}
	}
}


_MTF_END_NAMESPACE