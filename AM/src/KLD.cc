#include "mtf/AM/KLD.h"
#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/imgUtils.h"
#include "mtf/Utilities/histUtils.h"


_MTF_BEGIN_NAMESPACE

//! value constructor
KLDParams::KLDParams(const AMParams *am_params,
	int _n_bins, double _pre_seed,
	bool _partition_of_unity, bool _debug_mode) :
	AMParams(am_params){
	n_bins = _n_bins;
	pre_seed = _pre_seed;
	partition_of_unity = _partition_of_unity;
	debug_mode = _debug_mode;

}
//! default/copy constructor
KLDParams::KLDParams(const KLDParams *params) :
AMParams(params),
n_bins(KLD_N_BINS),
pre_seed(KLD_PRE_SEED),
partition_of_unity(KLD_POU),
debug_mode(KLD_DEBUG_MODE){
	if(params){
		n_bins = params->n_bins;
		pre_seed = params->pre_seed;
		partition_of_unity = params->partition_of_unity;
		debug_mode = params->debug_mode;
	}
}

KLD::KLD(const ParamType *kld_params) : AppearanceModel(kld_params),
params(kld_params){
	printf("\n");
	printf("Using Kullback Leibler Divergence AM with:\n");
	printf("n_bins: %d\n", params.n_bins);
	printf("pre_seed: %f\n", params.pre_seed);
	printf("partition_of_unity: %d\n", params.partition_of_unity);
	printf("debug_mode: %d\n", params.debug_mode);

	name = "kld";
	log_fname = "log/mtf_kld_log.txt";
	time_fname = "log/mtf_kld_times.txt";

	double norm_pix_min = 0, norm_pix_max = params.n_bins - 1;
	if(params.partition_of_unity){
		assert(params.n_bins > 3);
		norm_pix_min = 1;
		norm_pix_max = params.n_bins - 2;
	}
	printf("norm_pix_min: %f\n", norm_pix_min);
	printf("norm_pix_max: %f\n", norm_pix_max);

	pix_norm_mult = (norm_pix_max - norm_pix_min) / (PIX_MAX - PIX_MIN);
	pix_norm_add = norm_pix_min;

	hist_norm_mult = 1.0 / (static_cast<double>(n_pix)+params.pre_seed * params.n_bins);
	/* denominator of this factor is equal to the sum of all entries in the individual histograms,
	so multiplying hist with this factor will give the normalized hist whose entries sum to 1*/

	_std_bspl_ids.resize(params.n_bins, Eigen::NoChange);

	for(int i = 0; i < params.n_bins; i++) {
		_std_bspl_ids(i, 0) = max(0, i - 1);
		_std_bspl_ids(i, 1) = min(params.n_bins - 1, i + 2);
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
*	init_hist_grad :: gradient of the initial histogram w.r.t. pixel values
*/
void KLD::initializeSimilarity(){
	if(!is_initialized.similarity){
		init_hist.resize(params.n_bins);
		init_hist_mat.resize(params.n_bins, n_pix);
		init_hist_log.resize(params.n_bins);
		init_grad_factor.resize(params.n_bins);

		curr_hist.resize(params.n_bins);
		curr_hist_mat.resize(params.n_bins, n_pix);
		curr_hist_log.resize(params.n_bins);
		curr_grad_factor.resize(params.n_bins);

		_init_bspl_ids.resize(n_pix, Eigen::NoChange);
		_curr_bspl_ids.resize(n_pix, Eigen::NoChange);

		// since histogram and its gradients are computed simultaneously, both must be resized here too
		init_hist_grad.resize(params.n_bins, n_pix);
		curr_hist_grad.resize(params.n_bins, n_pix);
	}

	//! initial histogram and its gradient
	utils::getBSplHistWithGrad(init_hist, init_hist_mat, init_hist_grad,
		_init_bspl_ids, I0, _std_bspl_ids, 
		params.pre_seed, n_pix, hist_norm_mult);

	init_hist_log = init_hist.array().log();

	f = 0;

	if(!is_initialized.similarity){
		_curr_bspl_ids = _init_bspl_ids;
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
*	init_hist_grad :: initialize
* Computes :: Description
*	init_grad: gradient of initial KLD w.r.t. initial pixel values
*/
void KLD::initializeGrad(){

	if(!is_initialized.grad){
		df_dIt.resize(n_pix);
		df_dI0.resize(n_pix);
	}

	df_dI0 = -init_hist_grad.colwise().sum();
	if(!is_initialized.grad){
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
*	curr_joint_hist :: joint histogram between the current and initial pixel values
*	curr_hist_log :: entry wise logarithm of curr_hist
*	curr_joint_hist_log :: entry wise logarithm of curr_joint_hist
*	curr_hist_grad :: gradient of the current histogram w.r.t. current pixel values
*	curr_joint_hist_grad :: gradient of the current joint histogram w.r.t. current pixel values
*	similarity :: KLD between the current and initial pixel values (optional)
*/
void KLD::updateSimilarity(bool prereq_only){
	// compute both the histogram and its differential simultaneously
	// to take advantage of the many common computations involved
	utils::getBSplHistWithGrad(curr_hist, curr_hist_mat,
		curr_hist_grad, _curr_bspl_ids,	It, _std_bspl_ids,
		params.pre_seed, n_pix, hist_norm_mult);

	curr_hist_log = curr_hist.array().log();

	for(int hist_id = 0; hist_id < params.n_bins; hist_id++){
		init_grad_factor(hist_id) = curr_hist(hist_id) / init_hist(hist_id);
		curr_grad_factor(hist_id) = curr_hist_log(hist_id) - init_hist_log(hist_id);
	}

	if(prereq_only){ return; }

	f = 0;
	for(int hist_id = 0; hist_id < params.n_bins; hist_id++){
		f += curr_hist(hist_id) * curr_grad_factor(hist_id);
	}
}

/**
* updateInitGrad
* Prerequisites :: Computed in:
*	init_hist_grad, _init_bspl_ids, init_hist_log :: initialize
*	curr_hist_mat, _curr_bspl_ids, curr_joint_hist_log :: update
* Computes :: Description
*	init_joint_hist_grad :: gradient of the current joint histogram w.r.t. initial pixel values
*	init_grad_factor :: 1 + log(curr_joint_hist/init_hist) - useful for computing the KLD gradient too
*	init_grad: gradient of current KLD w.r.t. initial pixel values
*/
void KLD::updateInitGrad(){
	df_dI0.fill(0);
	for(int pix = 0; pix < n_pix; pix++){
		for(int id = _init_bspl_ids(pix, 0); id <= _init_bspl_ids(pix, 1); id++) {
			df_dI0(pix) -= init_hist_grad(id, pix) * init_grad_factor(id);
		}
	}	
}
/**
* updateCurrGrad
* Prerequisites :: Computed in:
*	curr_joint_hist_log, curr_hist_log, curr_joint_hist_grad :: update
* Computes :: Description
*	curr_grad_factor :: 1 + log(curr_joint_hist/curr_hist) - useful for computing the KLD gradient too
*	curr_grad: gradient of current KLD w.r.t. current pixel values
*/
void KLD::updateCurrGrad(){
	//utils::printMatrixToFile(_curr_bspl_ids.transpose().eval(), "_curr_bspl_ids", log_fname, "%d");
	//utils::printMatrixToFile(curr_hist_grad, "curr_hist_grad", log_fname, "%15.9f");
	//utils::printMatrixToFile(curr_grad, "curr_grad", log_fname, "%15.9f");

	df_dIt.fill(0);
	for(int pix_id = 0; pix_id < n_pix; pix_id++){
		//printf("pix_id: %d\n", pix_id);
		for(int hist_id = _curr_bspl_ids(pix_id, 0); hist_id <= _curr_bspl_ids(pix_id, 1); hist_id++) {
			//printf("hist_id: %d\n", hist_id);
			df_dIt(pix_id) += curr_hist_grad(hist_id, pix_id) * curr_grad_factor(hist_id);
		}
	}
}

void  KLD::cmptInitHessian(MatrixXd &hessian, const MatrixXd &init_pix_jacobian){
	int state_vec_size = init_pix_jacobian.cols();
	assert(hessian.rows() == state_vec_size && hessian.cols() == state_vec_size);

	MatrixXd init_hist_jacobian = init_hist_grad * init_pix_jacobian;

	MatrixXd curr_hess(state_vec_size, state_vec_size);
	hessian.fill(0);
	for(int id = 0; id < params.n_bins; id++){
		curr_hess.noalias() = init_hist_jacobian.row(id).transpose() * init_hist_jacobian.row(id);
		double hist_factor = init_grad_factor(id) / init_hist(id);
		hessian += curr_hess * hist_factor;
	}
#ifdef LOG_KLD_DATA
	if(params.debug_mode){
		printf("cmptInitHessian end: hessian.rows(): %d\t hessian.cols(): %d\n", (int)hessian.rows(), (int)hessian.cols());
	}
#endif
}
/**
* Prerequisites :: Computed in:
*	curr_joint_hist_grad, curr_hist_grad :: updateCurrErrVec if fast joint histogram is enabled; updateCurrErrVecGrad otherwise
*	curr_joint_hist, curr_hist :: updateCurrErrVec
* Computes::Description
*	hessian::first order Hessian of current KLD w.r.t. current pixel values
*/
void  KLD::cmptCurrHessian(MatrixXd &hessian, const MatrixXd &curr_pix_jacobian){
	int state_vec_size = curr_pix_jacobian.cols();
	assert(hessian.rows() == state_vec_size && hessian.cols() == state_vec_size);
	assert(curr_pix_jacobian.rows() == n_pix);

	//printf("First order Hessian\n");

	// jacobians of the current histogram w.r.t. SSM parameters
	MatrixXd curr_hist_jacobian = curr_hist_grad * curr_pix_jacobian;	
	MatrixXd curr_hess(state_vec_size, state_vec_size);
	hessian.fill(0);
	for(int id = 0; id < params.n_bins; id++){
		curr_hess.noalias() = curr_hist_jacobian.row(id).transpose() * curr_hist_jacobian.row(id);
		double hist_factor = init_grad_factor(id) * (1 - 1 / curr_hist(id));
		hessian += curr_hess * hist_factor;
	}
#ifdef LOG_KLD_DATA
	if(params.debug_mode){
		utils::printScalar(state_vec_size, "state_vec_size", "%d");
		utils::printMatrixToFile(curr_hist_jacobian, "curr_hist_jacobian", log_fname, "%15.9f", "a");
		utils::printMatrixToFile(hessian, "hessian", log_fname, "%15.9f", "a");
	}
#endif
}

void KLD::initializeHess(){
	if(!is_initialized.hess){
		init_hist_hess.resize(params.n_bins, n_pix);
		curr_hist_hess.resize(params.n_bins, n_pix);
	}
	utils::getBSplHistHess(init_hist_hess, I0, _init_bspl_ids, n_pix, hist_norm_mult);

	is_initialized.hess = true;
}

void KLD::cmptInitHessian(MatrixXd &init_hessian, const MatrixXd &init_pix_jacobian,
	const MatrixXd &init_pix_hessian){

	int state_vec_size = init_pix_jacobian.cols();

	assert(init_hessian.rows() == state_vec_size && init_hessian.cols() == state_vec_size);
	assert(init_pix_jacobian.rows() == n_pix);
	assert(init_pix_hessian.rows() == state_vec_size*state_vec_size);
	
	cmptInitHessian(init_hessian, init_pix_jacobian);

	MatrixXd pix_jac_ssm(state_vec_size, state_vec_size);
	MatrixXd hessian_new(state_vec_size, state_vec_size);
	hessian_new.fill(0);
	MatrixXd pix_term(state_vec_size, state_vec_size);
	for(int pix_id = 0; pix_id < n_pix; pix_id++){
		Map<const MatrixXd> pix_hess_ssm(init_pix_hessian.col(pix_id).data(), state_vec_size, state_vec_size);
		pix_jac_ssm.noalias() = init_pix_jacobian.row(pix_id).transpose() * init_pix_jacobian.row(pix_id);
		for(int hist_id = _init_bspl_ids(pix_id, 0); hist_id <= _init_bspl_ids(pix_id, 1); hist_id++) {
			pix_term.noalias() = init_hist_hess(hist_id, pix_id)*pix_jac_ssm + init_hist_grad(hist_id, pix_id)*pix_hess_ssm;
			hessian_new += pix_term * init_grad_factor(hist_id);
		}
	}
	init_hessian -= hessian_new;
}
/**
* Prerequisites :: Computed in:
*		[curr_joint_hist_log, curr_hist_log] :: updateCurrGrad
* Computes :: Description:
*		curr_hessian :: hessian of the error norm w.r.t. current SSM parameters
*/
void KLD::cmptCurrHessian(MatrixXd &curr_hessian, const MatrixXd &curr_pix_jacobian,
	const MatrixXd &curr_pix_hessian){
	int state_vec_size = curr_pix_jacobian.cols();

	assert(curr_hessian.rows() == state_vec_size && curr_hessian.cols() == state_vec_size);
	assert(curr_pix_jacobian.rows() == n_pix);
	assert(curr_pix_hessian.rows() == state_vec_size*state_vec_size);

	// get first order hessian
	cmptCurrHessian(curr_hessian, curr_pix_jacobian);
	// get hessian of bspline histogram
	utils::getBSplHistHess(curr_hist_hess, It, _curr_bspl_ids, n_pix, hist_norm_mult);

	MatrixXd pix_jac_ssm(state_vec_size, state_vec_size);
	MatrixXd hessian_new(state_vec_size, state_vec_size);	

	hessian_new.fill(0);
	MatrixXd pix_term(state_vec_size, state_vec_size);
	for(int pix_id = 0; pix_id < n_pix; pix_id++){
		Map<const MatrixXd> pix_hess_ssm(curr_pix_hessian.col(pix_id).data(), state_vec_size, state_vec_size);
		pix_jac_ssm.noalias() = curr_pix_jacobian.row(pix_id).transpose() * curr_pix_jacobian.row(pix_id);
		for(int hist_id = _curr_bspl_ids(pix_id, 0); hist_id <= _curr_bspl_ids(pix_id, 1); hist_id++) {
			pix_term.noalias() = curr_hist_hess(hist_id, pix_id)*pix_jac_ssm + curr_hist_grad(hist_id, pix_id)*pix_hess_ssm;
			hessian_new += pix_term * (curr_grad_factor(hist_id) + 1.0 / init_grad_factor(hist_id));
		}
	}
	curr_hessian += hessian_new;

	if(params.debug_mode){
		utils::printMatrixToFile(curr_hessian, "second order hessian", log_fname, "%15.9f", "a");
	}
	//utils::printMatrix(hessian_new, "second order term");
	//utils::printMatrix(curr_hessian, "second order hessian");
}



void KLD::initializeDistFeat(){
	feat_size = params.n_bins;
	feat_vec.resize(feat_size);
}

void KLD::updateDistFeat(double* feat_addr){
	VectorXdM hist((double*)feat_addr, params.n_bins);
	hist.fill(0);
	for(size_t pix_id = 0; pix_id < n_pix; pix_id++) {
		int pix_val_floor = static_cast<int>(It(pix_id));
		double pix_diff = _std_bspl_ids(pix_val_floor, 0) - It(pix_id);
		for(int hist_id = _std_bspl_ids(pix_val_floor, 0); hist_id<=_std_bspl_ids(pix_val_floor, 1); hist_id++) {
			hist(hist_id) += utils::bSpl3(pix_diff++);
		}
	}
	//if(params.debug_mode){
	//	utils::printMatrix(hist, "hist");
	//}
	hist *= hist_norm_mult;
	//if(params.debug_mode){
	//	utils::printScalar(hist_norm_mult, "hist_norm_mult");
	//	utils::printMatrix(hist, "normalized hist");
	//}
}

double KLD::operator()(const double* hist1_addr, const double* hist2_addr,
	size_t hist_size, double worst_dist) const{
	assert(hist_size == feat_size);

	ResultType result = 0;
	for(int hist_id = 0; hist_id < hist_size; hist_id++){
		result -= (*hist1_addr) * log((*hist1_addr) / (*hist2_addr));
		hist1_addr++;
		hist2_addr++;
	}
	return result;
}

_MTF_END_NAMESPACE

