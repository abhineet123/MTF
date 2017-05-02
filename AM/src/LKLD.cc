#include "mtf/AM/LKLD.h"
#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/histUtils.h"

#define LKLD_N_BINS 8
#define LKLD_PRE_SEED 10
#define LKLD_POU true
#define LKLD_DEBUG_MODE false
#define LKLD_SUB_REGIONS 3
#define LKLD_SPACING 10

_MTF_BEGIN_NAMESPACE

//! value constructor
LKLDParams::LKLDParams(const AMParams *am_params,
	int _n_sub_regions_x, int _n_sub_regions_y,
	int _spacing_x, int _spacing_y,
	int _n_bins, double _pre_seed,
	bool _partition_of_unity,
	bool _debug_mode) :
	AMParams(am_params){
	n_sub_regions_x = _n_sub_regions_x;
	n_sub_regions_y = _n_sub_regions_y;
	spacing_x = _spacing_x;
	spacing_y = _spacing_y;
	n_bins = _n_bins;
	pre_seed = _pre_seed;
	partition_of_unity = _partition_of_unity;
	debug_mode = _debug_mode;

}
//! default/copy constructor
LKLDParams::LKLDParams(const LKLDParams *params) :
AMParams(params),
n_sub_regions_x(LKLD_SUB_REGIONS),
n_sub_regions_y(LKLD_SUB_REGIONS),
spacing_x(LKLD_SPACING),
spacing_y(LKLD_SPACING),
n_bins(LKLD_N_BINS),
pre_seed(LKLD_PRE_SEED),
partition_of_unity(LKLD_POU),
debug_mode(LKLD_DEBUG_MODE){
	if(!params){ return; }
	n_sub_regions_x = params->n_sub_regions_x;
	n_sub_regions_y = params->n_sub_regions_y;
	spacing_x = params->spacing_x;
	spacing_y = params->spacing_y;
	n_bins = params->n_bins;
	pre_seed = params->pre_seed;
	partition_of_unity = params->partition_of_unity;
	debug_mode = params->debug_mode;
}

LKLD::LKLD(const ParamType *kld_params) : 
AppearanceModel(kld_params), params(kld_params){
	printf("\n");
	printf("Using Localized Kullback Leibler Divergence AM with:\n");
	printf("sub_regions: %d x %d\n", params.n_sub_regions_x, params.n_sub_regions_y);
	printf("sub region spacing: %d x %d\n", params.spacing_x, params.spacing_y);
	printf("n_bins: %d\n", params.n_bins);
	printf("pre_seed: %f\n", params.pre_seed);
	printf("partition_of_unity: %d\n", params.partition_of_unity);
	printf("debug_mode: %d\n", params.debug_mode);

	name = "lkld";
	log_fname = "log/mtf_lkld_log.txt";
	time_fname = "log/mtf_lkld_times.txt";

	patch_size_x = resx;
	patch_size_y = resy;

	if(params.n_sub_regions_x <= 0 || params.n_sub_regions_y <= 0){
		printf("Using pixelwise KLD\n");
		params.n_sub_regions_x = patch_size_x;
		params.n_sub_regions_y = patch_size_y;
		params.spacing_x = params.spacing_y = 1;
	}

	sub_region_size_x = patch_size_x - (params.n_sub_regions_x - 1)*params.spacing_x;
	sub_region_size_y = patch_size_y - (params.n_sub_regions_y - 1)*params.spacing_y;
	sub_region_n_pix = sub_region_size_x*sub_region_size_y;
	n_sub_regions = params.n_sub_regions_x*params.n_sub_regions_y;


	if(sub_region_size_x <= 0 || sub_region_size_y <= 0){
		stringstream err_msg;
		err_msg << "LKLD :: Patch size : " << patch_size_x << "x" << patch_size_y <<
			"is not enough to use the specified region spacing and / or count";
		throw utils::InvalidArgument(err_msg.str());
	}
	printf("Using sub regions of size: %d x %d\n", sub_region_size_x, sub_region_size_y);

	double norm_pix_min = 0, norm_pix_max = params.n_bins - 1;
	if(params.partition_of_unity){
		if(params.n_bins < 4){
			throw utils::InvalidArgument("LKLD::Too few bins specified to enforce partition of unity constraint");
		}
		norm_pix_min = 1;
		norm_pix_max = params.n_bins - 2;
	}
	printf("norm_pix_min: %f\n", norm_pix_min);
	printf("norm_pix_max: %f\n", norm_pix_max);

	pix_norm_mult = (norm_pix_max - norm_pix_min) / (PIX_MAX - PIX_MIN);
	pix_norm_add = norm_pix_min;

	/* denominator of this factor is equal to the sum of all entries in the individual histograms,
	so multiplying hist with this factor will give the normalized hist whose entries sum to 1*/
	hist_norm_mult = 1.0 / (static_cast<double>(sub_region_n_pix) + params.pre_seed*params.n_bins);

	sub_region_x.resize(n_sub_regions, Eigen::NoChange);
	sub_region_y.resize(n_sub_regions, Eigen::NoChange);
	sub_region_pix_id.resize(n_sub_regions, sub_region_n_pix);

	int region_id = 0;
	for(int idx = 0; idx < params.n_sub_regions_x; idx++){
		for(int idy = 0; idy < params.n_sub_regions_y; idy++){

			sub_region_x(region_id, 0) = idx*params.spacing_x;
			sub_region_x(region_id, 1) = sub_region_x(region_id, 0) + sub_region_size_x - 1;
			sub_region_y(region_id, 0) = idy*params.spacing_y;
			sub_region_y(region_id, 1) = sub_region_y(region_id, 0) + sub_region_size_y - 1;

			int region_pix_id = 0;
			for(int x = sub_region_x(region_id, 0); x <= sub_region_x(region_id, 1); x++){
				for(int y = sub_region_y(region_id, 0); y <= sub_region_y(region_id, 1); y++){
					sub_region_pix_id(region_id, region_pix_id) = y*sub_region_size_x + x;
					++region_pix_id;
				}
			}
			++region_id;
		}
	}

	_std_bspl_ids.resize(params.n_bins, Eigen::NoChange);

	for(int i = 0; i < params.n_bins; i++) {
		_std_bspl_ids(i, 0) = max(0, i - 1);
		_std_bspl_ids(i, 1) = min(params.n_bins - 1, i + 2);
	}

	feat_size = params.n_bins;
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
void LKLD::initializeSimilarity(){
	if(!is_initialized.similarity){
		init_hists.resize(params.n_bins, n_sub_regions);
		init_hists_log.resize(params.n_bins, n_sub_regions);
		init_grad_factors.resize(params.n_bins, n_sub_regions);

		init_hist_mat.resize(params.n_bins, n_pix);

		curr_hists.resize(params.n_bins, n_sub_regions);
		curr_hists_log.resize(params.n_bins, n_sub_regions);
		curr_grad_factors.resize(params.n_bins, n_sub_regions);

		curr_hist_mat.resize(params.n_bins, n_pix);

		_init_bspl_ids.resize(n_pix, Eigen::NoChange);
		_curr_bspl_ids.resize(n_pix, Eigen::NoChange);

		// since histogram and its gradients are computed simultaneously, both must be resized here too
		init_hist_grad.resize(params.n_bins, n_pix);
		curr_hist_grad.resize(params.n_bins, n_pix);
	}

	
	_eig_set_zero(init_hist_grad, double);
	_eig_set_zero(init_hist_mat, double);

	double bspl, bspl_grad;
	for(unsigned int pix_id = 0; pix_id < n_pix; ++pix_id) {
		_init_bspl_ids.row(pix_id) = _std_bspl_ids.row(static_cast<int>(I0(pix_id)));
		double curr_diff = _init_bspl_ids(pix_id, 0) - I0(pix_id);
		for(int hist_id = _init_bspl_ids(pix_id, 0); hist_id <= _init_bspl_ids(pix_id, 1); hist_id++) {
			utils::bSpl3WithGrad(bspl, bspl_grad, curr_diff);
			init_hist_mat(hist_id, pix_id) = bspl;
			init_hist_grad(hist_id, pix_id) = -hist_norm_mult*bspl_grad;
			++curr_diff;
		}
	}
	init_hists.fill(params.pre_seed);
	for(int region_id = 0; region_id < n_sub_regions; region_id++){
		for(int region_pix_id = 0; region_pix_id < sub_region_n_pix; region_pix_id++){
			for(int hist_id = 0; hist_id < params.n_bins; hist_id++){
				init_hists(hist_id, region_id) += init_hist_mat(hist_id, sub_region_pix_id(region_id, region_pix_id));
			}
		}
	}
	init_hists *= hist_norm_mult;
	init_hists_log = init_hists.array().log();	

	if(!is_initialized.similarity){

		f = 0;

		_curr_bspl_ids = _init_bspl_ids;
		curr_hists = init_hists;
		curr_hist_mat = init_hist_mat;
		curr_hists_log = init_hists_log;
		curr_hist_grad = init_hist_grad;

		is_initialized.similarity = true;
	}
}
/**
* initializeGrad
* Prerequisites :: Computed in:
*	init_hist_grad :: initialize
* Computes :: Description
*	init_grad: gradient of initial LKLD w.r.t. initial pixel values
*/
void LKLD::initializeGrad(){

	if(!is_initialized.grad){
		df_dIt.resize(n_pix);
		df_dI0.resize(n_pix);
	}

	df_dI0 = init_hist_grad.colwise().sum();

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
*	similarity :: LKLD between the current and initial pixel values (optional)
*/
void LKLD::updateSimilarity(bool prereq_only){
	// compute both the histogram and its differential simultaneously
	// to take advantage of the many common computations involved
	_eig_set_zero(curr_hist_mat, double);
	_eig_set_zero(curr_hist_grad, double);
	for(unsigned int pix = 0; pix < n_pix; pix++) {
		_curr_bspl_ids.row(pix) = _std_bspl_ids.row(static_cast<int>(It(pix)));
		double curr_diff = _curr_bspl_ids(pix, 0) - It(pix);
		for(int id = _curr_bspl_ids(pix, 0); id <= _curr_bspl_ids(pix, 1); id++) {
			utils::bSpl3WithGrad(curr_hist_mat(id, pix), curr_hist_grad(id, pix), curr_diff);
			curr_hist_grad(id, pix) *= -hist_norm_mult;
			++curr_diff;
		}
	}
	curr_hists.fill(params.pre_seed);
	for(int region_id = 0; region_id < n_sub_regions; region_id++){
		for(int region_pix_id = 0; region_pix_id < sub_region_n_pix; region_pix_id++){
			for(int hist_id = 0; hist_id < params.n_bins; hist_id++){
				curr_hists(hist_id, region_id) += curr_hist_mat(hist_id, sub_region_pix_id(region_id, region_pix_id));
			}
		}
	}
	//curr_hists *= hist_norm_mult;
	//curr_hists_log = curr_hists.array().log();
	f = 0;
	for(int region_id = 0; region_id < n_sub_regions; region_id++){
		for(int hist_id = 0; hist_id < params.n_bins; hist_id++){
			curr_hists(hist_id, region_id) *= hist_norm_mult;
			curr_hists_log(hist_id, region_id) = log(curr_hists(hist_id, region_id));	

			init_grad_factors(hist_id, region_id) = curr_hists(hist_id, region_id) / init_hists(hist_id, region_id);
			curr_grad_factors(hist_id, region_id) = curr_hists_log(hist_id, region_id) - init_hists_log(hist_id, region_id);

			f -= curr_hists(hist_id, region_id) * curr_grad_factors(hist_id, region_id);
		}
	}
}

/**
* updateInitGrad
* Prerequisites :: Computed in:
*	init_hist_grad, _init_bspl_ids, init_hist_log :: initialize
*	curr_hist_mat, _curr_bspl_ids, curr_joint_hist_log :: update
* Computes :: Description
*	init_joint_hist_grad :: gradient of the current joint histogram w.r.t. initial pixel values
*	init_grad_factor :: 1 + log(curr_joint_hist/init_hist) - useful for computing the LKLD gradient too
*	init_grad: gradient of current LKLD w.r.t. initial pixel values
*/
void LKLD::updateInitGrad(){
	df_dI0.fill(0);
	for(int region_id = 0; region_id < n_sub_regions; region_id++){
		for(int region_pix_id = 0; region_pix_id < sub_region_n_pix; region_pix_id++){
			for(int hist_id = 0; hist_id < params.n_bins; hist_id++){
				int pix_id = sub_region_pix_id(region_id, region_pix_id);				
				df_dI0(pix_id) += init_hist_grad(hist_id, pix_id) * init_grad_factors(hist_id, region_id);
			}
		}
	}
}
/**
* updateCurrGrad
* Prerequisites :: Computed in:
*	curr_joint_hist_log, curr_hist_log, curr_joint_hist_grad :: update
* Computes :: Description
*	curr_grad_factor :: 1 + log(curr_joint_hist/curr_hist) - useful for computing the LKLD gradient too
*	curr_grad: gradient of current LKLD w.r.t. current pixel values
*/
void LKLD::updateCurrGrad(){
	//utils::printMatrixToFile(_curr_bspl_ids.transpose().eval(), "_curr_bspl_ids", log_fname, "%d");
	//utils::printMatrixToFile(curr_hist_grad, "curr_hist_grad", log_fname, "%15.9f");
	//utils::printMatrixToFile(curr_grad, "curr_grad", log_fname, "%15.9f");

	df_dIt.fill(0);
	for(int region_id = 0; region_id < n_sub_regions; region_id++){
		for(int region_pix_id = 0; region_pix_id < sub_region_n_pix; region_pix_id++){
			for(int hist_id = 0; hist_id < params.n_bins; hist_id++){
				int pix_id = sub_region_pix_id(region_id, region_pix_id);
				df_dIt(pix_id) -= curr_hist_grad(hist_id, pix_id) * curr_grad_factors(hist_id, region_id);
			}
		}
	}
}

void  LKLD::cmptInitHessian(MatrixXd &hessian, const MatrixXd &init_pix_jacobian){
	int ssm_state_size = init_pix_jacobian.cols();
	assert(hessian.rows() == ssm_state_size && hessian.cols() == ssm_state_size);
	assert(init_pix_jacobian.rows() == n_pix && init_pix_jacobian.cols() == ssm_state_size);

	MatrixXd init_hist_jacobian(params.n_bins, ssm_state_size);
	MatrixXd curr_hess(ssm_state_size, ssm_state_size);
	hessian.setZero();
	for(int region_id = 0; region_id < n_sub_regions; region_id++){		
		init_hist_jacobian.setZero();
		for(int hist_id = 0; hist_id < params.n_bins; hist_id++){
			for(int region_pix_id = 0; region_pix_id < sub_region_n_pix; region_pix_id++){
				int pix_id = sub_region_pix_id(region_id, region_pix_id);
				init_hist_jacobian.row(hist_id) += init_hist_grad(hist_id, pix_id)*init_pix_jacobian.row(pix_id);
			}
			curr_hess.noalias() = init_hist_jacobian.row(hist_id).transpose() * init_hist_jacobian.row(hist_id);
			hessian -= curr_hess * init_grad_factors(hist_id, region_id) / init_hists(hist_id, region_id);
		}
	}
}
/**
* Prerequisites :: Computed in:
*	curr_joint_hist_grad, curr_hist_grad :: updateCurrErrVec if fast joint histogram is enabled; updateCurrErrVecGrad otherwise
*	curr_joint_hist, curr_hist :: updateCurrErrVec
* Computes::Description
*	hessian::first order Hessian of current LKLD w.r.t. current pixel values
*/
void  LKLD::cmptCurrHessian(MatrixXd &hessian, const MatrixXd &curr_pix_jacobian){
	int ssm_state_size = curr_pix_jacobian.cols();
	assert(hessian.rows() == ssm_state_size && hessian.cols() == ssm_state_size);
	assert(curr_pix_jacobian.rows() == n_pix && curr_pix_jacobian.cols() == ssm_state_size);

	MatrixXd curr_hist_jacobian(params.n_bins, ssm_state_size);
	MatrixXd curr_hess(ssm_state_size, ssm_state_size);
	hessian.setZero();
	for(int region_id = 0; region_id < n_sub_regions; region_id++){
		curr_hist_jacobian.setZero();
		for(int hist_id = 0; hist_id < params.n_bins; hist_id++){
			for(int region_pix_id = 0; region_pix_id < sub_region_n_pix; region_pix_id++){
				int pix_id = sub_region_pix_id(region_id, region_pix_id);
				curr_hist_jacobian.row(hist_id) += curr_hist_grad(hist_id, pix_id)*curr_pix_jacobian.row(pix_id);
			}
			curr_hess.noalias() = curr_hist_jacobian.row(hist_id).transpose() * curr_hist_jacobian.row(hist_id);
			hessian -= curr_hess * init_grad_factors(hist_id, region_id) * (1 - 1 / curr_hists(hist_id, region_id));
		}
	}
}

void LKLD::initializeHess(){
	if(!is_initialized.hess){
		init_hist_hess.resize(params.n_bins, n_pix);
		curr_hist_hess.resize(params.n_bins, n_pix);
	}
	utils::getBSplHistHess(init_hist_hess, I0, _init_bspl_ids, n_pix, hist_norm_mult);

	is_initialized.hess = true;
}

void LKLD::cmptCurrHessian(MatrixXd &curr_hessian, const MatrixXd &curr_pix_jacobian,
	const MatrixXd &curr_pix_hessian){
	int ssm_state_size = curr_pix_jacobian.cols();

	assert(curr_hessian.rows() == ssm_state_size && curr_hessian.cols() == ssm_state_size);
	assert(curr_pix_jacobian.rows() == n_pix);
	assert(curr_pix_hessian.rows() == ssm_state_size*ssm_state_size);

	// get first order hessian
	cmptCurrHessian(curr_hessian, curr_pix_jacobian);

	for(unsigned int pix_id = 0; pix_id < n_pix; ++pix_id){
		curr_hessian += df_dIt(pix_id) * MatrixXdM((double*)curr_pix_hessian.col(pix_id).data(), ssm_state_size, ssm_state_size);
	}
}
void LKLD::cmptInitHessian(MatrixXd &init_hessian, const MatrixXd &init_pix_jacobian,
	const MatrixXd &init_pix_hessian){

	int ssm_state_size = init_pix_jacobian.cols();
	assert(init_pix_hessian.rows() == ssm_state_size*ssm_state_size);

	cmptInitHessian(init_hessian, init_pix_jacobian);

	for(unsigned int pix_id = 0; pix_id < n_pix; ++pix_id){
		init_hessian += df_dI0(pix_id) * MatrixXdM((double*)init_pix_hessian.col(pix_id).data(), ssm_state_size, ssm_state_size);
	}
}

void LKLD::cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian) {
	self_hessian.setZero();
}
void LKLD::cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian,
	const MatrixXd &curr_pix_hessian) {
	self_hessian.setZero();
}

void LKLD::initializeDistFeat(){
	feat_size = params.n_bins;
	feat_vec.resize(feat_size);
}

void LKLD::updateDistFeat(double* feat_addr){
	VectorXdM hist(feat_addr, params.n_bins);
	hist.fill(0);
	for(size_t pix_id = 0; pix_id < n_pix; ++pix_id) {
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

double LKLDDist::operator()(const double* hist1_addr, const double* hist2_addr,
	size_t hist_size, double worst_dist) const{
	assert(hist_size == feat_size);
	double result = 0;
	for(unsigned int hist_id = 0; hist_id < hist_size; ++hist_id){
		result -= (*hist1_addr) * log((*hist1_addr) / (*hist2_addr));
		hist1_addr++;
		hist2_addr++;
	}
	return result;
}

_MTF_END_NAMESPACE

