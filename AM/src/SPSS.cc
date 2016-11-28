#include "mtf/AM/SPSS.h"
#include "mtf/Utilities/imgUtils.h"

_MTF_BEGIN_NAMESPACE

SPSSParams::SPSSParams(const AMParams *am_params,
double _k, 
ImageBase *_pix_mapper) :
AMParams(am_params),
k(_k),
pix_mapper(_pix_mapper){}

SPSSParams::SPSSParams(const SPSSParams *params) :
AMParams(params),
k(SPSS_K),
pix_mapper(SPSS_PIX_MAPPER){
	if(params){
		k = params->k;		
		pix_mapper = params->pix_mapper;
	}
}

SPSS::SPSS(const ParamType *spss_params, const int _n_channels) :
AppearanceModel(spss_params, _n_channels),
params(spss_params){
	name = "spss";
	printf("\n");
	printf("Using Sum of Pixelwise Structural Similarity AM with...\n");
	printf("k: %f\n", params.k);
	printf("likelihood_alpha: %f\n", params.likelihood_alpha);
	printf("grad_eps: %e\n", params.grad_eps);
	printf("hess_eps: %e\n", params.hess_eps);

	c = params.k*(PIX_MAX - PIX_MIN);
	c *= c;
	printf("c: %f\n", c);

	if(!params.pix_mapper){
		printf("Pixel mapping is disabled\n");
	}
}

double SPSS::getLikelihood() const{
	return exp(params.likelihood_alpha*(f - patch_size));
}

void SPSS::initializePixVals(const Matrix2Xd& init_pts){
	if(!is_initialized.pix_vals){
		I0.resize(patch_size);
		It.resize(patch_size);
	}
	if(params.pix_mapper){
		params.pix_mapper->initializePixVals(init_pts);
		I0 = params.pix_mapper->getInitPixVals();

	} else{
		switch(n_channels){
		case 1:
			utils::getPixVals(I0, curr_img, init_pts, n_pix,
				img_height, img_width, pix_norm_mult, pix_norm_add);
			break;
		case 3:
			utils::getPixVals(I0, curr_img_cv, init_pts, n_pix,
				img_height, img_width, pix_norm_mult, pix_norm_add);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
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
void SPSS::updatePixVals(const Matrix2Xd& curr_pts){
	if(params.pix_mapper){
		params.pix_mapper->updatePixVals(curr_pts);
		It = params.pix_mapper->getCurrPixVals();
	} else{
		switch(n_channels){
		case 1:
			utils::getPixVals(It, curr_img, curr_pts, n_pix, img_height, img_width,
				pix_norm_mult, pix_norm_add); break;
		case 3:
			utils::getPixVals(It, curr_img_cv, curr_pts, n_pix, img_height, img_width,
				pix_norm_mult, pix_norm_add); break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	}
}

void SPSS::initializeSimilarity(){
	if(!is_initialized.similarity){
		curr_err_vec.resize(patch_size);
		curr_err_vec_den.resize(patch_size);

		init_pix_vals_sqr.resize(patch_size);
		curr_pix_vals_sqr.resize(patch_size);
		pix_vals_prod.resize(patch_size);
	}

	init_pix_vals_sqr = I0.array() * I0.array();


	if(!is_initialized.similarity){
		curr_pix_vals_sqr = init_pix_vals_sqr;
		pix_vals_prod = init_pix_vals_sqr;
		curr_err_vec_den = 2 * init_pix_vals_sqr.array() + c;
		curr_err_vec.fill(1);

		f = static_cast<double>(patch_size);
		is_initialized.similarity = true;
	}
}

void SPSS::initializeGrad(){
	if(is_initialized.grad)
		return;

	df_dIt.resize(patch_size);
	df_dI0.resize(patch_size);
	df_dI0.fill(0);
	df_dIt.fill(0);

	is_initialized.grad = true;
}

void SPSS::updateSimilarity(bool prereq_only){
	f = 0;
	for(int pix_id = 0; pix_id < patch_size; pix_id++){
		pix_vals_prod(pix_id) = I0(pix_id)*It(pix_id);
		curr_pix_vals_sqr(pix_id) = It(pix_id)*It(pix_id);
		curr_err_vec_den(pix_id) = init_pix_vals_sqr(pix_id) + curr_pix_vals_sqr(pix_id) + c;
		f += curr_err_vec(pix_id) = (2 * pix_vals_prod(pix_id) + c) / curr_err_vec_den(pix_id);
	}
}

void SPSS::updateInitGrad(){
	for(int pix_id = 0; pix_id < patch_size; pix_id++){
		df_dI0(pix_id) =
			2 * (It(pix_id)*(curr_pix_vals_sqr(pix_id) - init_pix_vals_sqr(pix_id))
			+
			c*(It(pix_id) - 2 * I0(pix_id)))
			/
			(curr_err_vec_den(pix_id) * curr_err_vec_den(pix_id));
	}
}
void SPSS::updateCurrGrad(){
	for(int pix_id = 0; pix_id < patch_size; pix_id++){
		df_dIt(pix_id) =
			2 * (I0(pix_id) - curr_err_vec(pix_id)*It(pix_id))
			/
			curr_err_vec_den(pix_id);
	}
}

void SPSS::cmptInitHessian(MatrixXd &init_hessian, const MatrixXd &init_pix_jacobian){
	assert(init_hessian.cols() == init_hessian.rows());

	init_hessian.fill(0);
	for(int pix_id = 0; pix_id < patch_size; pix_id++){
		double init_hess = -2 * (curr_err_vec(pix_id) + I0(pix_id)*df_dI0(pix_id)) / curr_err_vec_den(pix_id);
		init_hessian += init_pix_jacobian.row(pix_id).transpose() * init_pix_jacobian.row(pix_id) * init_hess;
	}
}

void SPSS::cmptInitHessian(MatrixXd &init_hessian, const MatrixXd &init_pix_jacobian,
	const MatrixXd &init_pix_hessian){
	int ssm_state_size = init_hessian.rows();

	assert(init_hessian.cols() == ssm_state_size);
	assert(init_pix_hessian.rows() == ssm_state_size * ssm_state_size);
	init_hessian.fill(0);
	for(int pix_id = 0; pix_id < patch_size; pix_id++){
		double init_hess = -2 * (curr_err_vec(pix_id) + I0(pix_id)*df_dI0(pix_id)) / curr_err_vec_den(pix_id);
		init_hessian += init_pix_jacobian.row(pix_id).transpose() * init_pix_jacobian.row(pix_id) * init_hess
			+ Map<const MatrixXd>(init_pix_hessian.col(pix_id).data(), ssm_state_size, ssm_state_size) * df_dI0(pix_id);
	}
}

void SPSS::cmptCurrHessian(MatrixXd &curr_hessian, const MatrixXd &curr_pix_jacobian){
	assert(curr_hessian.cols() == curr_hessian.rows());
	curr_hessian.fill(0);
	for(int pix_id = 0; pix_id < patch_size; pix_id++){
		double hess = -2 * (curr_err_vec(pix_id) + 3 * df_dIt(pix_id)*It(pix_id)) /
			curr_err_vec_den(pix_id);
		curr_hessian += curr_pix_jacobian.row(pix_id).transpose() * curr_pix_jacobian.row(pix_id) * hess;
	}
}
void SPSS::cmptCurrHessian(MatrixXd &curr_hessian, const MatrixXd &curr_pix_jacobian,
	const MatrixXd &curr_pix_hessian){
	int ssm_state_size = curr_hessian.rows();
	assert(curr_hessian.cols() == ssm_state_size);
	assert(curr_pix_hessian.rows() == ssm_state_size * ssm_state_size);

	curr_hessian.fill(0);
	for(int pix_id = 0; pix_id < patch_size; pix_id++){
		double hess = -2 * (curr_err_vec(pix_id) + 3 * df_dIt(pix_id)*It(pix_id)) /
			curr_err_vec_den(pix_id);
		curr_hessian += curr_pix_jacobian.row(pix_id).transpose() * curr_pix_jacobian.row(pix_id) * hess
			+ Map<const MatrixXd>(curr_pix_hessian.col(pix_id).data(), ssm_state_size, ssm_state_size) * df_dIt(pix_id);
	}
}

void SPSS::cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian){
	assert(self_hessian.cols() == self_hessian.rows());
	self_hessian.fill(0);
	for(int pix_id = 0; pix_id < patch_size; pix_id++){
		double self_hess = -2 / (2 * curr_pix_vals_sqr(pix_id) + c);
		self_hessian += curr_pix_jacobian.row(pix_id).transpose() * curr_pix_jacobian.row(pix_id) * self_hess;
	}
}

void SPSS::cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian,
	const MatrixXd &curr_pix_hessian){
	cmptSelfHessian(self_hessian, curr_pix_jacobian);
}

double SPSS::operator()(const double* a, const double* b, size_t size, double worst_dist) const{
	double result = double();
	double diff0, diff1, diff2, diff3;
	const double* last = a + size;
	const double* lastgroup = last - 3;

	/* Process 4 items with each loop for efficiency. */
	while(a < lastgroup) {
		diff0 = ((2 * a[0] * b[0] + c) / (a[0] * a[0] + b[0] * b[0] + c));
		diff1 = ((2 * a[1] * b[1] + c) / (a[1] * a[1] + b[1] * b[1] + c));
		diff2 = ((2 * a[2] * b[2] + c) / (a[2] * a[2] + b[2] * b[2] + c));
		diff3 = ((2 * a[3] * b[3] + c) / (a[3] * a[3] + b[3] * b[3] + c));
		result -= (diff0 + diff1 + diff2 + diff3);
		a += 4;
		b += 4;
	}
	/* Process last 0-3 pixels.  Not needed for standard vector lengths. */
	while(a < last) {
		diff0 = ((2 * (*a) * (*b) + c) / ((*a) * (*a) + (*b) * (*b) + c));
		a++;
		b++;
		result -= diff0;
	}
	return result;
}

double SPSS::accum_dist(const ElementType& a, const ElementType& b, int) const
{
	return -(2 * a*b + c) / (a*a + b*b + c);
}

void SPSS::updateDistFeat(double* feat_addr){
	for(size_t pix = 0; pix < patch_size; pix++) {
		*feat_addr++ = It(pix);
	}
}

_MTF_END_NAMESPACE

