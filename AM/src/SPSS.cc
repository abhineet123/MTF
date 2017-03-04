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
	if(params.pix_mapper){
		if(!is_initialized.pix_vals){
			I0.resize(patch_size);
			It.resize(patch_size);
		}
		params.pix_mapper->initializePixVals(init_pts);
		I0 = params.pix_mapper->getInitPixVals();
		if(!is_initialized.pix_vals){
			It = I0;
			is_initialized.pix_vals = true;
		}
	} else{
		ImageBase::initializePixVals(init_pts);
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
		ImageBase::updatePixVals(curr_pts);
	}
}

void SPSS::initializeSimilarity(){
	if(!is_initialized.similarity){
		f_vec.resize(patch_size);
		f_vec_den.resize(patch_size);

		I0_sqr.resize(patch_size);
		It_sqr.resize(patch_size);
	}

	I0_sqr = I0.array() * I0.array();


	if(!is_initialized.similarity){
		It_sqr = I0_sqr;
		f_vec_den = 2 * I0_sqr.array() + c;
		f_vec.fill(1);

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
	It_sqr = It.array() * It.array();
	f_vec_den = (I0_sqr + It_sqr).array() + c;
	f_vec = (2 * I0.array() * It.array() + c) / f_vec_den.array();

	if(prereq_only){ return; }

	f = f_vec.sum();

	//f = 0;
	//for(int pix_id = 0; pix_id < patch_size; pix_id++){
	//	pix_vals_prod(pix_id) = I0(pix_id)*It(pix_id);
	//	curr_pix_vals_sqr(pix_id) = It(pix_id)*It(pix_id);
	//	curr_err_vec_den(pix_id) = init_pix_vals_sqr(pix_id) + curr_pix_vals_sqr(pix_id) + c;
	//	f += curr_err_vec(pix_id) = (2 * pix_vals_prod(pix_id) + c) / curr_err_vec_den(pix_id);
	//}
}

void SPSS::updateInitGrad(){
	df_dI0 =
		2 * (It.array() * (It_sqr - I0_sqr).array()	+ c*(It - 2 * I0).array())
		/
		(f_vec_den.array() * f_vec_den.array());

	//for(int pix_id = 0; pix_id < patch_size; pix_id++){
	//	df_dI0(pix_id) =
	//		2 * (It(pix_id)*(curr_pix_vals_sqr(pix_id) - init_pix_vals_sqr(pix_id))
	//		+
	//		c*(It(pix_id) - 2 * I0(pix_id)))
	//		/
	//		(curr_err_vec_den(pix_id) * curr_err_vec_den(pix_id));
	//}
}
void SPSS::updateCurrGrad(){
	df_dIt = 2 * (I0.array() - f_vec.array()*It.array()) / f_vec_den.array();

	//for(int pix_id = 0; pix_id < patch_size; pix_id++){
	//	df_dIt(pix_id) =
	//		2 * (I0(pix_id) - curr_err_vec(pix_id)*It(pix_id))
	//		/
	//		curr_err_vec_den(pix_id);
	//}
}

void SPSS::cmptInitHessian(MatrixXd &d2f_dp2, const MatrixXd &dI0_dp){
	assert(d2f_dp2.cols() == d2f_dp2.rows());

	d2f_dp2 = (dI0_dp.array().colwise() *
		(-2 * (f_vec.array() + I0.array()*df_dI0.transpose().array()) / f_vec_den.array())
		).matrix().transpose() * dI0_dp;

	//d2f_dp2.fill(0);
	//for(int pix_id = 0; pix_id < patch_size; pix_id++){
	//	double init_hess = -2 * (f_vec(pix_id) + I0(pix_id)*df_dI0(pix_id)) / f_vec_den(pix_id);
	//	d2f_dp2 += dI0_dp.row(pix_id).transpose() * dI0_dp.row(pix_id) * init_hess;
	//}
}

void SPSS::cmptInitHessian(MatrixXd &d2f_dp2, const MatrixXd &dI0_dp,
	const MatrixXd &d2I0_dp2){
	int p_size = d2f_dp2.rows();

	assert(d2f_dp2.cols() == p_size);
	assert(d2I0_dp2.rows() == p_size * p_size);

	cmptInitHessian(d2f_dp2, dI0_dp);

	//Map<VectorXd>(d2f_dp2.data(), p_size*p_size) +=
	//	(d2I0_dp2.array().rowwise() * df_dI0.array()).matrix().rowwise().sum();

	for(int pix_id = 0; pix_id < patch_size; ++pix_id){
		d2f_dp2 +=  Map<const MatrixXd>(d2I0_dp2.col(pix_id).data(), p_size, p_size) * df_dI0(pix_id);
	}
}

void SPSS::cmptCurrHessian(MatrixXd &d2f_dp2, const MatrixXd &dIt_dp){
	assert(d2f_dp2.cols() == d2f_dp2.rows());

	d2f_dp2 = (dIt_dp.array().colwise() *
		(-2 * (f_vec.array() + 3 * df_dIt.transpose().array()*It.array()) / f_vec_den.array()) 
		).matrix().transpose() * dIt_dp;

	//curr_hessian.fill(0);
	//for(int pix_id = 0; pix_id < patch_size; pix_id++){
	//	double hess = -2 * (f_vec(pix_id) + 3 * df_dIt(pix_id)*It(pix_id)) /
	//		f_vec_den(pix_id);
	//	curr_hessian += dIt_dp.row(pix_id).transpose() * dIt_dp.row(pix_id) * hess;
	//}
}
void SPSS::cmptCurrHessian(MatrixXd &d2f_dp2, const MatrixXd &dIt_dp,
	const MatrixXd &d2It_dp2){
	int p_size = d2f_dp2.rows();
	assert(d2f_dp2.cols() == p_size);
	assert(d2It_dp2.rows() == p_size * p_size);

	cmptCurrHessian(d2f_dp2, dIt_dp);

	//Map<VectorXd>(d2f_dp2.data(), p_size*p_size) += 
	//	(d2It_dp2.array().rowwise() * df_dIt.array()).matrix().rowwise().sum();

	for(int pix_id = 0; pix_id < patch_size; pix_id++){
		d2f_dp2 += Map<const MatrixXd>(d2It_dp2.col(pix_id).data(), p_size, p_size) * df_dIt(pix_id);
	}
}

void SPSS::cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &dIt_dp){
	assert(self_hessian.cols() == self_hessian.rows());
	self_hessian = (dIt_dp.array().colwise() * 
		(-2 / (2 * It_sqr.array() + c))).matrix().transpose() * dIt_dp;

	//self_hessian.fill(0);
	//for(int pix_id = 0; pix_id < patch_size; pix_id++){
	//	double self_hess = -2 / (2 * curr_pix_vals_sqr(pix_id) + c);
	//	self_hessian += curr_pix_jacobian.row(pix_id).transpose() * curr_pix_jacobian.row(pix_id) * self_hess;
	//}
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

