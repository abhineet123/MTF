#include "mtf/AM/SSIM.h"
#include "mtf/Utilities/miscUtils.h"

_MTF_BEGIN_NAMESPACE

SSIMParams::SSIMParams(const AMParams *am_params,
double _k1, double _k2) :
AMParams(am_params),
k1(_k1),
k2(_k2){}

SSIMParams::SSIMParams(const SSIMParams *params) :
AMParams(params),
k1(SSIM_K1),
k2(SSIM_K2){
	if(params){
		k1 = params->k1;
		k2 = params->k2;		
	}
}

SSIM::SSIM(const ParamType *ssim_params, const int _n_channels) :
AppearanceModel(ssim_params, _n_channels), params(ssim_params){
	name = "ssim";
	printf("\n");
	printf("Using Structural Similarity AM with...\n");
	printf("grad_eps: %e\n", params.grad_eps);
	printf("hess_eps: %e\n", params.hess_eps);
	printf("k1: %f\n", params.k1);
	printf("k2: %f\n", params.k2);
	printf("likelihood_alpha: %f\n", params.likelihood_alpha);

	c1 = params.k1*(PIX_MAX - PIX_MIN);
	c1 *= c1;
	c2 = params.k2*(PIX_MAX - PIX_MIN);
	c2 *= c2;

	printf("c1: %f\n", c1);
	printf("c2: %f\n", c2);
}

double SSIM::getLikelihood() const{
	double d = (1.0 / f) - 1;
	return exp(-params.likelihood_alpha*d*d);
}

void SSIM::initializeSimilarity(){
	if(!is_initialized.similarity){
		init_pix_vals_cntr.resize(patch_size);
		curr_pix_vals_cntr.resize(patch_size);
	}

	init_pix_mean = I0.mean();
	init_pix_vals_cntr = (I0.array() - init_pix_mean);
	init_pix_var = init_pix_vals_cntr.squaredNorm() / (patch_size - 1);
	init_pix_mean2 = init_pix_mean*init_pix_mean;


	if(!is_initialized.similarity){
		a = c = 2 * init_pix_mean2 + c1;
		b = d = 2 * cross_pix_var + c2;
		cd = c*d;
		f = 1;

		curr_pix_mean = init_pix_mean;
		curr_pix_vals_cntr = init_pix_vals_cntr;
		curr_pix_var = init_pix_var;
		curr_pix_mean2 = init_pix_mean2;
		cross_pix_var = init_pix_var;

		is_initialized.similarity = true;
	}
}

void SSIM::initializeGrad(){
	if(!is_initialized.grad){
		df_dI0.resize(patch_size);
		df_dIt.resize(patch_size);

		init_grad_vec.resize(patch_size);
		curr_grad_vec.resize(patch_size);

		df_dI0.fill(0);
		df_dIt.fill(0);

		init_grad_vec.fill(0);
		curr_grad_vec.fill(0);

		init_grad_scal = curr_grad_scal = 0;

		is_initialized.grad = true;
	}
}

void SSIM::updateSimilarity(bool prereq_only){
	curr_pix_mean = It.mean();
	curr_pix_vals_cntr = (It.array() - curr_pix_mean);
	curr_pix_var = curr_pix_vals_cntr.squaredNorm() / (patch_size - 1);
	curr_pix_mean2 = curr_pix_mean*curr_pix_mean;

	a = 2 * curr_pix_mean*init_pix_mean + c1;
	cross_pix_var = (init_pix_vals_cntr.array() * curr_pix_vals_cntr.array()).sum() / (patch_size - 1);
	b = 2 * cross_pix_var + c2;
	c = curr_pix_mean2 + init_pix_mean2 + c1;
	d = curr_pix_var + init_pix_var + c2;
	cd = c*d;
	f = (a*b) / cd;
}

void SSIM::updateInitGrad(){
	double mult_factor = 2.0 / (patch_size*cd);
	//double fc = similarity*c, fd = similarity*d;
	//double add_factor = (curr_pix_mean*b - init_pix_mean*fd) / (patch_size - 1);
	//for(int i = 0; i < patch_size; i++){
	//	init_grad(i) = ((a*curr_pix_vals_cntr(i) - fc*init_pix_vals_cntr(i)) / patch_size + add_factor)* mult_factor;
	//}
	init_grad_scal = 2 * (curr_pix_mean*b - init_pix_mean*f*d) / (cd*(patch_size - 1));
	for(int patch_id = 0; patch_id < patch_size; patch_id++){
		init_grad_vec(patch_id) = mult_factor*(a*curr_pix_vals_cntr(patch_id) - f*c*init_pix_vals_cntr(patch_id));
		df_dI0(patch_id) = init_grad_vec(patch_id) + init_grad_scal;
	}
}
void SSIM::updateCurrGrad(){
	double mult_factor = 2.0 / (patch_size*cd);
	//double fc = similarity*c, fd = similarity*d;
	//double add_factor = (init_pix_mean*b - curr_pix_mean*fd) / (patch_size - 1);
	//for(int i = 0; i < patch_size; i++){
	//	curr_grad(i) = ((a*init_pix_vals_cntr(i) - fc*curr_pix_vals_cntr(i)) / patch_size + add_factor)* mult_factor;
	//}
	//P = mult_factor*(a*init_pix_vals_cntr - similarity*c*curr_pix_vals_cntr).transpose();

	curr_grad_scal = 2 * (init_pix_mean*b - curr_pix_mean*f*d) / (cd*(patch_size - 1));
	for(int patch_id = 0; patch_id < patch_size; patch_id++){
		curr_grad_vec(patch_id) = mult_factor*(a*init_pix_vals_cntr(patch_id) - f*c*curr_pix_vals_cntr(patch_id));
		df_dIt(patch_id) = curr_grad_vec(patch_id) + curr_grad_scal;
	}

	//curr_grad = (P.array() + q).matrix();
}

void SSIM::cmptInitHessian(MatrixXd &init_hessian, const MatrixXd &init_pix_jacobian){
	int ssm_state_size = init_hessian.rows();
	assert(init_hessian.cols() == ssm_state_size);

	double a_diff = 2.0 * curr_pix_mean / patch_size;
	double c_diff = 2.0 * init_pix_mean / patch_size;
	double mult_1 = 2.0 / (patch_size*cd);
	double mult_2 = 2.0 / ((patch_size - 1)*cd);
	RowVectorXd init_pix_jacobian_sum = init_pix_jacobian.colwise().sum();

	init_hessian = mult_1*(
		a_diff*init_pix_jacobian.transpose()*curr_pix_vals_cntr*init_pix_jacobian_sum
		-
		init_pix_jacobian.transpose()*init_pix_vals_cntr*(
		df_dI0.array()*c + f*c_diff
		).matrix()*init_pix_jacobian
		-
		f*c*(
		init_pix_jacobian.transpose()*init_pix_jacobian
		-
		(1.0 / patch_size)*init_pix_jacobian_sum.transpose()*init_pix_jacobian_sum
		)
		)
		-
		init_pix_jacobian.transpose()*(init_grad_vec.transpose())*(
		(2 / (patch_size - 1))*init_pix_vals_cntr.array() / d + (c_diff / c)
		).matrix()*init_pix_jacobian
		+
		init_pix_jacobian.transpose()*(
		mult_2*(
		curr_pix_vals_cntr*(2 * curr_pix_mean / (patch_size - 1))
		-
		(
		(df_dI0.transpose()*d + (2.0 / (patch_size - 1))*init_pix_vals_cntr*f).array()*init_pix_mean
		+
		f*d / patch_size
		).matrix()
		)
		-
		init_grad_scal*((2 / (patch_size - 1))*init_pix_vals_cntr.array() / d + (c_diff / c)).matrix()
		)*init_pix_jacobian_sum;
}

void SSIM::cmptInitHessian(MatrixXd &init_hessian, const MatrixXd &init_pix_jacobian,
	const MatrixXd &init_pix_hessian){
	int ssm_state_size = init_hessian.rows();
	assert(init_hessian.cols() == ssm_state_size);
	assert(init_pix_hessian.rows() == ssm_state_size * ssm_state_size);

	cmptInitHessian(init_hessian, init_pix_jacobian);
	for(int patch_id = 0; patch_id < patch_size; patch_id++){
		init_hessian += Map<const MatrixXd>(init_pix_hessian.col(patch_id).data(), ssm_state_size, ssm_state_size) * df_dI0(patch_id);;
	}

}

void SSIM::cmptCurrHessian(MatrixXd &curr_hessian, const MatrixXd &curr_pix_jacobian){
	int ssm_state_size = curr_hessian.rows();
	assert(curr_hessian.cols() == ssm_state_size);

	double a_diff = 2.0 * init_pix_mean / patch_size;
	double c_diff = 2.0 * curr_pix_mean / patch_size;
	double mult_1 = 2.0 / (patch_size*cd);
	double mult_2 = 2.0 / ((patch_size - 1)*cd);
	RowVectorXd curr_pix_jacobian_sum = curr_pix_jacobian.colwise().sum();

	curr_hessian = mult_1*(
		a_diff*curr_pix_jacobian.transpose()*init_pix_vals_cntr*curr_pix_jacobian_sum
		-
		curr_pix_jacobian.transpose()*curr_pix_vals_cntr*(df_dIt.array()*c + f*c_diff).matrix()*curr_pix_jacobian
		-
		f*c*(
		curr_pix_jacobian.transpose()*curr_pix_jacobian
		-
		(1.0 / patch_size)*curr_pix_jacobian_sum.transpose()*curr_pix_jacobian_sum
		)
		)
		-
		curr_pix_jacobian.transpose()*(curr_grad_vec.transpose())*(
		(2 / (patch_size - 1))*curr_pix_vals_cntr.array() / d + (c_diff / c)
		).matrix()*curr_pix_jacobian
		+
		curr_pix_jacobian.transpose()*(
		mult_2*(
		init_pix_vals_cntr*(2 * init_pix_mean / (patch_size - 1))
		-
		(
		(df_dIt.transpose()*d + (2.0 / (patch_size - 1))*curr_pix_vals_cntr*f).array()*curr_pix_mean
		+
		f*d / patch_size
		).matrix()
		)
		-
		curr_grad_scal*((2 / (patch_size - 1))*curr_pix_vals_cntr.array() / d + (c_diff / c)).matrix()
		)*curr_pix_jacobian_sum;

	//printf("ssm_state_size: %d\n", ssm_state_size);
	//utils::printMatrix(curr_hessian_1, "curr_hessian_1");
	//utils::printMatrix(curr_hessian_2, "curr_hessian_2");
	//utils::printMatrix(curr_hessian_3, "curr_hessian_3");
	//utils::printMatrix(curr_hessian, "first order curr_hessian");

	if(curr_hessian.rows() != ssm_state_size || curr_hessian.cols() != ssm_state_size){
		//utils::printMatrix(curr_hessian_1, "curr_hessian_1");
		//utils::printMatrix(curr_hessian_2, "curr_hessian_2");
		//utils::printMatrix(curr_hessian_3, "curr_hessian_3");
		utils::printMatrix(curr_hessian, "first order curr_hessian");
		throw std::domain_error("curr_hessian is not of correct size");
	}

}

void SSIM::cmptCurrHessian(MatrixXd &curr_hessian, const MatrixXd &curr_pix_jacobian,
	const MatrixXd &curr_pix_hessian){
	int ssm_state_size = curr_hessian.rows();
	assert(curr_hessian.cols() == ssm_state_size);
	assert(curr_pix_hessian.rows() == ssm_state_size * ssm_state_size);

	cmptCurrHessian(curr_hessian, curr_pix_jacobian);

	for(int patch_id = 0; patch_id < patch_size; patch_id++){
		curr_hessian += Map<const MatrixXd>(curr_pix_hessian.col(patch_id).data(), ssm_state_size, ssm_state_size) * df_dIt(patch_id);
	}
	//utils::printMatrix(curr_hessian, "second order curr_hessian");
}

void SSIM::cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian){
	assert(self_hessian.cols() == self_hessian.rows());

	c = 2 * curr_pix_mean2 + c1;
	d = 2 * curr_pix_var + c2;
	double cd_curr = c*d;
	double mult_1 = 2.0 / (patch_size*cd_curr);
	double mult_2 = 2.0 / ((patch_size - 1)*cd_curr);

	RowVectorXd curr_pix_jacobian_sum = curr_pix_jacobian.colwise().sum();
	self_hessian = c*mult_1*(
		(1.0 / patch_size)*curr_pix_jacobian_sum.transpose()*curr_pix_jacobian_sum
		-
		curr_pix_jacobian.transpose()*curr_pix_jacobian
		)
		-
		(mult_2*d / patch_size)*curr_pix_jacobian_sum.transpose()*curr_pix_jacobian_sum;
}


/*Support for FLANN library*/

int SSIM::getDistFeatSize(){ return patch_size + 3; }

void SSIM::initializeDistFeat(){
	curr_feat_vec.resize(getDistFeatSize());
}

void SSIM::updateDistFeat(double* feat_addr){
	curr_pix_mean = It.mean();
	double pix_var = 0;
	for(size_t patch_id = 0; patch_id < patch_size; patch_id++) {
		*feat_addr = It(patch_id) - curr_pix_mean;
		pix_var += *feat_addr * *feat_addr;
		feat_addr++;
	}
	pix_var /= patch_size - 1;
	*feat_addr++ = curr_pix_mean;
	*feat_addr++ = curr_pix_mean*curr_pix_mean;
	*feat_addr = pix_var;
}

void SSIM::updateDistFeat(){
	updateDistFeat(curr_feat_vec.data());
}

double SSIM::operator()(const double* a, const double* b, size_t size, double worst_dist) const
{
	double result = double();
	double cross_pix_var = 0;
	size_t _patch_size = size - 3;
	const double* last = a + _patch_size;
	const double* lastgroup = last - 3;

	/* Process 4 items with each loop for efficiency. */
	while(a < lastgroup) {

		cross_pix_var += a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
		a += 4;
		b += 4;
	}
	/* Process last 0-3 pixels.  Not needed for standard vector lengths. */
	while(a < last) {
		cross_pix_var += *a * *b;
		a++;
		b++;
	}
	cross_pix_var /= _patch_size - 1;
	result = -((2 * a[0] * b[0] + c1)*(2 * cross_pix_var + c2)) / ((a[1] + b[1] + c1)*(a[2] + b[2] + c2));
	return result;
}

_MTF_END_NAMESPACE

