#include "mtf/AM/RIU.h"
#include "mtf/Utilities/miscUtils.h"

#define RIU_DEBUG false

_MTF_BEGIN_NAMESPACE

//! value constructor
RIUParams::RIUParams(const AMParams *am_params,
bool _debug_mode) :
AMParams(am_params), debug_mode(_debug_mode){}

//! default/copy constructor
RIUParams::RIUParams(const RIUParams *params) :
AMParams(params),
debug_mode(RIU_DEBUG){
	if(params){
		debug_mode = params->debug_mode;
	}
}

RIUDist::RIUDist(const string &_name, const bool _dist_from_likelihood,
	const double _likelihood_alpha,
	const unsigned int _patch_size) : AMDist(_name),
	dist_from_likelihood(_dist_from_likelihood),
	likelihood_alpha(_likelihood_alpha), patch_size(_patch_size){}

RIU::RIU(const ParamType *riu_params, const int _n_channels) :
AppearanceModel(riu_params, _n_channels), params(riu_params){
	name = "riu";
	printf("\n");
	printf("Using Ratio Image Uniformity AM with...\n");
	printf("likelihood_alpha: %f\n", params.likelihood_alpha);
	printf("dist_from_likelihood: %d\n", params.dist_from_likelihood);
	printf("debug_mode: %d\n", params.debug_mode);
	printf("grad_eps: %e\n", grad_eps);
	printf("hess_eps: %e\n", hess_eps);
	N_inv = 1.0 / patch_size;
}

double RIU::getLikelihood() const{
	return exp(params.likelihood_alpha * f);
}

void RIU::initializeSimilarity(){
	if(is_initialized.similarity){ return; }
	r = VectorXd::Ones(patch_size);
	r_cntr = VectorXd::Zero(patch_size);
	r_mean = 1;
	r_var = 0;
	f = 0;
	is_initialized.similarity = true;
}

void RIU::initializeGrad(){
	if(is_initialized.grad){ return; }

	df_dIt.resize(patch_size);
	df_dI0.resize(patch_size);
	df_dr.resize(patch_size);
	dr_dI0.resize(patch_size);
	dr_dIt.resize(patch_size);
	df_dI0.fill(0);
	df_dIt.fill(0);
	dr_dI0 = 1.0 / (1 + I0.array());
	dr_dIt = dr_dI0;
	is_initialized.grad = true;
}

void RIU::updateSimilarity(bool prereq_only){
	r = (1 + It.array()) / (1 + I0.array());
	r_mean = r.mean();
	r_cntr = r.array() - r_mean;
	r_var = r_cntr.squaredNorm() / patch_size;
	r_mean_inv = 1.0 / r_mean;
	f = -r_var * r_mean_inv;
}

void RIU::updateInitGrad(){
	df_dr = (f * N_inv - 2 * r_cntr.array())*(N_inv*r_mean_inv);
	dr_dI0 = -r.array() / (1 + I0.array());
	df_dI0 = df_dr.array() * dr_dI0.array();
	//df_dI0 = -r.array() * (f / patch_size - 2 * r_cntr.array())
	//	*
	//	((N_inv*r_mean_inv)*(1 + I0.array()));
}
void RIU::updateCurrGrad(){
	df_dr = (f * N_inv - 2 * r_cntr.array())*(N_inv*r_mean_inv);
	df_dIt = df_dr.array() * dr_dIt.array();
}

void RIU::cmptInitHessian(MatrixXd &d2f_dp2, const MatrixXd &dI0_dp){
	assert(d2f_dp2.cols() == d2f_dp2.rows());
	assert(d2f_dp2.cols() == dI0_dp.cols());
	assert(dI0_dp.rows() == patch_size);

	MatrixXd dr_dp = dI0_dp.array().colwise() * dr_dI0.array();
	MatrixXd dr_dp_sum = dr_dp.colwise().sum();

	VectorXd d2r_dI02 = -2 * dr_dI0.array() / (1 + I0.array());
	MatrixXd dr_dp2 = (dI0_dp.array().colwise() * (d2r_dI02.array()*df_dr.array()).array()).colwise().sum();
	d2f_dp2 =
		-(
		(
		(dr_dp.transpose() * dr_dp * r_mean_inv)
		-
		(dr_dp.transpose() * r * dr_dp_sum) * (N_inv*r_mean_inv*r_mean_inv)
		) * 2
		-
		(
		dr_dp_sum.transpose()*
		((df_dr.array() - (f * r_mean_inv * N_inv)).matrix().transpose())
		* dr_dp * r_mean_inv * N_inv
		)
		) * N_inv
		+
		dr_dp2.transpose() * dr_dp2;

}
void RIU::cmptInitHessian(MatrixXd &d2f_dp2, const MatrixXd &dI0_dp,
	const MatrixXd &d2I0_dp2){
	int p_size = static_cast<int>(d2f_dp2.rows());
	assert(d2f_dp2.cols() == p_size);
	assert(d2I0_dp2.rows() == p_size * p_size);

	cmptInitHessian(d2f_dp2, dI0_dp);

	for(unsigned int patch_id = 0; patch_id < patch_size; ++patch_id){
		d2f_dp2 += Map<const MatrixXd>(d2I0_dp2.col(patch_id).data(), p_size, p_size) * df_dI0(patch_id);;
	}

}

void RIU::cmptCurrHessian(MatrixXd &d2f_dp2, const MatrixXd &dIt_dp){
	assert(d2f_dp2.cols() == d2f_dp2.rows());

	//mtf_clock_get(d2f_dp2_start);
	MatrixXd dr_dp = dIt_dp.array().colwise() * dr_dIt.array();
	MatrixXd dr_dp_sum = dr_dp.colwise().sum();
	d2f_dp2 =
		-(
		(
		(dr_dp.transpose() * dr_dp * r_mean_inv)
		-
		(dr_dp.transpose() * r * dr_dp_sum) * (N_inv*r_mean_inv*r_mean_inv)
		) * 2
		-
		(
		dr_dp_sum.transpose()*
		((df_dr.array() - (f * r_mean_inv * N_inv)).matrix().transpose())
		* dr_dp * r_mean_inv * N_inv
		)
		) * N_inv;
	//mtf_clock_get(d2f_dp2_end);

	/*
	mtf_clock_get(d2f_dp2_2_start);
	MatrixXd d2f_dp2_2 = -dIt_dp.transpose() *
	(
	(
	(
	(
	(
	(MatrixXd::Identity(patch_size, patch_size)*r_mean_inv).colwise()
	-
	(r * (N_inv*r_mean_inv*r_mean_inv))
	) * 2
	).rowwise()
	-
	(
	(df_dr.array() - (f * r_mean_inv * N_inv))*r_mean_inv * N_inv
	).matrix().transpose()
	) * N_inv
	).array().rowwise()
	*
	(dr_dIt.array()*dr_dIt.array()).transpose()
	).matrix()
	*dIt_dp;
	mtf_clock_get(d2f_dp2_2_end);


	double d2f_dp2_time, d2f_dp2_2_time;
	mtf_clock_measure(d2f_dp2_start, d2f_dp2_end, d2f_dp2_time);
	mtf_clock_measure(d2f_dp2_2_start, d2f_dp2_2_end, d2f_dp2_2_time);
	utils::printScalar(d2f_dp2_time, "d2f_dp2_time");
	utils::printScalar(d2f_dp2_2_time, "d2f_dp2_2_time");
	utils::printScalar(d2f_dp2_time / d2f_dp2_2_time, "d2f_dp2_speedup");

	MatrixXd d2f_dp2_diff = d2f_dp2_2 - d2f_dp2;
	utils::printMatrix(d2f_dp2, "d2f_dp2");
	utils::printMatrix(d2f_dp2_2, "d2f_dp2_2");
	utils::printMatrix(d2f_dp2_diff, "d2f_dp2_diff");
	*/

}
void RIU::cmptCurrHessian(MatrixXd &d2f_dp2, const MatrixXd &dIt_dp,
	const MatrixXd &d2It_dp2){
	int p_size = static_cast<int>(d2f_dp2.rows());
	assert(d2f_dp2.cols() == p_size);
	assert(d2It_dp2.rows() == p_size * p_size);

	cmptCurrHessian(d2f_dp2, dIt_dp);

	for(unsigned int patch_id = 0; patch_id < patch_size; ++patch_id){
		d2f_dp2 += Map<const MatrixXd>(d2It_dp2.col(patch_id).data(), p_size, p_size) * df_dIt(patch_id);
	}
	//utils::printMatrix(curr_hessian, "second order curr_hessian");
}
void RIU::cmptSelfHessian(MatrixXd &d2f_dp2, const MatrixXd &dIt_dp){
	assert(d2f_dp2.cols() == d2f_dp2.rows());
	MatrixXd dr_dp = dIt_dp.array().colwise() * dr_dIt.array();
	MatrixXd dr_dp_sum = dr_dp.colwise().sum();
	d2f_dp2 =
		-(
		(dr_dp.transpose() * dr_dp)
		-
		(dr_dp_sum.transpose() * dr_dp_sum) * N_inv
		) * 2 * N_inv;
}

void RIU::cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian,
	const MatrixXd &curr_pix_hessian){
	cmptSelfHessian(self_hessian, curr_pix_jacobian);
}

double RIUDist::operator()(const double* a, const double* b, size_t size, double worst_dist) const{
	assert(size == patch_size + 1);
	const double* _I0, *_It;
	if(a[0] == 1){
		_I0 = a + 1;
		_It = b + 1;
	} else{
		_I0 = b + 1;
		_It = a + 1;
	}
	VectorXd _r = Map<const VectorXd>(_It, size - 1).array() / Map<const VectorXd>(_I0, size - 1).array();
	double _r_mean = _r.mean();
	double dist = ((_r.array() - _r_mean).matrix().squaredNorm() / size) / (_r_mean);
	return dist_from_likelihood ?
		-exp(-likelihood_alpha * dist) : dist;
}

void RIU::updateDistFeat(double* feat_addr){
	*feat_addr++ = 0;
	for(size_t pix_id = 0; pix_id < patch_size; ++pix_id) {
		*feat_addr++ = 1 + It(pix_id);
	}
}

_MTF_END_NAMESPACE

