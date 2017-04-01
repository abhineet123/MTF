#include "mtf/AM/NGF.h"
#include "mtf/Utilities/miscUtils.h"

#define NGF_USE_SSD false
#define NGF_ETA 5.0

_MTF_BEGIN_NAMESPACE

//! value constructor
NGFParams::NGFParams(const AMParams *am_params,
double _eta, bool _use_ssd) :
AMParams(am_params),
eta(_eta),
use_ssd(_use_ssd){}
//! default/copy constructor
NGFParams::NGFParams(const NGFParams *params) :
AMParams(params),
eta(NGF_ETA),
use_ssd(NGF_USE_SSD){
	if(params){
		eta = params->eta;
		use_ssd = params->use_ssd;
	}
}

NGF::NGF(const ParamType *ngf_params, const int _n_channels) :
AppearanceModel(ngf_params, _n_channels), params(ngf_params){
	name = "ngf";
	printf("\n");
	printf("Using Normalized Gradient Fields AM with...\n");
	printf("eta: %f\n", params.eta);
	printf("use_ssd: %d\n", params.use_ssd);
	printf("likelihood_alpha: %f\n", params.likelihood_alpha);
	printf("likelihood_beta: %f\n", params.likelihood_beta);
	printf("grad_eps: %e\n", grad_eps);
	printf("hess_eps: %e\n", hess_eps);
	feat_size = patch_size * 2;
}

double NGF::getLikelihood() const{
	if(params.use_ssd){
		return exp(-sqrt(-params.likelihood_alpha * f / (static_cast<double>(patch_size))));
	}
	double d = 1 - (f / epsilon);
	return exp(-params.likelihood_alpha * d*d);
}

void NGF::initializePixVals(const Matrix2Xd& init_pts){
	if(params.use_ssd){
		_init_pts = init_pts;
		_curr_pts = _init_pts;
	}
	ImageBase::initializePixVals(init_pts);
}
void NGF::updatePixVals(const Matrix2Xd& curr_pts){
	if(params.use_ssd){
		_curr_pts = curr_pts;
	}
	ImageBase::updatePixVals(curr_pts);
}

void NGF::initializeSimilarity(){
	f = 0;
	if(params.use_ssd){
		norm_dI0_dx.resize(patch_size, Eigen::NoChange);
		norm_dIt_dx.resize(patch_size, Eigen::NoChange);
		initializePixGrad(_init_pts);
		double eps = params.eta*dI0_dx.cwiseAbs().sum() / static_cast<double>(patch_size);
		double eps_squared = eps*eps;
		for(int patch_id = 0; patch_id < patch_size; ++patch_id){
			norm_dI0_dx.row(patch_id) = dI0_dx.row(patch_id) / (dI0_dx.row(patch_id).norm() + eps_squared);
		}
	} else{

		//for(int patch_id = 0; patch_id < patch_size; ++patch_id){
		//	norm_dI0_dx.row(patch_id) = dI0_dx.row(patch_id) / (dI0_dx.row(patch_id).norm() + eps_squared);
		//	double dot_prod = norm_dI0_dx(patch_id, 0)*norm_dI0_dx(patch_id, 0) + norm_dI0_dx(patch_id, 1)*norm_dI0_dx(patch_id, 1);
		//	f += dot_prod*dot_prod;
		//}
		fac_0.resize(3 * n_pix);
		rc.resize(n_pix);
		grad_I0_x.resize(n_pix);
		grad_I0_y.resize(n_pix);
		grad_It_x.resize(n_pix);
		grad_It_y.resize(n_pix);
		grad_I0_norm.resize(n_pix);
		grad_It_norm.resize(n_pix);
		grad_I0_squared_norm.resize(n_pix);
		grad_It_squared_norm.resize(n_pix);
		const double eps_squared = 100 * 100;
		double r1, r2;
		for(int y = 0; y < resy; y++) {
			for(int x = 0; x < resx; x++) {
				unsigned int idx = x + y*resx;
				if(x == 0) {
					grad_It_x[idx] = grad_I0_x[idx] = (I0[idx + 1] - I0[idx]) / 2;
				} else if(x == resx - 1) {
					grad_It_x[idx] = grad_I0_x[idx] = (I0[idx] - I0[idx - 1]) / 2;
				} else {
					grad_It_x[idx] = grad_I0_x[idx] = (I0[idx + 1] - I0[idx - 1]) / 2;
				}
				if(y == 0) {
					grad_It_y[idx] = grad_I0_y[idx] = (I0[idx + resx] - I0[idx]) / 2;
				} else if(y == resy - 1) {
					grad_It_y[idx] = grad_I0_y[idx] = (I0[idx] - I0[idx - resx]) / 2;
				} else {
					grad_It_y[idx] = grad_I0_y[idx] = (I0[idx + resx] - I0[idx - resx]) / 2;
				}
				grad_It_squared_norm[idx] = grad_I0_squared_norm[idx] = grad_I0_x[idx] * grad_I0_x[idx] +
					grad_I0_y[idx] * grad_I0_y[idx]	+ eps_squared;
				grad_It_norm[idx] = grad_I0_norm[idx] = sqrt(grad_I0_squared_norm[idx]);
				r1 = grad_I0_x[idx] * grad_It_x[idx] + grad_I0_y[idx] * grad_It_y[idx];
				r2 = 1 / (grad_It_norm[idx] * grad_I0_norm[idx]);

				fac_0[idx] = r2 * (grad_It_x[idx] - (r1 * grad_I0_x[idx]) / (grad_I0_squared_norm[idx])) / 2;
				fac_0[idx + n_pix] = r2 * (grad_It_y[idx] - (r1 * grad_I0_y[idx]) / (grad_I0_squared_norm[idx])) / 2;

				rc[idx] = r1 * r2;
				f += rc[idx] * rc[idx];
			}
		}
		fac_t = fac_0;
	}
	epsilon = f + params.likelihood_beta;
	//utils::printScalar(f, "max_similarity");
	is_initialized.similarity = true;
}

void NGF::initializeGrad(){
	if(is_initialized.grad){ return; }
	df_dIt.resize(n_pix);
	df_dI0.resize(n_pix);
	df_dI0.fill(0);
	df_dIt.fill(0);
	dr_dI0.resize(n_pix, n_pix);
	dr_dIt.resize(n_pix, n_pix);
	unsigned int k = 0;
	std::vector<SpTr> _dr_dI;
	_dr_dI.reserve(6 * n_pix);
	for(int y = 0; y < resy; y++) {
		for(int x = 0; x < resx; x++) {
			unsigned int idx = x + y*resx;
			int drcI;
			double drcP;
			df_dI0[idx] = 0;
			if(y == 0) {
				if(x == 0) {
					drcI = idx;
					drcP = -fac_0[idx] - fac_0[idx + n_pix];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx + 1;
					drcP = -fac_0[idx + 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				} else if(x == resx - 1) {
					drcI = idx - 1;
					drcP = fac_0[idx - 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx;
					drcP = fac_0[idx] - fac_0[idx + n_pix];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				} else {
					drcI = idx - 1;
					drcP = fac_0[idx - 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx;
					drcP = -fac_0[idx + n_pix];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx + 1;
					drcP = -fac_0[idx + 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				}
				drcI = idx + resx;
				drcP = -fac_0[idx + n_pix + resx];
				df_dI0[idx] += rc[drcI] * drcP;
				_dr_dI.push_back(SpTr(drcI, idx, drcP));
				k++;
			} else if(y == resy - 1) {
				drcI = idx - resx;
				drcP = fac_0[idx + n_pix - resx];
				df_dI0[idx] += rc[drcI] * drcP;
				_dr_dI.push_back(SpTr(drcI, idx, drcP));
				k++;
				if(x == 0) {
					drcI = idx;
					drcP = -fac_0[idx] + fac_0[idx + n_pix];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx + 1;
					drcP = -fac_0[idx + 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				} else if(x == resx - 1) {
					drcI = idx - 1;
					drcP = fac_0[idx - 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx;
					drcP = fac_0[idx] + fac_0[idx + n_pix];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				} else {
					drcI = idx - 1;
					drcP = fac_0[idx - 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx;
					drcP = fac_0[idx + n_pix];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx + 1;
					drcP = -fac_0[idx + 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				}
			} else {
				drcI = idx - resx;
				drcP = fac_0[idx + n_pix - resx];
				df_dI0[idx] += rc[drcI] * drcP;
				_dr_dI.push_back(SpTr(drcI, idx, drcP));
				k++;
				if(x == 0) {
					drcI = idx;
					drcP = -fac_0[idx];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx + 1;
					drcP = -fac_0[idx + 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				} else if(x == resx - 1) {
					drcI = idx - 1;
					drcP = fac_0[idx - 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx;
					drcP = fac_0[idx];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				} else {
					drcI = idx - 1;
					drcP = fac_0[idx - 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx + 1;
					drcP = -fac_0[idx + 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				}
				drcI = idx + resx;
				drcP = -fac_0[idx + n_pix + resx];
				df_dI0[idx] += rc[drcI] * drcP;
				_dr_dI.push_back(SpTr(drcI, idx, drcP));
				k++;
			}
		}
	}
	df_dIt = df_dI0;
	dr_dIt.setFromTriplets(_dr_dI.begin(), _dr_dI.end());
	is_initialized.grad = true;
}
//! adapted from distances/NGFdotMexC.cpp in FAIR (http://www.siam.org/books/fa06/) 
void NGF::updateSimilarity(bool prereq_only){
	if(params.use_ssd){
		updatePixGrad(_curr_pts);
		double eps = params.eta*dIt_dx.cwiseAbs().sum() / static_cast<double>(patch_size);
		double eps_squared = eps*eps;
		for(int patch_id = 0; patch_id < patch_size; ++patch_id){
			norm_dIt_dx.row(patch_id) = dIt_dx.row(patch_id) / (dIt_dx.row(patch_id).norm() + eps_squared);
		}
		f = -(norm_dI0_dx - norm_dIt_dx).squaredNorm();
	} else{
		f = 0;
		//for(int patch_id = 0; patch_id < patch_size; ++patch_id){
		//	norm_dIt_dx.row(patch_id) = dIt_dx.row(patch_id) / (dIt_dx.row(patch_id).norm() + eps_squared);
		//	double dot_prod = norm_dI0_dx(patch_id, 0)*norm_dIt_dx(patch_id, 0) + norm_dI0_dx(patch_id, 1)*norm_dIt_dx(patch_id, 1);
		//	f += dot_prod*dot_prod;
		//}
		const double eps_squared = 100 * 100;
		double r1, r2;
		for(int y = 0; y < resy; y++) {
			for(int x = 0; x < resx; x++) {
				unsigned int idx = x + y*resx;
				if(x == 0) {
					grad_It_x[idx] = (It[idx + 1] - It[idx]) / 2;
				} else if(x == resx - 1) {
					grad_It_x[idx] = (It[idx] - It[idx - 1]) / 2;
				} else {
					grad_It_x[idx] = (It[idx + 1] - It[idx - 1]) / 2;
				}
				if(y == 0) {
					grad_It_y[idx] = (It[idx + resx] - It[idx]) / 2;
				} else if(y == resy - 1) {
					grad_It_y[idx] = (It[idx] - It[idx - resx]) / 2;
				} else {
					grad_It_y[idx] = (It[idx + resx] - It[idx - resx]) / 2;
				}
				grad_It_squared_norm[idx] = grad_It_x[idx] * grad_It_x[idx] + 
					grad_It_y[idx] * grad_It_y[idx] + eps_squared;
				grad_It_norm[idx] = sqrt(grad_It_squared_norm[idx]);
				r1 = grad_I0_x[idx] * grad_It_x[idx] + grad_I0_y[idx] * grad_It_y[idx];
				r2 = 1 / (grad_It_norm[idx] * grad_I0_norm[idx]);

				fac_t[idx] = r2*(grad_I0_x[idx] - (r1*grad_It_x[idx]) / (grad_It_squared_norm[idx])) / 2;
				fac_t[idx + n_pix] = r2*(grad_I0_y[idx] - (r1* grad_It_y[idx]) / (grad_It_squared_norm[idx])) / 2;

				fac_0[idx] = r2*(grad_It_x[idx] - (r1* grad_I0_x[idx]) / (grad_I0_squared_norm[idx])) / 2;
				fac_0[idx + n_pix] = r2*(grad_It_y[idx] - (r1 * grad_I0_y[idx]) / (grad_I0_squared_norm[idx])) / 2;

				rc[idx] = r1 * r2;
				f += rc[idx] * rc[idx];
			}
		}
		f /= 2;
	}
}

void NGF::updateInitGrad(){
	unsigned int k = 0;
	std::vector<SpTr> _dr_dI;
	_dr_dI.reserve(6 * n_pix);
	for(int y = 0; y < resy; y++) {
		for(int x = 0; x < resx; x++) {
			unsigned int idx = x + y*resx;
			int drcI;
			double drcP;
			df_dI0[idx] = 0;
			if(y == 0) {
				if(x == 0) {
					drcI = idx;
					drcP = -fac_0[idx] - fac_0[idx + n_pix];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx + 1;
					drcP = -fac_0[idx + 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				} else if(x == resx - 1) {
					drcI = idx - 1;
					drcP = fac_0[idx - 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx;
					drcP = fac_0[idx] - fac_0[idx + n_pix];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				} else {
					drcI = idx - 1;
					drcP = fac_0[idx - 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx;
					drcP = -fac_0[idx + n_pix];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx + 1;
					drcP = -fac_0[idx + 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				}
				drcI = idx + resx;
				drcP = -fac_0[idx + n_pix + resx];
				df_dI0[idx] += rc[drcI] * drcP;
				_dr_dI.push_back(SpTr(drcI, idx, drcP));
				k++;
			} else if(y == resy - 1) {
				drcI = idx - resx;
				drcP = fac_0[idx + n_pix - resx];
				df_dI0[idx] += rc[drcI] * drcP;
				_dr_dI.push_back(SpTr(drcI, idx, drcP));
				k++;
				if(x == 0) {
					drcI = idx;
					drcP = -fac_0[idx] + fac_0[idx + n_pix];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx + 1;
					drcP = -fac_0[idx + 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				} else if(x == resx - 1) {
					drcI = idx - 1;
					drcP = fac_0[idx - 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx;
					drcP = fac_0[idx] + fac_0[idx + n_pix];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				} else {
					drcI = idx - 1;
					drcP = fac_0[idx - 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx;
					drcP = fac_0[idx + n_pix];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx + 1;
					drcP = -fac_0[idx + 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				}
			} else {
				drcI = idx - resx;
				drcP = fac_0[idx + n_pix - resx];
				df_dI0[idx] += rc[drcI] * drcP;
				_dr_dI.push_back(SpTr(drcI, idx, drcP));
				k++;
				if(x == 0) {
					drcI = idx;
					drcP = -fac_0[idx];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx + 1;
					drcP = -fac_0[idx + 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				} else if(x == resx - 1) {
					drcI = idx - 1;
					drcP = fac_0[idx - 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx;
					drcP = fac_0[idx];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				} else {
					drcI = idx - 1;
					drcP = fac_0[idx - 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx + 1;
					drcP = -fac_0[idx + 1];
					df_dI0[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				}
				drcI = idx + resx;
				drcP = -fac_0[idx + n_pix + resx];
				df_dI0[idx] += rc[drcI] * drcP;
				_dr_dI.push_back(SpTr(drcI, idx, drcP));
				k++;
			}
		}
	}
	dr_dIt.setFromTriplets(_dr_dI.begin(), _dr_dI.end());
}
//! adapted from distances/NGFdotMexC.cpp in FAIR (http://www.siam.org/books/fa06/) 
void NGF::updateCurrGrad(){
	unsigned int k = 0;
	std::vector<SpTr> _dr_dI;
	_dr_dI.reserve(6 * n_pix);
	for(int y = 0; y < resy; y++) {
		for(int x = 0; x < resx; x++) {
			unsigned int idx = x + y*resx;
			int drcI;
			double drcP;
			df_dIt[idx] = 0;
			if(y == 0) {
				if(x == 0) {
					drcI = idx;
					drcP = -fac_t[idx] - fac_t[idx + n_pix];
					df_dIt[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx + 1;
					drcP = -fac_t[idx + 1];
					df_dIt[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				} else if(x == resx - 1) {
					drcI = idx - 1;
					drcP = fac_t[idx - 1];
					df_dIt[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx;
					drcP = fac_t[idx] - fac_t[idx + n_pix];
					df_dIt[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				} else {
					drcI = idx - 1;
					drcP = fac_t[idx - 1];
					df_dIt[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx;
					drcP = -fac_t[idx + n_pix];
					df_dIt[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx + 1;
					drcP = -fac_t[idx + 1];
					df_dIt[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				}
				drcI = idx + resx;
				drcP = -fac_t[idx + n_pix + resx];
				df_dIt[idx] += rc[drcI] * drcP;
				_dr_dI.push_back(SpTr(drcI, idx, drcP));
				k++;
			} else if(y == resy - 1) {
				drcI = idx - resx;
				drcP = fac_t[idx + n_pix - resx];
				df_dIt[idx] += rc[drcI] * drcP;
				_dr_dI.push_back(SpTr(drcI, idx, drcP));
				k++;
				if(x == 0) {
					drcI = idx;
					drcP = -fac_t[idx] + fac_t[idx + n_pix];
					df_dIt[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx + 1;
					drcP = -fac_t[idx + 1];
					df_dIt[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				} else if(x == resx - 1) {
					drcI = idx - 1;
					drcP = fac_t[idx - 1];
					df_dIt[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx;
					drcP = fac_t[idx] + fac_t[idx + n_pix];
					df_dIt[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				} else {
					drcI = idx - 1;
					drcP = fac_t[idx - 1];
					df_dIt[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx;
					drcP = fac_t[idx + n_pix];
					df_dIt[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx + 1;
					drcP = -fac_t[idx + 1];
					df_dIt[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				}
			} else {
				drcI = idx - resx;
				drcP = fac_t[idx + n_pix - resx];
				df_dIt[idx] += rc[drcI] * drcP;
				_dr_dI.push_back(SpTr(drcI, idx, drcP));
				k++;
				if(x == 0) {
					drcI = idx;
					drcP = -fac_t[idx];
					df_dIt[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx + 1;
					drcP = -fac_t[idx + 1];
					df_dIt[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				} else if(x == resx - 1) {
					drcI = idx - 1;
					drcP = fac_t[idx - 1];
					df_dIt[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx;
					drcP = fac_t[idx];
					df_dIt[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				} else {
					drcI = idx - 1;
					drcP = fac_t[idx - 1];
					df_dIt[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
					drcI = idx + 1;
					drcP = -fac_t[idx + 1];
					df_dIt[idx] += rc[drcI] * drcP;
					_dr_dI.push_back(SpTr(drcI, idx, drcP));
					k++;
				}
				drcI = idx + resx;
				drcP = -fac_t[idx + n_pix + resx];
				df_dIt[idx] += rc[drcI] * drcP;
				_dr_dI.push_back(SpTr(drcI, idx, drcP));
				k++;
			}
		}
	}
	dr_dIt.setFromTriplets(_dr_dI.begin(), _dr_dI.end());
}

void NGF::cmptSelfHessian(MatrixXd &d2f_dp2, const MatrixXd &dIt_dp){
	assert(d2f_dp2.cols() == d2f_dp2.rows());
	assert(dIt_dp.rows() == n_pix);
	d2f_dp2.noalias() = -(dr_dIt*dIt_dp).transpose() * (dr_dIt*dIt_dp);
}

void NGF::initializeDistFeat(){	
	curr_feat_vec.resize(feat_size);
	if(params.use_ssd){
		norm_dIt_dx.resize(patch_size, Eigen::NoChange);
		initializePixGrad(_init_pts);
	}
}

void NGF::updateDistFeat(double* feat_addr){
	if(params.use_ssd){
		updatePixGrad(_curr_pts);
		double eps = params.eta*dIt_dx.cwiseAbs().sum() / static_cast<double>(patch_size);
		double eps_squared = eps*eps;
		for(size_t patch_id = 0; patch_id < patch_size; ++patch_id) {
			norm_dIt_dx.row(patch_id) = dIt_dx.row(patch_id) / (dIt_dx.row(patch_id).norm() + eps_squared);
			feat_addr[patch_id * 2] = norm_dIt_dx(patch_id, 0);
			feat_addr[patch_id * 2 + 1] = norm_dIt_dx(patch_id, 1);
		}
	} else{
		const double eps_squared = 100 * 100;
		for(int y = 0; y < resy; y++) {
			for(int x = 0; x < resx; x++) {
				unsigned int idx = x + y*resx;
				if(x == 0) {
					feat_addr[idx * 2] = (It[idx + 1] - It[idx]) / 2;
				} else if(x == resx - 1) {
					feat_addr[idx * 2] = (It[idx] - It[idx - 1]) / 2;
				} else {
					feat_addr[idx * 2] = (It[idx + 1] - It[idx - 1]) / 2;
				}
				if(y == 0) {
					feat_addr[idx * 2 + 1] = (It[idx + resx] - It[idx]) / 2;
				} else if(y == resy - 1) {
					feat_addr[idx * 2 + 1] = (It[idx] - It[idx - resx]) / 2;
				} else {
					feat_addr[idx * 2 + 1] = (It[idx + resx] - It[idx - resx]) / 2;
				}
				double lengthGT = sqrt(feat_addr[idx * 2] * feat_addr[idx * 2] + feat_addr[idx * 2 + 1] * feat_addr[idx * 2 + 1]
					+ eps_squared);
				feat_addr[idx * 2] /= lengthGT;
				feat_addr[idx * 2 + 1] /= lengthGT;
			}
		}
	}
}

double NGF::operator()(const double* a, const double* b, size_t size, double worst_dist) const{
	assert(size == patch_size * 2);
	double dist = 0;
	if(params.use_ssd){
		double diff0, diff1, diff2, diff3;
		const double* last = a + size;
		const double* lastgroup = last - 3;

		/* Process 4 items with each loop for efficiency. */
		while(a < lastgroup){
			diff0 = (a[0] - b[0]);
			diff1 = (a[1] - b[1]);
			diff2 = (a[2] - b[2]);
			diff3 = (a[3] - b[3]);
			dist += diff0 * diff0 + diff1 * diff1 + diff2 * diff2 + diff3 * diff3;
			a += 4;
			b += 4;

			if((worst_dist > 0) && (dist > worst_dist)){
				return dist;
			}
		}
		/* Process last 0-3 pixels.  Not needed for standard vector lengths. */
		while(a < last){
			diff0 = (*a++ - *b++);
			dist += diff0 * diff0;
		}
	} else{
		for(size_t patch_id = 0; patch_id < patch_size; ++patch_id) {
			double dot_prod = a[2 * patch_id] * b[2 * patch_id] + a[2 * patch_id + 1] * b[2 * patch_id + 1];
			dist -= dot_prod*dot_prod;
		}
	}

	return dist;
}

_MTF_END_NAMESPACE

