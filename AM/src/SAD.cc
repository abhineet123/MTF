#include "mtf/AM/SAD.h"


_MTF_BEGIN_NAMESPACE

SAD::SAD(const AMParams *am_params, const int _n_channels) :
AppearanceModel(am_params, _n_channels){
	if(am_params){
		likelihood_alpha = am_params->likelihood_alpha;
	}
	printf("\n");
	printf("Using Sum of Absolute Differences AM with...\n");
	printf("grad_eps: %e\n", grad_eps);
	printf("hess_eps: %e\n", hess_eps);
	printf("likelihood_alpha: %f\n", likelihood_alpha);

	name = "sad";

	pix_norm_mult = 1;
	pix_norm_add = 0;
	feat_size = n_pix*n_channels;
}

void SAD::initializeSimilarity(){
	if(is_initialized.similarity)
		return;
	f = 0;
	is_initialized.similarity = true;
}

double SAD::getLikelihood() const{
	return exp(-likelihood_alpha * f / static_cast<double>(patch_size));
}

void SAD::updateSimilarity(bool prereq_only){
	f = -(It - I0).lpNorm<1>();
}
double SAD::operator()(const double* a, const double* b,
	size_t size, double worst_dist) const{
	double result = 0;
	const double* last = a + size;
	const double* lastgroup = last - 3;

	/* Process 4 items with each loop for efficiency. */
	while(a < lastgroup){
		result += fabs(a[0] - b[0]) + fabs(a[1] - b[1]) + fabs(a[2] - b[2]) + fabs(a[3] - b[3]);
		a += 4;
		b += 4;

		if((worst_dist > 0) && (result > worst_dist)){
			return result;
		}
	}
	/* Process last 0-3 pixels.  Not needed for standard vector lengths. */
	while(a < last){
		result += fabs(*a++ - *b++);
	}
	return result;
}

_MTF_END_NAMESPACE

