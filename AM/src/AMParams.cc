#include "mtf/AM/AMParams.h"

_MTF_BEGIN_NAMESPACE

AMParams::AMParams(
int _resx, int _resy,
double _grad_eps,
double _hess_eps,
bool _use_uchar_input,
double _likelihood_alpha,
double _likelihood_beta,
bool _dist_from_likelihood,
double _forgetting_factor,
IlluminationModel *_ilm) :
ImgParams(_resx, _resy, 
_grad_eps, _hess_eps, _use_uchar_input),
likelihood_alpha(_likelihood_alpha),
likelihood_beta(_likelihood_beta),
dist_from_likelihood(_dist_from_likelihood),
learning_rate(_forgetting_factor),
ilm(_ilm){}

AMParams::AMParams(const AMParams *am_params) :
ImgParams(am_params),
likelihood_alpha(AM_LIKELIHOOD_ALPHA),
likelihood_beta(AM_LIKELIHOOD_BETA),
dist_from_likelihood(AM_DIST_FROM_LIKELIHOOD),
learning_rate(AM_LEARNING_RATE),
ilm(nullptr){
	if(am_params){
		likelihood_alpha = am_params->likelihood_alpha;
		likelihood_beta = am_params->likelihood_beta;
		dist_from_likelihood = am_params->dist_from_likelihood;
		learning_rate = am_params->learning_rate;
		ilm = am_params->ilm;
	}
}
_MTF_END_NAMESPACE

