#include "ImageBase.h"
#include "IlluminationModel.h"
#include <memory>

#define AM_LIKELIHOOD_ALPHA 1
#define AM_LIKELIHOOD_BETA 0
#define AM_DIST_FROM_LIKELIHOOD false
#define AM_LEARNING_RATE 0.5

_MTF_BEGIN_NAMESPACE

struct AMParams : ImgParams{
	typedef shared_ptr<IlluminationModel> ILM;
	//! multiplicative and additive factors for the exponent in the likelihood
	double likelihood_alpha, likelihood_beta;
	//! use negative of likelihood as the distance measure
	bool dist_from_likelihood;
	//! optional factor to control the rate of online learning
	double learning_rate;
	//! optional parametric function of pixel values that can account for lighting changes
	ILM ilm;
	AMParams(int _resx, int _resy,
		double _grad_eps = GRAD_EPS,
		double _hess_eps = HESS_EPS,
		bool _uchar_input = UCHAR_INPUT,
		double _likelihood_alpha = AM_LIKELIHOOD_ALPHA,
		double _likelihood_beta = AM_LIKELIHOOD_BETA,
		bool _dist_from_likelihood = AM_DIST_FROM_LIKELIHOOD,
		double _learning_rate = AM_LEARNING_RATE,
		IlluminationModel *_ilm = nullptr);
	AMParams(const AMParams *am_params = nullptr);
};
_MTF_END_NAMESPACE

