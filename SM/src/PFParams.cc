#include "mtf/SM/PFParams.h"
#include "mtf/Utilities/miscUtils.h"

_MTF_BEGIN_NAMESPACE
const char* PFParams::toString(DynamicModel _dyn_model){
	switch(_dyn_model){
	case DynamicModel::RandomWalk:
		return "RandomWalk";
	case DynamicModel::AutoRegression1:
		return "AutoRegression(1)";
	default:
		throw std::invalid_argument("Invalid dynamic model provided");
	}
}
const char* PFParams::toString(UpdateType _upd_type){
	switch(_upd_type){
	case UpdateType::Additive:
		return "Additive";
	case UpdateType::Compositional:
		return "Compositional";
	default:
		throw std::invalid_argument("Invalid dynamic model provided");
	}
}
const char* PFParams::toString(ResamplingType _resampling_type){
	switch(_resampling_type){
	case ResamplingType::None:
		return "None";
	case ResamplingType::BinaryMultinomial:
		return "BinaryMultinomial";
	case ResamplingType::LinearMultinomial:
		return "LinearMultinomial";
	case ResamplingType::Residual:
		return "Residual";
	default:
		throw std::invalid_argument("Invalid resampling type provided");
	}
}
const char* PFParams::toString(LikelihoodFunc _likelihood_func){
	switch(_likelihood_func){
	case LikelihoodFunc::AM:
		return "AM";
	case LikelihoodFunc::Gaussian:
		return "Gaussian";
	case LikelihoodFunc::Reciprocal:
		return "Reciprocal";
	default:
		throw std::invalid_argument("Invalid likelihood function provided");
	}
}
const char* PFParams::toString(MeanType _likelihood_func){
	switch(_likelihood_func){
	case MeanType::None:
		return "None";
	case MeanType::SSM:
		return "SSM";
	case MeanType::Corners:
		return "Corners";
	default:
		throw std::invalid_argument("Invalid mean type provided");
	}
}

PFParams::PFParams(int _max_iters, int _n_particles, double _epsilon,
	DynamicModel _dynamic_model, UpdateType _update_type,
	LikelihoodFunc _likelihood_func,
	ResamplingType _resampling_type,
	MeanType _mean_type, bool _reset_to_mean,
	const vectorvd &_ssm_sigma, const vectorvd &_ssm_mean,
	bool _update_distr_wts, double _min_distr_wt,
	double _adaptive_resampling_thresh,
	const vectord &_pix_sigma, double _measurement_sigma,
	int _show_particles, bool _enable_learning,
	bool _jacobian_as_sigma, bool _debug_mode) :
	max_iters(_max_iters),
	n_particles(_n_particles),
	epsilon(_epsilon),
	dynamic_model(_dynamic_model),
	update_type(_update_type),
	likelihood_func(_likelihood_func),
	resampling_type(_resampling_type),
	reset_to_mean(_reset_to_mean),
	mean_type(_mean_type),
	ssm_mean(_ssm_mean),
	update_distr_wts(_update_distr_wts),
	min_distr_wt(_min_distr_wt),
	adaptive_resampling_thresh(_adaptive_resampling_thresh),
	measurement_sigma(_measurement_sigma),
	show_particles(_show_particles),
	enable_learning(_enable_learning),
	jacobian_as_sigma(_jacobian_as_sigma),
	debug_mode(_debug_mode){
	ssm_sigma = _ssm_sigma;
	pix_sigma = _pix_sigma;
}

PFParams::PFParams(const PFParams *params) :
max_iters(PF_MAX_ITERS),
n_particles(PF_N_PARTICLES),
epsilon(PF_EPSILON),
dynamic_model(static_cast<DynamicModel>(PF_DYN_MODEL)),
update_type(static_cast<UpdateType>(PF_UPD_TYPE)),
likelihood_func(static_cast<LikelihoodFunc>(PF_LIKELIHOOD_FUNC)),
resampling_type(static_cast<ResamplingType>(PF_RESAMPLING_TYPE)),
mean_type(static_cast<MeanType>(PF_MEAN_TYPE)),
reset_to_mean(PF_RESET_TO_MEAN),
update_distr_wts(PF_UPDATE_SAMPLER_WTS),
min_distr_wt(PF_MIN_DISTR_WT),
adaptive_resampling_thresh(PF_ADAPTIVE_RESAMPLING_THRESH),
pix_sigma(PF_PIX_SIGMA),
measurement_sigma(PF_MEASUREMENT_SIGMA),
show_particles(PF_SHOW_PARTICLES),
enable_learning(PF_UPDATE_TEMPLATE),
jacobian_as_sigma(PF_JACOBIAN_AS_SIGMA),
debug_mode(PF_DEBUG_MODE){
	if(params){
		max_iters = params->max_iters;
		n_particles = params->n_particles;
		epsilon = params->epsilon;
		dynamic_model = params->dynamic_model;
		update_type = params->update_type;
		likelihood_func = params->likelihood_func;
		resampling_type = params->resampling_type;
		reset_to_mean = params->reset_to_mean;
		mean_type = params->mean_type;
		ssm_sigma = params->ssm_sigma;
		ssm_mean = params->ssm_mean;
		update_distr_wts = params->update_distr_wts;
		min_distr_wt = params->min_distr_wt;
		adaptive_resampling_thresh = params->adaptive_resampling_thresh;
		pix_sigma = params->pix_sigma;
		show_particles = params->show_particles;
		enable_learning = params->enable_learning;
		measurement_sigma = params->measurement_sigma;
		jacobian_as_sigma = params->jacobian_as_sigma;
		debug_mode = params->debug_mode;
	}
}


bool PFParams::processDistributions(vector<VectorXd> &state_sigma,
	vector<VectorXd> &state_mean, VectorXi &distr_n_samples,
	int &n_distr, int ssm_state_size){
	bool using_pix_sigma;
	if(pix_sigma.empty() || pix_sigma[0] <= 0){
		if(ssm_sigma.empty()){
			throw std::invalid_argument("Sigma must be provided for at least one sampler");
		}
		using_pix_sigma = false;
		n_distr = max(ssm_sigma.size(), ssm_mean.size());
	} else{
		using_pix_sigma = true;
		n_distr = pix_sigma.size();
		utils::printMatrix(Map<const RowVectorXd>(pix_sigma.data(), pix_sigma.size()),
			"pix_sigma");
	}

	state_sigma.resize(n_distr);
	state_mean.resize(n_distr);
	distr_n_samples.resize(n_distr);

	bool default_mean = true;

	if(!using_pix_sigma){
		if(ssm_mean.empty()){
			ssm_mean.push_back(vectord(ssm_state_size, 0));
		} else{
			default_mean = false;
			printf("state_mean:\n");
		}
		int sampler_id = 0, sigma_id = 0, mean_id = 0;
		while(sampler_id < n_distr){
			if(ssm_sigma[sigma_id].size() == 1){
				state_sigma[sampler_id].resize(ssm_state_size);
				state_sigma[sampler_id].fill(ssm_sigma[sigma_id][0]);
			} else if(ssm_sigma[sigma_id].size() < ssm_state_size){
				throw std::invalid_argument(
					cv::format("PFParams :: SSM sigma for distribution %d has invalid size: %d",
					sampler_id, ssm_sigma[sigma_id].size()));
			} else{
				state_sigma[sampler_id] = Map<const VectorXd>(ssm_sigma[sigma_id].data(), ssm_state_size);
			}
			if(ssm_mean[mean_id].size() == 1){
				state_mean[sampler_id].resize(ssm_state_size);
				state_mean[sampler_id].fill(ssm_mean[mean_id][0]);
			} else if(ssm_mean[mean_id].size() < ssm_state_size){
				throw std::invalid_argument(
					cv::format("PFParams :: SSM mean for distribution %d has invalid size: %d",
					sampler_id, ssm_mean[mean_id].size()));
			} else{
				state_mean[sampler_id] = Map<const VectorXd>(ssm_mean[mean_id].data(), ssm_state_size);
			}

			//if(ssm_sigma[sigma_id].size() < ssm_state_size){
			//	throw std::invalid_argument(
			//		cv::format("PFParams :: SSM sigma for distribution %d has invalid size: %d",
			//		sampler_id, ssm_sigma[sigma_id].size()));
			//}
			//if(ssm_mean[mean_id].size() < ssm_state_size){
			//	throw std::invalid_argument(
			//		cv::format("PFParams :: SSM mean for distribution %d has invalid size: %d",
			//		sampler_id, ssm_mean[mean_id].size()));
			//}
			//state_sigma[sampler_id] = Map<const VectorXd>(ssm_sigma[sigma_id].data(), ssm_state_size);
			//state_mean[sampler_id] = Map<const VectorXd>(ssm_mean[mean_id].data(), ssm_state_size);

			if(!default_mean){
				printf("%d: ", sampler_id);
				utils::printMatrix(state_mean[sampler_id].transpose(), nullptr, "%e");
			}
			++sampler_id;
			if(sampler_id < ssm_sigma.size()){ ++sigma_id; }
			if(sampler_id < ssm_mean.size()){ ++mean_id; }
		}
	}
	//! get the no. of samples generated by each sampler
	int particles_per_distr = n_particles / n_distr;
	for(int distr_id = 0; distr_id < n_distr; ++distr_id){
		distr_n_samples[distr_id] = particles_per_distr;
	}
	int residual_particles = n_particles - n_distr*particles_per_distr;
	if(residual_particles >= n_distr){
		throw std::logic_error(
			cv::format("PFParams :: Residual particle count: %d exceeds the no. of distributions: %d",
			residual_particles, n_distr));
	}
	//! distribute the residual samples evenly among the distributions;
	for(int distr_id = 0; distr_id < residual_particles; ++distr_id){
		++distr_n_samples[distr_id];
	}
	return using_pix_sigma;
}

_MTF_END_NAMESPACE
