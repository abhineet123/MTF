#include "mtf/SM/PF.h"
#include "mtf/Utilities/miscUtils.h" 
#include <boost/random/random_device.hpp>
#include <boost/random/seed_seq.hpp>
#include "opencv2/highgui/highgui.hpp"

//! OpenMP scheduler
#ifndef PF_OMP_SCHD
#define PF_OMP_SCHD auto
#endif

_MTF_BEGIN_NAMESPACE

template <class AM, class SSM>
PF<AM, SSM >::PF(const ParamType *pf_params,
	const AMParams *am_params, const SSMParams *ssm_params) :
	SearchMethod<AM, SSM>(am_params, ssm_params),
	params(pf_params), max_wt_id(0),
	enable_adaptive_resampling(false), min_eff_particles(0){
	printf("\n");
	printf("Using Particle Filter SM with:\n");
	printf("max_iters: %d\n", params.max_iters);
	printf("n_particles: %d\n", params.n_particles);
	printf("epsilon: %f\n", params.epsilon);
	printf("dynamic_model: %s\n", PFParams::toString(params.dynamic_model));
	printf("update_type: %s\n", PFParams::toString(params.update_type));
	printf("likelihood_func: %s\n", PFParams::toString(params.likelihood_func));
	printf("resampling_type: %s\n", PFParams::toString(params.resampling_type));
	printf("adaptive_resampling_thresh: %f\n", params.adaptive_resampling_thresh);
	printf("mean_type: %s\n", PFParams::toString(params.mean_type));
	printf("reset_to_mean: %d\n", params.reset_to_mean);
	ssm_state_size = ssm.getStateSize();
	if(params.pix_sigma.empty() || params.pix_sigma[0] <= 0){
		if(params.ssm_sigma.empty()){
			throw utils::InvalidArgument("Sigma must be provided for at least one sampler");
		}
		using_pix_sigma = false;
	} else{
		using_pix_sigma = true;
		printf("pix_sigma: %f\n", params.pix_sigma[0]);
	}
	printf("measurement_sigma: %f\n", params.measurement_sigma);
	printf("show_particles: %d\n", params.show_particles);
	printf("enable_learning: %d\n", params.enable_learning);
	printf("debug_mode: %d\n", params.debug_mode);
	printf("appearance model: %s\n", am.name.c_str());
	printf("state space model: %s\n", ssm.name.c_str());
	printf("ssm_state_size: %d\n", ssm_state_size);
	printf("\n");

	name = "pf";
	log_fname = "log/pf_debug.txt";
	time_fname = "log/pf_times.txt";
	frame_id = 0;

	const double pi = 3.14159265358979323846;
	measurement_factor = 1.0 / sqrt(2 * pi * params.measurement_sigma);

	for(int set_id = 0; set_id < 2; ++set_id){
		particle_states[set_id].resize(params.n_particles);
		particle_ar[set_id].resize(params.n_particles);
		for(int particle_id = 0; particle_id < params.n_particles; ++particle_id){
			particle_states[set_id][particle_id].resize(ssm_state_size);
			particle_ar[set_id][particle_id].resize(ssm_state_size);
		}
	}
	curr_set_id = 0;
	particle_wts.resize(params.n_particles);
	particle_cum_wts.setZero(params.n_particles);

	perturbed_state.resize(ssm_state_size);
	perturbed_ar.resize(ssm_state_size);
	mean_state.resize(ssm_state_size);
	state_sigma.resize(ssm_state_size);
	state_mean.resize(ssm_state_size);

	if(!using_pix_sigma){
		if(params.ssm_sigma[0].size() < ssm_state_size){
			throw utils::InvalidArgument(
				cv::format("SSM sigma has invalid size: %d",
				params.ssm_sigma[0].size()));
		}
		if(params.ssm_mean.empty()){
			state_mean = VectorXd::Zero(ssm_state_size);
		} else{
			if(params.ssm_mean[0].size() < ssm_state_size){
				throw utils::InvalidArgument(
					cv::format("SSM ssm_mean has invalid size: %d",
					params.ssm_mean[0].size()));
			}
			state_mean = Map<const VectorXd>(params.ssm_mean[0].data(), ssm_state_size);
		}
		state_sigma = Map<const VectorXd>(params.ssm_sigma[0].data(), ssm_state_size);
		utils::printMatrix(state_sigma.transpose(), "state_sigma");
	}
	// initialize random number generators for resampling and measurement function
	boost::random_device r;
	boost::random::seed_seq measurement_seed{ r(), r(), r(), r(), r(), r(), r(), r() };

	measurement_gen = RandGenT(measurement_seed);
	measurement_dist = MeasureDistT(0, params.measurement_sigma);

	boost::random::seed_seq resample_seed{ r(), r(), r(), r(), r(), r(), r(), r() };
	resample_gen = RandGenT(resample_seed);
	resample_dist = ResampleDistT(0, 1);

	if(params.adaptive_resampling_thresh > 0 && params.adaptive_resampling_thresh <= 1){
		printf("Using adaptive resampling\n");
		enable_adaptive_resampling = true;
		min_eff_particles = params.adaptive_resampling_thresh*params.n_particles;
	}

	if(params.debug_mode){
		fclose(fopen(log_fname, "w"));
		resample_ids.resize(params.n_particles);
		uniform_rand_nums.resize(params.n_particles);
	}

#ifdef ENABLE_PARALLEL
	for(int particle_id = 0; particle_id < params.n_particles; particle_id++){
		am_vec.push_back(AMPTr(new AM(am_params)));
		ssm_vec.push_back(SSMPTr(new SSM(ssm_params)));
	}
#endif
#if defined ENABLE_OMP
printf(" ******* Parallelization is enabled using OpenMP ******* \n");
#endif
}

template <class AM, class SSM>
void PF<AM, SSM >::initialize(const cv::Mat &corners){

	ssm.initialize(corners, am.getNChannels());

	if(using_pix_sigma){
		//! estimate SSM parameter sigma from pixel sigma
		ssm.estimateStateSigma(state_sigma, params.pix_sigma[0]);
		state_mean = VectorXd::Zero(ssm_state_size);
	}
	//! initialize SSM sampler with the first distribution
	ssm.initializeSampler(state_sigma, state_mean);

	state_sigma = ssm.getSamplerSigma();
	state_mean = ssm.getSamplerMean();

	am.initializePixVals(ssm.getPts());
	am.initializeSimilarity();
	max_similarity = am.getSimilarity();

	if(params.debug_mode){
		//! print the sigma and mean for the SSM distributions
		utils::printMatrix(state_sigma.transpose(), "state_sigma");
		utils::printMatrix(state_mean.transpose(), "state_mean");
		utils::printScalar(max_similarity, "max_similarity");
	}
	initializeParticles();

	prev_corners = ssm.getCorners();
	ssm.getCorners(cv_corners_mat);
#ifdef ENABLE_PARALLEL
	for(int particle_id = 0; particle_id < params.n_particles; particle_id++){
		ssm_vec[particle_id]->initialize(corners, am.getNChannels());
		ssm_vec[particle_id]->initializeSampler(state_sigma, VectorXd::Zero(ssm_state_size));
		am_vec[particle_id]->initializePixVals(ssm.getPts());
		am_vec[particle_id]->initializeSimilarity();
	}
#endif
}

template <class AM, class SSM>
void PF<AM, SSM >::initializeParticles(){
	double init_wt = 1.0 / params.n_particles;
	for(int particle_id = 0; particle_id < params.n_particles; particle_id++){
		particle_states[curr_set_id][particle_id] = ssm.getState();
		particle_wts[particle_id] = init_wt;
		if(particle_id > 0){
			particle_cum_wts[particle_id] = particle_wts[particle_id] + particle_cum_wts[particle_id - 1];
		} else{
			particle_cum_wts[particle_id] = particle_wts[particle_id];
		}
		particle_ar[curr_set_id][particle_id].setZero();
	}
}

template <class AM, class SSM>
void PF<AM, SSM >::update(){
	++frame_id;
	am.setFirstIter();
	int pause_after_show = 1;
	for(int i = 0; i < params.max_iters; i++){
		if(params.show_particles){
			am.getCurrImg().convertTo(curr_img_uchar, CV_8UC1);
		}
		double max_wt = std::numeric_limits<double>::lowest();
#ifdef ENABLE_OMP
#pragma omp parallel for schedule(PF_OMP_SCHD)
#endif	
		for(int particle_id = 0; particle_id < params.n_particles; ++particle_id){
#ifdef ENABLE_PARALLEL
			SSM &ssm = *ssm_vec[particle_id];
			AM &am = *am_vec[particle_id];
#endif
			switch(params.dynamic_model){
			case DynamicModel::AutoRegression1:
				switch(params.update_type){
				case UpdateType::Additive:
					ssm.additiveAutoRegression1(perturbed_state, perturbed_ar,
						particle_states[curr_set_id][particle_id], particle_ar[curr_set_id][particle_id]);
					break;
				case UpdateType::Compositional:
					ssm.compositionalAutoRegression1(perturbed_state, perturbed_ar,
						particle_states[curr_set_id][particle_id], particle_ar[curr_set_id][particle_id]);
					break;
				}
				particle_ar[curr_set_id][particle_id] = perturbed_ar;
				break;
			case DynamicModel::RandomWalk:
				switch(params.update_type){
				case UpdateType::Additive:
					ssm.additiveRandomWalk(perturbed_state, particle_states[curr_set_id][particle_id]);
					break;
				case UpdateType::Compositional:
					ssm.compositionalRandomWalk(perturbed_state, particle_states[curr_set_id][particle_id]);
					break;
				}
				break;
			}
			particle_states[curr_set_id][particle_id] = perturbed_state;

			ssm.setState(particle_states[curr_set_id][particle_id]);
			am.updatePixVals(ssm.getPts());
			am.updateSimilarity(false);

			// a positive number that measures the dissimilarity between the
			// template and the patch corresponding to this particle
			double measuremnt_val = max_similarity - am.getSimilarity();

			// convert this dissimilarity to a likelihood proportional to the dissimilarity
			switch(params.likelihood_func){
			case LikelihoodFunc::AM:
				measurement_likelihood = am.getLikelihood();
				break;
			case LikelihoodFunc::Gaussian:
				measurement_likelihood = measurement_factor * exp(-0.5*measuremnt_val / params.measurement_sigma);
				break;
			case LikelihoodFunc::Reciprocal:
				measurement_likelihood = 1.0 / (1.0 + measuremnt_val);
				break;
			}
			if(params.show_particles){
				cv::Point2d corners[4];
				ssm.getCorners(corners);
				utils::drawCorners(curr_img_uchar, corners,
					cv::Scalar(0, 0, 255), cv::format("%d: %5.3e", particle_id + 1, measurement_likelihood));
				//printf("measurement_likelihood: %e\n", measurement_likelihood);
				if((particle_id + 1) % params.show_particles == 0){
					cv::imshow("Particles", curr_img_uchar);
					int key = cv::waitKey(1 - pause_after_show);
					if(key == 27){
						cv::destroyWindow("Particles");
						params.show_particles = 0;
					} else if(key == 32){
						pause_after_show = 1 - pause_after_show;
					}
					am.getCurrImg().convertTo(curr_img_uchar, CV_8UC1);
				}
			}
			particle_wts[particle_id] = measurement_likelihood;
			if(particle_id > 0){
				particle_cum_wts[particle_id] = particle_wts[particle_id] + particle_cum_wts[particle_id - 1];
			} else{
				particle_cum_wts[particle_id] = particle_wts[particle_id];
			}
			if(particle_wts[particle_id] >= max_wt){
				max_wt = particle_wts[particle_id];
				max_wt_id = particle_id;
			}
		}
		if(params.debug_mode){
			utils::printMatrixToFile(particle_wts.transpose(), "particle_wts", log_fname, "%e");
			utils::printMatrixToFile(particle_cum_wts.transpose(), "particle_cum_wts", log_fname, "%e");
		}
		bool perform_resampling = true;
		if(enable_adaptive_resampling){
			double n_eff_particles = (particle_wts / particle_wts.sum()).squaredNorm();
			n_eff_particles = n_eff_particles == 0 ? 0 :
				1.0 / n_eff_particles;
			if(n_eff_particles > min_eff_particles){
				perform_resampling = false;
			}
		}
		if(perform_resampling){
			switch(params.resampling_type){
			case ResamplingType::None:
				break;
			case ResamplingType::BinaryMultinomial:
				binaryMultinomialResampling();
				break;
			case ResamplingType::LinearMultinomial:
				linearMultinomialResampling();
				break;
			case ResamplingType::Residual:
				residualResampling();
				break;
			}
		}
		switch(params.mean_type){
		case MeanType::None:
			//! set the SSM state to that of the highest weighted particle
			ssm.setState(particle_states[curr_set_id][max_wt_id]);
			break;
		case MeanType::SSM:
			ssm.estimateMeanOfSamples(mean_state, particle_states[curr_set_id], params.n_particles);
			ssm.setState(mean_state);
			break;
		case MeanType::Corners:
			updateMeanCorners();
			ssm.setCorners(mean_corners);
		}

		double update_norm = (prev_corners - ssm.getCorners()).squaredNorm();
		prev_corners = ssm.getCorners();
		if(update_norm < params.epsilon){
			if(params.debug_mode){
				printf("n_iters: %d\n", i + 1);
			}
			break;
		}
		am.clearFirstIter();
	}
	if(params.reset_to_mean){
		initializeParticles();
	}
	if(params.enable_learning){
		am.updateModel(ssm.getPts());
	}
	ssm.getCorners(cv_corners_mat);
}

// uses binary search to find the particle with the smallest
// index whose cumulative weight is greater than the provided
// random number supposedly drawn from a uniform distribution 
// between 0 and max cumulative weight
template <class AM, class SSM>
void PF<AM, SSM >::binaryMultinomialResampling(){
	// change the range of the uniform distribution used for resampling instead of normalizing the weights
	//resample_dist.param(ResampleDistParamT(0, particle_cum_wts[params.n_particles - 1]));

	// normalize the cumulative weights and leave the uniform distribution range to (0, 1]
	particle_cum_wts /= particle_cum_wts[params.n_particles - 1];
	if(params.debug_mode){
		utils::printMatrixToFile(particle_cum_wts.transpose(), "normalized particle_cum_wts", log_fname, "%e");
	}
	double max_wt = std::numeric_limits<double>::lowest();
	for(int particle_id = 0; particle_id < params.n_particles; ++particle_id){
		double uniform_rand_num = resample_dist(resample_gen);
		int lower_id = 0, upper_id = params.n_particles - 1;
		int resample_id = (lower_id + upper_id) / 2;
		int iter_id = 0;
		while(upper_id > lower_id){
			if(particle_cum_wts[resample_id] >= uniform_rand_num){
				upper_id = resample_id;
			} else{
				lower_id = resample_id + 1;
			}
			resample_id = (lower_id + upper_id) / 2;
			if(params.debug_mode){
				printf("iter_id: %d upper_id: %d lower_id: %d resample_id: %d\n", iter_id, upper_id,
					lower_id, resample_id);
			}
			++iter_id;
		}

		if(params.debug_mode){
			printf("particle_id: %d resample_id: %d\n", particle_id, resample_id);
			resample_ids[particle_id] = resample_id;
			uniform_rand_nums[particle_id] = uniform_rand_num;
		}

		// place the resampled particle states into the other set so as not to overwrite the current one
		particle_states[1 - curr_set_id][particle_id] = particle_states[curr_set_id][resample_id];
		particle_ar[1 - curr_set_id][particle_id] = particle_ar[curr_set_id][resample_id];
		if(particle_wts[resample_id] >= max_wt){
			max_wt = particle_wts[resample_id];
			max_wt_id = particle_id;
		}
	}
	if(params.debug_mode){
		utils::printMatrixToFile(resample_ids.transpose(), "resample_ids", log_fname, "%d");
		utils::printMatrixToFile(uniform_rand_nums.transpose(), "uniform_rand_nums", log_fname, "%e");
	}
	// make the other particle set the current one
	curr_set_id = 1 - curr_set_id;
}

template <class AM, class SSM>
void PF<AM, SSM >::linearMultinomialResampling(){
	// change the range of the uniform distribution used for resampling instead of normalizing the weights
	//resample_dist.param(ResampleDistParamT(0, particle_cum_wts[params.n_particles - 1]));

	// normalize the cumulative weights and leave the uniform distribution range to (0, 1]
	particle_cum_wts /= particle_cum_wts[params.n_particles - 1];
	if(params.debug_mode){
		utils::printMatrix(particle_cum_wts.transpose(), "normalized particle_cum_wts");
	}
	double max_wt = std::numeric_limits<double>::lowest();
	for(int particle_id = 0; particle_id < params.n_particles; ++particle_id){
		double uniform_rand_num = resample_dist(resample_gen);
		int resample_id = 0;
		while(particle_cum_wts[resample_id] < uniform_rand_num){ ++resample_id; }

		if(params.debug_mode){
			utils::printScalar(uniform_rand_num, "uniform_rand_num");
			utils::printScalar(resample_id, "resample_id", "%d");
		}

		// place the resampled particle states into the other set so as not to overwrite the current one
		particle_states[1 - curr_set_id][particle_id] = particle_states[curr_set_id][resample_id];
		particle_ar[1 - curr_set_id][particle_id] = particle_ar[curr_set_id][resample_id];
		if(particle_wts[resample_id] >= max_wt){
			max_wt = particle_wts[resample_id];
			max_wt_id = particle_id;
		}
	}
	// make the other particle set the current one
	curr_set_id = 1 - curr_set_id;
}



template <class AM, class SSM>
void PF<AM, SSM >::residualResampling() {
	// normalize the weights
	particle_wts /= particle_cum_wts[params.n_particles - 1];
	// vector of particle indies
	VectorXi particle_idx = VectorXi::LinSpaced(params.n_particles, 0, params.n_particles - 1);
	if(params.debug_mode){
		utils::printMatrix(particle_wts.transpose(), "normalized particle_wts");
		utils::printMatrix(particle_idx.transpose(), "particle_idx", "%d");
	}
	// sort, with highest weight first
	std::sort(particle_idx.data(), particle_idx.data() + params.n_particles - 1,
		[&](int a, int b){
		return particle_wts[a] > particle_wts[b];
	});
	if(params.debug_mode){
		utils::printMatrix(particle_idx.transpose(), "sorted particle_idx", "%d");
	}

	// now we append	
	int particles_found = 0;
	for(int particle_id = 0; particle_id < params.n_particles; ++particle_id) {
		int resample_id = particle_idx[particle_id];
		int particle_copies = static_cast<int>(round(particle_wts[resample_id] * params.n_particles));
		for(int copy_id = 0; copy_id < particle_copies; ++copy_id) {
			particle_states[1 - curr_set_id][particles_found] = particle_states[curr_set_id][resample_id];
			particle_ar[1 - curr_set_id][particles_found] = particle_ar[curr_set_id][resample_id];
			if(++particles_found == params.n_particles) { break; }
		}
		if(particles_found == params.n_particles) { break; }
	}
	int resample_id = particle_idx[0];
	for(int particle_id = particles_found; particle_id < params.n_particles; ++particle_id) {
		// duplicate particle with highest weight to get exactly same number again
		particle_states[1 - curr_set_id][particle_id] = particle_states[curr_set_id][resample_id];
		particle_ar[1 - curr_set_id][particle_id] = particle_ar[curr_set_id][resample_id];
	}
	curr_set_id = 1 - curr_set_id;
	max_wt_id = particle_idx[0];
}

template <class AM, class SSM>
void PF<AM, SSM >::updateMeanCorners(){
	mean_corners.setZero();
	for(int particle_id = 0; particle_id < params.n_particles; ++particle_id) {
		// compute running average of corners corresponding to the resampled particle states
		ssm.setState(particle_states[curr_set_id][particle_id]);
		mean_corners += (ssm.getCorners() - mean_corners) / (particle_id + 1);
	}
}
template <class AM, class SSM>
void PF<AM, SSM >::setRegion(const cv::Mat& corners){
	ssm.setCorners(corners);
	ssm.getCorners(cv_corners_mat);
	initializeParticles();
}

_MTF_END_NAMESPACE

#ifndef HEADER_ONLY_MODE
#include "mtf/Macros/register.h"
_REGISTER_TRACKERS(PF);
#endif