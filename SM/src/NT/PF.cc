#include "mtf/SM/NT/PF.h"
#include "mtf/Utilities/miscUtils.h" 
#include <boost/random/random_device.hpp>
#include <boost/random/seed_seq.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "mtf/Utilities/histUtils.h"
#ifndef DISABLE_GRAPH_UTILS
#include "mtf/Utilities/graphUtils.h"
#endif

#define reset_file(fname) fclose(fopen(fname, "w"));


_MTF_BEGIN_NAMESPACE
namespace nt{
	PF::PF(AM _am, SSM _ssm, const ParamType *pf_params) :
		SearchMethod(_am, _ssm), params(pf_params),
		max_wt_id(0), enable_adaptive_resampling(false),
		min_eff_particles(0){
		printf("\n");
		printf("Using Particle Filter (NT) SM with:\n");
		printf("max_iters: %d\n", params.max_iters);
		printf("n_particles: %d\n", params.n_particles);
		printf("epsilon: %f\n", params.epsilon);
		printf("dynamic_model: %s\n", PFParams::toString(params.dynamic_model));
		printf("update_type: %s\n", PFParams::toString(params.update_type));
		printf("likelihood_func: %s\n", PFParams::toString(params.likelihood_func));
		printf("resampling_type: %s\n", PFParams::toString(params.resampling_type));
		printf("mean_type: %s\n", PFParams::toString(params.mean_type));
		printf("reset_to_mean: %d\n", params.reset_to_mean);
		printf("update_distr_wts: %d\n", params.update_distr_wts);
		printf("min_distr_wt: %f\n", params.min_distr_wt);
		printf("adaptive_resampling_thresh: %f\n", params.adaptive_resampling_thresh);
		printf("measurement_sigma: %f\n", params.measurement_sigma);
		printf("show_particles: %d\n", params.show_particles);
		printf("enable_learning: %d\n", params.enable_learning);
		printf("jacobian_as_sigma: %d\n", params.jacobian_as_sigma);
		printf("debug_mode: %d\n", params.debug_mode);
		printf("appearance model: %s\n", am->name.c_str());
		printf("state space model: %s\n", ssm->name.c_str());
		printf("\n");

		name = "pf_nt";
		log_fname = "log/pf_debug.txt";
		time_fname = "log/pf_times.txt";
		wts_fname = "log/pf_particle_wts.txt";
		cum_wts_fname = "log/pf_particle_cum_wts.txt";
		state_fname = "log/pf_states.txt";
		corners_fname = "log/pf_corners.txt";

		frame_id = 0;

		ssm_state_size = ssm->getStateSize();
		printf("ssm_state_size: %d\n", ssm_state_size);

		using_pix_sigma = params.processDistributions(state_sigma, state_mean,
			distr_n_particles, n_distr, ssm_state_size);

		if(params.jacobian_as_sigma){
			dI_dp.resize(am->getPatchSize(), ssm_state_size);
			df_dp.resize(ssm_state_size);
			d2f_dp2.resize(ssm_state_size, ssm_state_size);
			n_distr = 1;
			using_pix_sigma = false;
		}
		printf("n_distr: %d\n", n_distr);

		if(n_distr == 1){ params.update_distr_wts = false; }

		const double pi = 3.14159265358979323846;
		measurement_factor = 1.0 / sqrt(2 * pi * params.measurement_sigma);

		for(int set_id = 0; set_id < 2; ++set_id){
			particle_states[set_id].resize(params.n_particles);
			particle_distr[set_id].resize(params.n_particles);
			particle_ar[set_id].resize(params.n_particles);
			for(int particle_id = 0; particle_id < params.n_particles; ++particle_id){
				particle_states[set_id][particle_id].resize(ssm_state_size);
				particle_ar[set_id][particle_id].resize(ssm_state_size);
			}
		}

		min_particles = (params.n_particles*params.min_distr_wt) / static_cast<double>(n_distr);
		dynamic_particles = (params.n_particles - min_particles*n_distr);

		curr_set_id = 0;
		particle_wts.resize(params.n_particles);
		particle_cum_wts.setZero(params.n_particles);
		distr_wts.resize(n_distr);
		distr_n_particles.resize(n_distr);

		perturbed_state.resize(ssm_state_size);
		perturbed_ar.resize(ssm_state_size);
		mean_state.resize(ssm_state_size);

		state_sigma.resize(n_distr);
		state_mean.resize(n_distr);


		//! initialize random number generators for resampling and measurement function
		boost::random_device r;
		boost::random::seed_seq measurement_seed{ r(), r(), r(), r(), r(), r(), r(), r() };

		measurement_gen = RandGenT(measurement_seed);
		measurement_dist = MeasureDistT(0, params.measurement_sigma);

		boost::random::seed_seq resample_seed{ r(), r(), r(), r(), r(), r(), r(), r() };
		resample_gen = RandGenT(resample_seed);
		resample_dist = ResampleDistT(0, 1);

		distr_id_gen = RandGenT(r());


		if(params.adaptive_resampling_thresh > 0 && params.adaptive_resampling_thresh <= 1){
			enable_adaptive_resampling = true;
			min_eff_particles = params.adaptive_resampling_thresh*params.n_particles;
		}

		if(params.debug_mode){
			reset_file(log_fname);
			reset_file(wts_fname);
			reset_file(cum_wts_fname);
			resample_ids.resize(params.n_particles);
			resample_ids_hist.resize(params.n_particles);
			uniform_rand_nums.resize(params.n_particles);
		}
		if(params.show_particles){
			reset_file(state_fname);
			reset_file(corners_fname);
			printf("Writing particle states to; %s\n", state_fname);
			printf("Writing particle corners to; %s\n", corners_fname);
		}
	}

	void PF::initialize(const cv::Mat &corners){
		am->clearInitStatus();
		ssm->clearInitStatus();

		ssm->initialize(corners, am->getNChannels());

		if(using_pix_sigma){
			//! estimate SSM parameter sigma from pixel sigma
			for(int distr_id = 0; distr_id < n_distr; ++distr_id){
				state_sigma[distr_id].resize(ssm_state_size);
				state_mean[distr_id] = VectorXd::Zero(ssm_state_size);
				ssm->estimateStateSigma(state_sigma[distr_id], params.pix_sigma[distr_id]);
			}
		}
		//! initialize SSM sampler with the first distribution
		ssm->initializeSampler(state_sigma[0], state_mean[0]);
		am->initializePixVals(ssm->getPts());
		am->initializeSimilarity();
		max_similarity = am->getSimilarity();

		if(params.jacobian_as_sigma){
			am->initializeGrad();
			am->initializePixGrad(ssm->getPts());
			if(params.update_type == UpdateType::Additive){
				ssm->cmptPixJacobian(dI_dp, am->getInitPixGrad());
			} else{
				ssm->cmptWarpedPixJacobian(dI_dp, am->getInitPixGrad());
			}
			am->cmptSelfHessian(d2f_dp2, dI_dp);
		}

		initializeParticles();
		initializeDistributions();

		prev_corners = ssm->getCorners();
		ssm->getCorners(cv_corners_mat);

		if(params.debug_mode){
			//! print the sigma for the SSM distributions
			printf("state_sigma:\n");
			for(int distr_id = 0; distr_id < n_distr; ++distr_id){
				if(n_distr > 1){ printf("%d: ", distr_id); }
				utils::printMatrix(state_sigma[distr_id].transpose(), nullptr, "%e");
			}
			utils::printScalar(max_similarity, "max_similarity");
		}
	}


	void PF::initializeParticles(){
		double init_wt = 1.0 / params.n_particles;
		for(int particle_id = 0; particle_id < params.n_particles; ++particle_id){
			particle_states[curr_set_id][particle_id] = ssm->getState();
			particle_wts[particle_id] = init_wt;
			if(particle_id > 0){
				particle_cum_wts[particle_id] = particle_wts[particle_id] + particle_cum_wts[particle_id - 1];
			} else{
				particle_cum_wts[particle_id] = particle_wts[particle_id];
			}
			particle_ar[curr_set_id][particle_id].setZero();
		}

	}
	void PF::initializeDistributions(){
		double init_distr_wt = 1.0 / n_distr;
		for(int distr_id = 0; distr_id < n_distr; ++distr_id){
			distr_wts[distr_id] = init_distr_wt;
			distr_n_particles[distr_id] = 0;
		}
	}

	void PF::update(){
		++frame_id;
		if(params.debug_mode){
			utils::printScalarToFile(frame_id, "\n\n-------------------\nframe_id", log_fname, "%d");
		}
		am->setFirstIter();
		int pause_after_show = 1;
		if(params.jacobian_as_sigma){
			am->updatePixVals(ssm->getPts());
			am->updateSimilarity();
			am->updateCurrGrad();
			am->updatePixGrad(ssm->getPts());
			if(params.update_type == UpdateType::Additive){
				ssm->cmptPixJacobian(dI_dp, am->getCurrPixGrad());
			} else{
				ssm->cmptWarpedPixJacobian(dI_dp, am->getCurrPixGrad());
			}
			am->cmptCurrJacobian(df_dp, dI_dp);
			state_sigma[0] = -d2f_dp2.colPivHouseholderQr().solve(df_dp.transpose());

			ssm->setSampler(state_sigma[0], state_mean[0]);
		}
		if(params.show_particles){
			utils::printScalarToFile(frame_id, "frame_id", state_fname, "%d");
			utils::printScalarToFile(frame_id, "frame_id", corners_fname, "%d");
			utils::printMatrixToFile(ssm->getState().transpose(), nullptr, state_fname);
			utils::printMatrixToFile(ssm->getCorners(), nullptr, corners_fname);
		}
		int distr_id = 0;
		for(int iter_id = 0; iter_id < params.max_iters; ++iter_id){
			if(params.show_particles){
				am->getCurrImg().convertTo(curr_img_uchar, CV_8UC1);
			}
			if(n_distr > 1){
				distr_id_dist = DistrDistT(distr_wts.begin(), distr_wts.end());
				if(params.show_particles){
					cout << "Distribution weights:\n";
					for(int i = 0; i < n_distr; ++i){
						std::cout << distr_wts[i] << ' ';
					}
					std::cout << "\n";
					cout << "Distribution probablities:\n";
					for(int i = 0; i < n_distr; ++i){
						std::cout << distr_id_dist.probabilities()[i] << ' ';
					}
					std::cout << "\n";
				}
				for(int i = 0; i < n_distr; ++i){
					distr_wts[i] = 0;
					distr_n_particles[i] = 0;
				}
			}
			double max_wt = std::numeric_limits<double>::lowest();
			for(int particle_id = 0; particle_id < params.n_particles; ++particle_id){
				if(n_distr > 1){
					int new_distr_id = distr_id_dist(distr_id_gen);
					if(new_distr_id != distr_id){
						distr_id = new_distr_id;
						//! need to reset SSM sampler only if multiple distributions are in use
						ssm->setSampler(state_sigma[distr_id], state_mean[distr_id]);
					}
				}
				particle_distr[curr_set_id][particle_id] = distr_id;
				switch(params.dynamic_model){
				case DynamicModel::AutoRegression1:
					switch(params.update_type){
					case UpdateType::Additive:
						ssm->additiveAutoRegression1(perturbed_state, perturbed_ar,
							particle_states[curr_set_id][particle_id], particle_ar[curr_set_id][particle_id]);
						break;
					case UpdateType::Compositional:
						ssm->compositionalAutoRegression1(perturbed_state, perturbed_ar,
							particle_states[curr_set_id][particle_id], particle_ar[curr_set_id][particle_id]);
						break;
					}
					particle_ar[curr_set_id][particle_id] = perturbed_ar;
					break;
				case DynamicModel::RandomWalk:
					switch(params.update_type){
					case UpdateType::Additive:
						ssm->additiveRandomWalk(perturbed_state, particle_states[curr_set_id][particle_id]);
						break;
					case UpdateType::Compositional:
						ssm->compositionalRandomWalk(perturbed_state, particle_states[curr_set_id][particle_id]);
						break;
					}
					break;
				}
				particle_states[curr_set_id][particle_id] = perturbed_state;

				//printf("PF:: calling setState for particle %d set %d with: \n", particle_id, curr_set_id);
				//utils::printMatrix(perturbed_state.transpose(), "perturbed_state");
				//utils::printMatrix(particle_states[curr_set_id][particle_id].transpose(), "ssm_state");

				ssm->setState(particle_states[curr_set_id][particle_id]);
				am->updatePixVals(ssm->getPts());
				am->updateSimilarity(false);
				/**
				a positive number that measures the dissimilarity between the
				template and the patch corresponding to this particle
				*/
				double measuremnt_val = max_similarity - am->getSimilarity();

				// convert this dissimilarity to a likelihood proportional to the dissimilarity
				switch(params.likelihood_func){
				case LikelihoodFunc::AM:
					measurement_likelihood = am->getLikelihood();
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
					ssm->getCorners(corners);
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
						am->getCurrImg().convertTo(curr_img_uchar, CV_8UC1);
					}
					utils::printMatrixToFile(ssm->getState().transpose(), nullptr, state_fname);
					utils::printMatrixToFile(ssm->getCorners(), nullptr, corners_fname);
				}
				particle_wts[particle_id] = measurement_likelihood;				
				particle_cum_wts[particle_id] = particle_id == 0 ? particle_wts[particle_id] :
					particle_wts[particle_id] + particle_cum_wts[particle_id - 1];
				if(params.update_distr_wts){
					distr_wts[distr_id] += particle_wts[particle_id];
					distr_n_particles[distr_id] += 1;
				}
				if(particle_wts[particle_id] >= max_wt){
					max_wt = particle_wts[particle_id];
					max_wt_id = particle_id;
				}
			}
			if(params.update_distr_wts){
				double wt_sum = 0;
				//! set distribution weights to average particle weights
				for(int i = 0; i < n_distr; ++i){
					if(distr_n_particles[i]>0){
						distr_wts[i] /= distr_n_particles[i];
						wt_sum += distr_wts[i];
					}
				}
				//! normalize weights to sum to 1 while maintaining the minimum threshold
				for(int i = 0; i < n_distr; ++i){
					distr_wts[i] /= wt_sum;
					if(distr_wts[i] < params.min_distr_wt){
						distr_wts[i] = params.min_distr_wt;
					}
				}
				if(params.debug_mode){
					utils::printMatrixToFile(particle_wts.transpose(), nullptr, wts_fname, "%e");
					utils::printMatrixToFile(particle_cum_wts.transpose(), nullptr, cum_wts_fname, "%e");
					utils::printMatrixToFile(distr_n_particles.transpose(), "distr_n_particles", log_fname, "%d");
#ifndef DISABLE_GRAPH_UTILS
					VectorXf particle_wts_float = particle_wts.cast<float>();
					cv::imshow("Particle Weights", utils::drawFloatGraph(particle_wts_float.data(), params.n_particles));
#endif
				}
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
				//printf("PF:: calling setState with: \n");
				//printf("max_wt_id :%d\n", max_wt_id);
				//utils::printMatrix(particle_wts.transpose(), "particle_wts");
				//utils::printMatrix(particle_states[curr_set_id][max_wt_id].transpose(), "ssm_state");

				//! set the SSM state to that of the highest weighted particle
				ssm->setState(particle_states[curr_set_id][max_wt_id]);
				break;
			case MeanType::SSM:
				ssm->estimateMeanOfSamples(mean_state, particle_states[curr_set_id], params.n_particles);
				ssm->setState(mean_state);
				break;
			case MeanType::Corners:
				updateMeanCorners();
				ssm->setCorners(mean_corners);
			}
			double update_norm = (prev_corners - ssm->getCorners()).squaredNorm();
			prev_corners = ssm->getCorners();
			if(update_norm < params.epsilon){
				if(params.debug_mode){
					printf("n_iters: %d\n", iter_id + 1);
				}
				break;
			}
			am->clearFirstIter();
		}
		if(params.reset_to_mean){
			initializeParticles();
		}
		if(params.enable_learning){
			am->updateModel(ssm->getPts());
			max_similarity = am->getSimilarity();
		}
		ssm->getCorners(cv_corners_mat);
	}

	/**
	uses binary search to find the particle with the smallest
	index whose cumulative weight is greater than the provided
	random number supposedly drawn from a uniform distribution
	between 0 and max cumulative weight
	*/
	void PF::binaryMultinomialResampling(){
		//! change the range of the uniform distribution used for resampling instead of normalizing the weights
		//resample_dist.param(ResampleDistParamT(0, particle_cum_wts[params.n_particles - 1]));

		//! normalize the cumulative weights and leave the uniform distribution range to (0, 1]
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
				//if(params.debug_mode){
				//	printf("iter_id: %d upper_id: %d lower_id: %d resample_id: %d\n", iter_id, upper_id,
				//		lower_id, resample_id);
				//}
				++iter_id;
			}

			if(params.debug_mode){
				resample_ids[particle_id] = resample_id;
				uniform_rand_nums[particle_id] = uniform_rand_num;
			}

			//! place the resampled particle states into the other set so as not to overwrite the current one
			particle_states[1 - curr_set_id][particle_id] = particle_states[curr_set_id][resample_id];
			particle_ar[1 - curr_set_id][particle_id] = particle_ar[curr_set_id][resample_id];
			if(particle_wts[resample_id] >= max_wt){
				max_wt = particle_wts[resample_id];
				max_wt_id = particle_id;
			}
		}
		if(params.debug_mode){
#ifndef DISABLE_GRAPH_UTILS

			utils::getDiracHist(resample_ids_hist, resample_ids, params.n_particles);
			cv::imshow("Resampled ID Histogram", utils::drawIntGraph(resample_ids_hist.data(), resample_ids_hist.size()));
#endif
			utils::printMatrixToFile(resample_ids.transpose(), "resample_ids", log_fname, "%d");
			utils::printMatrixToFile(uniform_rand_nums.transpose(), "uniform_rand_nums", log_fname, "%e");
		}
		//! make the other particle set the current one
		curr_set_id = 1 - curr_set_id;
	}


	void PF::linearMultinomialResampling(){
		//! change the range of the uniform distribution used for resampling instead of normalizing the weights
		//resample_dist.param(ResampleDistParamT(0, particle_cum_wts[params.n_particles - 1]));

		//! normalize the cumulative weights and leave the uniform distribution range to (0, 1]
		particle_cum_wts /= particle_cum_wts[params.n_particles - 1];
		//if(params.debug_mode){
		//	utils::printMatrix(particle_cum_wts.transpose(), "normalized particle_cum_wts");
		//}
		double max_wt = std::numeric_limits<double>::lowest();
		for(int particle_id = 0; particle_id < params.n_particles; ++particle_id){
			double uniform_rand_num = resample_dist(resample_gen);
			int resample_id = 0;
			while(particle_cum_wts[resample_id] < uniform_rand_num){ ++resample_id; }

			//if(params.debug_mode){
			//	utils::printScalar(uniform_rand_num, "uniform_rand_num");
			//	utils::printScalar(resample_id, "resample_id", "%d");
			//}

			// place the resampled particle states into the other set so as not to overwrite the current one
			particle_states[1 - curr_set_id][particle_id] = particle_states[curr_set_id][resample_id];
			particle_ar[1 - curr_set_id][particle_id] = particle_ar[curr_set_id][resample_id];
			if(particle_wts[resample_id] >= max_wt){
				max_wt = particle_wts[resample_id];
				max_wt_id = particle_id;
			}
		}
		//! make the other particle set the current one
		curr_set_id = 1 - curr_set_id;
	}

	void PF::residualResampling() {
		// normalize the weights
		particle_wts /= particle_cum_wts[params.n_particles - 1];
		// vector of particle indies
		VectorXi particle_idx = VectorXi::LinSpaced(params.n_particles, 0, params.n_particles - 1);
		//if(params.debug_mode){
		//	utils::printMatrix(particle_wts.transpose(), "normalized particle_wts");
		//	utils::printMatrix(particle_idx.transpose(), "particle_idx", "%d");
		//}
		// sort, with highest weight first
		std::sort(particle_idx.data(), particle_idx.data() + params.n_particles - 1,
			[&](int a, int b){
			return particle_wts[a] > particle_wts[b];
		});
		//if(params.debug_mode){
		//	utils::printMatrix(particle_idx.transpose(), "sorted particle_idx", "%d");
		//}

		//! now we append	
		int particles_found = 0;
		for(int particle_id = 0; particle_id < params.n_particles; ++particle_id) {
			int resample_id = particle_idx[particle_id];
			int particle_copies = round(particle_wts[resample_id] * params.n_particles);
			for(int copy_id = 0; copy_id < particle_copies; ++copy_id) {
				particle_states[1 - curr_set_id][particles_found] = particle_states[curr_set_id][resample_id];
				particle_ar[1 - curr_set_id][particles_found] = particle_ar[curr_set_id][resample_id];
				if(++particles_found == params.n_particles) { break; }
			}
			if(particles_found == params.n_particles) { break; }
		}
		int resample_id = particle_idx[0];
		for(int particle_id = particles_found; particle_id < params.n_particles; ++particle_id) {
			//! duplicate particle with highest weight to get exactly same number again
			particle_states[1 - curr_set_id][particle_id] = particle_states[curr_set_id][resample_id];
			particle_ar[1 - curr_set_id][particle_id] = particle_ar[curr_set_id][resample_id];
		}
		curr_set_id = 1 - curr_set_id;
		max_wt_id = particle_idx[0];
	}


	void PF::updateMeanCorners(){
		mean_corners.setZero();
		for(int particle_id = 0; particle_id < params.n_particles; ++particle_id) {
			// compute running average of corners corresponding to the resampled particle states
			ssm->setState(particle_states[curr_set_id][particle_id]);
			mean_corners += (ssm->getCorners() - mean_corners) / (particle_id + 1);
		}
	}

	void PF::setRegion(const cv::Mat& corners){
		ssm->setCorners(corners);
		ssm->getCorners(cv_corners_mat);
		initializeParticles();
	}
}

_MTF_END_NAMESPACE
