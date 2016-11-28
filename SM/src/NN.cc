#include "mtf/SM/NN.h"
#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/excpUtils.h"
#include <fstream> 
#include "opencv2/highgui/highgui.hpp"
#include <flann/io/hdf5.h>

_MTF_BEGIN_NAMESPACE

template <class AM, class SSM>
NN<AM, SSM >::NN(const ParamType *nn_params,
	const FLANNParams *_flann_params,
	const AMParams *am_params, const SSMParams *ssm_params) :
	SearchMethod<AM, SSM>(am_params, ssm_params),
	params(nn_params),
	flann_params(_flann_params),
	flann_index(nullptr),
	gnn_index(nullptr),
	flann_dataset(nullptr),
	dataset_loaded(false){
	printf("\n");
	printf("Using Nearest Neighbor SM with:\n");
	printf("max_iters: %d\n", params.max_iters);
	printf("n_samples: %d\n", params.n_samples);
	printf("epsilon: %f\n", params.epsilon);
	printf("index_type: %d (%s)\n", flann_params.index_type,
		FLANNParams::toString(flann_params.index_type));
	printf("search_type: %d (%s)\n", flann_params.search_type,
		FLANNParams::toString(flann_params.search_type));
	printf("additive_update: %d\n", params.additive_update);
	printf("show_samples: %d\n", params.show_samples);
	printf("add_points: %d\n", params.add_points);
	printf("remove_points: %d\n", params.remove_points);
	printf("save_index: %d\n", params.save_index);
	printf("debug_mode: %d\n", params.debug_mode);

	printf("appearance model: %s\n", am.name.c_str());
	printf("state space model: %s\n", ssm.name.c_str());

	printf("\n");

	name = "nn";
	log_fname = "log/nn_debug.txt";
	time_fname = "log/nn_times.txt";
	frame_id = 0;

	ssm_state_size = ssm.getStateSize();
	am_dist_size = am.getDistFeatSize();
	printf("ssm_state_size: %d\n", ssm_state_size);
	printf("am_dist_size: %d\n", am_dist_size);

	using_pix_sigma = params.processDistributions(state_sigma, state_mean,
		distr_n_samples, n_distr, ssm_state_size);
	printf("n_distr: %d\n", n_distr);

	eig_dataset.resize(params.n_samples, am_dist_size);
	eig_result.resize(1);
	eig_dists.resize(1);

	flann_dataset.reset(new flannMatT(const_cast<double *>(eig_dataset.data()),
		eig_dataset.rows(), eig_dataset.cols()));

	ssm_perturbations.resize(params.n_samples);
	inv_state_update.resize(ssm_state_size);

	string fname_template = cv::format("%s_%s_%d_%d", am.name.c_str(), ssm.name.c_str(),
		params.n_samples, am_dist_size);
	saved_db_path = cv::format("%s/%s.db", params.saved_index_dir.c_str(), fname_template.c_str());
	saved_idx_path = cv::format("%s/%s_%s.idx", params.saved_index_dir.c_str(),
		fname_template.c_str(), FLANNParams::toString(flann_params.index_type));	
}

template <class AM, class SSM>
NN<AM, SSM >::~NN(){
	state_sigma.clear();
	state_mean.clear();
}

template <class AM, class SSM>
void NN<AM, SSM >::initialize(const cv::Mat &corners){
	start_timer();

	am.clearInitStatus();
	ssm.clearInitStatus();

	ssm.initialize(corners, am.getNChannels());

	if(using_pix_sigma){
		//! estimate SSM parameter sigma from pixel sigma
		for(int distr_id = 0; distr_id < n_distr; ++distr_id){
			state_sigma[distr_id].resize(ssm_state_size);
			state_mean[distr_id] = VectorXd::Zero(ssm_state_size);
			ssm.estimateStateSigma(state_sigma[distr_id], params.pix_sigma[distr_id]);
		}
	}
	if(params.debug_mode){
		//! print the sigma for the SSM samplers
		printf("state_sigma:\n");
		for(int distr_id = 0; distr_id < n_distr; ++distr_id){
			if(n_distr > 1){ printf("%d: ", distr_id); }
			utils::printMatrix(state_sigma[distr_id].transpose(), nullptr, "%e");
		}
	}
	//! initialize SSM sampler with the first distribution
	ssm.initializeSampler(state_sigma[0], state_mean[0]);

	am.initializePixVals(ssm.getPts());
	am.initializeDistFeat();

	//utils::printMatrix(ssm.getCorners(), "init_corners original");
	//utils::printMatrix(ssm.getCorners(), "init_corners after");
	//utils::printScalarToFile("initializing NN...", " ", log_fname, "%s", "w");

	if(params.show_samples){
		am.getCurrImg().convertTo(curr_img_uchar, CV_8UC1);
	}

	if(params.load_index){ loadDataset(); }

	if(!dataset_loaded){
		//! have to generate the dataset
		int pause_after_show = 1;
		printf("building feature dataset...\n");
		mtf_clock_get(db_start_time);
		int sample_id = 0;
		for(int distr_id = 0; distr_id < n_distr; ++distr_id){
			if(n_distr > 1){
				//! need to reset SSM sampler only if multiple samplers are in use
				//! since it was initialized with the first one
				ssm.setSampler(state_sigma[distr_id], state_mean[distr_id]);
			}
			for(int distr_sample_id = 0; distr_sample_id < distr_n_samples[distr_id]; ++distr_sample_id){
				ssm_perturbations[sample_id].resize(ssm_state_size);
				ssm.generatePerturbation(ssm_perturbations[sample_id]);
				//utils::printMatrix(ssm_perturbations[sample_id], "state_update");

				if(params.additive_update){
					inv_state_update = -ssm_perturbations[sample_id];
					ssm.additiveUpdate(inv_state_update);
				} else{
					ssm.invertState(inv_state_update, ssm_perturbations[sample_id]);
					ssm.compositionalUpdate(inv_state_update);
				}
				//utils::printMatrix(inv_state_update, "inv_state_update");

				am.updatePixVals(ssm.getPts());
				am.updateDistFeat(eig_dataset.row(sample_id).data());

				if(params.show_samples){
					cv::Point2d sample_corners[4];
					ssm.getCorners(sample_corners);
					utils::drawCorners(curr_img_uchar, sample_corners,
						cv::Scalar(0, 0, 255), to_string(sample_id + 1));
					if((sample_id + 1) % params.show_samples == 0){
						cv::imshow("Samples", curr_img_uchar);
						int key = cv::waitKey(1 - pause_after_show);
						if(key == 27){
							cv::destroyWindow("Samples");
							params.show_samples = 0;
						} else if(key == 32){
							pause_after_show = 1 - pause_after_show;
						}
						am.getCurrImg().convertTo(curr_img_uchar, CV_8UC1);
					}
				}
				// reset SSM to previous state
				if(params.additive_update){
					ssm.additiveUpdate(ssm_perturbations[sample_id]);
				} else{
					ssm.compositionalUpdate(ssm_perturbations[sample_id]);
				}
				++sample_id;
			}
		}
		double db_time;
		mtf_clock_get(db_end_time);
		mtf_clock_measure(db_start_time, db_end_time, db_time);
		printf("Time taken: %f secs\n", db_time);

		if(params.save_index){	saveDataset(); }
	}
	double idx_time;
	mtf_clock_get(idx_start_time);
	if(flann_params.index_type == IdxType::GNN){
		gnn_index.reset(new FGNN(&am, params.n_samples, am_dist_size,
			am.isSymmetrical(), &params.gnn));
		if(params.load_index){
			gnn_index->loadGraph(saved_idx_path.c_str());
		} else{
			printf("building GNN graph...\n");
			if(flann_params.fgnn_index_type != IdxType::GNN){
				printf("Using FLANN %s index to build the graph...\n",
					FLANNParams::toString(flann_params.fgnn_index_type));
				flann_index.reset(new FLANN(*flann_dataset, flann_params.getIndexParams(
					flann_params.fgnn_index_type, params.load_index, saved_idx_path),am));
				flann_index->buildIndex();
				gnn_index->buildGraph(eig_dataset.data(), flann_index.get(), flann_params.search);
			} else{
				gnn_index->buildGraph(eig_dataset.data());
			}
		}
	} else{
		printf("building FLANN index...\n");
		flann_index.reset(new FLANN(*flann_dataset, flann_params.getIndexParams(
			flann_params.index_type, params.load_index, saved_idx_path),am));
		flann_index->buildIndex();
	}
	mtf_clock_get(idx_end_time);
	mtf_clock_measure(idx_start_time, idx_end_time, idx_time);
	printf("Time taken: %f secs\n", idx_time);

	if(params.save_index){
		if(flann_params.index_type == IdxType::GNN){
			gnn_index->saveGraph(saved_idx_path.c_str());
		} else{
			printf("Saving FLANN index to: %s\n", saved_idx_path.c_str());
			flann_index->save(saved_idx_path);
		}
	}
	if(params.debug_mode){
		flann::save_to_file<double>(*flann_dataset, "log/flann_log.hdf5", "flann_dataset");
	}
	ssm.getCorners(cv_corners_mat);

	end_timer();
	write_interval(time_fname, "w");
}

template <class AM, class SSM>
void NN<AM, SSM >::update(){
	++frame_id;
	write_frame_id(frame_id);

	am.setFirstIter();
	for(int iter_id = 0; iter_id < params.max_iters; ++iter_id){
		init_timer();

		am.updatePixVals(ssm.getPts());
		record_event("am.updatePixVals");

		am.updateDistFeat();
		record_event("am.updateDistFeat");

		if(flann_params.index_type == IdxType::GNN){
			gnn_index->searchGraph(am.getDistFeat(), eig_dataset.data(),
				&best_idx, &best_dist);
		} else{
			flannMatT flann_query(const_cast<double*>(am.getDistFeat()), 1, am_dist_size);
			flannResultT flann_result(&best_idx, 1, 1);
			flannMatT flann_dists(&best_dist, 1, 1);
			record_event("flann_query/dists/result");
			switch(flann_params.search_type){
			case SearchType::KNN:
				flann_index->knnSearch(flann_query, flann_result, flann_dists, 1, flann_params.search);
				record_event("flann_index->knnSearch");
				break;
			case SearchType::Radius:
				flann_index->radiusSearch(flann_query, flann_result, flann_dists, 1, flann_params.search);
				record_event("flann_index->radiusSearch");
				break;
			default: throw invalid_argument(
				cv::format("Invalid search type specified: %d....\n", flann_params.search_type));
			}
		}

		//if(params.debug_mode){
		//	printf("The nearest neighbor is at index %d with distance %f\n", best_idx, best_dist);
		//	//flann::save_to_file(flann_result, "log/flann_log.hdf5", "flann_result");
		//	//flann::save_to_file(flann_dists, "log/flann_log.hdf5", "flann_dists");
		//	//utils::printMatrix(ssm_update, "ssm_update");
		//}

		prev_corners = ssm.getCorners();

		if(params.additive_update){
			ssm.additiveUpdate(ssm_perturbations[best_idx]);
			record_event("ssm.additiveUpdate");
		} else{
			ssm.compositionalUpdate(ssm_perturbations[best_idx]);
			record_event("ssm.compositionalUpdate");
		}

		double update_norm = (prev_corners - ssm.getCorners()).squaredNorm();
		record_event("update_norm");

		write_data(time_fname);

		if(update_norm < params.epsilon){
			if(params.debug_mode){
				printf("n_iters: %d\n", iter_id + 1);
			}
			break;
		}
		am.clearFirstIter();
	}
	if(params.remove_points){
		if(flann_params.index_type == IdxType::GNN){
			throw utils::FunctonNotImplemented("NN :: GNN does not currently support removing points");
		} else{
			flann_index->removePoint(best_idx);
		}
	}
	if(params.add_points){
		if(flann_params.index_type == IdxType::GNN){
			throw utils::FunctonNotImplemented("NN :: GNN does not currently support adding points");
		} else{
			flannMatT flann_point(const_cast<double*>(am.getDistFeat()), 1, am_dist_size);
			flann_index->addPoints(flann_point);
			ssm_perturbations.push_back(ssm_perturbations[best_idx]);
		}
	}
	ssm.getCorners(cv_corners_mat);
}

template <class AM, class SSM>
void NN<AM, SSM >::loadDataset(){
	ifstream in_file(saved_db_path, ios::in | ios::binary);
	if(in_file.good()){
		printf("Loading feature dataset from: %s\n", saved_db_path.c_str());
		mtf_clock_get(db_start_time);
		in_file.read((char*)(eig_dataset.data()), sizeof(double)*eig_dataset.size());
		for(int sample_id = 0; sample_id < params.n_samples; ++sample_id){
			ssm_perturbations[sample_id].resize(ssm_state_size);
			in_file.read((char*)(ssm_perturbations[sample_id].data()), sizeof(double)*ssm_state_size);
		}
		in_file.close();
		double db_time;
		mtf_clock_get(db_end_time);
		mtf_clock_measure(db_start_time, db_end_time, db_time);
		printf("Time taken: %f secs\n", db_time);
		dataset_loaded = true;
		if(!ifstream(saved_idx_path, ios::in | ios::binary).good()){
			// index file does not exist or is unreadable
			params.load_index = false;
		} else{
			params.save_index = false;
		}
	} else{
		printf("Failed to load feature dataset from: %s\n", saved_db_path.c_str());
		// index must be rebuilt if dataset could not loaded
		params.load_index = false;
	}
}

template <class AM, class SSM>
void NN<AM, SSM >::saveDataset(){
	ofstream out_file(saved_db_path, ios::out | ios::binary);
	if(out_file.good()){
		printf("Saving dataset to: %s\n", saved_db_path.c_str());
		out_file.write((char*)(eig_dataset.data()), sizeof(double)*eig_dataset.size());
		for(int sample_id = 0; sample_id < params.n_samples; ++sample_id){
			out_file.write((char*)(ssm_perturbations[sample_id].data()), sizeof(double)*ssm_state_size);
		}
		out_file.close();
	} else{
		printf("Failed to save dataset to: %s\n", saved_db_path.c_str());
	}
}

_MTF_END_NAMESPACE

#ifndef HEADER_ONLY_MODE
#include "mtf/Macros/register.h"
_REGISTER_TRACKERS(NN);
#endif