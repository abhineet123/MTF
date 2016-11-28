#ifndef MTF_NN_PARAMS_H
#define MTF_NN_PARAMS_H

#include "mtf/SM/GNN.h"
#include "mtf/Macros/common.h"

#define NN_MAX_ITERS 1
#define NN_EPSILON 0.01
#define NN_N_SAMPLES 1000
#define NN_N_TREES 6
#define NN_ADDITIVE_UPDATE 1
#define NN_SHOW_SAMPLES 1
#define NN_ADD_POINTS 0
#define NN_REMOVE_POINTS 0
#define NN_LOAD_INDEX 0
#define NN_SAVE_INDEX 0
#define NN_INDEX_FILE_TEMPLATE "nn_saved_index"
#define NN_DEBUG_MODE false

_MTF_BEGIN_NAMESPACE

struct NNParams{

	gnn::GNNParams gnn;

	int n_samples;

	//! maximum iterations of the NN algorithm to run for each frame
	int max_iters;
	//! maximum L2 norm of the state update vector at which to stop the iterations	
	double epsilon;

	vectorvd ssm_sigma;
	vectorvd ssm_mean;
	vectord pix_sigma;

	bool additive_update;
	int show_samples;
	int add_points;
	int remove_points;

	bool save_index;
	bool load_index;
	std::string saved_index_dir;

	//! decides whether logging data will be printed for debugging purposes; 
	//! only matters if logging is enabled at compile time
	bool debug_mode;

	NNParams(
		const gnn::GNNParams *_gnn,
		int _n_samples,
		int _max_iters,
		double _epsilon,
		const vectorvd &_ssm_sigma,
		const vectorvd &_ssm_mean,
		const vectord &_pix_sigma,
		bool _additive_update,
		int _show_samples,
		int _add_points,
		int _remove_points,
		bool load_index,
		bool _save_index,
		std::string _saved_index_dir,
		bool _debug_mode);
	NNParams(const NNParams *params = nullptr);
	/**
	parse the provided mean and sigma and apply several priors
	to get the final parameters for all distributions
	*/
	bool processDistributions(vector<VectorXd> &state_sigma,
		vector<VectorXd> &state_mean, VectorXi &distr_n_samples,
		int &n_distr, int ssm_state_size);
};

_MTF_END_NAMESPACE

#endif

