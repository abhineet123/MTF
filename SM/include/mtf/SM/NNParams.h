#ifndef MTF_NN_PARAMS_H
#define MTF_NN_PARAMS_H

#include "mtf/SM/GNNParams.h"
#include "mtf/Macros/common.h"

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

	//! gap between frames at which the index is updated with new samples 
	//! 0 disables the addition of samples
	int add_samples_gap;
	//! no. of samples added to the index at each update
	int n_samples_to_add;

	int remove_samples;

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
		int _add_samples_gap,
		int _n_samples_to_add,
		int _remove_samples,
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
		unsigned int &n_distr, unsigned int ssm_state_size);
};

_MTF_END_NAMESPACE

#endif

