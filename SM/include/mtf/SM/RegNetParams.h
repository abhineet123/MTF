#ifndef MTF_RG_PARAMS_H
#define MTF_RG_PARAMS_H

#include <vector>
#include <string>

#define RG_MAX_ITERS 10
#define RG_EPSILON 0.01
#define RG_N_SAMPLES 1000
#define RG_N_TREES 6
#define RG_CONFIG_FILE "Config/nn.cfg"
#define RG_ADDITIVE_UPDATE 1
#define RG_SHOW_SAMPLES 1
#define RG_ADD_POINTS 0
#define RG_REMOVE_POINTS 0
#define RG_LOAD_INDEX 0
#define RG_SAVE_INDEX 0
#define RG_INDEX_FILE_TEMPLATE "nn_saved_index"
#define RG_DEBUG_MODE false
#define NEPOCHS 10
#define BS 128
#define PREPROC 1
#define SOLVERF ""
#define TRAINF ""
#define MEANF ""
#define DEBUG 0
#define PRETRAINED 0

typedef std::vector<double> vectord;
typedef std::vector<vectord> vectorvd;

namespace mtf{
	struct RegNetParams{
		int max_iters; //! maximum iterations of the NN algorithm to run for each frame
		int n_samples;
		double epsilon; //! maximum L2 norm of the state update vector at which to stop the iterations	

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

        int nepochs;
        int bs;
        bool enable_preproc;
        char *solver_file, *train_file, *mean_file;
        bool debug;
        bool load_pretrained;

		RegNetParams(int _max_iters, int _n_samples, double _epsilon,
			const vectorvd &_ssm_sigma, vectorvd _ssm_mean,
			vectord _pix_sigma, bool _additive_update, int _show_samples,
			int _add_points, int _remove_points,bool load_index, 
			bool _save_index, std::string _saved_index_dir,
			bool _debug_mode, int _n_epochs, int _bs, 
            bool _preproc, char *_solver_f, char *_train_f,
            char *_mean_f, bool _debug, bool _pretrained);
		RegNetParams(const RegNetParams *params = nullptr);
	};
}

#endif

