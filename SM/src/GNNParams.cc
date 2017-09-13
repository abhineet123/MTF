#include "mtf/SM/GNNParams.h"

#define GNN_DEGREE 250
#define GNN_MAX_STEPS 10
#define GNN_CMPT_DIST_THRESH 10000
#define GNN_RANDOM_START 0
#define GNN_VERBOSE 0

_MTF_BEGIN_NAMESPACE

namespace gnn{
	GNNParams::GNNParams(int _degree, int _max_steps,
		int _cmpt_dist_thresh, bool _random_start,
		bool _verbose) :
		degree(_degree),
		max_steps(_max_steps),
		cmpt_dist_thresh(_cmpt_dist_thresh),
		random_start(_random_start),
		verbose(_verbose){}

	GNNParams::GNNParams(const GNNParams *params) :
		degree(GNN_DEGREE),
		max_steps(GNN_MAX_STEPS),
		cmpt_dist_thresh(GNN_CMPT_DIST_THRESH),
		random_start(GNN_RANDOM_START),
		verbose(GNN_VERBOSE){
		if(params){
			degree = params->degree;
			max_steps = params->max_steps;
			cmpt_dist_thresh = params->cmpt_dist_thresh;
			random_start = params->random_start;
			verbose = params->verbose;
		}
	}
}
_MTF_END_NAMESPACE
