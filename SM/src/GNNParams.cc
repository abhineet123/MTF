#include "mtf/SM/GNNParams.h"

_MTF_BEGIN_NAMESPACE

namespace gnn{
	GNNParams::GNNParams(int _degree, int _max_steps,
		int _cmpt_dist_thresh, bool _verbose) :
		degree(_degree),
		max_steps(_max_steps),
		cmpt_dist_thresh(_cmpt_dist_thresh),
		verbose(_verbose){}

	GNNParams::GNNParams(const GNNParams *params) :
		degree(GNN_DEGREE),
		max_steps(GNN_MAX_STEPS),
		cmpt_dist_thresh(GNN_CMPT_DIST_THRESH),
		verbose(GNN_VERBOSE){
		if(params){
			degree = params->degree;
			max_steps = params->max_steps;
			cmpt_dist_thresh = params->cmpt_dist_thresh;
			verbose = params->verbose;
		}
	}
}
_MTF_END_NAMESPACE
