#ifndef MTF_GNN_PARAMS_H
#define MTF_GNN_PARAMS_H

#include "mtf/Macros/common.h"

#define GNN_DEGREE 250
#define GNN_MAX_STEPS 10
#define GNN_CMPT_DIST_THRESH 10000
#define GNN_VERBOSE 0

_MTF_BEGIN_NAMESPACE

namespace gnn{	
	struct GNNParams{
		int degree;
		int max_steps;
		int cmpt_dist_thresh;
		bool verbose;
		GNNParams(int _dgree, int _max_steps,
			int _cmpt_dist_thresh, bool _verbose);
		GNNParams(const GNNParams *params = nullptr);
	};	
}
_MTF_END_NAMESPACE

#endif

