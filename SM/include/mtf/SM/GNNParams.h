#ifndef MTF_GNN_PARAMS_H
#define MTF_GNN_PARAMS_H

#include "mtf/Macros/common.h"

_MTF_BEGIN_NAMESPACE

namespace gnn{	
	struct GNNParams{
		int degree;
		int max_steps;
		int cmpt_dist_thresh;
		bool random_start;
		bool verbose;
		GNNParams(int _dgree, int _max_steps,
			int _cmpt_dist_thresh, bool _random_start,
			bool _verbose);
		GNNParams(const GNNParams *params = nullptr);
	};
}
_MTF_END_NAMESPACE

#endif

