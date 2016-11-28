#include "mtf/SM/FGNN.h"
#include "mtf/Utilities//miscUtils.h"

#include <fstream> 

_MTF_BEGIN_NAMESPACE
namespace gnn{

	template <class AM>
	void  FGNN<AM>::buildGraph(const double *dataset, flannIdxT* flann_index,
		const flann::SearchParams &search_params){
		VectorXi nn_ids(params.degree);
		VectorXd nn_dists(params.degree);
		flannResultT flann_result(static_cast<int*>(nn_ids.data()), 1, params.degree);
		flannMatT flann_dists(static_cast<double*>(nn_dists.data()), 1, params.degree);
		nodes.resize(n_samples);
		if(params.verbose){
			printf("Processing graph nodes using FLANN...\n");
		}
		mtf_clock_get(build_state_time);
		for(int id1 = 0; id1 < n_samples; ++id1){
			nodes[id1].nns_inds.resize(params.degree);
			nodes[id1].capacity = params.degree;
			nodes[id1].size = 0;
			flannMatT flann_query(const_cast<double*>(dataset + (id1*n_dims)), 1, n_dims);
			flann_index->knnSearch(flann_query, flann_result, flann_dists, params.degree, search_params);
			for(int id2 = 0; id2 < params.degree; ++id2){
				addNode(&nodes[id1], nn_ids[id2]);
			}
			if(params.verbose){
				mtf_clock_get(end_time);
				double elapsed_time;
				mtf_clock_measure(build_state_time, end_time, elapsed_time);
				if((id1 + 1) % 100 == 0){
					printf("Done %d/%d nodes (%6.2f%%). Time elapsed: %f secs...\n",
						id1 + 1, n_samples, double(id1 + 1) / double(n_samples) * 100,
						elapsed_time);
				}
			}
		}
	}
}
_MTF_END_NAMESPACE

#include "mtf/Macros/register.h"
_REGISTER_AM_TRACKERS(gnn::FGNN);
