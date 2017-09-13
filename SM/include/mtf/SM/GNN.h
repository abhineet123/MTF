#ifndef MTF_GNN_H
#define MTF_GNN_H

#include "mtf/SM/GNNParams.h"

#include <vector>
#include <memory>

_MTF_BEGIN_NAMESPACE

namespace gnn{
	struct Node{
		VectorXi nns_inds;
		int size;
		int capacity;
	};
	struct IndxDist{
		double dist;
		int idx;
	};
	inline int cmpQsort(const void *a, const void *b){
		IndxDist *a1 = (IndxDist *)a;
		IndxDist *b1 = (IndxDist *)b;
		// Ascending
		if(a1->dist > b1->dist) return 1;
		if(a1->dist == b1->dist) return 0;
		return -1;
	}

	template<class DistType>
	class GNN{
	public:

		typedef GNNParams ParamType;
		typedef std::shared_ptr<const DistType> DistTypePtr;

		GNN(DistTypePtr _dist_func, int _n_samples, int _n_dims,
			bool _is_symmetrical = true, const ParamType *gnn_params = nullptr);
		~GNN(){}
		void computeDistances(const double *dataset);
		void buildGraph(const double *dataset);
		void searchGraph(const double *query, const double *dataset, 
			int *nn_ids, double *nn_dists, int K = 1);
		void saveGraph(const char* file_name);
		void loadGraph(const char* file_name);

		void  buildGraph(const double *X, int k);
		int searchGraph(const double *Xq, const double *X, 
			int NNs, int K);

	protected:

		DistTypePtr dist_func;
		const int n_samples, n_dims;
		const bool is_symmetrical;
		ParamType params;
		std::vector<Node> nodes;
		MatrixXd dataset_distances;

		int start_node_idx;
		bool dist_computed;

		int getRandNum(int lb, int ub){
			//  time_t sec;
			//  time(&sec);
			//  srand((unsigned int) sec);
			return (rand() % (ub - lb + 1) + lb);
		}
		template<typename ScalarT>
		void swap(ScalarT *i, ScalarT *j){
			ScalarT temp;
			temp = *i;
			*i = *j;
			*j = temp;
		}
		void knnSearch2(const double *Q, IndxDist *dists, const double *X,
			int rows, int cols, int k);
		void knnSearch11(const double *Q, IndxDist *dists, const double *X, int rows,
			int cols, int k, int *X_inds);

		int min(int a, int b){ return a < b ? a : b; }

		void pickKNNs(IndxDist *vis_nodes, int visited, IndxDist **gnn_dists,
			int K, int *gnns_cap);

		void addNode(Node *node_i, int nn);
	};
}
_MTF_END_NAMESPACE

#endif

