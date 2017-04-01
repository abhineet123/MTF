#ifndef MTF_GNN_NT_H
#define MTF_GNN_NT_H

#include "mtf/AM/AppearanceModel.h"

#define GNN_DEGREE 250
#define GNN_MAX_STEPS 10
#define GNN_CMPT_DIST_THRESH 10000
#define GNN_VERBOSE 0

_MTF_BEGIN_NAMESPACE
namespace nt{
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

		class GNN{
		public:
			typedef AppearanceModel AM;
			typedef GNNParams ParamType;

			GNN(const AM *_dist_func, int _n_samples, int _n_dims,
				bool _is_symmetrical = true, const ParamType *gnn_params = nullptr);
			~GNN(){}
			void buildGraph(const double *dataset);
			void searchGraph(const double *query, const double *dataset,
				int *nn_ids, double *nn_dists, int K = 1);
			void saveGraph(const char* file_name);
			void loadGraph(const char* file_name);

			void  build_graph(const double *X, int k);
			int search_graph(const double *Xq, const double *X,
				int NNs, int K);

		protected:

			struct node{
				int *nns_inds;
				int size;
				int capacity;
			};

			struct indx_dist{
				double dist;
				int idx;
			};

			inline int cmp_qsort(const void *a, const void *b){
				indx_dist *a1 = (indx_dist *)a;
				indx_dist *b1 = (indx_dist *)b;

				// Ascending
				if(a1->dist > b1->dist) return 1;
				if(a1->dist == b1->dist) return 0;
				return -1;
			}

			const AM *dist_func;
			int n_samples, n_dims;
			bool is_symmetrical;

			ParamType params;
			node *Nodes;
			MatrixXd dataset_distances;

			int start_node_idx;
			bool dist_computed;

			void computeDistances(const double *dataset);

			int my_rand(int lb, int ub);
			void swap_int(int *i, int *j);
			void swap_double(double *i, double *j);

			void knn_search2(const double *Q, indx_dist *dists, const double *X,
				int rows, int cols, int k);
			void knn_search11(const double *Q, indx_dist *dists, const double *X, int rows,
				int cols, int k, int *X_inds);

			int min(int a, int b){ return a < b ? a : b; }

			void pick_knns(indx_dist *vis_nodes, int visited, indx_dist **gnn_dists,
				int K, int *gnns_cap);

			void add_node(node *node_i, int nn);
		};
	}
}
_MTF_END_NAMESPACE

#endif

