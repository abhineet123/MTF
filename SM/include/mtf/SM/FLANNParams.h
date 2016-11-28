#ifndef MTF_FLANN_PARAMS_H
#define MTF_FLANN_PARAMS_H

#include "mtf/Macros/common.h"
#include <flann/flann.hpp>

#define NN_SEARCH_TYPE 0
#define NN_INDEX_TYPE 1
#define NN_FGNN_INDEX_TYPE 0
#define NN_SRCH_CHECKS 50
#define NN_SRCH_EPS 0.0
#define NN_SRCH_SORTED true
#define NN_SRCH_MAX_NEIGHBORS -1
#define NN_SRCH_CORES 1
#define NN_SRCH_MATRICES_IN_GPU_RAM false
#define NN_SRCH_USE_HEAP flann::FLANN_Undefined
#define NN_KDT_TREES 6
#define NN_KM_BRANCHING 32
#define NN_KM_ITERATIONS 11
#define NN_KM_CENTERS_INIT flann::FLANN_CENTERS_RANDOM
#define NN_KM_CB_INDEX 0.2
#define NN_KDTS_LEAF_MAX_SIZE 10
#define NN_KDTC_LEAF_MAX_SIZE 64
#define NN_HC_BRANCHING 32
#define NN_HC_CENTERS_INIT flann::FLANN_CENTERS_RANDOM
#define NN_HC_TREES 4
#define NN_HC_LEAF_MAX_SIZE 100
#define NN_AUTO_TARGET_PRECISION 0.9
#define NN_AUTO_BUILD_WEIGHT 0.01
#define NN_AUTO_MEMORY_WEIGHT 0
#define NN_AUTO_SAMPLE_FRACTION 0.1

_MTF_BEGIN_NAMESPACE

//! index specific params
struct FLANNParams{

	enum class IdxType{
		GNN, KDTree, HierarchicalClustering, KMeans, Composite, Linear,
		KDTreeSingle, KDTreeCuda3d, Autotuned
	};
	enum class SearchType{ KNN, Radius };
	static const char* toString(IdxType index_type);
	static const char* toString(SearchType index_type);

	const flann::IndexParams getIndexParams(IdxType _index_type = IdxType::KDTree,
		bool load_index=false, std::string saved_idx_path="");
	void updateSearchParams();

	flann::SearchParams search;

	void printParams();

	SearchType search_type;
	IdxType index_type;
	IdxType fgnn_index_type;

	int srch_checks;
	float srch_eps;
	bool srch_sorted;
	int srch_max_neighbors;
	int srch_cores;
	bool srch_matrices_in_gpu_ram;
	flann::tri_type srch_use_heap;
	int kdt_trees;
	int km_branching;
	int km_iterations;
	flann::flann_centers_init_t km_centers_init;
	float km_cb_index;
	int kdts_leaf_max_size;
	int kdtc_leaf_max_size;
	int hc_branching;
	flann::flann_centers_init_t hc_centers_init;
	int hc_trees;
	int hc_leaf_max_size;
	float auto_target_precision;
	float auto_build_weight;
	float auto_memory_weight;
	float auto_sample_fraction;

	FLANNParams(
		SearchType _search_type,
		IdxType _index_type,
		IdxType _fgnn_index_type,
		int _srch_checks,
		float _srch_eps,
		bool _srch_sorted,
		int _srch_max_neighbors,
		int _srch_cores,
		bool _srch_matrices_in_gpu_ram,
		flann::tri_type _srch_use_heap,
		int kdt_trees,
		int km_branching,
		int km_iterations,
		flann::flann_centers_init_t _km_centers_init,
		float km_cb_index,
		int kdts_leaf_max_size,
		int kdtc_leaf_max_size,
		int hc_branching,
		flann::flann_centers_init_t _hc_centers_init,
		int hc_trees,
		int hc_leaf_max_size,
		float auto_target_precision,
		float auto_build_weight,
		float auto_memory_weight,
		float auto_sample_fraction);
	FLANNParams(const FLANNParams *params = nullptr);
};

_MTF_END_NAMESPACE

#endif

