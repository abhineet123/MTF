#ifndef MTF_FLANN_PARAMS_H
#define MTF_FLANN_PARAMS_H

#include "mtf/Macros/common.h"
#include <flann/flann.hpp>

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

