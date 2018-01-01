#include "mtf/SM/FLANNParams.h"
#include "mtf/Utilities/excpUtils.h"

#define NN_SEARCH_TYPE 0
#define NN_INDEX_TYPE 1
#define NN_FGNN_INDEX_TYPE 0
#define NN_SRCH_CHECKS 50
#define NN_SRCH_EPS 0.0f
#define NN_SRCH_SORTED true
#define NN_SRCH_MAX_NEIGHBORS -1
#define NN_SRCH_CORES 1
#define NN_SRCH_MATRICES_IN_GPU_RAM false
#define NN_SRCH_USE_HEAP flann::FLANN_Undefined
#define NN_KDT_TREES 6
#define NN_KM_BRANCHING 32
#define NN_KM_ITERATIONS 11
#define NN_KM_CENTERS_INIT flann::FLANN_CENTERS_RANDOM
#define NN_KM_CB_INDEX 0.2f
#define NN_KDTS_LEAF_MAX_SIZE 10
#define NN_KDTC_LEAF_MAX_SIZE 64
#define NN_HC_BRANCHING 32
#define NN_HC_CENTERS_INIT flann::FLANN_CENTERS_RANDOM
#define NN_HC_TREES 4
#define NN_HC_LEAF_MAX_SIZE 100
#define NN_AUTO_TARGET_PRECISION 0.9f
#define NN_AUTO_BUILD_WEIGHT 0.01f
#define NN_AUTO_MEMORY_WEIGHT 0.0f
#define NN_AUTO_SAMPLE_FRACTION 0.1f

_MTF_BEGIN_NAMESPACE

FLANNParams::FLANNParams(
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
	int _kdt_trees,
	int _km_branching,
	int _km_iterations,
	flann::flann_centers_init_t _km_centers_init,
	float _km_cb_index,
	int _kdts_leaf_max_size,
	int _kdtc_leaf_max_size,
	int _hc_branching,
	flann::flann_centers_init_t _hc_centers_init,
	int _hc_trees,
	int _hc_leaf_max_size,
	float _auto_target_precision,
	float _auto_build_weight,
	float _auto_memory_weight,
	float _auto_sample_fraction) :
search_type(_search_type),
index_type(_index_type),
fgnn_index_type(_fgnn_index_type),
srch_checks(_srch_checks),
srch_eps(_srch_eps),
srch_sorted(_srch_sorted),
srch_max_neighbors(_srch_max_neighbors),
srch_cores(_srch_cores),
srch_matrices_in_gpu_ram(_srch_matrices_in_gpu_ram),
srch_use_heap(_srch_use_heap),
kdt_trees(_kdt_trees),
km_branching(_km_branching),
km_iterations(_km_iterations),
km_centers_init(_km_centers_init),
km_cb_index(_km_cb_index),
kdts_leaf_max_size(_kdts_leaf_max_size),
kdtc_leaf_max_size(_kdtc_leaf_max_size),
hc_branching(_hc_branching),
hc_centers_init(_hc_centers_init),
hc_trees(_hc_trees),
hc_leaf_max_size(_hc_leaf_max_size),
auto_target_precision(_auto_target_precision),
auto_build_weight(_auto_build_weight),
auto_memory_weight(_auto_memory_weight),
auto_sample_fraction(_auto_sample_fraction){
	updateSearchParams();
	//printf("FLANNParams::Value Constructor:\n");
	//printParams();
}

FLANNParams::FLANNParams(const FLANNParams *params) :
search_type(static_cast<SearchType>(NN_SEARCH_TYPE)),
index_type(static_cast<IdxType>(NN_INDEX_TYPE)),
fgnn_index_type(static_cast<IdxType>(NN_FGNN_INDEX_TYPE)),
srch_checks(NN_SRCH_CHECKS),
srch_eps(NN_SRCH_EPS),
srch_sorted(NN_SRCH_SORTED),
srch_max_neighbors(NN_SRCH_MAX_NEIGHBORS),
srch_cores(NN_SRCH_CORES),
srch_matrices_in_gpu_ram(NN_SRCH_MATRICES_IN_GPU_RAM),
srch_use_heap(NN_SRCH_USE_HEAP),
kdt_trees(NN_KDT_TREES),
km_branching(NN_KM_BRANCHING),
km_iterations(NN_KM_ITERATIONS),
km_centers_init(NN_KM_CENTERS_INIT),
km_cb_index(NN_KM_CB_INDEX),
kdts_leaf_max_size(NN_KDTS_LEAF_MAX_SIZE),
kdtc_leaf_max_size(NN_KDTC_LEAF_MAX_SIZE),
hc_branching(NN_HC_BRANCHING),
hc_centers_init(NN_HC_CENTERS_INIT),
hc_trees(NN_HC_TREES),
hc_leaf_max_size(NN_HC_LEAF_MAX_SIZE),
auto_target_precision(NN_AUTO_TARGET_PRECISION),
auto_build_weight(NN_AUTO_BUILD_WEIGHT),
auto_memory_weight(NN_AUTO_MEMORY_WEIGHT),
auto_sample_fraction(NN_AUTO_SAMPLE_FRACTION){
	if(params){
		search_type = params->search_type;
		index_type = params->index_type;
		fgnn_index_type = params->fgnn_index_type;

		srch_checks = params->srch_checks;
		srch_eps = params->srch_eps;
		srch_sorted = params->srch_sorted;
		srch_max_neighbors = params->srch_max_neighbors;
		srch_cores = params->srch_cores;
		srch_matrices_in_gpu_ram = params->srch_matrices_in_gpu_ram;
		srch_use_heap = params->srch_use_heap;
		kdt_trees = params->kdt_trees;
		km_branching = params->km_branching;
		km_iterations = params->km_iterations;
		km_centers_init = params->km_centers_init;
		km_cb_index = params->km_cb_index;
		kdts_leaf_max_size = params->kdts_leaf_max_size;
		kdtc_leaf_max_size = params->kdtc_leaf_max_size;
		hc_branching = params->hc_branching;
		hc_centers_init = params->hc_centers_init;
		hc_trees = params->hc_trees;
		hc_leaf_max_size = params->hc_leaf_max_size;
		auto_target_precision = params->auto_target_precision;
		auto_build_weight = params->auto_build_weight;
		auto_memory_weight = params->auto_memory_weight;
		auto_sample_fraction = params->auto_sample_fraction;
	}
	updateSearchParams();
	//printf("FLANNParams::Copy Constructor:\n");
	//printParams();
}

void FLANNParams::updateSearchParams(){
	search.checks = srch_checks;
	search.cores = srch_cores;
	search.eps = srch_eps;
	search.matrices_in_gpu_ram = srch_matrices_in_gpu_ram;
	search.max_neighbors = srch_max_neighbors;
	search.sorted = srch_sorted;
	search.use_heap = srch_use_heap;
}


const flann::IndexParams FLANNParams::getIndexParams(IdxType _index_type,
	bool load_index, std::string saved_idx_path){
	if(load_index){
		printf("Loading FLANN index from: %s\n", saved_idx_path.c_str());
		return flann::SavedIndexParams(saved_idx_path);
	}
	switch(_index_type){
	case IdxType::GNN:
		throw utils::InvalidArgument("GNN is not a valid FLANN index type");
	case IdxType::Linear:
		printf("Using Linear index\n");
		return flann::LinearIndexParams();
	case IdxType::KDTree:
		printf("Using KD Tree index with:\n");
		printf("n_trees: %d\n", kdt_trees);
		return flann::KDTreeIndexParams(kdt_trees);
	case IdxType::KMeans:
		if(!(km_centers_init == flann::FLANN_CENTERS_RANDOM ||
			km_centers_init == flann::FLANN_CENTERS_GONZALES ||
			km_centers_init == flann::FLANN_CENTERS_KMEANSPP)){
			printf("Invalid method provided for selecting initial centers: %d. Using random centers...\n",
				km_centers_init);
			km_centers_init = flann::FLANN_CENTERS_RANDOM;
		}
		printf("Using KMeans index with:\n");
		printf("branching: %d\n", km_branching);
		printf("iterations: %d\n", km_iterations);
		printf("centers_init: %d\n", km_centers_init);
		printf("cb_index: %f\n", km_cb_index);
		return flann::KMeansIndexParams(km_branching, km_iterations,
			km_centers_init, km_cb_index);
	case IdxType::Composite:
		if(!(km_centers_init == flann::FLANN_CENTERS_RANDOM ||
			km_centers_init == flann::FLANN_CENTERS_GONZALES ||
			km_centers_init == flann::FLANN_CENTERS_KMEANSPP)){
			printf("Invalid method provided for selecting initial centers: %d. Using random centers...\n", km_centers_init);
			km_centers_init = flann::FLANN_CENTERS_RANDOM;
		}
		printf("Using Composite index with:\n");
		printf("n_trees: %d\n", kdt_trees);
		printf("branching: %d\n", km_branching);
		printf("iterations: %d\n", km_iterations);
		printf("centers_init: %d\n", km_centers_init);
		printf("cb_index: %f\n", km_cb_index);
		return flann::CompositeIndexParams(kdt_trees, km_branching, km_iterations,
			km_centers_init, km_cb_index);
	case IdxType::HierarchicalClustering:
		if(!(hc_centers_init == flann::FLANN_CENTERS_RANDOM ||
			hc_centers_init == flann::FLANN_CENTERS_GONZALES ||
			hc_centers_init == flann::FLANN_CENTERS_KMEANSPP)){
			printf("Invalid method provided for selecting initial centers: %d. Using random centers...\n",
				km_centers_init);
			hc_centers_init = flann::FLANN_CENTERS_RANDOM;
		}
		printf("Using Hierarchical Clustering index with:\n");
		printf("branching: %d\n", hc_branching);
		printf("centers_init: %d\n", hc_centers_init);
		printf("trees: %d\n", hc_trees);
		printf("leaf_max_size: %d\n", hc_leaf_max_size);
		return flann::HierarchicalClusteringIndexParams(hc_branching,
			hc_centers_init, hc_trees, hc_leaf_max_size);
	case IdxType::KDTreeSingle:
		printf("Using KDTreeSingle index with:\n");
		printf("leaf_max_size: %d\n", kdts_leaf_max_size);
		return flann::KDTreeSingleIndexParams(kdts_leaf_max_size);
#ifdef FLANN_USE_CUDA
	case IdxType::KDTreeCuda3d:
		printf("Using KDTreeCuda3d index with:\n");
		printf("leaf_max_size: %d\n", kdtc_leaf_max_size);
		return flann::KDTreeCuda3dIndexParams(kdtc_leaf_max_size);
#endif
	case IdxType::Autotuned:
		printf("Using Autotuned index with:\n");
		printf("target_precision: %f\n", auto_target_precision);
		printf("build_weight: %f\n", auto_build_weight);
		printf("memory_weight: %f\n", auto_memory_weight);
		printf("sample_fraction: %f\n", auto_sample_fraction);
		return flann::AutotunedIndexParams(auto_target_precision, auto_build_weight,
			auto_memory_weight, auto_sample_fraction);
	default:
		printf("Invalid index type specified: %d. Using KD Tree index by default...\n", _index_type);
		return flann::KDTreeIndexParams(kdt_trees);
	}
}

const char* FLANNParams::toString(IdxType index_type){
	switch(index_type){
	case IdxType::GNN:
		return "GNN";
	case IdxType::KDTree:
		return "KDTree";
	case IdxType::HierarchicalClustering:
		return "HierarchicalClustering";
	case IdxType::KMeans:
		return "KMeans";
	case IdxType::Composite:
		return "Composite";
	case IdxType::Linear:
		return "Linear";
	case IdxType::KDTreeSingle:
		return "KDTreeSingle";
	case IdxType::KDTreeCuda3d:
		return "KDTreeCuda3d";
	case IdxType::Autotuned:
		return "Autotuned";
	default:
		throw utils::InvalidArgument("Invalid index type provided");
	}
}

const char* FLANNParams::toString(SearchType search_type){
	switch(search_type){
	case SearchType::KNN:
		return "KNN";
	case SearchType::Radius:
		return "Radius";
	default:
		throw utils::InvalidArgument("Invalid search type provided");
	}
}

void FLANNParams::printParams(){
	printf("search_type: %s\n", toString(search_type));
	printf("index_type: %s\n", toString(index_type));
	printf("fgnn_index_type: %s\n", toString(fgnn_index_type));
	printf("srch_checks: %d\n", srch_checks);
	printf("srch_eps: %f\n", srch_eps);
	printf("srch_sorted: %d\n", srch_sorted);
	printf("srch_max_neighbors: %d\n", srch_max_neighbors);
	printf("srch_cores: %d\n", srch_cores);
	printf("srch_matrices_in_gpu_ram: %d\n", srch_matrices_in_gpu_ram);
	printf("srch_use_heap: %d\n", srch_use_heap);
	printf("kdt_trees: %d\n", kdt_trees);
	printf("km_branching: %d\n", km_branching);
	printf("km_iterations: %d\n", km_iterations);
	printf("km_centers_init: %d\n", km_centers_init);
	printf("km_cb_index: %f\n", km_cb_index);
	printf("kdts_leaf_max_size: %d\n", kdts_leaf_max_size);
	printf("kdtc_leaf_max_size: %d\n", kdtc_leaf_max_size);
	printf("hc_branching: %d\n", hc_branching);
	printf("hc_centers_init: %d\n", hc_centers_init);
	printf("hc_trees: %d\n", hc_trees);
	printf("hc_leaf_max_size: %d\n", hc_leaf_max_size);
	printf("hc_centers_init: %d\n", hc_centers_init);
	printf("auto_target_precision: %f\n", auto_target_precision);
	printf("auto_build_weight: %f\n", auto_build_weight);
	printf("auto_memory_weight: %f\n", auto_memory_weight);
	printf("auto_sample_fraction: %f\n", auto_sample_fraction);
}

_MTF_END_NAMESPACE
