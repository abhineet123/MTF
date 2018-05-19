#include "mtf/SM/FLANNCVParams.h"
#include "mtf/Utilities/excpUtils.h"

#define FLANNCV_INDEX_TYPE 1
#define FLANNCV_SRCH_CHECKS 50
#define FLANNCV_SRCH_EPS 0.0f
#define FLANNCV_SRCH_SORTED true
#define FLANNCV_KDT_TREES 6
#define FLANNCV_KM_BRANCHING 32
#define FLANNCV_KM_ITERATIONS 11
#define FLANNCV_KM_CENTERS_INIT cvflann::FLANN_CENTERS_RANDOM
#define FLANNCV_KM_CB_INDEX 0.2f
#define FLANNCV_KDTS_LEAF_MAX_SIZE 10
#define FLANNCV_KDTC_LEAF_MAX_SIZE 64
#define FLANNCV_HC_BRANCHING 32
#define FLANNCV_HC_CENTERS_INIT cvflann::FLANN_CENTERS_RANDOM
#define FLANNCV_HC_TREES 4
#define FLANNCV_HC_LEAF_MAX_SIZE 100
#define FLANNCV_AUTO_TARGET_PRECISION 0.9f
#define FLANNCV_AUTO_BUILD_WEIGHT 0.01f
#define FLANNCV_AUTO_MEMORY_WEIGHT 0.0f
#define FLANNCV_AUTO_SAMPLE_FRACTION 0.1f

_MTF_BEGIN_NAMESPACE

FLANNCVParams::FLANNCVParams(
IdxType _index_type,
int _srch_checks,
float _srch_eps,
bool _srch_sorted,
int _kdt_trees,
int _km_branching,
int _km_iterations,
cvflann::flann_centers_init_t _km_centers_init,
float _km_cb_index,
int _kdts_leaf_max_size,
int _kdtc_leaf_max_size,
int _hc_branching,
cvflann::flann_centers_init_t _hc_centers_init,
int _hc_trees,
int _hc_leaf_max_size,
float _auto_target_precision,
float _auto_build_weight,
float _auto_memory_weight,
float _auto_sample_fraction) :
index_type(_index_type),
srch_checks(_srch_checks),
srch_eps(_srch_eps),
srch_sorted(_srch_sorted),
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
auto_sample_fraction(_auto_sample_fraction){}

FLANNCVParams::FLANNCVParams(const FLANNCVParams *params) :
index_type(static_cast<IdxType>(FLANNCV_INDEX_TYPE)),
srch_checks(FLANNCV_SRCH_CHECKS),
srch_eps(FLANNCV_SRCH_EPS),
srch_sorted(FLANNCV_SRCH_SORTED),
kdt_trees(FLANNCV_KDT_TREES),
km_branching(FLANNCV_KM_BRANCHING),
km_iterations(FLANNCV_KM_ITERATIONS),
km_centers_init(FLANNCV_KM_CENTERS_INIT),
km_cb_index(FLANNCV_KM_CB_INDEX),
kdts_leaf_max_size(FLANNCV_KDTS_LEAF_MAX_SIZE),
kdtc_leaf_max_size(FLANNCV_KDTC_LEAF_MAX_SIZE),
hc_branching(FLANNCV_HC_BRANCHING),
hc_centers_init(FLANNCV_HC_CENTERS_INIT),
hc_trees(FLANNCV_HC_TREES),
hc_leaf_max_size(FLANNCV_HC_LEAF_MAX_SIZE),
auto_target_precision(FLANNCV_AUTO_TARGET_PRECISION),
auto_build_weight(FLANNCV_AUTO_BUILD_WEIGHT),
auto_memory_weight(FLANNCV_AUTO_MEMORY_WEIGHT),
auto_sample_fraction(FLANNCV_AUTO_SAMPLE_FRACTION){
	if(params){
		index_type = params->index_type;
		srch_checks = params->srch_checks;
		srch_eps = params->srch_eps;
		srch_sorted = params->srch_sorted;
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
}

const FLANNCVParams::SearchParamsPtr& FLANNCVParams::getSearchParams(){
	return SearchParamsPtr(new cv::flann::SearchParams(
		srch_checks, srch_eps, srch_sorted));
}

const FLANNCVParams::IndexParamsPtr& FLANNCVParams::getIndexParams(IdxType _index_type){
	switch(_index_type){
	case IdxType::Linear:
		printf("Using Linear index\n");
		return IndexParamsPtr(new cv::flann::LinearIndexParams());
	case IdxType::KDTree:
		printf("Using KD Tree index with:\n");
		printf("n_trees: %d\n", kdt_trees);
		return IndexParamsPtr(new cv::flann::KDTreeIndexParams(kdt_trees));
	case IdxType::KMeans:
		if(!(km_centers_init == cvflann::FLANN_CENTERS_RANDOM ||
			km_centers_init == cvflann::FLANN_CENTERS_GONZALES ||
			km_centers_init == cvflann::FLANN_CENTERS_KMEANSPP)){
			printf("Invalid method provided for selecting initial centers: %d. Using random centers...\n",
				km_centers_init);
			km_centers_init = cvflann::FLANN_CENTERS_RANDOM;
		}
		printf("Using KMeans index with:\n");
		printf("branching: %d\n", km_branching);
		printf("iterations: %d\n", km_iterations);
		printf("centers_init: %d\n", km_centers_init);
		printf("cb_index: %f\n", km_cb_index);
		return IndexParamsPtr(new cv::flann::KMeansIndexParams(km_branching, km_iterations,
			km_centers_init, km_cb_index));
	case IdxType::Composite:
		if(!(km_centers_init == cvflann::FLANN_CENTERS_RANDOM ||
			km_centers_init == cvflann::FLANN_CENTERS_GONZALES ||
			km_centers_init == cvflann::FLANN_CENTERS_KMEANSPP)){
			printf("Invalid method provided for selecting initial centers: %d. Using random centers...\n", km_centers_init);
			km_centers_init = cvflann::FLANN_CENTERS_RANDOM;
		}
		printf("Using Composite index with:\n");
		printf("n_trees: %d\n", kdt_trees);
		printf("branching: %d\n", km_branching);
		printf("iterations: %d\n", km_iterations);
		printf("centers_init: %d\n", km_centers_init);
		printf("cb_index: %f\n", km_cb_index);
		return IndexParamsPtr(new cv::flann::CompositeIndexParams(kdt_trees, km_branching, km_iterations,
			km_centers_init, km_cb_index));
	case IdxType::HierarchicalClustering:
		if(!(hc_centers_init == cvflann::FLANN_CENTERS_RANDOM ||
			hc_centers_init == cvflann::FLANN_CENTERS_GONZALES ||
			hc_centers_init == cvflann::FLANN_CENTERS_KMEANSPP)){
			printf("Invalid method provided for selecting initial centers: %d. Using random centers...\n",
				km_centers_init);
			hc_centers_init = cvflann::FLANN_CENTERS_RANDOM;
		}
		printf("Using Hierarchical Clustering index with:\n");
		printf("branching: %d\n", hc_branching);
		printf("centers_init: %d\n", hc_centers_init);
		printf("trees: %d\n", hc_trees);
		printf("leaf_max_size: %d\n", hc_leaf_max_size);
		return IndexParamsPtr(new cv::flann::HierarchicalClusteringIndexParams(hc_branching,
			hc_centers_init, hc_trees, hc_leaf_max_size));
	case IdxType::Autotuned:
		printf("Using Autotuned index with:\n");
		printf("target_precision: %f\n", auto_target_precision);
		printf("build_weight: %f\n", auto_build_weight);
		printf("memory_weight: %f\n", auto_memory_weight);
		printf("sample_fraction: %f\n", auto_sample_fraction);
		return IndexParamsPtr(new cv::flann::AutotunedIndexParams(auto_target_precision, auto_build_weight,
			auto_memory_weight, auto_sample_fraction));
	default:
		printf("Invalid index type specified: %d. Using KD Tree index by default...\n", _index_type);
		return IndexParamsPtr(new cv::flann::KDTreeIndexParams(kdt_trees));
	}
}

const char* FLANNCVParams::toString(IdxType index_type){
	switch(index_type){
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
	case IdxType::Autotuned:
		return "Autotuned";
	default:
		throw utils::InvalidArgument("FLANNCVParams :: Invalid index type provided");
	}
}

void FLANNCVParams::printParams(){
	printf("index_type: %s\n", toString(index_type));
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
