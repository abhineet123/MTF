#ifndef MTF_FLANNCV_PARAMS_H
#define MTF_FLANNCV_PARAMS_H

#include "mtf/Macros/common.h"
#include "opencv2/features2d/features2d.hpp"

_MTF_BEGIN_NAMESPACE

//! index specific params
struct FLANNCVParams{

	enum class IdxType{
		KDTree, HierarchicalClustering, KMeans, Composite, Linear,
		LSH, Autotuned
	};
	static const char* toString(IdxType index_type);

	const cv::flann::IndexParams getIndexParams(
		IdxType _index_type = IdxType::KDTree);


	void printParams();

	IdxType index_type;

	cv::flann::SearchParams getSearchParams(){
		return cv::flann::SearchParams(
			srch_checks, srch_eps, srch_sorted);
	}
	int srch_checks;
	float srch_eps;
	bool srch_sorted;
	int kdt_trees;
	int km_branching;
	int km_iterations;
	cvflann::flann_centers_init_t km_centers_init;
	float km_cb_index;
	int kdts_leaf_max_size;
	int kdtc_leaf_max_size;
	int hc_branching;
	cvflann::flann_centers_init_t hc_centers_init;
	int hc_trees;
	int hc_leaf_max_size;
	float auto_target_precision;
	float auto_build_weight;
	float auto_memory_weight;
	float auto_sample_fraction;


	FLANNCVParams(
		IdxType _index_type,
		int _srch_checks,
		float _srch_eps,
		bool _srch_sorted,
		int kdt_trees,
		int km_branching,
		int km_iterations,
		cvflann::flann_centers_init_t _km_centers_init,
		float km_cb_index,
		int kdts_leaf_max_size,
		int kdtc_leaf_max_size,
		int hc_branching,
		cvflann::flann_centers_init_t _hc_centers_init,
		int hc_trees,
		int hc_leaf_max_size,
		float auto_target_precision,
		float auto_build_weight,
		float auto_memory_weight,
		float auto_sample_fraction);
	FLANNCVParams(const FLANNCVParams *params = nullptr);
};

_MTF_END_NAMESPACE

#endif

