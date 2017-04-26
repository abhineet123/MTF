#ifndef MTF_FGNN_H
#define MTF_FGNN_H

#include "mtf/SM/GNN.h"
#include "mtf/Macros/common.h"
#include <flann/flann.hpp>
#include <memory>

_MTF_BEGIN_NAMESPACE

//! GNN with FLANN support
namespace gnn{
	template<class DistType>
	class FGNN : public GNN<DistType>{
	public:
		typedef std::shared_ptr<const DistType> DistTypePtr;
		typedef typename GNN<DistType>::ParamType ParamType;
		typedef flann::Matrix<double> flannMatT;
		typedef flann::Matrix<int> flannResultT;
		typedef flann::Index<DistType> flannIdxT;

		FGNN(DistTypePtr _dist_func, int _n_samples, int _n_dims,
			bool _is_symmetrical = true, const ParamType *gnn_params = nullptr):
			GNN<DistType>(_dist_func, _n_samples, _n_dims, _is_symmetrical, gnn_params){}
		~FGNN(){}
		using GNN<DistType>::n_samples;
		using GNN<DistType>::n_dims;
		using GNN<DistType>::nodes;
		using GNN<DistType>::params;
		using GNN<DistType>::addNode;
		using GNN<DistType>::loadGraph;
		using GNN<DistType>::buildGraph;
		void buildGraph(const double *dataset, flannIdxT* flann_index, 
			const flann::SearchParams &search_params);
	};
}
_MTF_END_NAMESPACE

#endif

