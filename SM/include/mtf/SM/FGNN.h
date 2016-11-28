#ifndef MTF_FGNN_H
#define MTF_FGNN_H

#include "mtf/SM/GNN.h"
#include "mtf/Macros/common.h"
#include <flann/flann.hpp>

_MTF_BEGIN_NAMESPACE

//! GNN with FLANN support
namespace gnn{
	template<class AM>
	class FGNN : public GNN<AM>{
	public:

		typedef typename GNN<AM>::ParamType ParamType;
		typedef flann::Matrix<double> flannMatT;
		typedef flann::Matrix<int> flannResultT;
		typedef flann::Index<AM> flannIdxT;

		FGNN(const AM *_dist_func, int _n_samples, int _n_dims,
			bool _is_symmetrical = true, const ParamType *gnn_params = nullptr):
			GNN<AM>(_dist_func, _n_samples, _n_dims, _is_symmetrical, gnn_params){}
		~FGNN(){}
		using GNN<AM>::n_samples;
		using GNN<AM>::n_dims;
		using GNN<AM>::nodes;
		using GNN<AM>::params;
		using GNN<AM>::addNode;
		using GNN<AM>::loadGraph;
		using GNN<AM>::buildGraph;
		void buildGraph(const double *dataset, flannIdxT* flann_index, 
			const flann::SearchParams &search_params);
	};
}
_MTF_END_NAMESPACE

#endif

