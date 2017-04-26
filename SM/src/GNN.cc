#include "mtf/SM/GNN.h"
#include "mtf/Utilities//miscUtils.h"
#include <fstream> 

#define GNN_DEGREE 250
#define GNN_MAX_STEPS 10
#define GNN_CMPT_DIST_THRESH 10000
#define GNN_RANDOM_START 0
#define GNN_VERBOSE 0

_MTF_BEGIN_NAMESPACE
namespace gnn{
	GNNParams::GNNParams(int _degree, int _max_steps,
		int _cmpt_dist_thresh, bool _random_start,
		bool _verbose) :
		degree(_degree),
		max_steps(_max_steps),
		cmpt_dist_thresh(_cmpt_dist_thresh),
		random_start(_random_start),
		verbose(_verbose){}

	GNNParams::GNNParams(const GNNParams *params) :
		degree(GNN_DEGREE),
		max_steps(GNN_MAX_STEPS),
		cmpt_dist_thresh(GNN_CMPT_DIST_THRESH),
		random_start(GNN_RANDOM_START),
		verbose(GNN_VERBOSE){
		if(params){
			degree = params->degree;
			max_steps = params->max_steps;
			cmpt_dist_thresh = params->cmpt_dist_thresh;
			random_start = params->random_start;
			verbose = params->verbose;
		}
	}

	template <class DistType>
	GNN<DistType>::GNN(DistTypePtr _dist_func, int _n_samples, int _n_dims,
		bool _is_symmetrical, const ParamType *gnn_params) :
		dist_func(_dist_func),
		n_samples(_n_samples),
		n_dims(_n_dims),
		is_symmetrical(_is_symmetrical),
		params(gnn_params)	{
		if(params.degree == 0 || params.degree > n_samples){
			params.degree = n_samples;
		} else if(params.degree < 0){
			params.degree = -n_samples / params.degree;
		}
		printf("Using Graph based NN with:\n");
		printf("degree: %d\n", params.degree);
		printf("max_steps: %d\n", params.max_steps);
		printf("cmpt_dist_thresh: %d\n", params.cmpt_dist_thresh);
		printf("random_start: %d\n", params.random_start);
		printf("verbose: %d\n", params.verbose);

		dist_computed = false;
		start_node_idx = getRandNum(0, n_samples - 1);
	}

	template <class DistType>
	void GNN<DistType>::computeDistances(const double *dataset){
		dataset_distances.resize(n_samples, n_samples);
		if(params.verbose){
			printf("Computing distances between samples...\n");
		}
		mtf_clock_get(dist_state_time);
		for(int id1 = 0; id1 < n_samples; ++id1){
			const double* sample1 = dataset + (id1*n_dims);
			dataset_distances(id1, id1) = (*dist_func)(sample1, sample1, n_dims);
			for(int id2 = id1 + 1; id2 < n_samples; ++id2){
				const double* sample2 = dataset + (id2*n_dims);
				dataset_distances(id1, id2) = (*dist_func)(sample1, sample2, n_dims);
				dataset_distances(id2, id1) = is_symmetrical ? dataset_distances(id1, id2) :
					(*dist_func)(sample2, sample1, n_dims);
			}
			if(params.verbose){
				mtf_clock_get(end_time);
				double elapsed_time;
				mtf_clock_measure(dist_state_time, end_time, elapsed_time);
				if((id1 + 1) % 100 == 0){
					printf("Done %d/%d samples (%6.2f%%). Time elapsed: %f secs...\n",
						id1 + 1, n_samples, double(id1 + 1) / double(n_samples) * 100,
						elapsed_time);
				}
			}
		}
		dist_computed = true;
	}
	template <class DistType>
	void GNN<DistType>::buildGraph(const double *dataset){
		if(!dist_computed && n_samples <= params.cmpt_dist_thresh){
			// distance is pre computed and stored only if the no. of samples is not large enough to 
			// cause a bad_alloc error on attempting to allocate memory for this
			computeDistances(dataset);
		}
		nodes.resize(n_samples);
		std::vector<IndxDist> dists(params.degree + 1);
		if(params.verbose){
			printf("Processing graph nodes...\n");
		}
		mtf_clock_get(build_state_time);
		for(int id1 = 0; id1 < n_samples; ++id1){
			nodes[id1].nns_inds.resize(params.degree);
			nodes[id1].capacity = params.degree;
			nodes[id1].size = 0;
			int count = 0;
			for(int id2 = 0; id2 < n_samples; ++id2){
				double dist = dist_computed ? dataset_distances(id1, id2) :
					(*dist_func)(dataset + (id1*n_dims), dataset + (id2*n_dims), n_dims);
				if(count < params.degree + 1){
					dists[count].idx = id2;
					dists[count].dist = dist;
					++count;
				} else if(dist < dists[count - 1].dist || (dist == dists[count - 1].dist &&
					id2 < dists[count - 1].idx)) {
					dists[count - 1].idx = id2;
					dists[count - 1].dist = dist;
				} else {
					continue;
				}
				int id3 = count - 1;
				while(id3 >= 1 && (dists[id3].dist < dists[id3 - 1].dist || (dists[id3].dist == dists[id3 - 1].dist &&
					dists[id3].idx < dists[id3 - 1].idx))){
					swap<int>(&dists[id3].idx, &dists[id3 - 1].idx);
					swap<double>(&dists[id3].dist, &dists[id3 - 1].dist);
					--id3;
				}
			}
			for(int j = 0; j < params.degree; j++){
				int nns_ind = dists[j + 1].idx;
				addNode(&nodes[id1], nns_ind);
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
	template <class DistType>
	void GNN<DistType>::searchGraph(const double *query, const double *dataset,
		int *nn_ids, double *nn_dists, int K){
		int gnns_cap = K;
		int visited_cap = K * 4;  //avg depth = 4
		int visited = 0;   // number of visited nodes
		IndxDist *gnn_dists = static_cast<IndxDist*>(malloc(gnns_cap * sizeof(IndxDist))); // graph nn-dists
		IndxDist *visited_nodes = static_cast<IndxDist*>(malloc(visited_cap * sizeof(IndxDist)));

		if(params.random_start){
			start_node_idx = getRandNum(0, n_samples - 1);
		}

		int r = start_node_idx;
		int parent_dist = (*dist_func)(query, &dataset[r*n_dims], n_dims);

		visited_nodes[0].idx = r;
		visited_nodes[0].dist = parent_dist;
		visited++;
		bool nn_found = false;
		for(int step_id = 0; step_id < params.max_steps; ++step_id){
			if(nodes[r].size > gnns_cap) {
				gnns_cap = nodes[r].size;
				gnn_dists = static_cast<IndxDist*>(realloc(gnn_dists, gnns_cap * sizeof(IndxDist)));
			}
			int count = 0;
			//printf("Nodes[%d].size: %d\n", r, Nodes[r].size);
			for(int id1 = 0; id1 < nodes[r].size; id1++){
				//printf("Nodes[%d].nns_inds[%d]: %d\n", r, id1, Nodes[r].nns_inds[id1]);
				const double *point = dataset + nodes[r].nns_inds[id1] * n_dims;
				double dist = (*dist_func)(query, point, n_dims);
				if(count < K){
					gnn_dists[count].idx = id1; // the ids stored in gnn_dists are w.r.t. the current parent node
					// rather than the dataset itself
					gnn_dists[count].dist = dist;
					++count;
				} else if(dist < gnn_dists[count - 1].dist || (dist == gnn_dists[count - 1].dist &&
					id1 < gnn_dists[count - 1].idx)) {
					gnn_dists[count - 1].idx = id1;
					gnn_dists[count - 1].dist = dist;
				} else {
					continue;
				}

				int id2 = count - 1;
				while(id2 >= 1 && (gnn_dists[id2].dist < gnn_dists[id2 - 1].dist || (gnn_dists[id2].dist == gnn_dists[id2 - 1].dist &&
					gnn_dists[id2].idx < gnn_dists[id2 - 1].idx))){
					swap<int>(&gnn_dists[id2].idx, &gnn_dists[id2 - 1].idx);
					swap<double>(&gnn_dists[id2].dist, &gnn_dists[id2 - 1].dist);
					id2--;
				}
			}
			int m = min(K, nodes[r].size); // no. of connected nodes visited for the current parent node
			if((visited + m) > visited_cap){
				do {
					visited_cap *= 2;
				} while(visited_cap < (visited + m));
				visited_nodes = static_cast<IndxDist*>(realloc(visited_nodes, visited_cap *sizeof(IndxDist)));
			}
			// add the visited nodes of the current parent node to the list of visited nodes
			for(int i = 0; i < m; i++){
				visited_nodes[visited + i].idx = nodes[r].nns_inds[gnn_dists[i].idx];
				visited_nodes[visited + i].dist = gnn_dists[i].dist;
			}
			visited = visited + m;

			if(parent_dist <= gnn_dists[0].dist){
				nn_found = true;
				break;
			}
			r = nodes[r].nns_inds[gnn_dists[0].idx]; // move to the nearest neighbor of the current parent node
			parent_dist = (*dist_func)(query, &dataset[r*n_dims], n_dims);
		}
		if(params.verbose && !nn_found){
			printf("GNN::Maximum steps reached\n");

		}
		qsort(visited_nodes, visited, sizeof(visited_nodes[0]), cmpQsort); //Ascending...
		/**
		next search will start at the nearest neighbor found in this search
		to facilitate faster convergence if the next quey point is similar
		to the current one - a valid assumption with tracking where nearby frames
		exhibit high correlation
		*/
		start_node_idx = visited_nodes[0].idx;
		for(int i = 0; i < K; i++){
			nn_ids[i] = visited_nodes[i].idx;
			nn_dists[i] = visited_nodes[i].dist;
		}
	}

	template <class DistType>
	void GNN<DistType>::saveGraph(const char* saved_graph_path){
		ofstream out_file(saved_graph_path, ios::out | ios::binary);
		if(out_file.good()){
			printf("Saving GNN graph to: %s\n", saved_graph_path);
			out_file.write((char*)(&n_samples), sizeof(int));
			out_file.write((char*)(&n_dims), sizeof(int));
			out_file.write((char*)(nodes.data()), n_samples*sizeof(Node));
			for(int node_id = 0; node_id < n_samples; node_id++){
				out_file.write((char*)(nodes[node_id].nns_inds.data()),
					nodes[node_id].capacity*sizeof(int));
			}
			out_file.close();
		} else{
			printf("Failed to saved GNN graph to: %s\n", saved_graph_path);
		}
	}
	template <class DistType>
	void GNN<DistType>::loadGraph(const char* saved_graph_path){
		ifstream in_file(saved_graph_path, ios::in | ios::binary);
		if(in_file.good()){
			printf("Loading GNN graph from: %s\n", saved_graph_path);
			in_file.read((char*)(&n_samples), sizeof(int));
			in_file.read((char*)(&n_dims), sizeof(int));
			printf("n_samples: %d\n", n_samples);
			printf("n_dims: %d\n", n_dims);
			nodes.resize(n_samples);
			in_file.read((char*)(nodes.data()), n_samples*sizeof(Node));
			for(int node_id = 0; node_id < n_samples; node_id++){
				nodes[node_id].nns_inds.resize(nodes[node_id].capacity);
				in_file.read((char*)(nodes[node_id].nns_inds.data()), nodes[node_id].capacity*sizeof(int));
			}
			in_file.close();
		} else{
			printf("Failed to load GNN graph from: %s\n", saved_graph_path);
		}
	}
	template <class DistType>
	void GNN<DistType>::buildGraph(const double *X, int k){
		nodes.resize(n_samples);
		for(int i = 0; i < n_samples; i++){
			nodes[i].nns_inds.resize(k);
			//   memset(Nodes[i].nns_inds, -1, k*sizeof(int));
			nodes[i].capacity = k;
			nodes[i].size = 0;
		}

		//  struct indx_dist *dists = malloc(n_samples * sizeof(struct indx_dist));
		//  check_pointer(dists, "Couldn't malloc dists");

		std::vector<IndxDist> dists(k + 1);

		//  int *nns_ind = malloc(k*sizeof(int));
		//  check_pointer(nns_ind, "Couldn't malloc nns_ind");
		//  int *query = malloc(n_dims*sizeof(int));
		//  check_pointer(query, "Couldn't malloc query");

		for(int i = 0; i < n_samples; i++){
			knnSearch2(X + (i*n_dims), dists.data(), X, n_samples, n_dims, k + 1);   // index of 1st node is 0
			for(int j = 0; j < k; j++){
				int nns_ind = dists[j + 1].idx;
				addNode(&nodes[i], nns_ind);
			}
		}
	}
	template <class DistType>
	int GNN<DistType>::searchGraph(const double *query, const double *X, int NNs, int K){

		IndxDist *gnn_dists, *knns, *visited_nodes;

		int gnns_cap = K; //NNs*3;
		gnn_dists = static_cast<IndxDist*>(malloc(gnns_cap * sizeof(IndxDist))); // graph nn-dists

		int dist_cnt, depth;
		//tnn_dists = realloc(tnn_dists, K * sizeof(struct indx_dist)); // shrinks the size to K   

		// Graph search time 
		int visited_cap = K * 4;  //avg depth = 4
		int visited = 0;   // number of visited nodes
		visited_nodes = static_cast<IndxDist*>(malloc(visited_cap * sizeof(IndxDist)));

		double parent_dist, dd;

		depth = 0;
		dist_cnt = 0;
		int r = start_node_idx;
		parent_dist = (*dist_func)(query, &X[r*n_dims], n_dims);
		// parent_dist = my_dist(query, x_row, n_dims);

		visited_nodes[0].idx = r;
		visited_nodes[0].dist = parent_dist;
		visited++;

		while(true){
			//      X1 = sample_X(Nodes, r, X); //contains the neighbors of node r
			if(nodes[r].size > gnns_cap) //Nodes[r].size != gnns_size)
			{
				gnns_cap = nodes[r].size;
				gnn_dists = static_cast<IndxDist*>(realloc(gnn_dists, gnns_cap * sizeof(IndxDist)));
			}

			// knn_search(query, gnn_dists, X1, Nodes[r].size, n_dims);
			// knn_search1(query, gnn_dists, X, Nodes[r].size, n_dims, Nodes[r].nns_inds);
			knnSearch11(query, gnn_dists, X, nodes[r].size, n_dims, K, nodes[r].nns_inds.data());

			//      free(X1); 

			int m = min(K, nodes[r].size);
			if((visited + m) > visited_cap){
				do {
					visited_cap *= 2;
				} while(visited_cap < (visited + m));
				visited_nodes = static_cast<IndxDist*>(realloc(visited_nodes, visited_cap *sizeof(IndxDist)));
			}
			for(int i = 0; i < m; i++){
				visited_nodes[visited + i].idx = nodes[r].nns_inds[gnn_dists[i].idx];
				visited_nodes[visited + i].dist = gnn_dists[i].dist;
			}
			visited = visited + m;

			if(parent_dist <= gnn_dists[0].dist)
				break;
			else
				dd = gnn_dists[0].dist;

			r = nodes[r].nns_inds[gnn_dists[0].idx];
			depth++;
			dist_cnt += nodes[r].size;
			// parent_dist = my_dist(query, x_row, n_dims);
			parent_dist = (*dist_func)(query, &X[r*n_dims], n_dims);
		}
		//gnns_size = K;
		//gnn_dists = realloc(gnn_dists, gnns_size * sizeof(struct indx_dist));
		//check_pointer(gnn_dists, "Couldn't realloc gnn_dists");

		// Given visited_nodes and their dists, selects the first knns and puts into gnns_dists
		pickKNNs(visited_nodes, visited, &gnn_dists, K, &gnns_cap);
		start_node_idx = gnn_dists[0].idx;
		return gnn_dists[0].idx;
	}
	template <class DistType>
	void GNN<DistType>::pickKNNs(IndxDist *vis_nodes, int visited,
		IndxDist **gnn_dists, int K, int *gnns_cap){
		qsort(vis_nodes, visited, sizeof(vis_nodes[0]), cmpQsort); //Ascending...
		if(K > *gnns_cap){
			*gnns_cap = K;
			*gnn_dists = static_cast<IndxDist*>(realloc(*gnn_dists, K*sizeof(IndxDist))); // not needed? not sure
		}
		int found = 0, ii = 0, jj = 0;
		(*gnn_dists)[jj].idx = vis_nodes[ii].idx;
		(*gnn_dists)[jj].dist = vis_nodes[ii].dist;
		ii++; jj++;

		while(jj < K){
			//  i++;
			found = 0;
			for(int j = 0; j < jj; j++)
				if((*gnn_dists)[j].idx == vis_nodes[ii].idx){
					found = 1; break;
				}

			if(found){
				ii++; continue;
			} else{
				(*gnn_dists)[jj].idx = vis_nodes[ii].idx;
				(*gnn_dists)[jj].dist = vis_nodes[ii].dist;
				jj++; ii++;
			}
		}
	}

	template <class DistType>
	void GNN<DistType>::knnSearch2(const double *Q, IndxDist *dists,
		const double *X, int rows, int cols, int k){
		// Faster version of knn_search
		// Calculates the distance of query to all data points in X and returns the sorted dist array
		/*  for (i=0; i<rows; i++)
		{
		dists[i].dist = (*dist_func)(Q, X+i*cols, cols);
		dists[i].idx = i;
		}
		mergesort(dists, 0, rows-1);
		*/
		int index, ii, count = 0;
		//int capacity = k;
		for(index = 0; index < rows; index++){
			const double *point = X + index*cols;
			for(ii = 0; ii < count; ++ii) {
				if(dists[ii].idx == ii) continue; //return false;
			}
			//addPoint(point);
			double dist = (*dist_func)(Q, point, cols);
			if(count < k){
				dists[count].idx = index;
				dists[count].dist = dist;
				++count;
			} else if(dist < dists[count - 1].dist || (dist == dists[count - 1].dist &&
				index < dists[count - 1].idx)) {
				//         else if (dist < dists[count-1]) {
				dists[count - 1].idx = index;
				dists[count - 1].dist = dist;
			} else {
				continue;   //  return false;
			}

			int i = count - 1;
			while(i >= 1 && (dists[i].dist < dists[i - 1].dist || (dists[i].dist == dists[i - 1].dist &&
				dists[i].idx < dists[i - 1].idx))){
				swap<int>(&dists[i].idx, &dists[i - 1].idx);
				swap<double>(&dists[i].dist, &dists[i - 1].dist);
				i--;
			}

			//return false;
		}
	}

	template <class DistType>
	void GNN<DistType>::knnSearch11(const double *Q, IndxDist *dists, const double *X,
		int rows, int cols, int k, int *X_inds){
		// Faster version of knn_search1
		// Calculates the distance of query to all data points in X and returns the sorted dist array

		int count = 0;
		for(int i = 0; i < rows; i++){

			//for(int j = 0; j < count; ++j) {
			//	if(dists[j].idx == j) continue; //return false;
			//}
			//printf("X_inds[%d]: %d\n", i, X_inds[i]);
			const double *point = X + X_inds[i] * cols;
			double dist = (*dist_func)(Q, point, cols);
			if(count < k){
				dists[count].idx = i;
				dists[count].dist = dist;
				++count;
			} else if(dist < dists[count - 1].dist || (dist == dists[count - 1].dist &&
				i < dists[count - 1].idx)) {
				//         else if (dist < dists[count-1]) {
				dists[count - 1].idx = i;
				dists[count - 1].dist = dist;
			} else {
				continue;      //  return false;
			}

			int ii = count - 1;
			while(ii >= 1 && (dists[ii].dist < dists[ii - 1].dist || (dists[ii].dist == dists[ii - 1].dist &&
				dists[ii].idx < dists[ii - 1].idx))){
				swap<int>(&dists[ii].idx, &dists[ii - 1].idx);
				swap<double>(&dists[ii].dist, &dists[ii - 1].dist);
				ii--;
			}
		}
	}

	template <class DistType>
	void GNN<DistType>::addNode(Node *node_i, int nn){
		int size = node_i->size++;
		if(size >= node_i->capacity){
			//node_i->nns_inds = static_cast<int*>(realloc(node_i->nns_inds, /*size*2*/ (size + 10)*sizeof(int)));
			node_i->nns_inds.resize(size + 10);
			node_i->capacity = /*size*2*/ size + 10;
		}
		node_i->nns_inds[size] = nn;
	}
}

_MTF_END_NAMESPACE
//! for NT version of NN
#include "mtf/AM/AppearanceModel.h"
template class mtf::gnn::GNN<mtf::AMDist>;
#ifndef ENABLE_ONLY_NT
//! for standard version of NN
#include "mtf/Macros/register.h"
_REGISTER_TRACKERS_DIST(gnn::GNN);
#endif
