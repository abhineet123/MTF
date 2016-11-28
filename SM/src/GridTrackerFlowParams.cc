#include "mtf/SM/GridTrackerFlowParams.h"

_MTF_BEGIN_NAMESPACE


GridTrackerFlowParams::GridTrackerFlowParams(
int _grid_size_x, int _grid_size_y,
int _search_window_x, int _search_window_y,
int _pyramid_levels, bool _use_const_grad,
double _min_eig_thresh, int _max_iters,
double _epsilon, bool _show_trackers,
bool _debug_mode) :
grid_size_x(_grid_size_x),
grid_size_y(_grid_size_y),
search_window_x(_search_window_x),
search_window_y(_search_window_y),
pyramid_levels(_pyramid_levels),
use_const_grad(_use_const_grad),
min_eig_thresh(_min_eig_thresh),
max_iters(_max_iters),
epsilon(_epsilon),
show_trackers(_show_trackers),
debug_mode(_debug_mode){
	resx = grid_size_x;
	resy = grid_size_y;
}
GridTrackerFlowParams::GridTrackerFlowParams(const GridTrackerFlowParams *params) :
grid_size_x(GTFLOW_GRID_SIZE_X),
grid_size_y(GTFLOW_GRID_SIZE_Y),
search_window_x(GTFLOW_SEARCH_WINDOW_X),
search_window_y(GTFLOW_SEARCH_WINDOW_Y),
pyramid_levels(GTFLOW_PYRAMID_LEVELS),
use_const_grad(GTFLOW_USE_MIN_EIG_VALS),
min_eig_thresh(GTFLOW_MIN_EIG_THRESH),
max_iters(GTFLOW_MAX_ITERS), epsilon(GTFLOW_EPSILON),
show_trackers(GTFLOW_SHOW_TRACKERS),
debug_mode(GTFLOW_DEBUG_MODE){
	if(params){
		grid_size_x = params->grid_size_x;
		grid_size_y = params->grid_size_y;
		search_window_x = params->search_window_x;
		search_window_y = params->search_window_y;
		pyramid_levels = params->pyramid_levels;
		use_const_grad = params->use_const_grad;
		min_eig_thresh = params->min_eig_thresh;
		max_iters = params->max_iters;
		epsilon = params->epsilon;
		show_trackers = params->show_trackers;
		debug_mode = params->debug_mode;
	}
	resx = grid_size_x;
	resy = grid_size_y;
}

_MTF_END_NAMESPACE
