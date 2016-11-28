#ifndef MTF_GRID_TRACKER_FLOW_PARAMS_H
#define MTF_GRID_TRACKER_FLOW_PARAMS_H

#include "mtf/Macros/common.h"

#define GTFLOW_GRID_SIZE_X 10
#define GTFLOW_GRID_SIZE_Y 10
#define GTFLOW_SEARCH_WINDOW_X 10
#define GTFLOW_SEARCH_WINDOW_Y 10
#define GTFLOW_PYRAMID_LEVELS 0
#define GTFLOW_USE_MIN_EIG_VALS 0
#define GTFLOW_MIN_EIG_THRESH 1e-4
#define GTFLOW_MAX_ITERS 30
#define GTFLOW_EPSILON 0.01
#define GTFLOW_SHOW_TRACKERS 0
#define GTFLOW_SHOW_TRACKER_EDGES 0
#define GTFLOW_DEBUG_MODE 0

_MTF_BEGIN_NAMESPACE

struct GridTrackerFlowParams{

	int grid_size_x, grid_size_y;
	int search_window_x, search_window_y;

	int pyramid_levels;
	bool use_const_grad;
	double min_eig_thresh;

	int max_iters; //! maximum iterations of the GridTrackerFlow algorithm to run for each frame
	double epsilon;

	bool show_trackers;// show the locations of individual patch trackers

	bool debug_mode; //! decides whether logging data will be printed for debugging purposes; 
	//! only matters if logging is enabled at compile time

	GridTrackerFlowParams(
		int _grid_size_x, int _grid_size_y,
		int _search_window_x, int _search_window_y,
		int _pyramid_levels, bool _use_const_grad,
		double _min_eig_thresh, int _max_iters,
		double _epsilon, bool _show_trackers,
		bool _debug_mode);
	GridTrackerFlowParams(const GridTrackerFlowParams *params = nullptr);

	int getResX() const{ return resx; }
	int getResY() const{ return resy; }

private:
	int resx, resy;
};

_MTF_END_NAMESPACE

#endif

