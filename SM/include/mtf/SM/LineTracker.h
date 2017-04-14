#ifndef MTF_LINE_TRACKER_H
#define MTF_LINE_TRACKER_H

#include "CompositeBase.h"
#include "mtf/Macros/common.h"

#define LINE_GRID_SIZE_X 5
#define LINE_GRID_SIZE_Y 5
#define LINE_PATCH_SIZE 25
#define LINE_USE_CONSTANT_SLOPE false
#define LINE_USE_LS false
#define LINE_INTER_ALPHA_THRESH 0.1
#define LINE_INTRA_ALPHA_THRESH 0.05
#define LINE_RESET_POS false
#define LINE_RESET_TEMPLATE false
#define LINE_DEBUG_MODE false

#define INF_VAL 1e5
#define TINY_VAL 1e-5
#define SMALL_VAL 2

#define PI_RAD 3.14159265358979323846

_MTF_BEGIN_NAMESPACE

struct lineParams {
	double m;
	double c;
	double c_diff;
	double theta;
	int is_vert;
	lineParams(double _m = 0, double _c = 0, double _c_diff=0,
		int _is_vert = 0) {
		m = _m;
		c = _c;
		c_diff = _c_diff;
		is_vert = _is_vert;
	}
};

struct gridPoint {
	double x;
	double y;
	double tracker_x;
	double tracker_y;
	double xx;
	double xy;

	double wt;
	double wt_y;
	double wt_x;
	double alpha_wt[2];
	double alpha[2];

	double inter_alpha_diff[2];
	double intra_alpha_diff[2];

	int is_successful[2];

};

struct gridLine {
	int no_of_pts;
	lineParams *params;
	gridPoint **pts;

	gridPoint *start_pt;
	gridPoint *end_pt;

	int type;

	int inited;
	int is_successful;

	gridLine();
	~gridLine() {
		if(inited) {
			delete(params);
			delete(pts);
		}
	}

	void init(int no_of_pts, int type);
	void assignAddrs(gridPoint *start_addr = nullptr, int offset_diff = 1);
	void updateParams();
};
struct LineTrackerParams {
	int grid_size_x, grid_size_y;
	int patch_size;
	int use_constant_slope;
	int use_ls;
	double inter_alpha_thresh;
	double intra_alpha_thresh;
	int reset_pos;
	int reset_template;
	int debug_mode;
	LineTrackerParams(int grid_size_x, int grid_size_y,
		int patch_size, bool use_constant_slope, bool use_ls,
		double inter_alpha_thresh, double intra_alpha_thresh,
		bool reset_pos, bool reset_template, bool debug_mode);
	LineTrackerParams(const LineTrackerParams *params = nullptr);
};

class LineTracker : public CompositeBase {

public:

	typedef  LineTrackerParams ParamType;

	LineTracker(const vector<TrackerBase*> trackers,
	 const ParamType *line_params = nullptr);
	~LineTracker(){}
	void initialize(const cv::Mat& cv_corners) override;
	void update() override;

private:
	ParamType params;

	int corner_tracker_ids[4];
	int frame_id;

	cv::Point2d *tracker_pos;
	cv::Point2d wt_mean_pt;

	gridPoint *curr_grid_pts;
	gridPoint *prev_grid_pts;

	gridLine *curr_vert_lines;
	gridLine *curr_horz_lines;
	gridLine *prev_vert_lines;
	gridLine *prev_horz_lines;

	lineParams *mean_horz_params;
	lineParams *mean_vert_params;

	int is_initialized;

	void initGridPositions(const cv::Mat& cv_corners);
	void updateAlpha(gridLine* curr_lines, gridLine* prev_lines, 
		int no_of_lines, int line_type);
	 int updateLineParamsLS(gridLine* curr_lines, gridLine* prev_lines, lineParams *mean_params,
		 int no_of_lines, int line_type);
	 int updateLineParamsWeightedLS(gridLine* curr_lines, gridLine* prev_lines, lineParams *mean_params,
		 int no_of_lines, int line_type);
	void resetLineParamsToPrev(gridLine* curr_lines,
		gridLine* prev_lines, int no_of_lines);
	 void resetLineParamsToMean(gridLine* curr_lines, gridLine* prev_lines, lineParams* mean_params,
		 int no_of_lines, int reset_c = 0);
	 void updateGridWithLineIntersections();

	 void updateDistanceWeights();
	 void resetTrackerStates();
	 void updateCVCorners();
};
_MTF_END_NAMESPACE
#endif