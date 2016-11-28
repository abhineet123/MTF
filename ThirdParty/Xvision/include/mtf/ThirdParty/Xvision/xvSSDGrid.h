#ifndef MTF_XVSSDGRID
#define MTF_XVSSDGRID

#include "mtf/TrackerBase.h"

#include "xvSSDTrans.h"
#include "xvSSDAffine.h"
#include "xvSSDSE2.h"
#include "xvSSDRT.h"
#include "xvSSDRotate.h"
#include "xvSSDPyramidTrans.h"
#include "xvSSDPyramidAffine.h"
#include "xvSSDPyramidSE2.h"
#include "xvSSDPyramidRT.h"
#include "xvSSDPyramidRotate.h"
#include "xvColor.h"
#include "xvSSDHelper.h"
#include "common.h"

#define DEF_GRID_SIZE 4
#define DEF_PATCH_SIZE 16
#define DEF_SPF 20
#define DEF_TRACKER XV_TRANS

double *vals_to_sort;

bool compareTrackerWts(int i, int j) {
	return (vals_to_sort[i] < vals_to_sort[j]);
}

inline void sortIndexes(double* vals, vector<int> &idx){
	vals_to_sort = vals;
	std::sort(idx.begin(), idx.end(), compareTrackerWts);
}

struct cvLineSeg {
	Point2d start_pt;
	Point2d end_pt;
};

using namespace mtf;

class XVSSDGrid : public TrackerBase {

public:

	typedef  XVSSDMain::ParamType ParamType;

	XVSSDMain** trackers;
	int tracker_type;
	int grid_size_x;
	int grid_size_y;
	int patch_size;
	int no_of_trackers;
	int trackers_to_reset;
	int steps_per_frame;
	int reset_pos;
	int reset_template;
	int reset_wts;
	int adjust_lines;
	int update_wts;
	int debug_mode;

	int corner_tracker_ids[4];

	double sel_reset_thresh;

	double *tracker_wts;
	double *tracker_wts_x;
	double *tracker_wts_y;
	vector<int> sorted_idx;

	IMAGE_TYPE * xv_frame;
	Point2d *tracker_pos;
	double *grid_pos_x;
	double *grid_pos_y;
	Point2d mean_pt;
	Point2d wt_mean_pt;

	cvLineSeg *vert_lines;
	cvLineSeg *horz_lines;


	int init_size_x;
	int init_size_y;

	int is_initialized;

	void setImage(const cv::Mat &img) override{
		if(!xv_frame){
			xv_frame = new IMAGE_TYPE(img.cols, img.rows);
		}
		xv_frame->remap((PIX_TYPE*)(img.data), false);
	}
	int inputType() const  override{ return CV_8UC3; }

	XVSSDGrid(XVParams *xv_params,
		int tracker_type = DEF_TRACKER, int grid_size_x = DEF_GRID_SIZE, int grid_size_y = -1,
		int patch_size = DEF_PATCH_SIZE, int reset_pos = 0, int reset_template = 0, 
		double sel_reset_thresh = 1.0, int reset_wts = 0, int adjust_lines = 0, 
		int update_wts = 0, int debug_mode = 0) : TrackerBase() {

		this->grid_size_x = grid_size_x;
		this->grid_size_y = grid_size_y;
		this->tracker_type = tracker_type;
		this->patch_size = patch_size;
		this->steps_per_frame = steps_per_frame;
		this->reset_pos = reset_pos;
		this->reset_template = reset_template;
		this->sel_reset_thresh = sel_reset_thresh;
		this->reset_wts = reset_wts;
		this->adjust_lines = adjust_lines;
		this->update_wts = update_wts;
		this->debug_mode = debug_mode;

		if(this->grid_size_y < 0)
			this->grid_size_y = this->grid_size_x;

		name = "ssd_grid";
		no_of_trackers = this->grid_size_x * this->grid_size_y;

		trackers_to_reset = sel_reset_thresh*no_of_trackers;

		printf("%d out of %d trackers will be reset\n", trackers_to_reset, no_of_trackers);

		trackers = new XVSSDMain*[no_of_trackers];
		tracker_pos = new Point2d[no_of_trackers];
		grid_pos_x = new double[no_of_trackers];
		grid_pos_y = new double[no_of_trackers];
		tracker_wts = new double[no_of_trackers];
		tracker_wts_x = new double[no_of_trackers];
		tracker_wts_y = new double[no_of_trackers];

		vert_lines = new cvLineSeg[this->grid_size_x];
		horz_lines = new cvLineSeg[this->grid_size_y];

		corner_tracker_ids[0] = 0;
		corner_tracker_ids[1] = this->grid_size_x - 1;
		corner_tracker_ids[3] = no_of_trackers - this->grid_size_x;
		corner_tracker_ids[2] = no_of_trackers - 1;

		is_initialized = 0;

		int no_of_levels = 2;
		double scale = 0.5;

		for(int i = 0; i < no_of_trackers; i++) {
			switch(tracker_type) {
			case XV_TRANS: {
				trackers[i] = new XVSSDTrans(xv_params);
				break;
			}
			case XV_AFFINE: {
				trackers[i] = new XVSSDAffine(xv_params);
				break;
			}
			case XV_SE2: {
				trackers[i] = new XVSSDSE2(xv_params);
				break;
			}
			case XV_RT: {
				trackers[i] = new XVSSDRT(xv_params);
				break;
			}
			case XV_ROTATE: {
				trackers[i] = new XVSSDRotate(xv_params);
				break;
			}
			case XV_PYRAMID_TRANS: {
				trackers[i] = new XVSSDPyramidTrans(xv_params);
				break;
			}
			case XV_PYRAMID_AFFINE: {
				trackers[i] = new XVSSDPyramidAffine(xv_params);
				break;
			}
			case XV_PYRAMID_SE2: {
				trackers[i] = new XVSSDPyramidSE2(xv_params);
				break;
			}
			case XV_PYRAMID_RT: {
				trackers[i] = new XVSSDPyramidRT(xv_params);
				break;
			}
			case XV_PYRAMID_ROTATE: {
				trackers[i] = new XVSSDPyramidRotate(xv_params);
				break;
			}
			default:
				printf("Invalid tracker type: %d specified\n", tracker_type);
			}
			tracker_wts[i] = 1.0 / no_of_trackers;
			tracker_wts_x[i] = tracker_wts_y[i] = tracker_wts[i];
			sorted_idx.push_back(i);
		}
		xv_frame = NULL;
	}

	~XVSSDGrid() {
		for(int i = 0; i < no_of_trackers; i++) {
			delete(trackers[i]);
		}
		delete(trackers);
		delete(tracker_pos);
		delete(tracker_wts);
		delete(tracker_wts_x);
		delete(tracker_wts_y);
		delete(grid_pos_x);
		delete(grid_pos_y);
	}
	void initialize(const Mat& cv_corners){
		double pos_x = (cv_corners.at<double>(0, 0) + cv_corners.at<double>(0, 1) +
			cv_corners.at<double>(0, 2) + cv_corners.at<double>(0, 3)) / 4;
		double pos_y = (cv_corners.at<double>(1, 0) + cv_corners.at<double>(1, 1) +
			cv_corners.at<double>(1, 2) + cv_corners.at<double>(1, 3)) / 4;
		double size_x = ((cv_corners.at<double>(0, 1) - cv_corners.at<double>(0, 0)) + (cv_corners.at<double>(0, 2) - cv_corners.at<double>(0, 3))) / 2;
		double size_y = ((cv_corners.at<double>(1, 3) - cv_corners.at<double>(1, 0)) + (cv_corners.at<double>(1, 2) - cv_corners.at<double>(1, 1))) / 2;
		initialize(pos_x, pos_y, size_x, size_y);
	}

	void initialize(double pos_x, double pos_y, double size_x, double size_y,
		IMAGE_TYPE *xv_frame_in = NULL) {
		printf("Using Grid SSD Tracker with:\n\t");
		printf("grid_size_x=%d\n\t", grid_size_x);
		printf("grid_size_y=%d\n\t", grid_size_y);
		printf("tracker_type=%c\n\t", tracker_type);
		printf("steps_per_frame=%d\n\t", steps_per_frame);
		printf("reset_pos=%d\n\t", reset_pos);
		printf("reset_template = %d\n\t", reset_template);
		printf("sel_reset_thresh = %f\n\t", sel_reset_thresh);
		printf("reset_wts = %d\n\t", reset_wts);
		printf("adjust_lines = %d\n\t", adjust_lines);
		printf("update_wts = %d\n\t", update_wts);
		printf("debug_mode = %d\n", debug_mode);

		if(xv_frame_in)
			xv_frame = xv_frame_in;

		init_size_x = size_x;
		init_size_y = size_y;
		wt_mean_pt.x = mean_pt.x = pos_x;
		wt_mean_pt.y = mean_pt.y = pos_y;

		updateGridPositions(pos_x, pos_y, size_x, size_y);

		for(int i = 0; i < no_of_trackers; i++) {
			trackers[i]->initialize(grid_pos_x[i], grid_pos_y[i], patch_size, patch_size, xv_frame);
			tracker_pos[i].x = grid_pos_x[i];
			tracker_pos[i].y = grid_pos_y[i];
		}
		updateCVCorners();
		is_initialized = 1;
	}

	void update(){ update(NULL); }

	void update(IMAGE_TYPE *xv_frame_in) {
		if(xv_frame_in)
			xv_frame = xv_frame_in;
		long sum_x = 0;
		long sum_y = 0;
		double wt_sum_x = 0;
		double wt_sum_y = 0;

		for(int i = 0; i < no_of_trackers; i++) {
			if(debug_mode){
				printf("\tPoint2d: %d pos: (%12.6f, %12.6f) wt: %12.6f wt_x: %12.6f wt_y: %12.6f\n", i, tracker_pos[i].x, tracker_pos[i].y,
					tracker_wts[i], tracker_wts_x[i], tracker_wts_y[i]);
			}
			trackers[i]->update(xv_frame);
			tracker_pos[i].x = (trackers[i]->corners[0].PosX() + trackers[i]->corners[2].PosX()) / 2;
			tracker_pos[i].y = (trackers[i]->corners[0].PosY() + trackers[i]->corners[2].PosY()) / 2;
			sum_x += tracker_pos[i].x;
			sum_y += tracker_pos[i].y;
			wt_sum_x += tracker_wts[i] * tracker_pos[i].x;
			wt_sum_y += tracker_wts[i] * tracker_pos[i].y;
		}
		mean_pt.x = sum_x / no_of_trackers;
		mean_pt.y = sum_y / no_of_trackers;
		wt_mean_pt.x = wt_sum_x;
		wt_mean_pt.y = wt_sum_y;
		if(debug_mode){
			printf("\nmean_pt: (%12.6f, %12.6f) wt_mean_pt: (%12.6f, %12.6f)\n", mean_pt.x, mean_pt.y, wt_mean_pt.x, wt_mean_pt.y);
		}

		updateGridPositions(wt_mean_pt.x, wt_mean_pt.y, init_size_x, init_size_y);
		if(update_wts){
			updateTrackerWeights();
		}
		sortIndexes(tracker_wts, sorted_idx);
		resetTrackerStates(wt_mean_pt.x, wt_mean_pt.y, init_size_x, init_size_y);

		if(debug_mode){
			printWeights();
		}
		if(adjust_lines){
			updateHorzLines();
			updateVertLines();
		}
		updateCVCorners();
	}

	inline void printWeights(){
		printf("Tracker weights:\n");
		for(int i = 0; i < no_of_trackers; i++) {
			int idx = sorted_idx[i];
			printf("\tTracker %d\t idx: %3d\t wt: %12.6f\n", i, idx, tracker_wts[idx]);
		}
	}

	inline void updateHorzLines() {
		for(int line_id = 0; line_id < grid_size_y; line_id++) {
			int sum_y = 0;
			int start_id = line_id * grid_size_x;
			for(int i = 0; i < grid_size_x; i++) {
				sum_y += tracker_pos[start_id + i].y;
			}
			int mean_y = sum_y / grid_size_x;
			horz_lines[line_id].start_pt.y = mean_y;
			horz_lines[line_id].end_pt.y = mean_y;

			horz_lines[line_id].start_pt.x = tracker_pos[start_id].x;
			horz_lines[line_id].end_pt.x = tracker_pos[start_id + grid_size_x - 1].x;
		}
	}

	inline void updateVertLines() {
		for(int line_id = 0; line_id < grid_size_x; line_id++) {
			int sum_x = 0;
			for(int i = 0; i < grid_size_y; i++) {
				int pt_id = line_id + i * grid_size_x;
				sum_x += tracker_pos[pt_id].x;
			}
			int mean_x = sum_x / grid_size_y;
			vert_lines[line_id].start_pt.x = mean_x;
			vert_lines[line_id].end_pt.x = mean_x;

			vert_lines[line_id].start_pt.y = tracker_pos[line_id].y;
			vert_lines[line_id].end_pt.y = tracker_pos[line_id + grid_size_x * (grid_size_y - 1)].y;
		}
	}

	inline void updateGridPositions(double pos_x, double pos_y, double size_x, double size_y) {
		double ulx = pos_x - size_x / 2;
		double uly = pos_y - size_y / 2;

		//int tracker_dist_x=size_x/grid_size_x;
		//int tracker_dist_y=size_y/grid_size_y;
		//int start_x=ulx+tracker_dist_x/2;
		//int start_y=uly+tracker_dist_y/2;

		double tracker_dist_x = size_x / (grid_size_x - 1);
		double tracker_dist_y = size_y / (grid_size_x - 1);
		double start_x = ulx;
		double start_y = uly;

		for(int i = 0; i < no_of_trackers; i++) {
			int grid_id_x = i % grid_size_x;
			int grid_id_y = i / grid_size_x;

			grid_pos_x[i] = start_x + grid_id_x * tracker_dist_x;
			grid_pos_y[i] = start_y + grid_id_y * tracker_dist_y;
		}
	}

	void resetTrackerStates(double pos_x, double pos_y, double size_x, double size_y) {
		for(int i = 0; i < trackers_to_reset; i++) {
			int idx = sorted_idx[i];
			if(reset_template) {
				trackers[idx]->resetTrackerTemplate(xv_frame, grid_pos_x[idx], grid_pos_y[idx], patch_size, patch_size);
				if(reset_wts){
					tracker_wts[idx] = tracker_wts[sorted_idx[no_of_trackers - 1]];
					tracker_wts_x[idx] = tracker_wts_y[idx] = tracker_wts[idx];
				}
				//tracker_pos[idx].x=grid_pos_x[idx];
				//tracker_pos[idx].y=grid_pos_y[idx];
			} else if(reset_pos) {
				trackers[idx]->resetTrackerPosition(grid_pos_x[idx], grid_pos_y[idx]);
				if(reset_wts){
					tracker_wts[idx] = tracker_wts[sorted_idx[no_of_trackers - 1]];
					tracker_wts_x[idx] = tracker_wts_y[idx] = tracker_wts[idx];
				}
				//tracker_pos[idx].x=grid_pos_x[idx];
				//tracker_pos[idx].y=grid_pos_y[idx];
			} else{
				return;
			}
		}
		if(reset_wts){
			normalizeWeights();
		}
	}

	inline void normalizeWeights(){
		double wt_sum = 0.0, wt_x_sum = 0.0, wt_y_sum = 0.0;
		for(int i = 0; i < no_of_trackers; i++) {
			wt_sum += tracker_wts[i];
			wt_x_sum += tracker_wts_x[i];
			wt_y_sum += tracker_wts_y[i];
		}
		for(int i = 0; i < no_of_trackers; i++) {
			tracker_wts[i] /= wt_sum;
			tracker_wts_x[i] /= wt_x_sum;
			tracker_wts_y[i] /= wt_y_sum;
		}
	}
	inline void updateTrackerWeights() {

		double wt_sum = 0.0, wt_sum_x = 0.0, wt_sum_y = 0.0;
		for(int i = 0; i < no_of_trackers; i++) {

			double dist_x = fabs(grid_pos_x[i] - tracker_pos[i].x);
			double dist_y = fabs(grid_pos_y[i] - tracker_pos[i].y);

			tracker_wts_x[i] = 1.0 / (1.0 + dist_x);
			tracker_wts_y[i] = 1.0 / (1.0 + dist_y);
			tracker_wts[i] = 1.0 / (1.0 + dist_x + dist_y);

			wt_sum += tracker_wts[i];
			wt_sum_x += tracker_wts_x[i];
			wt_sum_y += tracker_wts_y[i];
		}
		for(int i = 0; i < no_of_trackers; i++) {
			tracker_wts[i] /= wt_sum;
			tracker_wts_x[i] /= wt_sum_x;
			tracker_wts_y[i] /= wt_sum_y;
			//printf("Tracker %d wt: %f\n", i, tracker_wts[i]);
		}

	}
	inline void updateCVCorners() {
		for(int c = 0; c < NCORNERS; c++) {
			/*cout<<"Corner "<<c<<":\t";
			cout<<corners[c].PosX()<<" "<<corners[c].PosY()<<"\n";*/
			cv_corners[c] = tracker_pos[corner_tracker_ids[c]];
		}
	}

};
#endif