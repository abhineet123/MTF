#ifndef MTF_XVSSDMAIN
#define MTF_XVSSDMAIN

#define STEPS_PER_FRAME 10

#include"mtf/TrackerBase.h"

#include <XVSSD.h>
#include <XVTracker.h>
//#include <XVWindowX.h>
#include<stdio.h>
#include<stdlib.h>
#include<iostream>
#include<string>

#include "common.h"

#define DEBUG_MODE 0

struct XVParams{
	bool show_xv_window;
	int steps_per_frame;
	bool copy_frame;
	bool refresh_win;

	XVParams() : show_xv_window(false), steps_per_frame(STEPS_PER_FRAME),
		copy_frame(false), refresh_win(false){}
	XVParams(int _show_xv_window, double _steps_per_frame,
		bool _copy_frame, bool _refresh_win){
		show_xv_window = _show_xv_window;
		steps_per_frame = _steps_per_frame;
		copy_frame = _copy_frame;
		refresh_win = _refresh_win;
	}
	XVParams(XVParams *params) : show_xv_window(false), steps_per_frame(STEPS_PER_FRAME),
		copy_frame(false), refresh_win(false){
		if(params){
			show_xv_window = params->show_xv_window;
			steps_per_frame = params->steps_per_frame;
			copy_frame = params->copy_frame;
			refresh_win = params->refresh_win;
		}
	}
};

typedef struct AffineState {
	double tx;
	double ty;
	double angle;
	double scale;
	double shear_x;
	double shear_y;
	AffineState(double tx0 = 0, double ty0 = 0, double angle0 = 0,
		double scale0 = 1, double shear_x0 = 0, double shear_y0 = 0) {
		tx = tx0;
		ty = ty0;
		angle = angle0;
		scale = scale0;
		shear_x = shear_x0;
		shear_y = shear_y0;
	}
} TrackerStateAffine;

typedef struct SE2State {
	double tx;
	double ty;
	double angle;
	double scale;
	SE2State(double tx0 = 0, double ty0 = 0, double angle0 = 0, double scale0 = 1) {
		tx = tx0;
		ty = ty0;
		angle = angle0;
		scale = scale0;
	}
	void setState(double tx0 = 0, double ty0 = 0, double angle0 = 0, double scale0 = 1) {
		tx = tx0;
		ty = ty0;
		angle = angle0;
		scale = scale0;
	}
	void writeToFile(ofstream &fout) {

		fout << tx << "\t" << ty << "\t" << angle << "\t" << scale << "\n";
		//fout.width(12);
		//fout.precision(12);
		//fout << std::right << tx << "\t";
		//fout.width(12);
		//fout.precision(12);
		//      fout << std::right << ty << "\t";
		//fout.width(12);
		//fout.precision(12);
		//      fout << std::right << angle << "\t";
		//fout.width(12);
		//fout.precision(12);
		//      fout << std::right << scale << "\n";
	}
} TrackerStateSE2;
using namespace mtf;
typedef TrackerStateSE2 TrackerState;

class XVSSDMain : public TrackerBase {

public:
	typedef XVParams ParamType;

	ParamType params;

	IMAGE_TYPE* xv_frame = NULL;
	IMAGE_TYPE init_template;
	IMAGE_TYPE_GS init_template_gs;
	IMAGE_TYPE_GS xv_frame_gs;
	IMAGE_TYPE_GS warped_img;
	IMAGE_TYPE_GS template_img;


	XVSize *init_size;
	XVPosition *init_pos, *init_pos_min;

	//WIN_INT *win_int = NULL;

	TrackerState *current_tracker_state;
	TrackerState *diff_tracker_state;
	vector<TrackerState> tracker_states;

	int img_height;
	int img_width;

	//int warp_height;
	//int warp_width;

	int no_of_pixels;
	int xv_line_width;

	int no_of_frames;
	int no_of_vals;
	int no_of_pts;
	int xv_row_size;
	int xv_nch;

	vector<int*> init_pixel_vals;
	vector<PIX_TYPE_GS> init_pixel_vals_gs;
	vector<double> curr_pixel_vals_gs;

	vector<double*> ssd_log;
	vector<double*> error_log;
	vector<double*> region_ssd_log;

	int region_thresh;
	int region_size;

	XVPosition corners[NCORNERS];
	XVPositionD points[NCORNERS];

	vector<XVPositionD> init_pts;
	vector<XVPositionD> trans_pts;
	double* current_pts;

	vector<int> init_xv_locs;
	vector<int> current_xv_locs;

	int id;

	int write_frame_data;
	int write_pts;
	char *frame_dir;
	char *pts_dir;

	int write_tracker_states;
	ofstream tracker_state_fout, diff_state_fout;
	char *tracker_state_fname;

	/*virtual functions to be implemented by specific trackers*/
	virtual void initTracker() = 0;
	virtual double updateTrackerState() = 0;
	virtual void updateCorners() = 0;
	virtual void resetTrackerPosition(double pos_x, double pos_y) = 0;
	virtual void resetTrackerTemplate(IMAGE_TYPE *img, double pos_x, double pos_y,
		double size_x, double size_y) = 0;
	virtual void getTransformedPoints(vector<XVPositionD> &in_pts, vector<XVPositionD> &out_pts) = 0;


	/*helper functions*/
	void getInitPoints();
	void getInitPixelValues();
	void getInitPixelValues2();
	void writeTransformedPoints();
	void writeTransformedPointsBin();
	void writeCurrentFrameBin();
	void writeCurrentFrameGS();
	void writeCurrentFrameGSBin();
	void writeInitData();
	void writeParamsBin();
	void printSSDOfRegion(int tracker_id);
	void printSSDOfRegion(double *region_ssd);
	void printSSSDLog(int tracker_id);
	void printErrorLog(int tracker_id);
	void getCurrentPixelValues();
	int* getPixelValue(int x, int y, IMAGE_TYPE* img);
	double* getPixelValue(double x, double y, IMAGE_TYPE* img);
	PIX_TYPE_GS getPixelValueGS(int x, int y, IMAGE_TYPE_GS* img);
	double getPixelValueGS(double x, double y, IMAGE_TYPE_GS* img);
	double getCurrentSSD();
	double getCurrentSSDGS();
	double* getSSDOfRegion();

	XVSSDMain(const ParamType *xv_params = NULL) : TrackerBase(),
		params(xv_params){
		this->xv_line_width = 3;
		this->no_of_frames = 0;
		this->region_thresh = 50;
		this->region_size = (2 * region_thresh + 1) * (2 * region_thresh + 1);
		this->init_pos_min = new XVPosition;
		this->init_pos = new XVPosition;
		this->init_size = new XVSize;
	}

	~XVSSDMain() {
		if(write_tracker_states) {
			delete(tracker_state_fname);
		}
	}

	void setImage(const cv::Mat &img) override{
		if(!xv_frame){
			img_height = img.rows;
			img_width = img.cols;
			xv_frame = new IMAGE_TYPE(img_width, img_height);
		}
		xv_frame->remap((PIX_TYPE*)(img.data), false);
	}
	int inputType() const  override{ return CV_8UC3; }

	void initialize(const Mat& cv_corners){
		double pos_x = (cv_corners.at<double>(0, 0) + cv_corners.at<double>(0, 1) +
			cv_corners.at<double>(0, 2) + cv_corners.at<double>(0, 3)) / 4;
		double pos_y = (cv_corners.at<double>(1, 0) + cv_corners.at<double>(1, 1) +
			cv_corners.at<double>(1, 2) + cv_corners.at<double>(1, 3)) / 4;
		double size_x = ((cv_corners.at<double>(0, 1) - cv_corners.at<double>(0, 0)) + (cv_corners.at<double>(0, 2) - cv_corners.at<double>(0, 3))) / 2;
		double size_y = ((cv_corners.at<double>(1, 3) - cv_corners.at<double>(1, 0)) + (cv_corners.at<double>(1, 2) - cv_corners.at<double>(1, 1))) / 2;
		initialize(pos_x, pos_y, size_x, size_y);
	}

	void initialize(double pos_x, double pos_y,
		double size_x, double size_y,
		IMAGE_TYPE *xv_frame_in = NULL, int id = 0,
		int write_frame_data = 0, int write_pts = 0, int write_tracker_states = 0,
		char *frame_dir = NULL, char *pts_dir = NULL, char* states_dir = NULL) {
		//printf("XVSSDMain::size_x=%f\n", size_x);
		//printf("XVSSDMain::size_y=%f\n", size_y);
		//printf("right after XVSSDMain::init_size:Width=%d\n",init_size->Width());
		//printf("right after XVSSDMain::init_size:Height=%d\n",init_size->Height());

		if(xv_frame_in)
			xv_frame = xv_frame_in;

		if(!frame_dir)
			write_frame_data = 0;

		if(!pts_dir)
			write_pts = 0;

		this->id = id;
		this->write_frame_data = write_frame_data;
		this->write_pts = write_pts;
		this->write_tracker_states = write_tracker_states;
		this->frame_dir = frame_dir;
		this->pts_dir = pts_dir;

		init_pos->setX(pos_x);
		init_pos->setY(pos_y);

		updateTemplate(xv_frame, pos_x, pos_y, size_x, size_y);

		printf("Using %s tracker with:\n\t", name.c_str());
		printf("size_x=%d\n\t", init_size->Width());
		printf("size_y=%d\n\t", init_size->Height());
		printf("pos_x=%d\n\t", init_pos->PosX());
		printf("pos_y=%d\n\t", init_pos->PosY());
		printf("params.steps_per_frame=%d\n", params.steps_per_frame);

		//printf("xv_frame=%lu\n", (unsigned long)xv_frame);
		//printf("xv_frame data=%lu\n", (unsigned long)xv_frame->data());
		/*printf("write_frame_data=%d\n", write_frame_data);
		printf("write_pts=%d\n", write_pts);*/

		img_height = xv_frame->Height();
		img_width = xv_frame->Width();
		no_of_pixels = img_height * img_width;

		/*fprintf(stdout, "img_height=%d\n", img_height);
		fprintf(stdout, "img_width=%d\n", img_width);
		fprintf(stdout, "no_of_pixels=%d\n", no_of_pixels);*/

		//if(params.show_xv_window) {
		//	//fprintf(stdout, "Calling initWindow\n");
		//	initWindow(win_int_in);
		//	//fprintf(stdout, "Calling updateWindow\n");
		//	updateWindow("0.0");
		//}

		XVPositionD tmpPoint = XVPositionD(init_size->Width() / 2,
			init_size->Height() / 2);
		points[0] = -tmpPoint;
		points[1] = XVPositionD(tmpPoint.PosX(), -tmpPoint.PosY());
		points[2] = tmpPoint;
		points[3] = XVPositionD(-tmpPoint.PosX(), tmpPoint.PosY());

		xv_nch = sizeof(PIX_TYPE);
		xv_row_size = xv_frame->SizeX() * xv_nch;


		no_of_pts = init_size->Width() * init_size->Height();
		no_of_vals = 2 * no_of_pts * params.steps_per_frame;
		current_pts = new double[no_of_vals];
		//curr_pixel_vals_gs=new double*[no_of_pts*params.steps_per_frame];

		//printf("XVSSDMain::init_size:Width=%d\n",init_size->Width());
		//printf("XVSSDMain::init_size:Height=%d\n",init_size->Height());

		//printf("Calling initTracker\n");
		current_tracker_state = new TrackerState();
		diff_tracker_state = new TrackerState();
		initTracker();
		updateCorners();
		//printf("Done Calling initTracker\n");

		if(write_tracker_states) {
			tracker_state_fname = new char[100];
			snprintf(tracker_state_fname, 100, "%s/tracker_states_%s_%d.txt", states_dir, name.c_str(), params.steps_per_frame);
			tracker_state_fout.open(tracker_state_fname, ios::out);
			snprintf(tracker_state_fname, 100, "%s/diff_states_%s_%d.txt", states_dir, name.c_str(), params.steps_per_frame);
			diff_state_fout.open(tracker_state_fname, ios::out);
			current_tracker_state->writeToFile(tracker_state_fout);
			diff_tracker_state->writeToFile(diff_state_fout);
		}

		ofstream init_template_fout("log/init_template.txt");
		ofstream init_template_gs_fout("log/init_template_gs.txt");
		ofstream template_img_fout("log/template_img.txt");

		init_template_fout << init_template;
		init_template_gs_fout << init_template_gs;
		template_img_fout << template_img;

		init_template_fout.close();
		init_template_gs_fout.close();
		template_img_fout.close();


		if(write_pts) {
			//printf("pts_dir=%s\n", pts_dir);

			//printf("Calling getInitPoints\n");
			getInitPoints();
			//printf("Calling getInitPixelValues\n");
			getInitPixelValues2();
			//printf("Calling writeParamsBin\n");
			writeParamsBin();
			//printf("Calling writeInitData\n");
			writeInitData();
		}

		if(write_frame_data) {
			//printf("frame_dir=%s\n", frame_dir);

			//printf("Calling writeCurrentFrameGSBin\n");
			writeCurrentFrameBin();			
			writeCurrentFrameGSBin();
		}
		updateCVCorners();
		//printf("Done initializing\n");
	}
	void update(){ update(NULL); }

	void update(IMAGE_TYPE *xv_frame_in, char* fps = "0.0") {
		//printf("Updating with frame %d...\n", no_of_frames);
		if(xv_frame_in)
			xv_frame = xv_frame_in;
		//writeCurrentFrameBin();
		//writeCurrentFrameGS();
		//printf("Calling writeCurrentFrameGSBin...\n");
		//printf("Done\n");
		double* frame_error = new double[params.steps_per_frame];
		//double* frame_ssd=new double[params.steps_per_frame];
		for(int i = 0; i < params.steps_per_frame; i++) {
			frame_error[i] = updateTrackerState();
			if(write_tracker_states) {
				current_tracker_state->writeToFile(tracker_state_fout);
				diff_tracker_state->writeToFile(diff_state_fout);
			}
			/*getTransformedPoints(init_pts, trans_pts);
			int start_id=2*i*no_of_pts;
			for(int j=0;j<no_of_pts;j++){
			current_pts[start_id + 2*j]=trans_pts[j].PosX();
			current_pts[start_id + 2*j+1]=trans_pts[j].PosY();
			}*/
			//writeTransformedPoints();
			//getCurrentPixelValues();

			//frame_ssd[i]=getCurrentSSD();
			//frame_ssd[i]=getCurrentSSDGS();
		}
		updateCorners();
		no_of_frames++;

		if(write_tracker_states) {
			tracker_state_fout << "\n";
			diff_state_fout << "\n";
		}

		if(write_frame_data) {
			writeCurrentFrameBin();
			writeCurrentFrameGSBin();
		}
		if(write_pts) {
			//printf("Calling writeTransformedPointsBin\n");
			writeTransformedPointsBin();
		}
		updateCVCorners();
		//double *region_ssd=getSSDOfRegion();
		//printSSDOfRegion(region_ssd);
		//region_ssd_log.push_back(region_ssd);
		//ssd_log.push_back(frame_ssd);
		//error_log.push_back(frame_error);
	}
	inline void updateTemplate(IMAGE_TYPE *img, double pos_x, double pos_y,
		double size_x, double size_y){
		double min_x = pos_x - size_x / 2.0;
		double min_y = pos_y - size_y / 2.0;
		init_pos_min->setX(min_x);
		init_pos_min->setY(min_y);
		init_size->resize(size_x, size_y);
		XVROI roi(*init_size, *init_pos_min);
		init_template = subimage(*img, roi);
		RGBtoScalar(init_template, init_template_gs);
	}
	inline void scaleCorners(float scale = 1.0) {
		for(int i = 0; i < 4; i++) {
			corners[i].setX((int)(corners[i].x() / scale));
			corners[i].setY((int)(corners[i].y() / scale));
		}
	}
	inline void updateCVCorners() {
		for(int c = 0; c < NCORNERS; c++) {
			/*cout<<"Corner "<<c<<":\t";
			cout<<corners[c].PosX()<<" "<<corners[c].PosY()<<"\n";*/
			cv_corners_mat.at<double>(0, c) = corners[c].PosX();
			cv_corners_mat.at<double>(1, c) = corners[c].PosY();
		}
	}
	inline void closeFiles(){
		if(write_tracker_states){
			tracker_state_fout.close();
			diff_state_fout.close();
		}
	}
};

#endif
