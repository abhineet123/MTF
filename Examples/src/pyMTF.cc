#include <Python.h>
#include <numpy/arrayobject.h>

#include "mtf/mtf.h"
#include "mtf/Config/parameters.h"
#include "mtf/Config/datasets.h"
#include "mtf/Utilities/miscUtils.h"
// tools for reading in images from various sources like image sequences, 
// videos and cameras as well as for pre processing them
#include "mtf/Tools/pipeline.h"

#include <time.h>
#include <string.h>
#include <vector>
#include <memory>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#ifdef _WIN32
#define start_input_timer() \
	clock_t start_time_with_input = clock()
#define start_tracking_timer() \
	clock_t start_time = clock()
#define end_both_timers(fps, fps_win) \
	clock_t end_time = clock();\
	fps = CLOCKS_PER_SEC / static_cast<double>(end_time - start_time);\
	fps_win = CLOCKS_PER_SEC / static_cast<double>(end_time - start_time_with_input)
#else
#define start_input_timer() \
	timespec start_time_with_input;\
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start_time_with_input)
#define start_tracking_timer() \
	timespec start_time;\
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start_time)
#define end_both_timers(fps, fps_win) \
	timespec end_time;\
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end_time);\
	fps = 1.0 / ((double)(end_time.tv_sec - start_time.tv_sec) +\
		1e-9*(double)(end_time.tv_nsec - start_time.tv_nsec));\
	fps_win = 1.0 / ((double)(end_time.tv_sec - start_time_with_input.tv_sec) +\
		1e-9*(double)(end_time.tv_nsec - start_time_with_input.tv_nsec))
#endif

using namespace std;
using namespace mtf::params;


static PyObject* initialize(PyObject* self, PyObject* args);
static PyObject* update(PyObject* self, PyObject* args);
static PyObject* setRegion(PyObject* self, PyObject* args);
static PyObject* getRegion(PyObject* self, PyObject* args);

static PyArrayObject *img_py;
static PyArrayObject *init_corners_py;
static PyArrayObject *out_corners_py;
static double* out_corners_data;

typedef unique_ptr<mtf::TrackerBase> Tracker_;
static vector<Tracker_> trackers;
static vector<PreProc_> pre_proc;

static int img_height, img_width;
static vector<cv::Scalar> obj_cols;

static cv::Point fps_origin(10, 20);
static double fps_font_size = 0.50;
static cv::Scalar fps_color(0, 255, 0);
static char fps_text[100];
static int frame_id;
double fps, fps_win;
static char* config_root_dir = "C++/MTF/Config";



static PyMethodDef pyMTFMethods[] = {
	{ "initialize", initialize, METH_VARARGS },
	{ "update", update, METH_VARARGS },
	{ "setRegion", setRegion, METH_VARARGS },
	{ "getRegion", getRegion, METH_VARARGS },
	{ NULL, NULL }     /* Sentinel - marks the end of this structure */
};

/* ==== Initialize the C_test functions ====================== */
// Module name must be pyMTF in compile and linked
PyMODINIT_FUNC initpyMTF()  {
	(void)Py_InitModule("pyMTF", pyMTFMethods);
	import_array();  // Must be present for NumPy.  Called first after above line.
}

/* ==== initialize tracker ==== */

static PyObject* initialize(PyObject* self, PyObject* args) {

	/*parse first input array*/
	if(!PyArg_ParseTuple(args, "O!O!z", &PyArray_Type, &img_py, &PyArray_Type, &init_corners_py, &config_root_dir)) {
		printf("\n----pyMTF::initialize: Input arguments could not be parsed----\n\n");
		return NULL;
	}

	if(img_py == NULL) {
		printf("\n----pyMTF::initialize::init_img is NULL----\n\n");
		return NULL;
	}
	if(init_corners_py == NULL) {
		printf("\n----pyMTF::initialize::init_corners is NULL----\n\n");
		return NULL;
	}

	if(init_corners_py->dimensions[0] != 2 || init_corners_py->dimensions[1] != 4){
		printf("pyMTF::Initial corners matrix has incorrect dimensions: %ld, %ld\n",
			init_corners_py->dimensions[0], init_corners_py->dimensions[1]);
		return NULL;
	}
	if(!config_root_dir){
		config_root_dir = "C++/MTF/Config";
		printf("Using default configuration folder: %s\n", config_root_dir);
	} else{
		printf("Reading MTF configuration files from: %s\n", config_root_dir);
	}
#ifdef USE_TBB
	Eigen::initParallel();
#endif

	img_height = img_py->dimensions[0];
	img_width = img_py->dimensions[1];

	printf("img_height: %d\n", img_height);
	printf("img_width: %d\n", img_width);

	cv::Mat init_img_cv(img_height, img_width, CV_8UC3, img_py->data);

	cv::Mat temp(2, 4, CV_64FC1, init_corners_py->data);
	cv::Mat init_corners_cv(2, 4, CV_64FC1);

	//double *in_corners_data = (double*)init_corners_py->data;
	//for(int i = 0; i < 4; i++) {
	//	init_corners_cv.at<double>(0, i) = in_corners_data[i];
	//	init_corners_cv.at<double>(1, i) = in_corners_data[i + init_corners_py->strides[1]];
	//}
	init_corners_cv.at<double>(0, 0) = temp.at<double>(0, 0);
	init_corners_cv.at<double>(1, 0) = temp.at<double>(0, 1);
	init_corners_cv.at<double>(0, 1) = temp.at<double>(0, 2);
	init_corners_cv.at<double>(1, 1) = temp.at<double>(0, 3);
	init_corners_cv.at<double>(0, 2) = temp.at<double>(1, 0);
	init_corners_cv.at<double>(1, 2) = temp.at<double>(1, 1);
	init_corners_cv.at<double>(0, 3) = temp.at<double>(1, 2);
	init_corners_cv.at<double>(1, 3) = temp.at<double>(1, 3);


	printf("init_corners_cv:\n");
	for(int i = 0; i < 4; i++) {
		printf("%d: (%f, %f)\n", i, init_corners_cv.at<double>(0, i), init_corners_cv.at<double>(1, i));
	}

	double min_x = init_corners_cv.at<double>(0, 0);
	double min_y = init_corners_cv.at<double>(1, 0);
	double max_x = init_corners_cv.at<double>(0, 2);
	double max_y = init_corners_cv.at<double>(1, 2);
	double size_x = max_x - min_x;
	double size_y = max_y - min_y;

	if(!readParams(0, nullptr)){ return Py_BuildValue("i", 0); }

	printf("*******************************\n");
	printf("Using parameters:\n");
	printf("n_trackers: %d\n", n_trackers);
	printf("actor_id: %d\n", actor_id);
	printf("source_id: %d\n", source_id);
	printf("source_name: %s\n", source_name.c_str());
	printf("actor: %s\n", actor.c_str());
	printf("pipeline: %c\n", pipeline);
	printf("img_source: %c\n", img_source);
	printf("show_cv_window: %d\n", show_cv_window);
	printf("read_obj_from_gt: %d\n", read_obj_from_gt);
	printf("write_tracking_data: %d\n", write_tracking_data);
	printf("mtf_sm: %s\n", mtf_sm);
	printf("mtf_am: %s\n", mtf_am);
	printf("mtf_ssm: %s\n", mtf_ssm);
	printf("*******************************\n");


	/*********************************** initialize trackers ***********************************/
	if(n_trackers > 1){
		printf("Multi tracker setup enabled\n");
		write_tracking_data = 0;
	}
	if(res_from_size){
		printf("Getting sampling resolution from object size...\n");
	}
	FILE *multi_fid = NULL;
	for(int tracker_id = 0; tracker_id < n_trackers; tracker_id++) {
		if(n_trackers > 1){
			multi_fid = readTrackerParams(multi_fid);
		}
		printf("Using tracker %d with object of size %f x %f\n", tracker_id,
			size_x, size_y);
		if(res_from_size){
			resx = size_x / res_from_size;
			resy = size_y / res_from_size;
		}
		mtf::TrackerBase* new_tracker = mtf::getTracker(mtf_sm, mtf_am, mtf_ssm, mtf_ilm);
		if(!new_tracker){
			printf("Tracker could not be initialized successfully\n");
			return Py_BuildValue("i", 0);
		}
		PreProc_ new_pre_proc = getPreProc(new_tracker->inputType(), pre_proc_type);
		new_pre_proc->initialize(init_img_cv);
		for(PreProc_ curr_obj = new_pre_proc; curr_obj; curr_obj = curr_obj->next){
			new_tracker->setImage(curr_obj->getFrame());
		}
		new_tracker->initialize(init_corners_cv);
		trackers.push_back(Tracker_(new_tracker));
		pre_proc.push_back(new_pre_proc);
	}
	if(show_cv_window) {
		cv::namedWindow("OpenCV Window", cv::WINDOW_AUTOSIZE);
	}
	obj_cols.push_back(cv::Scalar(0, 0, 255));
	obj_cols.push_back(cv::Scalar(0, 255, 0));
	obj_cols.push_back(cv::Scalar(255, 0, 0));
	obj_cols.push_back(cv::Scalar(255, 255, 0));
	obj_cols.push_back(cv::Scalar(255, 0, 255));
	obj_cols.push_back(cv::Scalar(0, 255, 255));
	obj_cols.push_back(cv::Scalar(255, 255, 255));
	obj_cols.push_back(cv::Scalar(0, 0, 0));


	int dims[] = { 2, 4 };
	out_corners_py = (PyArrayObject *)PyArray_FromDims(2, dims, NPY_DOUBLE);
	out_corners_data = (double*)out_corners_py->data;

	frame_id = 0;

	return Py_BuildValue("i", 1);
}

static PyObject* update(PyObject* self, PyObject* args) {

	/*parse first input array*/
	if(!PyArg_ParseTuple(args, "O!", &PyArray_Type, &img_py)) {
		printf("\n----pyMTF::update: Input arguments could not be parsed----\n\n");
		return NULL;
	}

	if(img_py == NULL) {
		printf("\n----pyMTF::img_py is NULL----\n\n");
		return NULL;
	}
	frame_id++;

	cv::Mat curr_img_cv(img_height, img_width, CV_8UC3, img_py->data);
	start_input_timer();
	//update trackers
	for(int tracker_id = 0; tracker_id < n_trackers; tracker_id++) {
		// update pre processed image
		pre_proc[tracker_id]->update(curr_img_cv);
		start_tracking_timer();
		trackers[tracker_id]->update();// this is equivalent to :             
		// trackers[tracker_id]->update(pre_proc->getFrame());        
		// as the image has been passed at the time of initialization 
		// and does not need to be passed again as long as the new 
		// image is read into the same locatioon
		end_both_timers(fps, fps_win);
		if(reset_template){
			trackers[tracker_id]->initialize(trackers[tracker_id]->getRegion());
		}
	}

	cv::Point2d corners[4];
	if(show_cv_window) {
		/* draw tracker positions to OpenCV window */
		for(int tracker_id = 0; tracker_id < n_trackers; tracker_id++) {
			int col_id = tracker_id % obj_cols.size();
			mtf::utils::Corners(trackers[tracker_id]->getRegion()).points(corners);
			line(curr_img_cv, corners[0], corners[1], obj_cols[col_id], line_thickness);
			line(curr_img_cv, corners[1], corners[2], obj_cols[col_id], line_thickness);
			line(curr_img_cv, corners[2], corners[3], obj_cols[col_id], line_thickness);
			line(curr_img_cv, corners[3], corners[0], obj_cols[col_id], line_thickness);
			putText(curr_img_cv, trackers[tracker_id]->name, corners[0],
				cv::FONT_HERSHEY_SIMPLEX, fps_font_size, obj_cols[col_id]);
		}
		snprintf(fps_text, 100, "frame: %d c: %12.6f cw: %12.6f", frame_id, fps, fps_win);
		putText(curr_img_cv, fps_text, fps_origin, cv::FONT_HERSHEY_SIMPLEX, fps_font_size, fps_color);
		imshow("PyMTF", curr_img_cv);
		cv::waitKey(1);
	}
	mtf::utils::Corners(trackers[0]->getRegion()).points(corners);
	//mtf::utils::printMatrix<double>(trackers[0]->getRegion(), "mtf corners");
	for(int corner_id = 0; corner_id < 4; corner_id++) {
		out_corners_data[corner_id] = corners[corner_id].x;
		out_corners_data[corner_id + 4] = corners[corner_id].y;
	}
	return Py_BuildValue("O", out_corners_py);
}
static PyObject* setRegion(PyObject* self, PyObject* args) {
	/*parse first input array*/
	if(!PyArg_ParseTuple(args, "O!", &PyArray_Type, &init_corners_py)) {
		printf("\n----pyMTF::initialize: Input arguments could not be parsed----\n\n");
		return Py_BuildValue("i", 0);
	}

	if(init_corners_py == NULL) {
		printf("\n----pyMTF::initialize::init_corners is NULL----\n\n");
		return Py_BuildValue("i", 0);
	}

	if(init_corners_py->dimensions[0] != 2 || init_corners_py->dimensions[1] != 4){
		printf("pyMTF::Initial corners matrix has incorrect dimensions: %ld, %ld\n",
			init_corners_py->dimensions[0], init_corners_py->dimensions[1]);
		return Py_BuildValue("i", 0);
	}
	cv::Mat corners(2, 4, CV_64FC1, init_corners_py->data);
	trackers[0]->setRegion(corners);
	return Py_BuildValue("i", 1);

}
static PyObject* getRegion(PyObject* self, PyObject* args){
	return Py_BuildValue("O", out_corners_py);
}
