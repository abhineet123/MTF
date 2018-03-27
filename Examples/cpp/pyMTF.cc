#ifndef DISABLE_VISP
//! needed to avoid a weird bug in ViSP
#define PNG_SKIP_SETJMP_CHECK
#endif
#ifdef _WIN32
#define hypot _hypot
#endif
#include "mtf/mtf.h"
//! tools for reading in images from various sources like image sequences, 
//! videos and cameras as well as for pre processing them
#include "mtf/pipeline.h"
#include "mtf/Config/parameters.h"
#include "mtf/Utilities/miscUtils.h"

#include <vector>
#include <memory>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <Python.h>
#include <numpy/arrayobject.h>

using namespace std;
using namespace mtf::params;

static PyObject* initInput(PyObject* self, PyObject* args);
static PyObject* create(PyObject* self, PyObject* args);
static PyObject* initialize(PyObject* self, PyObject* args);
static PyObject* update(PyObject* self, PyObject* args);
static PyObject* setRegion(PyObject* self, PyObject* args);
static PyObject* remove(PyObject* self, PyObject* args);

typedef unique_ptr<mtf::TrackerBase> Tracker_;
static std::vector<Tracker_> trackers;
static std::vector<PreProc_> pre_procs;
static std::vector<bool> tracker_initialized;

static PyMethodDef pyMTFMethods[] = {
	{ "create", create, METH_VARARGS },
	{ "initialize", initialize, METH_VARARGS },
	{ "update", update, METH_VARARGS },
	{ "setRegion", setRegion, METH_VARARGS },
	{ "remove", remove, METH_VARARGS },
	{ NULL, NULL }     /* Sentinel - marks the end of this structure */
};

/* ==== Initialize the C_test functions ====================== */
// Module name must be pyMTF in compile and linked
PyMODINIT_FUNC initpyMTF()  {
	(void)Py_InitModule("pyMTF", pyMTFMethods);
	import_array();  // Must be present for NumPy.  Called first after above line.
}

static PyObject* create(PyObject* self, PyObject* args) {
	char* config_root_dir;
	if(!PyArg_ParseTuple(args, "z", &config_root_dir)) {
		PySys_WriteStdout("\n----pyMTF::create: Input arguments could not be parsed----\n\n");
		return Py_BuildValue("i", 0);
	}
#ifdef USE_TBB
	Eigen::initParallel();
#endif
	if(!config_root_dir){
		config_root_dir = "../../Config";
		PySys_WriteStdout("Using default configuration folder: %s\n", config_root_dir);
	} else{
		PySys_WriteStdout("Reading MTF configuration files from: %s\n", config_root_dir);
	}
	config_dir = std::string(config_root_dir);
	if(!readParams(0, nullptr)){ return Py_BuildValue("i", 0); }

	PySys_WriteStdout("*******************************\n");
	PySys_WriteStdout("Using parameters:\n");
	PySys_WriteStdout("mtf_sm: %s\n", mtf_sm);
	PySys_WriteStdout("mtf_am: %s\n", mtf_am);
	PySys_WriteStdout("mtf_ssm: %s\n", mtf_ssm);
	PySys_WriteStdout("py_visualize: %d\n", py_visualize);
	PySys_WriteStdout("*******************************\n");

	/*********************************** initialize tracker ***********************************/
	mtf::TrackerBase *tracker;
	PreProc_ pre_proc;
	try{
		tracker = mtf::getTracker(mtf_sm, mtf_am, mtf_ssm, mtf_ilm);			
		if(!tracker){
			PySys_WriteStdout("Tracker could not be created successfully\n");
			return Py_BuildValue("i", 0);
		}
	} catch(const mtf::utils::Exception &err){
		PySys_WriteStdout("Exception of type %s encountered while creating the tracker: %s\n",
			err.type(), err.what());
		return Py_BuildValue("i", 0);
	}
	try{
		pre_proc = mtf::getPreProc(pre_procs, tracker->inputType(), pre_proc_type);
	} catch(const mtf::utils::Exception &err){
		PySys_WriteStdout("Exception of type %s encountered while creating the pre processor: %s\n",
			err.type(), err.what());
		return Py_BuildValue("i", 0);
	}
	trackers.push_back(Tracker_(tracker));
	pre_procs.push_back(pre_proc);
	tracker_initialized.push_back(false);
	return Py_BuildValue("i", 1);
}

/* ==== initialize tracker ==== */
static PyObject* initialize(PyObject* self, PyObject* args) {
	PyArrayObject *img_py, *in_corners_py;
	unsigned int tracker_id = trackers.size() == 0 ? 0 : trackers.size() - 1;	
	/*parse first input array*/
	if(!PyArg_ParseTuple(args, "O!O!|i", &PyArray_Type, &img_py, 
		&PyArray_Type, &in_corners_py, &tracker_id)) {
		PySys_WriteStdout("\n----pyMTF::initialize: Input arguments could not be parsed----\n\n");
		return Py_BuildValue("i", 0);
	}
	if(img_py == NULL) {
		PySys_WriteStdout("\n----pyMTF::initialize::init_img is NULL----\n\n");
		return Py_BuildValue("i", 0);
	}
	if(in_corners_py == NULL) {
		PySys_WriteStdout("\n----pyMTF::initialize::init_corners is NULL----\n\n");
		return Py_BuildValue("i", 0);
	}
	if(in_corners_py->dimensions[0] != 2 || in_corners_py->dimensions[1] != 4){
		PySys_WriteStdout("pyMTF::Initial corners matrix has incorrect dimensions: %ld, %ld\n",
			in_corners_py->dimensions[0], in_corners_py->dimensions[1]);
		return Py_BuildValue("i", 0);
	}
	if(tracker_id >= trackers.size()){
		PySys_WriteStdout("\n----pyMTF::initialize: tracker_id %d is invalid as only %lu trackers exist----\n\n",
			tracker_id, trackers.size());
		return Py_BuildValue("i", 0);
	}
	int img_height = img_py->dimensions[0];
	int img_width = img_py->dimensions[1];
	int n_channels = img_py->nd == 3 ? img_py->dimensions[2] : 1;

	if(n_channels != 1 && n_channels != 3){
		PySys_WriteStdout("pyMTF:: Only grayscale and RGB images are supported\n");
		return Py_BuildValue("i", 0);
	}

	PySys_WriteStdout("img_height: %d\n", img_height);
	PySys_WriteStdout("img_width: %d\n", img_width);
	PySys_WriteStdout("n_channels: %d\n", n_channels);

	int img_type = n_channels == 3 ? CV_8UC3 : CV_8UC1;
	cv::Mat init_img_cv(img_height, img_width, img_type, img_py->data);
	cv::Mat temp(2, 4, CV_64FC1, in_corners_py->data);
	cv::Mat init_corners_cv(2, 4, CV_64FC1);
	init_corners_cv.at<double>(0, 0) = temp.at<double>(0, 0);
	init_corners_cv.at<double>(1, 0) = temp.at<double>(0, 1);
	init_corners_cv.at<double>(0, 1) = temp.at<double>(0, 2);
	init_corners_cv.at<double>(1, 1) = temp.at<double>(0, 3);
	init_corners_cv.at<double>(0, 2) = temp.at<double>(1, 0);
	init_corners_cv.at<double>(1, 2) = temp.at<double>(1, 1);
	init_corners_cv.at<double>(0, 3) = temp.at<double>(1, 2);
	init_corners_cv.at<double>(1, 3) = temp.at<double>(1, 3);

	// PySys_WriteStdout("init_corners_cv:\n");
	// for(unsigned int corner_id = 0; corner_id < 4; ++corner_id) {
	// PySys_WriteStdout("%d: (%f, %f)\n", corner_id, init_corners_cv.at<double>(0, corner_id), init_corners_cv.at<double>(1, corner_id));
	// }

	double min_x = init_corners_cv.at<double>(0, 0);
	double min_y = init_corners_cv.at<double>(1, 0);
	double max_x = init_corners_cv.at<double>(0, 2);
	double max_y = init_corners_cv.at<double>(1, 2);
	double size_x = max_x - min_x;
	double size_y = max_y - min_y;

	/*********************************** initialize tracker ***********************************/
	try{
		pre_procs[tracker_id]->initialize(init_img_cv);
	} catch(const mtf::utils::Exception &err){
		PySys_WriteStdout("Exception of type %s encountered while initializing the pre processor: %s\n",
			err.type(), err.what());
		return Py_BuildValue("i", 0);
	}
	try{
		for(PreProc_ curr_obj = pre_procs[tracker_id]; curr_obj; curr_obj = curr_obj->next){
			trackers[tracker_id]->setImage(curr_obj->getFrame());
		}
		PySys_WriteStdout("Initializing tracker with object of size %f x %f\n", size_x, size_y);
		trackers[tracker_id]->initialize(init_corners_cv);
	} catch(const mtf::utils::Exception &err){
		PySys_WriteStdout("Exception of type %s encountered while initializing the tracker: %s\n",
			err.type(), err.what());
		return Py_BuildValue("i", 0);
	}
	if(py_visualize) {
		cv::namedWindow("PyMTF", cv::WINDOW_AUTOSIZE);
	}
	tracker_initialized[tracker_id] = true;
	return Py_BuildValue("i", 1);
}

static PyObject* update(PyObject* self, PyObject* args) {
	PyArrayObject *img_py, *out_corners_py;
	unsigned int tracker_id = trackers.size() == 0 ? 0 : trackers.size() - 1;
	/*parse first input array*/
	if(!PyArg_ParseTuple(args, "O!O!|i", &PyArray_Type, &img_py, &PyArray_Type, &out_corners_py, &tracker_id)) {
		PySys_WriteStdout("\n----pyMTF::update: Input arguments could not be parsed----\n\n");
		return Py_BuildValue("i", 0);
	}
	if(!tracker_initialized[tracker_id]){
		PySys_WriteStdout("\n----pyMTF::update: Tracker must be initialized before it can be updated ----\n\n");
		return Py_BuildValue("i", 0);
	}
	if(img_py == NULL) {
		PySys_WriteStdout("\n----pyMTF::update::input image is NULL----\n\n");
		return Py_BuildValue("i", 0);
	}
	if(tracker_id >= trackers.size()){
		PySys_WriteStdout("\n----pyMTF::update: tracker_id %d is invalid as only %lu trackers exist----\n\n",
			tracker_id, trackers.size());
		return Py_BuildValue("i", 0);
	}
	int img_height = img_py->dimensions[0];
	int img_width = img_py->dimensions[1];
	int n_channels = img_py->nd == 3 ? img_py->dimensions[2] : 1;
	int img_type = n_channels == 3 ? CV_8UC3 : CV_8UC1;

	cv::Mat curr_img_cv(img_height, img_width, img_type, img_py->data);
	double fps = 0, fps_win = 0;
	double tracking_time, tracking_time_with_input;
	mtf_clock_get(start_time_with_input);
	//update tracker
	try{
		//! update pre processor
		pre_procs[tracker_id]->update(curr_img_cv);
		mtf_clock_get(start_time);
		//! update tracker
		trackers[tracker_id]->update();
		mtf_clock_get(end_time);
		mtf_clock_measure(start_time, end_time, tracking_time);
		mtf_clock_measure(start_time_with_input, end_time, tracking_time_with_input);
		fps = 1.0 / tracking_time;
		fps_win = 1.0 / tracking_time_with_input;
		if(reset_template){
			trackers[tracker_id]->initialize(trackers[tracker_id]->getRegion());
		}
	} catch(const mtf::utils::Exception &err){
		PySys_WriteStdout("Exception of type %s encountered while updating the tracker: %s\n",
			err.type(), err.what());
		return Py_BuildValue("i", 0);
	}
	if(py_visualize) {
		/* draw tracker positions to OpenCV window */
		cv::Point fps_origin(10, 20);
		double fps_font_size = 0.50;
		cv::Scalar fps_color(0, 255, 0);
		cv::Point2d corners[4];
		mtf::utils::Corners(trackers[tracker_id]->getRegion()).points(corners);
		line(curr_img_cv, corners[0], corners[1], CV_RGB(255, 0, 0), line_thickness);
		line(curr_img_cv, corners[1], corners[2], CV_RGB(255, 0, 0), line_thickness);
		line(curr_img_cv, corners[2], corners[3], CV_RGB(255, 0, 0), line_thickness);
		line(curr_img_cv, corners[3], corners[0], CV_RGB(255, 0, 0), line_thickness);
		putText(curr_img_cv, trackers[tracker_id]->name, corners[0],
			cv::FONT_HERSHEY_SIMPLEX, fps_font_size, CV_RGB(255, 0, 0));
		std::string fps_text = cv::format("c: %12.6f cw: %12.6f", fps, fps_win);
		putText(curr_img_cv, fps_text, fps_origin, cv::FONT_HERSHEY_SIMPLEX, fps_font_size, fps_color);
		imshow("PyMTF", curr_img_cv);
		cv::waitKey(1);
	}
	cv::Mat out_corners = trackers[tracker_id]->getRegion();
	double* out_corners_data = (double*)out_corners_py->data;
	for(unsigned int corner_id = 0; corner_id < 4; ++corner_id) {
		out_corners_data[corner_id] = out_corners.at<double>(0, corner_id);
		out_corners_data[corner_id + 4] = out_corners.at<double>(1, corner_id);
	}
	return Py_BuildValue("i", 1);
}
static PyObject* setRegion(PyObject* self, PyObject* args) {
	PyArrayObject *in_corners_py;
	unsigned int tracker_id = trackers.size() == 0 ? 0 : trackers.size() - 1;
	/*parse first input array*/
	if(!PyArg_ParseTuple(args, "O!|i", &PyArray_Type, &in_corners_py, &tracker_id)) {
		PySys_WriteStdout("\n----pyMTF::setRegion: Input arguments could not be parsed----\n\n");
		return Py_BuildValue("i", 0);
	}
	if(in_corners_py == NULL) {
		PySys_WriteStdout("\n----pyMTF::setRegion::init_corners is NULL----\n\n");
		return Py_BuildValue("i", 0);
	}
	if(in_corners_py->dimensions[0] != 2 || in_corners_py->dimensions[1] != 4){
		PySys_WriteStdout("pyMTF:: Corners matrix has incorrect dimensions: %ld, %ld\n",
			in_corners_py->dimensions[0], in_corners_py->dimensions[1]);
		return Py_BuildValue("i", 0);
	}
	if(tracker_id >= trackers.size()){
		PySys_WriteStdout("\n----pyMTF::setRegion: tracker_id %d is invalid as only %lu trackers exist----\n\n",
			tracker_id, trackers.size());
		return Py_BuildValue("i", 0);
	}
	cv::Mat temp(2, 4, CV_64FC1, in_corners_py->data);
	cv::Mat corners(2, 4, CV_64FC1);
	corners.at<double>(0, 0) = temp.at<double>(0, 0);
	corners.at<double>(1, 0) = temp.at<double>(0, 1);
	corners.at<double>(0, 1) = temp.at<double>(0, 2);
	corners.at<double>(1, 1) = temp.at<double>(0, 3);
	corners.at<double>(0, 2) = temp.at<double>(1, 0);
	corners.at<double>(1, 2) = temp.at<double>(1, 1);
	corners.at<double>(0, 3) = temp.at<double>(1, 2);
	corners.at<double>(1, 3) = temp.at<double>(1, 3);
	try{
		trackers[tracker_id]->setRegion(corners);
	} catch(const mtf::utils::Exception &err){
		PySys_WriteStdout("Exception of type %s encountered while resetting the tracker: %s\n",
			err.type(), err.what());
		return Py_BuildValue("i", 0);
	}
	return Py_BuildValue("i", 1);

}
static PyObject* remove(PyObject* self, PyObject* args) {
	unsigned int tracker_id = trackers.size() == 0 ? 0 : trackers.size() - 1;

	/*parse first input array*/
	if(!PyArg_ParseTuple(args, "|i", &PyArray_Type, &tracker_id)) {
		PySys_WriteStdout("\n----pyMTF::remove: Input arguments could not be parsed----\n\n");
		return Py_BuildValue("i", 0);
	}
	if(tracker_id >= trackers.size()){
		PySys_WriteStdout("\n----pyMTF::remove: tracker_id %d is invalid as only %lu trackers exist----\n\n",
			tracker_id, trackers.size());
		return Py_BuildValue("i", 0);
	}
	try{
		trackers.erase(trackers.begin() + tracker_id);
	} catch(const mtf::utils::Exception &err){
		PySys_WriteStdout("Exception of type %s encountered while removing the tracker: %s\n",
			err.type(), err.what());
		return Py_BuildValue("i", 0);
	}
	return Py_BuildValue("i", 1);
}
