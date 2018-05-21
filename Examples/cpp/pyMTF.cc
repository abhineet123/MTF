#ifndef DISABLE_VISP
//! needed to avoid a weird bug in ViSP
#define PNG_SKIP_SETJMP_CHECK
#endif
#ifdef _WIN32
#define hypot _hypot
#endif
#include "mtf/TrackerStruct.h"
#include "mtf/Utilities/miscUtils.h"

#include <vector>
#include <map>
#include <memory>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <Python.h>
#include <numpy/arrayobject.h>

using namespace std;
using namespace mtf::params;

static PyObject* initInput(PyObject* self, PyObject* args);
static PyObject* create(PyObject* self, PyObject* args);
static PyObject* getRegion(PyObject* self, PyObject* args);
static PyObject* setRegion(PyObject* self, PyObject* args);
static PyObject* remove(PyObject* self, PyObject* args);

//typedef unique_ptr<mtf::TrackerBase> Tracker_;
//static std::vector<Tracker_> trackers;
//static std::vector<PreProc_> pre_procs;
//static std::vector<bool> tracker_initialized;

static PyMethodDef pyMTFMethods[] = {
	{ "create", create, METH_VARARGS },
	{ "getRegion", getRegion, METH_VARARGS },
	{ "setRegion", setRegion, METH_VARARGS },
	{ "remove", remove, METH_VARARGS },
	{ NULL, NULL, 0, NULL }     /* Sentinel - marks the end of this structure */
};

#if PY_MAJOR_VERSION < 3
PyMODINIT_FUNC initpyMTF()  {
	(void)Py_InitModule("pyMTF", pyMTFMethods);
	import_array();  // Must be present for NumPy.  Called first after above line.
}
#else
static struct PyModuleDef pyMTFModule = {
	PyModuleDef_HEAD_INIT,
	"pyMTF",   /* name of module */
	NULL, /* module documentation, may be NULL */
	-1,       /* size of per-interpreter state of the module,
			  or -1 if the module keeps state in global variables. */
	pyMTFMethods
};
PyMODINIT_FUNC PyInit_pyMTF(void) {
	import_array();
	return PyModule_Create(&pyMTFModule);
}
#endif


static std::map<int, TrackerStruct> trackers;
static unsigned int _tracker_id = 0;

static PyObject* create(PyObject* self, PyObject* args) {
	char* config_root_dir;
	PyArrayObject *img_py, *in_corners_py;

	if(!PyArg_ParseTuple(args, "O!O!|z", &PyArray_Type, &img_py,
		&PyArray_Type, &in_corners_py, &config_root_dir)) {
		PySys_WriteStdout("\n----pyMTF::create: Input arguments could not be parsed----\n\n");
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

	/*********************************** create and initialize tracker ***********************************/

	TrackerStruct tracker;
	if(!tracker.create(init_img_cv, init_corners_cv)){
		return Py_BuildValue("i", 0);
	}
	++_tracker_id;
	trackers.insert(std::pair<int, TrackerStruct>(_tracker_id, tracker));
	if(py_visualize) {
		cv::namedWindow("PyMTF", cv::WINDOW_AUTOSIZE);
	}
	PySys_WriteStdout("Created tracker with ID: %u\n", _tracker_id);

	return Py_BuildValue("I", _tracker_id);
}

static PyObject* getRegion(PyObject* self, PyObject* args) {
	PyArrayObject *img_py, *out_corners_py;
	unsigned int tracker_id;
	/*parse first input array*/
	if(!PyArg_ParseTuple(args, "O!O!I", &PyArray_Type, &img_py, &PyArray_Type, &out_corners_py, &tracker_id)) {
		PySys_WriteStdout("\n----pyMTF::update: Input arguments could not be parsed----\n\n");
		return Py_BuildValue("i", 0);
	}
	if(img_py == NULL) {
		PySys_WriteStdout("\n----pyMTF::update::input image is NULL----\n\n");
		return Py_BuildValue("i", 0);
	}
	int img_height = img_py->dimensions[0];
	int img_width = img_py->dimensions[1];
	int n_channels = img_py->nd == 3 ? img_py->dimensions[2] : 1;
	int img_type = n_channels == 3 ? CV_8UC3 : CV_8UC1;

	cv::Mat curr_img_cv(img_height, img_width, img_type, img_py->data);

	std::map<int, TrackerStruct>::iterator it = trackers.find(tracker_id);
	if(it == trackers.end()){
		printf("Invalid tracker ID: %d\n", tracker_id);
		return Py_BuildValue("i", 0);
	}
	cv::Mat out_corners;
	if(!it->second.update(curr_img_cv, out_corners)){
		return Py_BuildValue("i", 0);
	}

	if(py_visualize) {
		/* draw tracker positions to OpenCV window */
		cv::Point fps_origin(10, 20);
		double fps_font_size = 0.50;
		cv::Scalar fps_color(0, 255, 0);
		cv::Point2d corners[4];
		mtf::utils::Corners(out_corners).points(corners);
		line(curr_img_cv, corners[0], corners[1], CV_RGB(255, 0, 0), line_thickness);
		line(curr_img_cv, corners[1], corners[2], CV_RGB(255, 0, 0), line_thickness);
		line(curr_img_cv, corners[2], corners[3], CV_RGB(255, 0, 0), line_thickness);
		line(curr_img_cv, corners[3], corners[0], CV_RGB(255, 0, 0), line_thickness);
		putText(curr_img_cv, it->second.name(), corners[0],
			cv::FONT_HERSHEY_SIMPLEX, fps_font_size, CV_RGB(255, 0, 0));
		//std::string fps_text = cv::format("c: %12.6f cw: %12.6f", fps, fps_win);
		//putText(curr_img_cv, fps_text, fps_origin, cv::FONT_HERSHEY_SIMPLEX, fps_font_size, fps_color);
		imshow("PyMTF", curr_img_cv);
		cv::waitKey(1);
	}
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
	if(!PyArg_ParseTuple(args, "O!I", &PyArray_Type, &in_corners_py, &tracker_id)) {
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
	cv::Mat corners(2, 4, CV_64FC1);

	//cv::Mat temp(2, 4, CV_64FC1, in_corners_py->data);
	//corners.at<double>(0, 0) = temp.at<double>(0, 0);
	//corners.at<double>(1, 0) = temp.at<double>(0, 1);
	//corners.at<double>(0, 1) = temp.at<double>(0, 2);
	//corners.at<double>(1, 1) = temp.at<double>(0, 3);
	//corners.at<double>(0, 2) = temp.at<double>(1, 0);
	//corners.at<double>(1, 2) = temp.at<double>(1, 1);
	//corners.at<double>(0, 3) = temp.at<double>(1, 2);
	//corners.at<double>(1, 3) = temp.at<double>(1, 3);

	double* in_corners_data = (double*)in_corners_py->data;
	for(unsigned int corner_id = 0; corner_id < 4; ++corner_id) {
		corners.at<double>(0, corner_id) = in_corners_data[corner_id];
		corners.at<double>(1, corner_id) = in_corners_data[corner_id + 4];
	}

	//cout << "pyMTF :: setRegion: setting tracker to: \n" << corners << "\n";

	std::map<int, TrackerStruct>::iterator it = trackers.find(tracker_id);
	if(it == trackers.end()){
		printf("Invalid tracker ID: %d\n", tracker_id);
		return Py_BuildValue("i", 0);
	}
	try{
		it->second.setRegion(corners);
	} catch(const mtf::utils::Exception &err){
		printf("Exception of type %s encountered while resetting the tracker: %s\n",
			err.type(), err.what());
		return Py_BuildValue("i", 0);
	}
	return Py_BuildValue("i", 1);

}
static PyObject* remove(PyObject* self, PyObject* args) {
	unsigned int tracker_id;
	/*parse first input array*/
	if(!PyArg_ParseTuple(args, "I", &tracker_id)) {
		PySys_WriteStdout("\n----pyMTF::remove: Input arguments could not be parsed----\n\n");
		return Py_BuildValue("i", 0);
	}
	std::map<int, TrackerStruct>::iterator it = trackers.find(tracker_id);
	if(it == trackers.end()){
		printf("Invalid tracker ID: %d\n", tracker_id);
		return Py_BuildValue("i", 0);
	}
	trackers.erase(it);
	return Py_BuildValue("i", 1);
}
