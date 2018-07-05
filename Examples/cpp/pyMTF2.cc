#ifndef DISABLE_VISP
//! needed to avoid a weird bug in ViSP
#define PNG_SKIP_SETJMP_CHECK
#endif
#ifdef _WIN32
#define hypot _hypot
#endif

#include "mtf/TrackerStrct_mt.h"

#include <Python.h>
#include <numpy/arrayobject.h>

using namespace std;
using namespace mtf::params;

static PyObject* init(PyObject* self, PyObject* args);
static PyObject* isInitialized(PyObject* self, PyObject* args);
static PyObject* quit(PyObject* self, PyObject* args);
static PyObject* getFrame(PyObject* self, PyObject* args);
static PyObject* createTracker(PyObject* self, PyObject* args, PyObject *keywds);
static PyObject* createTrackers(PyObject* self, PyObject* args);
static PyObject* getRegion(PyObject* self, PyObject* args);
static PyObject* setRegion(PyObject* self, PyObject* args);
static PyObject* removeTracker(PyObject* self, PyObject* args);
static PyObject* removeTrackers(PyObject* self, PyObject* args);

static PyMethodDef pyMTF2Methods[] = {
	{ "init", init, METH_VARARGS },
	{ "isInitialized", isInitialized, METH_VARARGS },
	{ "quit", quit, METH_VARARGS },
	{ "getFrame", getFrame, METH_VARARGS },
	{ "createTracker", (PyCFunction)createTracker, METH_VARARGS | METH_KEYWORDS },
	{ "createTrackers", createTrackers, METH_VARARGS },
	{ "getRegion", getRegion, METH_VARARGS },
	{ "setRegion", setRegion, METH_VARARGS },
	{ "removeTracker", removeTracker, METH_VARARGS },
	{ "removeTrackers", removeTrackers, METH_VARARGS },
	{ NULL, NULL, 0, NULL }     /* Sentinel - marks the end of this structure */
};

#if PY_MAJOR_VERSION < 3
PyMODINIT_FUNC initpyMTF2()  {
	(void)Py_InitModule("pyMTF2", pyMTF2Methods);
	import_array();  // Must be present for NumPy.  Called first after above line.
}
#else
static struct PyModuleDef pyMTF2Module = {
	PyModuleDef_HEAD_INIT,
	"pyMTF2",   /* name of module */
	NULL, /* module documentation, may be NULL */
	-1,       /* size of per-interpreter state of the module,
			  or -1 if the module keeps state in global variables. */
			  pyMTF2Methods
};
PyMODINIT_FUNC PyInit_pyMTF2(void) {
	import_array();
	return PyModule_Create(&pyMTF2Module);
}
#endif

static InputStructPtr input;
static std::map<int, TrackerStruct> trackers;

static unsigned int _tracker_id = 0;

bool createInput() {
	if(input_buffer_size <= 1){
		input_buffer_size = 100;
	}
	Input _input;
	try{
		_input.reset(mtf::getInput(pipeline));
		if(!_input->initialize()){
			PySys_WriteStdout("Pipeline could not be initialized successfully\n");
			return false;
		}
	} catch(const mtf::utils::Exception &err){
		PySys_WriteStdout("Exception of type %s encountered while initializing the input pipeline: %s\n",
			err.type(), err.what());
		return false;
	}
	if(input) {
		input->reset(_input);
	} else {
		input.reset(new InputStruct(_input));
	}

	return true;
}
bool checkInput(bool verbose = true) {
	if(!input){
		if(verbose){
			PySys_WriteStdout("Input pipeline has not been initialized\n");
		}
		return false;
	}
	if(!input->isValid()) {
		if(verbose){
			PySys_WriteStdout("Input pipeline is not active\n");
		}
		return false;
	}
	return true;
}
bool initializeTracker(Tracker &tracker, PreProc &pre_proc,
	const cv::Mat &init_img, const cv::Mat &init_corners) {
	try{
		pre_proc->initialize(init_img);
	} catch(const mtf::utils::Exception &err){
		PySys_WriteStdout("Exception of type %s encountered while initializing the pre processor: %s\n",
			err.type(), err.what());
		return false;
	}
	try{
		for(PreProc curr_obj = pre_proc; curr_obj; curr_obj = curr_obj->next){
			tracker->setImage(curr_obj->getFrame());
		}
		double min_x = init_corners.at<double>(0, 0);
		double min_y = init_corners.at<double>(1, 0);
		double max_x = init_corners.at<double>(0, 2);
		double max_y = init_corners.at<double>(1, 2);
		double size_x = max_x - min_x;
		double size_y = max_y - min_y;
		PySys_WriteStdout("Initializing tracker with object of size %f x %f...\n", size_x, size_y);

		tracker->initialize(init_corners);
	} catch(const mtf::utils::Exception &err){
		PySys_WriteStdout("Exception of type %s encountered while initializing the tracker: %s\n",
			err.type(), err.what());
		return false;
	}
	PySys_WriteStdout("Initialization successful.\n");
	return true;
}
bool createTracker(const cv::Mat &init_corners) {
	if(!input->isValid()){
		PySys_WriteStdout("Input has not been initialized\n");
		return false;
	}
	Tracker tracker;
	try{
		tracker.reset(mtf::getTracker(mtf_sm, mtf_am, mtf_ssm, mtf_ilm));
		if(!tracker){
			PySys_WriteStdout("Tracker could not be created successfully\n");
			return false;
		}
	} catch(const mtf::utils::Exception &err){
		PySys_WriteStdout("Exception of type %s encountered while creating the tracker: %s\n",
			err.type(), err.what());
		return false;
	}
	PreProc pre_proc;
	try{
		pre_proc = mtf::getPreProc(tracker->inputType(), pre_proc_type);
	} catch(const mtf::utils::Exception &err){
		PySys_WriteStdout("Exception of type %s encountered while creating the pre processor: %s\n",
			err.type(), err.what());
		return false;
	}
	if(!initializeTracker(tracker, pre_proc, input->getFrame(), init_corners)){
		PySys_WriteStdout("Tracker initialization failed");
		return false;
	}
	++_tracker_id;
	trackers.insert(std::pair<int, TrackerStruct>(_tracker_id,
		TrackerStruct(tracker, pre_proc, input, _tracker_id, py_visualize, "pyMTF2")));
	return true;
}

static PyObject* init(PyObject* self, PyObject* args) {
	if(checkInput(false)) {
		PySys_WriteStdout("Input pipeline has already been initialized\n");
		return Py_BuildValue("i", 1);
	}
	char* _params = nullptr;
	if(!PyArg_ParseTuple(args, "|z", &_params)) {
		PySys_WriteStdout("\n----pyMTF::init: Input arguments could not be parsed----\n\n");
		return Py_BuildValue("i", 0);
	}
	if(!readParams(_params)) {
		PySys_WriteStdout("Parameters could not be parsed");
		return Py_BuildValue("i", 0);
	}
	if(!createInput()){
		return Py_BuildValue("i", 0);
	}
	return Py_BuildValue("i", 1);
}

static PyObject* isInitialized(PyObject* self, PyObject* args) {
	if(checkInput(false)) {
		return Py_BuildValue("i", 1);
	}
	return Py_BuildValue("i", 0);
}

static PyObject* quit(PyObject* self, PyObject* args) {
	PySys_WriteStdout("pyMTF2 :: clearing up...");
	input->reset();
	for(auto it = trackers.begin(); it != trackers.end(); ++it){
		it->second.reset();
	}
	trackers.clear();
	PySys_WriteStdout("done\n");
	return Py_BuildValue("i", 1);
}

static PyObject* getFrame(PyObject* self, PyObject* args) {
	if(!checkInput(false)) {
		return Py_BuildValue("");
	}

	const cv::Mat frame = input->getFrame();
	int n_rows = frame.rows, n_cols = frame.cols;

	int dims[] = { n_rows, n_cols, 3 };
	PyArrayObject *frame_py = (PyArrayObject *)PyArray_FromDims(3, dims, NPY_UBYTE);
	unsigned char *result = (unsigned char*)frame_py->data;

	for(int row = 0; row < n_rows; ++row) {
		for(int col = 0; col < n_cols; ++col) {
			int np_location = col*frame_py->strides[1] + row*frame_py->strides[0];
			cv::Vec3b pix_vals = frame.at<cv::Vec3b>(row, col);
			for(int ch = 0; ch < 3; ++ch) {
				*(result + np_location + ch*frame_py->strides[2]) = pix_vals[ch];
			}
		}
	}
	return Py_BuildValue("O", frame_py);
}

static PyObject* createTracker(PyObject* self, PyObject* args, PyObject *keywds) {
	PyArrayObject *in_corners_py = nullptr;
	char* _params = nullptr;
	static char *kwlist[] = { "params", "corners", NULL };
	if(!PyArg_ParseTupleAndKeywords(args, keywds, "|zO!", kwlist, &_params, &PyArray_Type, &in_corners_py)) {
		PySys_WriteStdout("\n----pyMTF2::createTracker: Input arguments could not be parsed----\n\n");
		return Py_BuildValue("i", 0);
	}
	if(!readParams(_params)) {
		PySys_WriteStdout("Parameters could not be parsed\n");
		return Py_BuildValue("i", 0);
	}

	cv::Mat init_corners_cv(2, 4, CV_64FC1);
	if(in_corners_py) {
		cv::Mat temp(2, 4, CV_64FC1, in_corners_py->data);
		init_corners_cv.at<double>(0, 0) = temp.at<double>(0, 0);
		init_corners_cv.at<double>(1, 0) = temp.at<double>(0, 1);
		init_corners_cv.at<double>(0, 1) = temp.at<double>(0, 2);
		init_corners_cv.at<double>(1, 1) = temp.at<double>(0, 3);
		init_corners_cv.at<double>(0, 2) = temp.at<double>(1, 0);
		init_corners_cv.at<double>(1, 2) = temp.at<double>(1, 1);
		init_corners_cv.at<double>(0, 3) = temp.at<double>(1, 2);
		init_corners_cv.at<double>(1, 3) = temp.at<double>(1, 3);
	} else {
		ObjectSelectorThread obj_sel_thread(input, 1, py_live_init);
		boost::thread t = boost::thread{ boost::ref(obj_sel_thread) };
		try{
			t.join();
		} catch(boost::thread_interrupted) {
			PySys_WriteStdout("Caught exception from object selector thread\n");
		}
		if(!obj_sel_thread.success) {
			PySys_WriteStdout("Initial corners could not be obtained\n");
			return Py_BuildValue("i", 0);
		}
		init_corners_cv = obj_sel_thread.getCorners(0);
	}
	if(!createTracker(init_corners_cv)) {
		PySys_WriteStdout("Tracker creation was unsuccessful\n");
		return Py_BuildValue("i", 0);
	}
	return Py_BuildValue("I", _tracker_id);
}


static PyObject* createTrackers(PyObject* self, PyObject* args) {
	char* _params = nullptr;
	static char *kwlist[] = { "params", "corners", NULL };
	if(!PyArg_ParseTupleAndKeywords(args, keywds, "|zO!", kwlist, &_params)) {
		PySys_WriteStdout("\n----pyMTF2::createTracker: Input arguments could not be parsed----\n\n");
		return Py_BuildValue("i", 0);
	}
	if(!readParams(_params)) {
		PySys_WriteStdout("Parameters could not be parsed\n");
		return Py_BuildValue("i", 0);
	}
	ObjectSelectorThread obj_sel_thread(input, n_trackers, py_live_init);
	boost::thread t = boost::thread{ boost::ref(obj_sel_thread) };
	try{
		t.join();
	} catch(boost::thread_interrupted) {
		PySys_WriteStdout("Caught exception from object selector thread\n");
	}
	if(!obj_sel_thread.success) {
		PySys_WriteStdout("Initial corners could not be obtained\n");
		return Py_BuildValue("i", 0);
	}
	FILE *multi_fid = nullptr;
	std::vector<unsigned int> tracker_ids;
	for(unsigned int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
		if(n_trackers > 1){ multi_fid = readTrackerParams(multi_fid); }
		if(!createTracker(obj_sel_thread.getCorners(tracker_id))) {
			PySys_WriteStdout("pyMTF2 :: tracker %d creation was unsuccessful\n", tracker_id);
			return Py_BuildValue("i", 0);
		}
		tracker_ids.push_back(_tracker_id);
	}
	int dims[] = { n_trackers};
	PyArrayObject *tracker_ids_py = (PyArrayObject *)PyArray_FromDims(1, dims, NPY_UINT);
	unsigned int *out_ptr = (unsigned int*)tracker_ids_py->data;
	for(unsigned int _id = 0; _id < n_trackers; ++_id) {
		out_ptr[_id] = tracker_ids[_id];
	}
	return Py_BuildValue("O", tracker_ids_py);
}

static PyObject* getRegion(PyObject* self, PyObject* args) {
	unsigned int tracker_id;
	if(!PyArg_ParseTuple(args, "I", &tracker_id)) {
		PySys_WriteStdout("\n----pyMTF2::getRegion: input argument could not be parsed----\n\n");
		return Py_BuildValue("");
	}

	std::map<int, TrackerStruct>::iterator it = trackers.find(tracker_id);
	if(it == trackers.end()){
		PySys_WriteStdout("Invalid tracker ID: %d\n", tracker_id);
		return Py_BuildValue("");
	}
	if(!it->second.isRunning()) {
		return Py_BuildValue("");
	}
	cv::Mat corners = it->second.getRegion();
	int dims[] = { 2, 4 };
	PyArrayObject *corners_py = (PyArrayObject *)PyArray_FromDims(2, dims, NPY_DOUBLE);
	double* corners_py_data = (double*)corners_py->data;
	for(unsigned int corner_id = 0; corner_id < 4; ++corner_id) {
		corners_py_data[corner_id] = corners.at<double>(0, corner_id);
		corners_py_data[corner_id + 4] = corners.at<double>(1, corner_id);
	}
	return Py_BuildValue("O", corners_py);
}


static PyObject* setRegion(PyObject* self, PyObject* args) {
	unsigned int tracker_id;
	PyArrayObject *corners_py;
	if(!PyArg_ParseTuple(args, "O!I", &PyArray_Type, &corners_py, &tracker_id)) {
		PySys_WriteStdout("\n----pyMTF2::setRegion: Input arguments could not be parsed----\n\n");
		return Py_BuildValue("i", 0);
	}

	std::map<int, TrackerStruct>::iterator it = trackers.find(tracker_id);
	if(it == trackers.end()){
		PySys_WriteStdout("Invalid tracker ID: %d\n", tracker_id);
		return Py_BuildValue("i", 0);
	}
	if(corners_py == NULL) {
		PySys_WriteStdout("\n----pyMTF2::setRegion::init_corners is NULL----\n\n");
		return Py_BuildValue("i", 0);
	}
	if(corners_py->dimensions[0] != 2 || corners_py->dimensions[1] != 4){
		PySys_WriteStdout("pyMTF2::setRegion :: corners matrix has incorrect dimensions: %ld, %ld\n",
			corners_py->dimensions[0], corners_py->dimensions[1]);
		return Py_BuildValue("i", 0);
	}
	cv::Mat corners(2, 4, CV_64FC1);
	double* corners_py_data = (double*)corners_py->data;
	for(unsigned int corner_id = 0; corner_id < 4; ++corner_id) {
		corners.at<double>(0, corner_id) = corners_py_data[corner_id];
		corners.at<double>(1, corner_id) = corners_py_data[corner_id + 4];
	}
	try{
		it->second.setRegion(corners);
	} catch(const mtf::utils::Exception &err){
		PySys_WriteStdout("Exception of type %s encountered while resetting the tracker: %s\n",
			err.type(), err.what());
		return Py_BuildValue("i", 0);
	}
	return Py_BuildValue("i", 1);
}


static PyObject* removeTrackers(PyObject* self, PyObject* args) {
	PySys_WriteStdout("pyMTF2 :: removing all trackers...");
	for(auto it = trackers.begin(); it != trackers.end(); ++it){
		it->second.reset();
	}
	trackers.clear();
	PySys_WriteStdout("done\n");
	return Py_BuildValue("i", 1);
}

static PyObject* removeTracker(PyObject* self, PyObject* args) {
	unsigned int tracker_id;
	if(!PyArg_ParseTuple(args, "I", &tracker_id)) {
		PySys_WriteStdout("\n----pyMTF2::removeTracker: input arguments could not be parsed----\n\n");
		return Py_BuildValue("i", 0);
	}
	std::map<int, TrackerStruct>::iterator it = trackers.find(tracker_id);
	if(it == trackers.end()){
		PySys_WriteStdout("pyMTF2 :: invalid tracker ID: %d\n", tracker_id);
		return Py_BuildValue("i", 0);
	}
	trackers.erase(it);
	PySys_WriteStdout("pyMTF2 :: removed tracker with ID: %u\n", tracker_id);
	return Py_BuildValue("i", 1);
}