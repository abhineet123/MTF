#ifdef _WIN32
#define hypot _hypot
#endif

#include "opencv/cv.h"
#include "opencv2/highgui/highgui.hpp"
#include "math.h"
#include <limits>
#ifdef _CHAR16T
#define CHAR16_T
#endif
#include <Python.h>
#include <numpy/arrayobject.h>

#include "mtf/Utilities/miscUtils.h"

static const int MAX_COUNT = 500;
static const int MAX_IMG = 2;
static int win_size = 4;
static CvPoint2D32f* points[3] = { 0,0,0 };

static PyArrayObject *img_py_1, *img_py_2;
static PyArrayObject *pts_py_1, *pts_py_2;
static int Level;

static PyObject* get(PyObject* self, PyObject* args);

static PyMethodDef pyLKMethods[] = {
	{ "get", get, METH_VARARGS },
	{ NULL, NULL }     /* Sentinel - marks the end of this structure */
};

PyMODINIT_FUNC initpyLK() {
	(void)Py_InitModule("pyLK", pyLKMethods);
	import_array();  // Must be present for NumPy.  Called first after above line.
}

static void euclideanDistance(CvPoint2D32f *point1, CvPoint2D32f *point2, float *match, int nPts) {

	for(int i = 0; i < nPts; i++) {

		match[i] = sqrt((point1[i].x - point2[i].x)*(point1[i].x - point2[i].x) +
			(point1[i].y - point2[i].y)*(point1[i].y - point2[i].y));

	}
}

static void normCrossCorrelation(IplImage *imgI, IplImage *imgJ, CvPoint2D32f *points0, CvPoint2D32f *points1,
	int nPts, char *status, float *match, int winsize, int method) {


	IplImage *rec0 = cvCreateImage(cvSize(winsize, winsize), 8, 1);
	IplImage *rec1 = cvCreateImage(cvSize(winsize, winsize), 8, 1);
	IplImage *res = cvCreateImage(cvSize(1, 1), IPL_DEPTH_32F, 1);

	for(int i = 0; i < nPts; i++) {
		if(status[i] == 1) {
			cvGetRectSubPix(imgI, rec0, points0[i]);
			cvGetRectSubPix(imgJ, rec1, points1[i]);
			cvMatchTemplate(rec0, rec1, res, method);
			match[i] = ((float *)(res->imageData))[0];

		} else {
			match[i] = 0.0;
		}
	}
	cvReleaseImage(&rec0);
	cvReleaseImage(&rec1);
	cvReleaseImage(&res);

}

static PyObject* get(PyObject* self, PyObject* args) {

	PySys_WriteStdout("\nStarting pyLK\n");

	/*parse first input array*/
	if(!PyArg_ParseTuple(args, "O!O!O!O!|i",
		&PyArray_Type, &img_py_1,
		&PyArray_Type, &img_py_2,
		&PyArray_Type, &pts_py_1,
		&PyArray_Type, &pts_py_2, &Level)) {
		PyErr_SetString(PyExc_IOError, "Input arguments could not be parsed");
		PyErr_PrintEx(1);
	}
	if(img_py_1 == NULL || img_py_2 == NULL) {
		PyErr_SetString(PyExc_IOError, "img_py_1 or img_py_2 is NULL");
		PyErr_PrintEx(1);
	}
	if(pts_py_1 == NULL || pts_py_2 == NULL) {
		PyErr_SetString(PyExc_IOError, "pts_py_1 or pts_py_2 is NULL");
		PyErr_PrintEx(1);
	}
	if(img_py_1->nd != 2 || img_py_2->nd != 2) {
		PyErr_SetString(PyExc_IOError, "Both input images must be 2 dimensional arrays");
		PyErr_PrintEx(1);
	}
	if(pts_py_1->nd != 2 || pts_py_2->nd != 2) {
		PyErr_SetString(PyExc_IOError, "Both point sets must be 2 dimensional arrays");
		PyErr_PrintEx(1);
	}

	int img_height = img_py_1->dimensions[0];
	int img_width = img_py_1->dimensions[1];
	if(img_height != img_py_2->dimensions[0] || img_width != img_py_2->dimensions[1]) {
		PyErr_SetString(PyExc_IOError, "Input images have inconsistent dimensions");
		PyErr_PrintEx(1);
	}
	if(pts_py_1->dimensions[1] != 2 || pts_py_2->dimensions[1] != 2) {
		PyErr_SetString(PyExc_IOError, "Both point sets must have 2 columns");
		PyErr_PrintEx(1);
	}
	int nPts = pts_py_1->dimensions[0];
	if(nPts != pts_py_2->dimensions[0]) {
		PyErr_SetString(PyExc_IOError, "Both point sets must have the same number of points");
		PyErr_PrintEx(1);
	}
	PySys_WriteStdout("img_height: %d\t img_width: %d\n", img_height, img_width);
	PySys_WriteStdout("nPts: %d\n", nPts);
	PySys_WriteStdout("Level: %d\n", Level);

	//int dummy_input;
	//scanf("Press any key to continue: %d", &dummy_input);

	IplImage **IMG = (IplImage**)calloc(MAX_IMG, sizeof(IplImage*));
	IplImage **PYR = (IplImage**)calloc(MAX_IMG, sizeof(IplImage*));

	int I = 0;
	int J = 1;
	int Winsize = 10;

	// Images
	cv::Mat img_1(img_height, img_width, CV_8UC1, img_py_1->data);
	cv::Mat img_2(img_height, img_width, CV_8UC1, img_py_2->data);

	CvSize imageSize = cvSize(img_width, img_height);
	IMG[I] = new IplImage(img_1);
	PYR[I] = cvCreateImage(imageSize, 8, 1);
	IMG[J] = new IplImage(img_2);
	PYR[J] = cvCreateImage(imageSize, 8, 1);

	// Points	
	cv::Mat ptsI = cv::Mat(nPts, 2, CV_32FC1, pts_py_1->data);
	cv::Mat ptsJ = cv::Mat(nPts, 2, CV_32FC1, pts_py_2->data);

	points[0] = (CvPoint2D32f*)cvAlloc(nPts*sizeof(CvPoint2D32f)); // template
	points[1] = (CvPoint2D32f*)cvAlloc(nPts*sizeof(CvPoint2D32f)); // target
	points[2] = (CvPoint2D32f*)cvAlloc(nPts*sizeof(CvPoint2D32f)); // forward-backward

	for(int i = 0; i < nPts; i++) {
		points[0][i].x = ptsI.at<float>(i, 0); points[0][i].y = ptsI.at<float>(i, 1);
		points[1][i].x = ptsJ.at<float>(i, 0); points[1][i].y = ptsJ.at<float>(i, 1);
		points[2][i].x = ptsI.at<float>(i, 0); points[2][i].y = ptsI.at<float>(i, 1);
	}
	float *fb = (float*)cvAlloc(nPts*sizeof(float));
	char  *status = (char*)cvAlloc(nPts);
	cvCalcOpticalFlowPyrLK(IMG[I], IMG[J], PYR[I], PYR[J], points[0],
		points[1], nPts, cvSize(win_size, win_size), Level, status, 0,
		cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03), CV_LKFLOW_INITIAL_GUESSES);
	PySys_WriteStdout("Completed forward flow\n");
	cvCalcOpticalFlowPyrLK(IMG[J], IMG[I], PYR[J], PYR[I], points[1],
		points[2], nPts, cvSize(win_size, win_size), Level, status, 0,
		cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03),
		CV_LKFLOW_INITIAL_GUESSES | CV_LKFLOW_PYR_A_READY | CV_LKFLOW_PYR_B_READY);
	PySys_WriteStdout("Completed backward flow\n");

	float *ncc = (float*)cvAlloc(nPts*sizeof(float));
	normCrossCorrelation(IMG[I], IMG[J], points[0], points[1], nPts, status, ncc, Winsize, CV_TM_CCOEFF_NORMED);
	PySys_WriteStdout("Completed NCC computation \n");

	//float *ssd = (float*)cvAlloc(nPts*sizeof(float));
	//normCrossCorrelation(IMG[I],IMG[J],points[0],points[1],nPts, status, ssd, Winsize,CV_TM_SQDIFF);
	
	euclideanDistance(points[0], points[2], fb, nPts);
	PySys_WriteStdout("Completed Euclidean distance computation\n");

	// Output
	int dims[] = { 4, nPts };
	PyArrayObject *output_py = (PyArrayObject *)PyArray_FromDims(2, dims, NPY_FLOAT);
	cv::Mat output = cv::Mat(4, nPts, CV_32FC1, output_py->data);
	float nan = std::numeric_limits<float>::quiet_NaN();
	float inf = std::numeric_limits<float>::infinity();
	for(int i = 0; i < nPts; i++) {
		if(status[i] == 1) {
			output.at<float>(0, i) = points[1][i].x;
			output.at<float>(1, i) = points[1][i].y;
			output.at<float>(2, i) = fb[i];
			output.at<float>(3, i) = ncc[i];
		} else {
			output.at<float>(0, i) = nan;
			output.at<float>(1, i) = nan;
			output.at<float>(2, i) = nan;
			output.at<float>(3, i) = nan;
		}
	}
	PySys_WriteStdout("Completed writing to output matrix\n");

	mtf::utils::drawPts<float>(img_1, ptsI, cv::Scalar(0, 0, 0), 2);
	mtf::utils::drawPts<float>(img_2, ptsJ, cv::Scalar(0, 0, 0), 2);
	std::vector<cv::Mat> img_list;
	img_list.push_back(img_1);
	img_list.push_back(img_2);
	cv::Mat stacked_img = mtf::utils::stackImages(img_list);
	cv::imshow("Input Images", stacked_img);
	if(cv::waitKey(0) == 27) {
		Py_Exit(0);
	}
	mtf::utils::printMatrixToFile<float>(ptsI, nullptr, "ptsI.txt", "%.4f");
	mtf::utils::printMatrixToFile<float>(ptsJ, nullptr, "ptsJ.txt", "%.4f");


	// clean up
	for(int i = 0; i < MAX_IMG; i++) {
		//! sharing memory with Python array
		//cvReleaseImage(&(IMG[i]));
		cvReleaseImage(&(PYR[i]));
	}
	free(IMG);
	free(PYR);
	PySys_WriteStdout("Completed cleaning up\n");
	return Py_BuildValue("O", output_py);
}
