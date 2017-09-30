// Copyright 2011 Zdenek Kalal
//
// This file is part of TLD.
// 
// TLD is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// TLD is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with TLD.  If not, see <http://www.gnu.org/licenses/>.

#include <Python.h>
#include <numpy/arrayobject.h>
#include "opencv2/highgui/highgui.hpp"

#include <math.h>

#ifndef NAN
#define NAN 0/0
#endif

#ifndef M_PI
#define M_PI 3.14159265358979L
#endif
// rowwise access
#define coord(x, y, width, height) (x+y*width)
#define nextrow(tmp, width, height) ((tmp)+width)
#define nextcol(tmp, width, height) ((tmp)+1)
#define nextr_c(tmp, width, height) ((tmp)+width+1)

#define M(r, c) H[r*3+c]

static PyArrayObject *img_py, *H_py, *bb_py;

static PyObject* get(PyObject* self, PyObject* args);

static PyMethodDef pyWarpMethods[] = {
	{ "get", get, METH_VARARGS },
	{ NULL, NULL }     /* Sentinel - marks the end of this structure */
};

PyMODINIT_FUNC initpyWarp() {
	(void)Py_InitModule("pyWarp", pyWarpMethods);
	import_array();  // Must be present for NumPy.  Called first after above line.
}

/* Warps image of size w x h, using affine transformation matrix (2x2 part)
   and offset (center of warping) ofsx, ofsy. Result is the region of size
   defined with roi. */
void warp_image_roi(unsigned char *image, int w, int h, double *H,
	double xmin, double xmax, double ymin, double ymax,
	double fill, double *result)
{
	double curx, cury, curz, wx, wy, wz, ox, oy, oz;
	int x, y;
	unsigned char *tmp;
	double *output = result, i, j, xx, yy;
	/* precalulate necessary constant with respect to i,j offset
	   translation, H is column oriented (transposed) */
	ox = M(0, 2);
	oy = M(1, 2);
	oz = M(2, 2);

	yy = ymin;
	for(j = 0; j < (int)(ymax - ymin + 1); j++)
	{
		/* calculate x, y for current row */
		curx = M(0, 1)*yy + ox;
		cury = M(1, 1)*yy + oy;
		curz = M(2, 1)*yy + oz;
		xx = xmin;
		yy = yy + 1;
		for(i = 0; i < (int)(xmax - xmin + 1); i++)
		{
			/* calculate x, y in current column */
			wx = M(0, 0)*xx + curx;
			wy = M(1, 0)*xx + cury;
			wz = M(2, 0)*xx + curz;
			//       printf("%g %g, %g %g %g\n", xx, yy, wx, wy, wz);
			wx /= wz; wy /= wz;
			xx = xx + 1;

			x = (int)floor(wx);
			y = (int)floor(wy);

			if(x >= 0 && y >= 0)
			{
				wx -= x; wy -= y;
				if(x + 1 == w && wx == 1)
					x--;
				if(y + 1 == h && wy == 1)
					y--;
				if((x + 1) < w && (y + 1) < h)
				{
					tmp = &image[coord(x, y, w, h)];
					/* image[x,y]*(1-wx)*(1-wy) + image[x+1,y]*wx*(1-wy) +
					   image[x,y+1]*(1-wx)*wy + image[x+1,y+1]*wx*wy */
					*output++ =
						(*(tmp)* (1 - wx) + *nextcol(tmp, w, h) * wx) * (1 - wy) +
						(*nextrow(tmp, w, h) * (1 - wx) + *nextr_c(tmp, w, h) * wx) * wy;
				} else
					*output++ = fill;
			} else
				*output++ = fill;
		}
	}
}

void to_cv(cv::Mat &result, const double *image, int num_cols, int num_rows)
{
	// convert to matlab's column based representation
	int i, j;
	const double* s_ptr = image;
	double* d_ptr, *data;
	data = (double *)(result.data);
	for(i = 0; i < num_rows; i++)
	{
		d_ptr = &data[i];
		for(j = 0; j < num_cols; j++, d_ptr += num_rows, s_ptr++)
			(*d_ptr) = (*s_ptr);
	}
}

static PyObject* get(PyObject* self, PyObject* args) {
	/*parse first input array*/
	if(!PyArg_ParseTuple(args, "O!O!O!",
		&PyArray_Type, &img_py,
		&PyArray_Type, &H_py,
		&PyArray_Type, &bb_py)) {
		PyErr_SetString(PyExc_IOError, "Input arguments could not be parsed");
		PyErr_PrintEx(1);
	}
	if(img_py == NULL || H_py == NULL || bb_py == NULL) {
		PyErr_SetString(PyExc_IOError, "img_py, H_py or bb_py is NULL");
		PyErr_PrintEx(1);
	}
	if(img_py->nd != 2 || H_py->nd != 2) {
		PyErr_SetString(PyExc_IOError, "Both the input image and the warp matrix must be 2 dimensional arrays");
		PyErr_PrintEx(1);
	}
	if(bb_py->nd != 1) {
		PyErr_SetString(PyExc_IOError, "Bounding box must be a 1 dimensional array");
		PyErr_PrintEx(1);
	}

	int h = img_py->dimensions[0];
	int w = img_py->dimensions[1];
	unsigned char *im = (unsigned char*)img_py->data;
	double *H = (double*)H_py->data;
	double xmin, xmax, ymin, ymax, fill;
	//from_matlab(prhs[0], &im, &w, &h);
	double *B = (double*)bb_py->data;
	xmin = (*B++); xmax = (*B++);
	ymin = (*B++); ymax = (*B++);
	// Output
	int n_rows = (int)(ymax - ymin + 1);
	int n_cols = (int)(xmax - xmin + 1);
	int dims[] = { n_rows, n_cols };
	PyArrayObject *output_py = (PyArrayObject *)PyArray_FromDims(2, dims, NPY_DOUBLE);
	double *result = (double*)output_py->data;
	fill = 0;
	warp_image_roi(im, w, h, H, xmin, xmax, ymin, ymax, fill, result);
	PySys_WriteStdout("Completed obtaining the warped patch\n");
	return Py_BuildValue("O", output_py);
}
