#ifndef MTF_MEX_UTILS_H
#define MTF_MEX_UTILS_H

#include "mtf/Macros/common.h"

#include <mex.h>

#define _A3D_IDX_COLUMN_MAJOR(i,j,k,nrows,ncols) ((i)+((j)+(k)*ncols)*nrows)
// interleaved row-major indexing for 2-D OpenCV images
//#define _A3D_IDX_OPENCV(x,y,c,mat) (((y)*mat.step[0]) + ((x)*mat.step[1]) + (c))
#define _A3D_IDX_OPENCV(i,j,k,nrows,ncols,nchannels) (((i*ncols + j)*nchannels) + (k))

_MTF_BEGIN_NAMESPACE
namespace utils{
	/**
	* Copy the (image) data from Matlab-algorithm compatible (column-major) representation to cv::Mat.
	* The information about the image are taken from the OpenCV cv::Mat structure.
	adapted from OpenCV-Matlab package available at: https://se.mathworks.com/matlabcentral/fileexchange/41530-opencv-matlab
	*/
	template <typename T>
	inline void
		copyMatrixFromMatlab(const T* from, cv::Mat& to, int n_channels){

		const int n_rows = to.rows;
		const int n_cols = to.cols;

		T* pdata = (T*)to.data;

		for(int c = 0; c < n_channels; ++c){
			for(int x = 0; x < n_cols; ++x){
				for(int y = 0; y < n_rows; ++y){
					const T element = from[_A3D_IDX_COLUMN_MAJOR(y, x, c, n_rows, n_cols)];
					pdata[_A3D_IDX_OPENCV(y, x, c, rows, n_cols, n_channels)] = element;
				}
			}
		}
	}
	template <typename T>
	inline void
		copyMatrixToMatlab(const cv::Mat& from, T* to, int n_channels)
	{
		const int n_rows = from.rows;
		const int n_cols = from.cols;

		const T* pdata = (T*)from.data;

		for(int c = 0; c < n_channels; ++c){
			for(int x = 0; x < n_cols; ++x){
				for(int y = 0; y < n_rows; ++y){
					//const T element = pdata[_A3D_IDX_OPENCV(x,y,c,from)];
					const T element = pdata[_A3D_IDX_OPENCV(y, x, c, rows, n_cols, n_channels)];
					to[_A3D_IDX_COLUMN_MAJOR(y, x, n_channels - c - 1, n_rows, n_cols)] = element;
				}
			}
		}
	}
	inline unsigned int getID(const mxArray *mx_id){
		if(!mxIsClass(mx_id, "uint32")){
			mexErrMsgTxt("ID must be of 32 bit unsigned integral type");
		}
		unsigned int* id_ptr = (unsigned int*)mxGetData(mx_id);
		return *id_ptr;
	}
	inline cv::Mat getImage(const mxArray *mx_img){
		int img_n_dims = mxGetNumberOfDimensions(mx_img);
		if(!mxIsClass(mx_img, "uint8")){
			mexErrMsgTxt("Input image must be of 8 bit unsigned integral type");
		}
		if(img_n_dims < 2 || img_n_dims > 3){
			mexErrMsgTxt("Input image must have 2 or 3 dimensions");
		}
		int img_type = img_n_dims == 2 ? CV_8UC1 : CV_8UC3;
		const mwSize *img_dims = mxGetDimensions(mx_img);
		int height = img_dims[0];
		int width = img_dims[1];
		//printf("width: %d\t height=%d\t img_n_dims: %d\n", width, height, img_n_dims);
		unsigned char *img_ptr = (unsigned char*)mxGetData(mx_img);
		cv::Mat img(height, width, img_type);
		if(img_n_dims == 2){
			cv::Mat img_transpose(width, height, img_type, img_ptr);
			cv::transpose(img_transpose, img);
		} else{
			copyMatrixFromMatlab(img_ptr, img, 3);
		}
		return img;
	}
	inline cv::Mat getCorners(const mxArray *mx_corners){
		int corners_n_dims = mxGetNumberOfDimensions(mx_corners);
		if(!mxIsClass(mx_corners, "double")){
			mexErrMsgTxt("Input corner array must be of 64 bit floating point type");
		}
		if(corners_n_dims != 2){
			mexErrMsgTxt("Input corner array must have 2 dimensions");
		}
		const mwSize *corners_dims = mxGetDimensions(mx_corners);
		if(corners_dims[0] != 2 || corners_dims[1] != 4){
			mexErrMsgTxt("Input corner array must be of size 2 x 4");
		}
		double *corners_ptr = mxGetPr(mx_corners);
		cv::Mat corners_transposed(4, 2, CV_64FC1, corners_ptr);
		//cout << "corners_transposed: \n" << corners_transposed << "\n";
		cv::Mat corners(2, 4, CV_64FC1);
		cv::transpose(corners_transposed, corners);
		//cout << "corners: \n" << corners << "\n";
		return corners;
	}
	inline const char* toString(const mxArray *prhs){
		int param_str_len = mxGetM(prhs)*mxGetN(prhs) + 1;
		char* param_str = (char *)mxMalloc(param_str_len);
		mxGetString(prhs, param_str, param_str_len);
		return param_str;
	}

	inline mxArray *setCorners(const cv::Mat &corners){
		mxArray *mx_corners = mxCreateDoubleMatrix(2, 4, mxREAL);
		/* get a pointer to the real data in the output matrix */
		double *out_corners_ptr = mxGetPr(mx_corners);
		cv::Mat out_corners(4, 2, CV_64FC1, out_corners_ptr);
		cv::transpose(corners, out_corners);
		return mx_corners;
	}
}
_MTF_END_NAMESPACE
#endif