#ifndef MTF_COMMON_H
#define MTF_COMMON_H

#include <Eigen/Dense>
#include "opencv2/core/core.hpp"
#include <assert.h> 
#include <stdlib.h>
#include <stdio.h>
#include <vector>

#if defined(__GNUC__) || defined(__GNUG__)
#define _force_inline_ __attribute__ (always_inline)
#else
#define _force_inline_
#endif

//#ifdef _WIN32
//typedef long ClockType;
//#define INIT_TIMER(start_time) INIT_TIMER(start_time);
//#define END_TIMER(start_time, end_time, interval)\
//	end_time = clock();\
//	interval = ((double)(end_time - start_time)) / CLOCKS_PER_SEC;
//#define RECORD_EVENT(start_time, end_time, label, proc_times, proc_labels) \
//	end_time = clock();\
//	proc_times.push_back(((double) (end_time - start_time))/CLOCKS_PER_SEC);\
//	proc_labels.push_back(label);\
//	start_time = clock();
//#else
//typedef timespec ClockType;
//#define INIT_TIMER(start_time) clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start_time);
//#define END_TIMER(start_time, end_time, interval)\
//	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end_time);\
//	interval = (double)(end_time.tv_sec - start_time.tv_sec) + 1e-9*(double)(end_time.tv_nsec - start_time.tv_nsec);
//#define RECORD_EVENT(start_time, end_time, label, proc_times, proc_labels) \
//	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end_time);\
//	proc_times.push_back((double)(end_time.tv_sec - start_time.tv_sec) + 1e-9*(double)(end_time.tv_nsec - start_time.tv_nsec));\
//	proc_labels.push_back(label);\
//	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start_time);
//#endif
//
//#define TIME_EVENT(start_time, end_time, interval) \
//	end_time = clock();\
//	interval = ((double) (end_time - start_time))/CLOCKS_PER_SEC;\
//	INIT_TIMER(start_time);

#ifdef _WIN32
#define _USE_MATH_DEFINES
#include <time.h>
#define mtf_clock_get(time_instant) \
	clock_t time_instant = clock()
#define mtf_clock_measure(start_time, end_time, elapsed_time) \
	elapsed_time = static_cast<double>(end_time - start_time)/CLOCKS_PER_SEC

#else
#include <ctime> 
#define mtf_clock_get(time_instant) \
	timespec time_instant;\
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time_instant)
#define mtf_clock_measure(start_time, end_time, elapsed_time) \
	elapsed_time = ((double)(end_time.tv_sec - start_time.tv_sec) +\
		1e-9*(double)(end_time.tv_nsec - start_time.tv_nsec))
#endif

#ifdef ENABLE_PROFILING
#define init_profiling() \
	timespec _event_start_time, _event_end_time;\
	double _event_interval; \
	std::vector<double> proc_times;\
	std::vector<char*> proc_labels
#define inherit_profiling(BASE_CLASS) \
	using BASE_CLASS ::_event_start_time;\
	using BASE_CLASS ::_event_end_time;\
	using BASE_CLASS ::_event_interval;\
	using BASE_CLASS :: proc_times;\
	using BASE_CLASS :: proc_labels
#define write_frame_id(frame_id) \
	utils::printScalarToFile(frame_id, "\n\nframe_id", time_fname, "%6d", "a");
#define init_timer() \
	proc_times.clear();\
	proc_labels.clear();\
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &_event_start_time)
#define start_timer() \
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &_event_start_time)
#define end_timer() \
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &_event_end_time);\
	_event_interval = (double)(_event_end_time.tv_sec - _event_start_time.tv_sec) + 1e-9*(double)(_event_end_time.tv_nsec - _event_start_time.tv_nsec);
#define write_interval(time_fname, mode) \
	utils::printScalarToFile(_event_interval, "init_time", time_fname, "%15.9f", mode)
#define record_event(label) \
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &_event_end_time);\
	proc_times.push_back((double)(_event_end_time.tv_sec - _event_start_time.tv_sec) + 1e-9*(double)(_event_end_time.tv_nsec - _event_start_time.tv_nsec));\
	proc_labels.push_back(label);\
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &_event_start_time)
#define write_data(time_fname) \
	utils::writeTimesToFile(proc_times, proc_labels, time_fname, i)
#else
#define init_profiling()
#define inherit_profiling(BASE_CLASS)
#define write_frame_id(frame_id)
#define init_timer()
#define start_timer()
#define end_timer()
#define write_interval(time_fname, mode)
#define record_event(label) 
#define write_data(time_fname)
#endif

#define eig_set_zero(eig_mat, scalar_type) \
	memset(eig_mat.data(), 0, eig_mat.size()*sizeof(scalar_type))

#define func_not_implemeted(func_name) \
	throw mtf::utils::FunctonNotImplemented(cv::format("%s :: %s :: Not implemented Yet", name.c_str(), #func_name))

#define remap_data(var_name, var_type, data_loc, data_size) new (&var_name) Map< var_type >(data_loc, data_size)
//#define CV_TO_EIG_IMG(cv_mat) Map<const  MatrixXuc, Aligned>((EIG_PIX_TYPE*)(cv_mat.data), cv_mat.rows, cv_mat.cols);
//#define CV_TO_EIG_CORNERS(cv_mat) Map<const  Matrix24d, Aligned>((double*)cv_mat.data, cv_mat.rows, cv_mat.cols);

#ifndef DISABLE_SPI
#define spi_pt_check_mc(mask, id, ch_id) \
	if(mask && !mask[id]){ ch_id += n_channels; continue;}
#define spi_pt_check(mask, id) \
	if(mask && !mask[id]){ continue;}
#define spi_check_mc(mask, id, ch_id) \
	if(!mask[id]){ ch_id += n_channels; continue;}
#define spi_check(mask, id) \
	if(!mask[id]){continue;}
#else
#define spi_pt_check(mask, id)
#define spi_pt_check_mc(mask, id, ch_id) 
#define spi_check_mc(mask, id, ch_id) 
#define spi_check(mask, id)
#endif

#define corners_to_cv(cv_corners, eig_corners)\
	for(int corner_id = 0; corner_id < 4; ++corner_id){\
		cv_corners.at<double>(0, corner_id) = eig_corners(0, corner_id);\
		cv_corners.at<double>(1, corner_id) = eig_corners(1, corner_id);\
	}
#define corners_from_cv(eig_corners, cv_corners)\
	for(int corner_id = 0; corner_id < 4; ++corner_id){\
		eig_corners(0, corner_id) = cv_corners.at<double>(0, corner_id);\
		eig_corners(1, corner_id) = cv_corners.at<double>(1, corner_id);\
	}
#define corners_to_points(points, eig_corners)\
	for(int corner_id = 0; corner_id < 4; ++corner_id){\
		points[corner_id].x = curr_corners(0, corner_id);\
		points[corner_id].y = curr_corners(1, corner_id);\
	}
#define HETEROGENEOUS_INPUT -1

#define _MTF_BEGIN_NAMESPACE namespace mtf {
#define _MTF_END_NAMESPACE }

//! default sampling resolution for all MTF modules
#define MTF_RES 50

#if CV_MAJOR_VERSION < 3
#define cv_format(...) cv::format(__VA_ARGS__)
#else
#define cv_format(...) cv::format(__VA_ARGS__).c_str()
#endif

using namespace std;
using namespace Eigen;

_MTF_BEGIN_NAMESPACE

typedef unsigned char uchar;
typedef unsigned int uint;

typedef float EigPixT;
typedef float CVPixT;

typedef Matrix<double, 3, 4> Matrix34d;
typedef Matrix<double, 2, 4> Matrix24d;
typedef Matrix<double, 10, 1> Vector10d;
typedef Matrix<double, 8, 1> Vector8d;
typedef Matrix<double, 5, 1> Vector5d;
typedef Matrix<double, 6, 1> Vector6d;

typedef Matrix<double, 8, Dynamic> Matrix8Xd;
typedef Matrix<double, 16, Dynamic> Matrix16Xd;

typedef Matrix<double, Dynamic, 9> MatrixX9d;
typedef Matrix<double, 9, Dynamic> Matrix9Xd;

typedef Matrix<double, 8, 8> Matrix8d;
typedef Matrix<double, 7, 7> Matrix7d;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 2, 8> Matrix28d;
typedef Matrix<double, 2, 6> Matrix26d;
typedef Matrix<double, 9, 8> Matrix98d;
typedef Matrix<double, 8, 9> Matrix89d;
typedef Matrix<double, 8, 6> Matrix86d;
typedef Matrix<double, 8, 5> Matrix85d;
typedef Matrix<double, 8, 4> Matrix84d;
typedef Matrix<double, 8, 3> Matrix83d;
typedef Matrix<double, 7, 2> Matrix72d;
typedef Matrix<double, 6, 6> Matrix66d;
typedef Matrix<double, 5, 5> Matrix55d;
typedef Matrix<double, 4, 4> Matrix44d;
typedef Matrix<double, 4, 3> Matrix43d;
typedef Matrix<double, 3, 9> Matrix39d;
typedef Matrix<double, 2, 3> Matrix23d;

typedef Matrix<bool, Dynamic, 1> VectorXb;
typedef Array<bool, Dynamic, 1> ArryaXb;
typedef Matrix<double, 9, 1> Vector9d;
typedef Matrix<unsigned char, Dynamic, 1> VectorXc;

typedef Map<VectorXd> VectorXdM;
typedef Map<VectorXb> VectorXbM;
typedef Map<RowVectorXd> RowVectorXdM;
typedef Map<MatrixXd> MatrixXdM;
typedef Map<MatrixX2d> MatrixX2dM;
typedef Map<Matrix2d> Matrix2dM;

typedef Matrix<double, Dynamic, Dynamic, RowMajor> MatrixXdr;
typedef Matrix<double, Dynamic, 1, RowMajor> VectorXdr;
typedef Matrix<int, Dynamic, 1, RowMajor> VectorXir;
typedef Matrix<EigPixT, Dynamic, Dynamic, RowMajor> EigImgMat;

typedef Map<EigImgMat> EigImgT;
typedef Map<MatrixXdr> MatrixXdMr;

typedef VectorXd PixValT;
typedef MatrixX2d PixGradT;
typedef Matrix4Xd PixHessT;

typedef Matrix3d ProjWarpT;
typedef Matrix2Xd PtsT;
typedef Matrix2Xf PtsfT;
typedef Matrix24d CornersT;
typedef Matrix3Xd HomPtsT;
typedef Matrix34d HomCornersT;
typedef Matrix8Xd GradPtsT;
typedef Matrix16Xd HessPtsT;

typedef cv::Rect_<double> Rectd;

typedef std::vector<double> vectord;
typedef std::vector<float> vectorf;
typedef std::vector<int> vectori;
typedef std::vector<vectord> vectorvd;
typedef std::vector<vectori> vectorvi;

_MTF_END_NAMESPACE

#endif