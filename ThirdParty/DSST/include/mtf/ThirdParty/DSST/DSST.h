/*
 * DSST.h
 *
 *  Created on: 22 May, 2015
 *      Author: Sara & Mennatullah
 */

#define _USE_MATH_DEFINES

#ifndef MTF_DSST_H
#define MTF_DSST_H

#include <fstream>
#include <opencv2/core/core.hpp>

#include "Params.h"
#include "mtf/TrackerBase.h"
#include "mtf/Utilities/warpUtils.h"
//#include <windows.h>

using namespace std;
using namespace mtf;
//using namespace cv;


class DSSTTracker : public mtf::TrackerBase
{
	DSSTParams tParams;
	HOGParams hParams;
	cv::Mat currFrame;

public:
	trackingSetup tSetup;
	trackingSetup prev_tSetup;
	cv::Mat currCorners;
    bool first;
    int *idxs;
	cv::Mat convertFloatImg(cv::Mat &img);

	DSSTTracker(DSSTParams *params = NULL);
	void initialize(const cv::Mat& corners) override;
    void setRegion(const cv::Mat& corners) override;
	void update() override;
	void setImage(const cv::Mat &img) override{currFrame = img; }
	int inputType() const  override{ return CV_32FC1; }
	const cv::Mat& getRegion()  override{ return currCorners; }

private:
	//~DSSTTracker();
	cv::Mat inverseFourier(cv::Mat original, int flag=0);
    void reset_filters();
	cv::Mat createFourier(cv::Mat original, int flag=0);
	cv::Mat hann(int size);
	float *convert1DArray(cv::Mat &patch);
	cv::Mat convert2DImage(float *arr, int w, int h);
	cv::Point ComputeMaxDisplayfl(cv::Mat &img,string winName="FloatImg");
	cv::Mat *create_feature_map(cv::Mat& patch, int full, int &nChns, cv::Mat& Gray, bool scaling);
	//cv::Mat *create_feature_map2(cv::Mat& patch, int full, int &nChns, cv::Mat& Gray, bool scaling);
	cv::Mat get_scale_sample(cv::Mat img, trackingSetup tSetup, DSSTParams tParams, int &nDims,bool display = false);
	cv::Mat *get_translation_sample(cv::Mat img, trackingSetup tSet, int &nDims);
	void train(bool first, cv::Mat img, bool original);
	cv::Point updateCentroid(cv::Point oldC, int w , int h , int imgw, int imgh);
	cv::RotatedRect processFrame(cv::Mat img, bool enableScaling, bool enableRotating);
    cv::RotatedRect processFrame_bb(cv::Mat img, bool enableScaling, bool enableRotating, cv::RotatedRect bb);
	double computeCorrelationVariance(double *arr, int arrW, int arrH);
	cv::Mat convert2DImageFloat(double *arr, int w, int h);
	cv::Mat convertNormalizedFloatImg(cv::Mat &img);
	double *convert1DArrayDouble(cv::Mat &patch);
	double *computeMeanVariance(cv::Mat trans_response);
	void preprocess(cv::Mat img, cv::RotatedRect bb);
	cv::Mat visualize(cv::Rect rect, cv::Mat img, cv::Scalar scalar = cvScalarAll(0));
	cv::Point displayFloat(cv::Mat img);
    cv::Mat extract_rotated_patch(cv::Mat img, cv::RotatedRect bb);
	cv::Mat get_rot_sample(cv::Mat img, trackingSetup tSetup, DSSTParams tParams, int &nDims,bool display = false);
    void myGetQuadrangleSubPix(const cv::Mat& src, cv::Mat& dst,cv::Mat& m );
    cv::RotatedRect getBestFitRotatedRect(cv::Point2f tl, cv::Point2f tr, cv::Point2f bl, cv::Point2f br, cv::Mat corners);
    int getNearest(cv::Point2f pt, std::vector<cv::Point2f> pts);
    cv::Mat rotateBox(float angle, float cx, float cy, cv::Mat corners );
    void getIndices(std::vector<cv::Point2f> pts, int *indices);
    void getQuadrangleSubPix_8u32f_CnR( const uchar* src, size_t src_step, cv::Size src_size,float* dst, 
            size_t dst_step, cv::Size win_size,const double *matrix, int cn );
};

#endif
