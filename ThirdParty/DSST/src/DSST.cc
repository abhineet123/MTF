/*
 * DSST.cpp
 *
 *  Created on: 22 May, 2015
 *      Author: Sara & Mennatullah
 */

#include "mtf/ThirdParty/DSST/DSST.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <math.h>
#include "mtf/ThirdParty/DSST/HOG.h"
#include "mtf/Utilities/miscUtils.h"
#include "opencv2/calib3d/calib3d.hpp"

#include <string>

#ifdef _WIN32
#pragma warning(disable:4244)
#pragma warning(disable:4800)
#pragma warning(disable:4101)
#endif

#define eps 0.0001

// unit vectors used to compute gradient orientation
static double uu[9] = { 1.0000, 0.9397, 0.7660, 0.500, 0.1736, -0.1736, -0.5000, -0.7660, -0.9397 };
static double vv[9] = { 0.0000, 0.3420, 0.6428, 0.8660, 0.9848, 0.9848, 0.8660, 0.6428, 0.3420 };

/*static inline double round(double num)
{
return (num > 0.0) ? floor(num + 0.5) : ceil(num - 0.5);
}*/
static inline float min(float x, float y)
{
	return (x <= y ? x : y);
}
static inline float max(float x, float y)
{
	return (x <= y ? y : x);
}

/*static inline double round(double num)
{
return (num > 0.0) ? floor(num + 0.5) : ceil(num - 0.5);
}*/
/*static inline int min(int x, int y)
{
return (x <= y ? x : y);
}
static inline int max(int x, int y)
{
return (x <= y ? y : x);
}
*/

void DSST::reset_filters()
{
    for(int i=0; i<tSetup.nNumTrans; i++)
        tSetup.num_trans[i]= prev_tSetup.num_trans[i];
    tSetup.den_trans= prev_tSetup.den_trans;
    if(tParams.is_scaling)
    {
        for(int i=0; i<tSetup.nNumScale; i++)
            tSetup.num_scale[i]= prev_tSetup.num_scale[i];
        tSetup.den_scale= prev_tSetup.den_scale;
    }
    if(tParams.is_rotating)
    {
        for(int i=0; i<tSetup.nNumRot; i++)
            tSetup.num_rot[i]= prev_tSetup.num_rot[i];
        tSetup.den_rot= prev_tSetup.den_rot;
    }
}

void DSST::setRegion(const cv::Mat& corners){

    //Reset Correlation Filters
    reset_filters();
    
    //Parse New corners
    cv::Point2f bl((float)corners.at<double>(0,3), (float)corners.at<double>(1,3));
    cv::Point2f br((float)corners.at<double>(0,2), (float)corners.at<double>(1,2));
    cv::Point2f tl((float)corners.at<double>(0,0), (float)corners.at<double>(1,0));
    cv::Point2f tr((float)corners.at<double>(0,1), (float)corners.at<double>(1,1));
    cout<<"setregion point "<<tl.x<<" "<<tl.y<<endl;
    cout<<"setregion point "<<bl.x<<" "<<bl.y<<endl;
    cout<<"setregion point "<<tr.x<<" "<<tr.y<<endl;
    cout<<"setregion point "<<br.x<<" "<<br.y<<endl;

    cv::RotatedRect init_bb;
    if (!tParams.is_rotating)
    {
    	mtf::Rectd best_fit_rect = mtf::utils::getBestFitRectangle<double>(corners, currFrame.cols, currFrame.rows);
        init_bb.center.x = (best_fit_rect.x+best_fit_rect.width/2) / tParams.resize_factor;
        init_bb.center.y = (best_fit_rect.y+best_fit_rect.height/2) / tParams.resize_factor;
        init_bb.size = cv::Size(best_fit_rect.width / tParams.resize_factor, best_fit_rect.height / tParams.resize_factor);
        init_bb.angle=0;
    }
    else
    {
        std::vector<cv::Point2f> corners_vec;
//        for (int i=0; i<4; i++)
//            corners_vec.push_back(cv::Point2f(corners.at<double>(0,i)/tParams.resize_factor, corners.at<double>(1,i)/tParams.resize_factor));
        corners_vec.push_back(cv::Point2f(bl.x/tParams.resize_factor, bl.y/tParams.resize_factor));
        corners_vec.push_back(cv::Point2f(tl.x/tParams.resize_factor, tl.y/tParams.resize_factor));
        corners_vec.push_back(cv::Point2f(tr.x/tParams.resize_factor, tr.y/tParams.resize_factor));
        corners_vec.push_back(cv::Point2f(br.x/tParams.resize_factor, br.y/tParams.resize_factor));

		Vector3d result = mtf::utils::computeIsometryDLT(
			mtf::utils::Corners(tSetup.init_bb).eig(), mtf::utils::Corners(corners).eig());
       // init_bb.size= cv::Size(result[0]*tSetup.original.width,result[1]*tSetup.original.height);
        
        init_bb= cv::minAreaRect(corners_vec);
	    //Matrix3d result= computeSimilitudeDLT(tSetup.init_bb, corners);
        init_bb.angle= result[2];
        //
//        if (init_bb.angle <-10)
//            init_bb.angle += 90;
    }

    cout<<"Init point "<<init_bb.center<<" "<<init_bb.angle<<" "<<init_bb.size<<endl;

    //process new frame with new corners
 	cv::Mat scaledCurrFrame;
	resize(currFrame, scaledCurrFrame, cv::Size(currFrame.cols / tParams.resize_factor, currFrame.rows / tParams.resize_factor));
	cv::RotatedRect rect = processFrame_bb(scaledCurrFrame, tParams.is_scaling, tParams.is_rotating, init_bb);
   
    //output new corners
    rect.center.x= rect.center.x*tParams.resize_factor;
    rect.center.y= rect.center.y*tParams.resize_factor;
    rect.size.width= rect.size.width*tParams.resize_factor;
    rect.size.height= rect.size.height*tParams.resize_factor;
    
    cv::Point2f pts[4];
    rect.points(pts);
    for (int i=0; i<4; i++)
        cout<<"Tracked point "<<pts[i].x<<" "<<pts[i].y<<endl;
    std::vector<cv::Point2f> src;
    src.push_back(pts[0]);
    src.push_back(pts[1]);
    src.push_back(pts[2]);
    src.push_back(pts[3]);

    cv::Mat corners2(2, 4, CV_64FC1);
	corners2.at<double>(0, 0) = pts[idxs[0]].x; //tlx
    corners2.at<double>(0, 1) = pts[idxs[1]].x; //trx
	corners2.at<double>(0, 2) = pts[idxs[2]].x;//brx
	corners2.at<double>(0, 3) = pts[idxs[3]].x;//blx
	corners2.at<double>(1, 0) = pts[idxs[0]].y;//ulx
	corners2.at<double>(1, 1) = pts[idxs[1]].y;//urx
	corners2.at<double>(1, 2) = pts[idxs[2]].y;//bry
	corners2.at<double>(1, 3) = pts[idxs[3]].y;//bly
    currCorners = corners2;

 }

DSSTParams::DSSTParams(double padding_, double output_sigma_factor_, double scale_sigma_factor_, double lambda_,
	double learning_rate_, int number_scales_, double scale_step_, int number_rots_, double rot_step_, int resize_factor_, int is_scaling_, int is_rotating_, int bin_size_)
{
	padding = padding_;
	output_sigma_factor = output_sigma_factor_;
	scale_sigma_factor = scale_sigma_factor_;
	lambda = lambda_;
	learning_rate = learning_rate_;
	const_learning_rate = 0.01;
	number_scales = number_scales_;
	number_rots = number_rots_;
	scale_step = scale_step_;
	rot_step = rot_step_;
	resize_factor = resize_factor_;
	is_scaling = is_scaling_;
	is_rotating = is_rotating_;
	bin_size = bin_size_;
	scale_model_max_area = 512;
}

DSSTParams::DSSTParams(const DSSTParams *params) {
	if(params){
		padding = params->padding;//1;
		output_sigma_factor = params->output_sigma_factor;//1.0/16;
		scale_sigma_factor = params->scale_sigma_factor;//1.0/4;
		lambda = params->lambda;//1e-2;
		learning_rate = params->learning_rate;//0.035;//0.025;
		const_learning_rate = 0.01;
		number_scales = params->number_scales;//33;
		number_rots = params->number_rots;//33;
		scale_step = params->scale_step;//1.02;
		rot_step = params->rot_step;//1.02;
		resize_factor = params->resize_factor;
		is_scaling = params->is_scaling;
		is_rotating = params->is_rotating;
		bin_size = params->bin_size;
		//std::cout<<"Setting params : "<<padding<<" "<<output_sigma_factor<<" "<<scale_sigma_factor<<" "<<lambda;
		//std::cout << " " << learning_rate << " " << number_scales << " " << scale_step << " " << resize_factor << " " << is_scaling << " " << bin_size << std::endl;
		scale_model_max_area = 512;
	} else
	{
		padding = 1;
		output_sigma_factor = 1.0 / 16;
		scale_sigma_factor = 1.0 / 4;
		lambda = 1e-2;
		learning_rate = 0.035;//0.025;
		number_scales = 33;
		number_rots = 21;
		scale_step = 1.02;
		rot_step = 2;
		resize_factor = 2;
		is_scaling = 1;
		is_rotating = 1;
		bin_size = 1;
		scale_model_max_area = 512;
	}
}

cv::Mat DSST::convertFloatImg(cv::Mat &img)
{
	cv::Mat imgU;
	double minVal;
	double maxVal;
	cv::Point minLoc;
	cv::Point maxLoc;
	minMaxLoc(img, &minVal, &maxVal, &minLoc, &maxLoc);

	img -= minVal;
	//img.convertTo(imgU,CV_8U,255.0/(maxVal-minVal));
	//cout<<"Converting float image to 8 unsigned char"<<endl;
	img.convertTo(imgU, CV_8U);
	return imgU;
}

DSST::DSST(DSSTParams *params) :
TrackerBase(), tParams(params)
{
	name = "dsst";
	tSetup.num_trans = 0;
	//tSetup.enableScaling= false;
	tSetup.num_scale = 0;
	tSetup.num_rot = 0;

	printf("\n");
	printf("Using Discriminative Scale Space Tracker with:\n");
	printf("padding: %f\n", tParams.padding);
	printf("scale_sigma_factor: %f\n", tParams.scale_sigma_factor);
	printf("lambda: %f\n", tParams.lambda);
	printf("learning_rate: %f\n", tParams.learning_rate);
	printf("number_scales: %d\n", tParams.number_scales);
	printf("number_rots: %d\n", tParams.number_rots);
	printf("scale_step: %f\n", tParams.scale_step);
	printf("rot_step: %f\n", tParams.rot_step);
	printf("scale_model_max_area: %d\n", tParams.scale_model_max_area);
	printf("resize_factor: %d\n", tParams.resize_factor);
	printf("is_scaling: %d\n", tParams.is_scaling);
	printf("is_rotating: %d\n", tParams.is_rotating);
	printf("bin_size: %d\n", tParams.bin_size);
	printf("\n");

	//currFrame= convertFloatImg(img);
	//currFrame = img;
	//currFrame.data = img.data;
	//imshow("testing sth", currFrame);
	//waitKey();
}

cv::Mat DSST::rotateBox(float angle, float cx, float cy, cv::Mat corners )
{
    cv::Mat rotation_mat(3,3,CV_64FC1);
    rotation_mat.at<double>(0,0)= cos(angle*PI/180);
    rotation_mat.at<double>(0,1)= -sin(angle*PI/180);
    rotation_mat.at<double>(0,2)= -cx;
    rotation_mat.at<double>(1,0)= sin(angle*PI/180);
    rotation_mat.at<double>(1,1)= cos(angle*PI/180);
    rotation_mat.at<double>(1,2)= -cy;
    rotation_mat.at<double>(2,0)=0; 
    rotation_mat.at<double>(2,1)=0; 
    rotation_mat.at<double>(2,2)=1; 
    cv::Mat corners_hom(3,4, CV_64FC1);
    for(int i=0; i<2; i++)
        for(int j=0; j<4; j++)
            corners_hom.at<double>(i,j)= corners.at<double>(i,j);
    corners_hom.at<double>(2,0)=1;
    corners_hom.at<double>(2,1)=1;
    corners_hom.at<double>(2,2)=1;
    corners_hom.at<double>(2,3)=1;
    cv::Mat rotated_corners= rotation_mat*corners_hom;
    return rotated_corners;
}

cv::RotatedRect DSST::getBestFitRotatedRect(cv::Point2f tl, cv::Point2f tr, cv::Point2f bl, cv::Point2f br, cv::Mat corners)
{
    cv::RotatedRect init_bb;
    cv::Point2f A;
    if (br.y>bl.y)
        A=bl-br;
    else
        A= br-bl;
	float a[2] = {
		static_cast<float>(A.x / sqrt(pow(A.x, 2) + pow(A.y, 2))), 
		static_cast<float>(A.y / sqrt(pow(A.x, 2) + pow(A.y, 2)))
	};
    float b[2] = {1.0, 0.0};
    cv::Mat AA(1,2,CV_32FC1,a);
    cv::Mat BB(1,2,CV_32FC1,b);
    init_bb.angle= -1*acos(AA.dot(BB))*180/PI;

    init_bb.center.x=  (float)(tl.x+tr.x+br.x+bl.x)/4;
    init_bb.center.x= init_bb.center.x / tParams.resize_factor;
    init_bb.center.y= (float)(tl.y+tr.y+br.y+bl.y)/4;
    init_bb.center.y= init_bb.center.y / tParams.resize_factor;
    
    //rotate rectangle to its original position
    cv::Mat rotated_corners= rotateBox(init_bb.angle, init_bb.center.x, init_bb.center.y, corners);
    br.x= rotated_corners.at<double>(0,2)+init_bb.center.x;
    bl.x= rotated_corners.at<double>(0,3)+init_bb.center.x;
    bl.y= rotated_corners.at<double>(1,3)+init_bb.center.y;
    tl.y= rotated_corners.at<double>(1,0)+init_bb.center.y;
    float w= max(br.x, bl.x)- min(br.x, bl.x);
    w= w<4? 4 : w;
    float h= max(bl.y, tl.y)- min(bl.y, tl.y);
    h= h<4? 4 : h;

    cout<<"W= "<<w<<" "<<h<<endl;
    init_bb.size= cv::Size(w / tParams.resize_factor, h / tParams.resize_factor);
    cout<<"Size "<<init_bb.size<<endl;
    return init_bb;
}
void DSST::initialize(const cv::Mat& corners)
{
    cv::Point2f bl((float)corners.at<double>(0,3), (float)corners.at<double>(1,3));
    cv::Point2f br((float)corners.at<double>(0,2), (float)corners.at<double>(1,2));
    cv::Point2f tl((float)corners.at<double>(0,0), (float)corners.at<double>(1,0));
    cv::Point2f tr((float)corners.at<double>(0,1), (float)corners.at<double>(1,1));
 
    cv::RotatedRect init_bb;
    if (!tParams.is_rotating)
    {
    	mtf::Rectd best_fit_rect = mtf::utils::getBestFitRectangle<double>(corners, currFrame.cols, currFrame.rows);
        init_bb.center.x = (best_fit_rect.x+best_fit_rect.width/2) / tParams.resize_factor;
        init_bb.center.y = (best_fit_rect.y+best_fit_rect.height/2) / tParams.resize_factor;
        init_bb.size = cv::Size(best_fit_rect.width / tParams.resize_factor, best_fit_rect.height / tParams.resize_factor);
        init_bb.angle=0;
    }
    else
    {
        std::vector<cv::Point2f> corners_vec;
        for (int i=0; i<4; i++)
            corners_vec.push_back(cv::Point2f(corners.at<double>(0,i)/tParams.resize_factor, corners.at<double>(1,i)/tParams.resize_factor));
        init_bb= cv::minAreaRect(corners_vec);
    } 
    tSetup.init_bb= corners; 
    cv::Mat scaledCurrFrame;
	resize(currFrame, scaledCurrFrame, cv::Size(currFrame.cols / tParams.resize_factor, currFrame.rows / tParams.resize_factor));
    idxs= new int[4];

	preprocess(scaledCurrFrame, init_bb);
    first= true;

	cv::Point2f pts[4];
	init_bb.points(pts);

	currCorners = corners;
}

int DSST::getNearest(cv::Point2f pt, std::vector<cv::Point2f> pts)
{
    int minIndex=-1;
    float minD=1000;

    for(int i=0; i<4; i++)
    {
        float d= sqrt( pow(pts[i].x-pt.x, 2) + pow(pts[i].y-pt.y,2));

        if (d<minD)
        {
            minD= d;
            minIndex=i;
        }
    }
    return minIndex;
}

void DSST::getIndices(std::vector<cv::Point2f> pts, int *indices)
{
    for (int i=0; i<4; i++)
    {
        indices[i]= getNearest(cv::Point2f(currCorners.at<double>(0,i), currCorners.at<double>(1,i) ), pts);
    }
}
void DSST::update()
{
	cv::Mat scaledCurrFrame;
	resize(currFrame, scaledCurrFrame, cv::Size(currFrame.cols / tParams.resize_factor, currFrame.rows / tParams.resize_factor));
	cv::RotatedRect rect = processFrame(scaledCurrFrame, tParams.is_scaling, tParams.is_rotating);

    rect.center.x= rect.center.x*tParams.resize_factor;
    rect.center.y= rect.center.y*tParams.resize_factor;
    rect.size.width= rect.size.width*tParams.resize_factor;
    rect.size.height= rect.size.height*tParams.resize_factor;
    
    cv::Point2f pts[4];
    rect.points(pts);

    std::vector<cv::Point2f> src;
    src.push_back(pts[0]);
    src.push_back(pts[1]);
    src.push_back(pts[2]);
    src.push_back(pts[3]);

   if(first)
    {
        first= false;
        getIndices(src, idxs);
    }
    cv::Mat corners(2, 4, CV_64FC1);
	corners.at<double>(0, 0) = pts[idxs[0]].x; //tlx
    corners.at<double>(0, 1) = pts[idxs[1]].x; //trx
	corners.at<double>(0, 2) = pts[idxs[2]].x;//brx
	corners.at<double>(0, 3) = pts[idxs[3]].x;//blx
	corners.at<double>(1, 0) = pts[idxs[0]].y;//ulx
	corners.at<double>(1, 1) = pts[idxs[1]].y;//urx
	corners.at<double>(1, 2) = pts[idxs[2]].y;//bry
	corners.at<double>(1, 3) = pts[idxs[3]].y;//bly
    currCorners = corners;
}

cv::Mat DSST::inverseFourier(cv::Mat original, int flag)
{
	cv::Mat output;
	cv::idft(original, output, cv::DFT_REAL_OUTPUT | cv::DFT_SCALE);  // Applying DFT without padding
	return output;
}

cv::Mat DSST::createFourier(cv::Mat original, int flag)
{
	cv::Mat planes[] = { cv::Mat_<double>(original), cv::Mat::zeros(original.size(), CV_64F) };
	cv::Mat complexI;
	cv::merge(planes, 2, complexI);
	cv::dft(complexI, complexI, flag);  // Applying DFT without padding
	return complexI;
}

cv::Mat DSST::hann(int size)
{
	cv::Mat arr(size, 1, CV_32FC1);
	float multiplier;
	for(int i = 0; i < size; i++)
	{
		multiplier = 0.5 * (1 - cos(2 * M_PI*i / (size - 1)));
		*((float *)(arr.data + i*arr.step[0])) = multiplier;
	}
	return arr;
}

cv::Mat DSST::convert2DImageFloat(double *arr, int w, int h)
{
	int k = 0;
	cv::Mat img(h, w, CV_32FC1);

	for(int i = 0; i < img.cols; i++)
		for(int j = 0; j < img.rows; j++)
		{
			img.at<float>(j, i) = (float)arr[k];
			k++;
		}

	return img;
}

cv::Mat DSST::convertNormalizedFloatImg(cv::Mat &img)
{
	cv::Mat imgU;
	double minVal;
	double maxVal;
	cv::Point minLoc;
	cv::Point maxLoc;
	minMaxLoc(img, &minVal, &maxVal, &minLoc, &maxLoc);

	img -= minVal;
	img.convertTo(imgU, CV_64FC1, 1.0 / (maxVal - minVal));
	return imgU;
}

double *DSST::computeMeanVariance(cv::Mat trans_response)
{
	double mean = 0;
	for(int i = 0; i < trans_response.rows; i++)
	{
		for(int j = 0; j < trans_response.cols; j++)
		{
			mean += trans_response.at<double>(i, j);
		}
	}
	mean = mean / (trans_response.rows*trans_response.cols);

	double variance = 0;
	for(int i = 0; i < trans_response.rows; i++)
	{
		for(int j = 0; j < trans_response.cols; j++)
		{
			variance += pow(trans_response.at<double>(i, j) - mean, 2);
		}
	}
	//cout<<"Variance "<<variance<<endl;
	variance = variance / (trans_response.cols*trans_response.rows);
	//cout<<"Variance again "<<variance<<endl;
	double *params = new double[2];
	params[0] = mean;
	params[1] = sqrt(variance);

	//cout<<"Variance last time"<<params[1]<<endl;

	return params;
}

double DSST::computeCorrelationVariance(double *arr, int arrW, int arrH)
{
	//Assert on the width and height of the image!
	if(arrW != tSetup.padded.width || arrH != tSetup.padded.height)
	{
		cout << "Error in the size of the patch" << endl;
		return -1;
	}

	//Convert 1D array to 2D patch
	cv::Mat patch = convert2DImageFloat(arr, arrW, arrH);
	cv::Mat roiGrayFlot = patch;
	//imshow("testing patch ", patch);
	//waitKey();

	int nDims = 0;
	roiGrayFlot = roiGrayFlot.mul((float)1 / 255);
	roiGrayFlot = roiGrayFlot - 0.5;
	//imshow("testing float ", roiGrayFlot);
	//waitKey();

	//Compute HOG  features
	int nChns;
	cv::Mat *featureMap = create_feature_map(patch, 1, nChns, roiGrayFlot, false);

	//Compute Correlation
	nDims = nChns;
	for(int i = 0; i < nChns; i++)
		featureMap[i] = featureMap[i].mul(tSetup.trans_cos_win);

	cv::Mat *feature_map_fourier = new cv::Mat[nDims];
	for(int i = 0; i < nDims; i++)
	{
		cv::Mat feature_map_double(featureMap[i].rows, featureMap[i].cols, CV_64FC1);
		featureMap[i].convertTo(feature_map_double, CV_64FC1);
		feature_map_fourier[i] = createFourier(feature_map_double);
	}

	cv::Mat* temp = new cv::Mat[nDims];
	for(int i = 0; i < nDims; i++)
		mulSpectrums(tSetup.num_trans[i], feature_map_fourier[i], temp[i], 0, false);

	int w = tSetup.num_trans[0].cols, h = tSetup.num_trans[0].rows;

	cv::Mat sumDen(h, w, CV_64F);

	for(int j = 0; j < h; j++)
		for(int k = 0; k < w; k++)
			sumDen.at<double>(j, k) = tSetup.den_trans.at<cv::Vec2d>(j, k)[0] + tParams.lambda;

	cv::Mat sumTemp(h, w, CV_64FC2);
	sumTemp = cv::Mat::zeros(sumTemp.size(), CV_64FC2);
	for(int j = 0; j < h; j++)
		for(int k = 0; k < w; k++)
		{
			for(int i = 0; i < nDims; i++)
				sumTemp.at<cv::Vec2d>(j, k) += temp[i].at<cv::Vec2d>(j, k);

			sumTemp.at<cv::Vec2d>(j, k) /= sumDen.at<double>(j, k);
		}

	//Compute Final Translation Response
	cv::Mat trans_response = cv::Mat::zeros(sumTemp.rows, sumTemp.cols, CV_64FC1);
	trans_response = inverseFourier(sumTemp);
	cv::Mat trans2 = convertNormalizedFloatImg(trans_response);
	//imshow("Fake Translation Response ", trans2);
	//waitKey();

	double *params = computeMeanVariance(trans2);
	double var = params[1];

	delete[] params;
	delete[] featureMap;
	delete[] feature_map_fourier;
	delete[] temp;

	return var;
}

float *DSST::convert1DArray(cv::Mat &patch)
{
	float *img = (float*)calloc(patch.rows*patch.cols, sizeof(float));

	int k = 0;
	for(int i = 0; i < patch.cols; i++)
		for(int j = 0; j < patch.rows; j++)
		{
			img[k] = (float)patch.at<float>(j, i);
			k++;
		}
	return img;
}

double *DSST::convert1DArrayDouble(cv::Mat &patch)
{
	double *img = (double*)calloc(patch.rows*patch.cols, sizeof(double));

	int k = 0;
	for(int i = 0; i < patch.cols; i++)
		for(int j = 0; j < patch.rows; j++)
		{
			img[k] = (double)patch.at<float>(j, i);
			k++;
		}
	return img;
}

cv::Mat DSST::convert2DImage(float *arr, int w, int h)
{
	int k = 0;
	cv::Mat img(h, w, CV_32F);

	for(int i = 0; i < img.cols; i++)
		for(int j = 0; j < img.rows; j++)
		{
			img.at<float>(j, i) = arr[k];
			k++;
		}

	cv::Mat imgU;
	double minVal;
	double maxVal;
	cv::Point minLoc;
	cv::Point maxLoc;
	minMaxLoc(img, &minVal, &maxVal, &minLoc, &maxLoc);
	img -= minVal;
	img.convertTo(imgU, CV_8U, 255.0 / (maxVal - minVal));

	return imgU;
}
cv::Point DSST::ComputeMaxDisplayfl(cv::Mat &img, string winName)
{
	cv::Mat imgU;
	double minVal;
	double maxVal;
	cv::Point minLoc;
	cv::Point maxLoc;
	minMaxLoc(img, &minVal, &maxVal, &minLoc, &maxLoc);

	return maxLoc;
}

cv::Mat *DSST::create_feature_map(cv::Mat& patch, int full, int &nChns, cv::Mat& Gray, bool scaling)
{
	int h = patch.rows, w = patch.cols;
	float* M = (float*)calloc(h*w, sizeof(float));
	float* O = (float*)calloc(h*w, sizeof(float));

	float *img = convert1DArray(patch);
	//patch= convert2DImage(img, w, h);
	//cv::Mat patch8U= convertFloatImg(patch);
	//cout<<"Before Computing Gradients"<<endl;
	gradMag(img, M, O, h, w, 1, full);
	//imshow("Patch testing", patch8U);
	//waitKey();
	//cout<<"After Computing Gradients"<<endl;
	if(!scaling)
	{
		hParams.binSize = tParams.bin_size;//1;
	} else
	{
		hParams.binSize = 4;
	}
	int hb = h / hParams.binSize; int wb = w / hParams.binSize;

	nChns = hParams.nOrients * 3 + 5;
	float *H = (float*)calloc(hb*wb*nChns, sizeof(float));
	//cout<<"Before FHOG SSE"<<endl;
	fhogSSE(M, O, H, h, w, hParams.binSize, hParams.nOrients, hParams.softBin, hParams.clipHog);
	//cout<<"After FHOG SSE"<<endl;
	cv::Mat GrayRes;
	if(!scaling)
		resize(Gray, GrayRes, cv::Size(wb, hb));
	//cout<<"Gray size patch "<<GrayRes.cols<<" "<<GrayRes.rows<<" "<<wb<<" "<<hb<<endl;
	int l = 0;
	cv::Mat *featureMap;
	if(!scaling)
	{
		nChns = 28;
		featureMap = new cv::Mat[nChns];
		for(int i = 0; i < nChns; i++)
			featureMap[i] = cv::Mat(hb, wb, CV_32FC1);

		GrayRes.convertTo(featureMap[0], CV_32FC1);
		for(int j = 0; j < wb; j++)
			for(int i = 0; i < hb; i++)
				for(int k = 0; k < nChns - 1; k++)
					featureMap[k + 1].at<float>(i, j) = H[k*(hb*wb) + j*hb + i];

		//cout<<"finished the feature map "<<endl;
	} else
	{
		nChns = 31;
		featureMap = new cv::Mat[nChns];
		for(int i = 0; i < nChns; i++)
			featureMap[i] = cv::Mat(hb, wb, CV_32FC1);

		for(int j = 0; j < wb; j++)
			for(int i = 0; i < hb; i++)
				for(int k = 0; k < nChns; k++)
					featureMap[k].at<float>(i, j) = H[k*(hb*wb) + j*hb + i];


	}

	free(img);
	free(H);
	free(M);
	free(O);

	return featureMap;
}

void DSST::getQuadrangleSubPix_8u32f_CnR(const uchar* src, size_t src_step, cv::Size src_size,
	float* dst, size_t dst_step, cv::Size win_size,
	const double *matrix, int cn)
{
	int x, y, k;
	double A11 = matrix[0], A12 = matrix[1], A13 = matrix[2];
	double A21 = matrix[3], A22 = matrix[4], A23 = matrix[5];

	src_step /= sizeof(src[0]);
	dst_step /= sizeof(dst[0]);

	for(y = 0; y < win_size.height; y++, dst += dst_step)
	{
		double xs = A12*y + A13;
		double ys = A22*y + A23;
		double xe = A11*(win_size.width - 1) + A12*y + A13;
		double ye = A21*(win_size.width - 1) + A22*y + A23;

		if((unsigned)(cvFloor(xs) - 1) < (unsigned)(src_size.width - 3) &&
			(unsigned)(cvFloor(ys) - 1) < (unsigned)(src_size.height - 3) &&
			(unsigned)(cvFloor(xe) - 1) < (unsigned)(src_size.width - 3) &&
			(unsigned)(cvFloor(ye) - 1) < (unsigned)(src_size.height - 3))
		{
			for(x = 0; x < win_size.width; x++)
			{
				int ixs = cvFloor(xs);
				int iys = cvFloor(ys);
				const uchar *ptr = src + src_step*iys;
				float a = (float)(xs - ixs), b = (float)(ys - iys), a1 = 1.f - a, b1 = 1.f - b;
				float w00 = a1*b1, w01 = a*b1, w10 = a1*b, w11 = a*b;
				xs += A11;
				ys += A21;

				if(cn == 1)
				{
					ptr += ixs;
					dst[x] = ptr[0] * w00 + ptr[1] * w01 + ptr[src_step] * w10 + ptr[src_step + 1] * w11;
				} else if(cn == 3)
				{
					ptr += ixs * 3;
					float t0 = ptr[0] * w00 + ptr[3] * w01 + ptr[src_step] * w10 + ptr[src_step + 3] * w11;
					float t1 = ptr[1] * w00 + ptr[4] * w01 + ptr[src_step + 1] * w10 + ptr[src_step + 4] * w11;
					float t2 = ptr[2] * w00 + ptr[5] * w01 + ptr[src_step + 2] * w10 + ptr[src_step + 5] * w11;

					dst[x * 3] = t0;
					dst[x * 3 + 1] = t1;
					dst[x * 3 + 2] = t2;
				} else
				{
					ptr += ixs*cn;
					for(k = 0; k < cn; k++)
						dst[x*cn + k] = ptr[k] * w00 + ptr[k + cn] * w01 +
						ptr[src_step + k] * w10 + ptr[src_step + k + cn] * w11;
				}
			}
		} else
		{
			for(x = 0; x < win_size.width; x++)
			{
				int ixs = cvFloor(xs), iys = cvFloor(ys);
				float a = (float)(xs - ixs), b = (float)(ys - iys), a1 = 1.f - a, b1 = 1.f - b;
				float w00 = a1*b1, w01 = a*b1, w10 = a1*b, w11 = a*b;
				const uchar *ptr0, *ptr1;
				xs += A11; ys += A21;

				if((unsigned)iys < (unsigned)(src_size.height - 1))
					ptr0 = src + src_step*iys, ptr1 = ptr0 + src_step;
				else
					ptr0 = ptr1 = src + (iys < 0 ? 0 : src_size.height - 1)*src_step;

				if((unsigned)ixs < (unsigned)(src_size.width - 1))
				{
					ptr0 += ixs*cn; ptr1 += ixs*cn;
					for(k = 0; k < cn; k++)
						dst[x*cn + k] = ptr0[k] * w00 + ptr0[k + cn] * w01 + ptr1[k] * w10 + ptr1[k + cn] * w11;
				} else
				{
					ixs = ixs < 0 ? 0 : src_size.width - 1;
					ptr0 += ixs*cn; ptr1 += ixs*cn;
					for(k = 0; k < cn; k++)
						dst[x*cn + k] = ptr0[k] * b1 + ptr1[k] * b;
				}
			}
		}
	}
}

//----------------------------------------------------------
// 
//----------------------------------------------------------
void DSST::myGetQuadrangleSubPix(const cv::Mat& src, cv::Mat& dst, cv::Mat& m)
{
	CV_Assert(src.channels() == dst.channels());

	cv::Size win_size = dst.size();
	double matrix[6];
	cv::Mat M(2, 3, CV_64F, matrix);
	m.convertTo(M, CV_64F);
	double dx = (win_size.width - 1)*0.5;
	double dy = (win_size.height - 1)*0.5;
	matrix[2] -= matrix[0] * dx + matrix[1] * dy;
	matrix[5] -= matrix[3] * dx + matrix[4] * dy;

	if(src.depth() == CV_8U && dst.depth() == CV_32F)
		getQuadrangleSubPix_8u32f_CnR(src.data, src.step, src.size(),
		(float*)dst.data, dst.step, dst.size(),
		matrix, src.channels());
	else
	{
		CV_Assert(src.depth() == dst.depth());
		cv::warpAffine(src, dst, M, dst.size(),
			cv::INTER_LINEAR + cv::WARP_INVERSE_MAP,
			cv::BORDER_REPLICATE);
	}
}

cv::Mat DSST::extract_rotated_patch(cv::Mat img, cv::RotatedRect bb)
{
	//cv::Mat img_clone= img.clone();
	cv::Mat dst(bb.size, CV_32FC1);
	cv::Mat rot_matrix(2, 3, CV_64FC1);
	float ang = bb.angle*CV_PI / 180.0;
	rot_matrix.at<double>(0, 0) = cos(ang);
	rot_matrix.at<double>(1, 0) = sin(ang);
	rot_matrix.at<double>(0, 1) = -sin(ang);
	rot_matrix.at<double>(1, 1) = cos(ang);
	rot_matrix.at<double>(0, 2) = bb.center.x;
	rot_matrix.at<double>(1, 2) = bb.center.y;
	myGetQuadrangleSubPix(img, dst, rot_matrix);
	cv::Mat dst_d = convertFloatImg(dst);
	/*    cout<<"Angle is "<<bb.angle<<endl;
		cv::imshow("testing", dst_d);
		cv::waitKey();*/
	return dst;
}

cv::Mat DSST::get_rot_sample(cv::Mat img, trackingSetup tSetup, DSSTParams tParams, int &nDims, bool display)
{
	cv::Mat featureMapRot;
	CvRect patchSize;

	cv::Mat roiGray;
	cv::Mat roiResized;
	cv::Mat feature_map_rot_fourier;

	float pw = tSetup.original.width*tSetup.current_scale_factor;
	float ph = tSetup.original.height*tSetup.current_scale_factor;

	int centerX = tSetup.centroid.x + 1;
	int centerY = tSetup.centroid.y + 1;
	tSetup.current_bb.center = cv::Point2f(centerX, centerY);
	tSetup.current_bb.size = cv::Size(pw, ph);
	for(int i = 0; i < tParams.number_rots; i++)
	{
		//        cout<<"Current rotation factor is "<<tSetup.rotFactors[i]<<" at "<<i<<" "<<tSetup.current_rot_factor<<endl;
		float angle = tSetup.rotFactors[i] + tSetup.current_rot_factor;
		tSetup.current_bb.angle = angle;

		cv::Mat patch = extract_rotated_patch(img, tSetup.current_bb);
		/*        cv::Mat patchFloat= convertFloatImg(patch);
				cv::imshow("testing", patchFloat);
				cv::waitKey();
				*/
		int interpolation;
		if(tSetup.scale_model_sz.width > patch.cols)
			interpolation = cv::INTER_LINEAR;
		else
			interpolation = cv::INTER_AREA;
		resize(patch, roiResized, cv::Size(tSetup.scale_model_sz.width, tSetup.scale_model_sz.height), 0, 0, interpolation);
		int nChns;
		cv::Mat m = cv::Mat();
		cv::Mat *featureMap = create_feature_map(roiResized, 1, nChns, m, true);

		float s = tSetup.rot_cos_win.at<float>(i, 0);

		if(featureMapRot.data == NULL)
		{
			featureMapRot = cv::Mat(featureMap[0].rows*featureMap[0].cols*nChns, tParams.number_rots, CV_32FC1);
			feature_map_rot_fourier = cv::Mat(featureMap[0].rows*featureMap[0].cols*nChns, tParams.number_rots, CV_32FC1);
		}

		int k = 0;

		for(int j = 0; j < nChns; j++)
		{
			for(int m = 0; m < featureMap[j].cols; m++)
				for(int l = 0; l < featureMap[j].rows; l++)
				{
					featureMapRot.at<float>(k, i) = featureMap[j].at<float>(l, m) * s;
					k++;
				}
		}
		delete[]featureMap;
		if(display){
			imshow("roi", patch);
			cv::waitKey();
		}
	}
	cv::Mat featureMapTempDouble;
	featureMapRot.convertTo(featureMapTempDouble, CV_64FC1);

	cv::Mat feature_map_rot_fourier_temp = createFourier(featureMapTempDouble, cv::DFT_ROWS);

	nDims = tParams.number_rots;

	return feature_map_rot_fourier_temp;
}


cv::Mat DSST::get_scale_sample(cv::Mat img, trackingSetup tSetup, DSSTParams tParams, int &nDims, bool display)
{
	cv::Mat featureMapScale;
	CvRect patchSize;

	cv::Mat roiGray;
	cv::Mat roiResized;
	cv::Mat feature_map_scale_fourier;
	for(int i = 0; i < tParams.number_scales; i++)
	{
		float pw = tSetup.scaleFactors[i] * tSetup.current_scale_factor*tSetup.original.width;
		float ph = tSetup.scaleFactors[i] * tSetup.current_scale_factor*tSetup.original.height;

		int tlX1, tlY1, w1, w2, h1, h2;
		tlX1 = max(0, (int)ceil(tSetup.centroid.x + 1 - pw / 2));
		int padToX = (int)ceil(tSetup.centroid.x + 1 - pw / 2) < 0 ? (int)ceil(tSetup.centroid.x + 1 - pw / 2) : 0;
		w1 = (padToX + pw);
		w2 = (tlX1 + w1) >= img.cols ? img.cols - tlX1 : w1;

		tlY1 = max(0, (int)ceil(tSetup.centroid.y + 1 - ph / 2));
		int padToY = (int)ceil(tSetup.centroid.y + 1 - ph / 2) < 0 ? (int)ceil(tSetup.centroid.y + 1 - ph / 2) : 0;
		h1 = (padToY + ph);
		h2 = (tlY1 + h1) >= img.rows ? img.rows - tlY1 : h1;

		cv::Rect rect(tlX1, tlY1, w2, h2);
		cv::Mat patch = img(rect);
		cv::Mat roi;

		copyMakeBorder(patch, roi, abs(padToY), h1 - h2, abs(padToX), w1 - w2, cv::BORDER_REPLICATE);
		cv::Rect patchSize = cv::Rect(tlX1, tlY1, roi.cols, roi.rows);

		int interpolation;
		if(tSetup.scale_model_sz.width > patchSize.width)
			interpolation = cv::INTER_LINEAR;
		else
			interpolation = cv::INTER_AREA;
		resize(roi, roiResized, cv::Size(tSetup.scale_model_sz.width, tSetup.scale_model_sz.height), 0, 0, interpolation);
		int nChns;
		cv::Mat m = cv::Mat();
        cv::Mat *featureMap = create_feature_map(roiResized, 1, nChns, m, true);
		float s = tSetup.scale_cos_win.at<float>(i, 0);

		if(featureMapScale.data == NULL)//i==0)
		{
			featureMapScale = cv::Mat(featureMap[0].rows*featureMap[0].cols*nChns, tParams.number_scales, CV_32FC1);
			feature_map_scale_fourier = cv::Mat(featureMap[0].rows*featureMap[0].cols*nChns, tParams.number_scales, CV_32FC1);
		}

		int k = 0;

		for(int j = 0; j < nChns; j++)
		{
			for(int m = 0; m < featureMap[j].cols; m++)
				for(int l = 0; l < featureMap[j].rows; l++)
				{
					featureMapScale.at<float>(k, i) = featureMap[j].at<float>(l, m) * s;
					k++;
				}
		}
		delete[]featureMap;
		//     display= 1;
		if(display){
			imshow("roi", roi);
			cv::waitKey();
		}
	}
	cv::Mat featureMapTempDouble;
	featureMapScale.convertTo(featureMapTempDouble, CV_64FC1);

	cv::Mat feature_map_scale_fourier_temp = createFourier(featureMapTempDouble, cv::DFT_ROWS);

	nDims = tParams.number_scales;

	return feature_map_scale_fourier_temp;
}

cv::Mat *DSST::get_translation_sample(cv::Mat img, trackingSetup tSet, int &nDims)
{
	float pw = tSetup.padded.width*tSet.current_scale_factor;
	float ph = tSetup.padded.height*tSet.current_scale_factor;
	int centerX = tSetup.centroid.x + 1;
	int centerY = tSetup.centroid.y + 1;

	int tlX1, tlY1, w1, w2, h1, h2;
	tlX1 = max(0, (int)ceil(centerX - pw / 2));
	int padToX = (int)ceil(centerX - pw / 2) < 0 ? (int)ceil(centerX - pw / 2) : 0;
	w1 = (padToX + pw);
	w2 = (tlX1 + w1) >= img.cols ? img.cols - tlX1 : w1;

	tlY1 = max(0, (int)ceil(centerY - ph / 2));
	int padToY = (int)ceil(centerY - ph / 2) < 0 ? (int)ceil(centerY - ph / 2) : 0;
	h1 = (padToY + ph);
	h2 = (tlY1 + h1) >= img.rows ? img.rows - tlY1 : h1;

	cv::Rect rect(tlX1, tlY1, w2, h2);
	cv::Mat patch = img(rect);
	cv::Mat roi;
	copyMakeBorder(patch, roi, abs(padToY), h1 - h2, abs(padToX), w1 - w2, cv::BORDER_REPLICATE);
	//cv::Mat roiDisplay= convertFloatImg(roi);
	//imshow("testing " , roiDisplay);
	//waitKey();
	cv::Rect patchSize = cv::Rect(tlX1, tlY1, roi.cols, roi.rows);

	int interpolation;
	if(tSetup.padded.width > patchSize.width)
		interpolation = cv::INTER_LINEAR;
	else
		interpolation = cv::INTER_AREA;

	resize(roi, roi, cv::Size(tSetup.padded.width, tSetup.padded.height), 0, 0, interpolation);
	//cv::Mat roiGray= roi;
	//cv::cvtColor(roi, roiGray, CV_BGR2GRAY);

	//cv::Mat roiGrayFlot(roiGray.rows, roiGray.cols, CV_32FC1);
	//roiGray.convertTo(roiGrayFlot,CV_32FC1);
	cv::Mat roiGrayFlot = roi;
	roiGrayFlot = roiGrayFlot.mul((float)1 / 255);
	roiGrayFlot = roiGrayFlot - 0.5;

	int hb, wb, nChns;
	//cout<<"Before creating feature map"<<endl;
	cv::Mat *featureMap = create_feature_map(roi, 1, nChns, roiGrayFlot, false);
	//cout<<"After creating feature map"<<featureMap[0].cols<<" "<<featureMap[0].rows<<endl;
	nDims = nChns;

	for(int i = 0; i < nChns; i++)
		featureMap[i] = featureMap[i].mul(tSetup.trans_cos_win);
	//cout<<"Multiplied with cosine window "<<endl; 

	return featureMap;
}

void DSST::train(bool first, cv::Mat img, bool original)//false when called from setRegion, true otherwise
{
	//cout<<"Entered training "<<endl;
	//Model update:
	//1- Extract samples ftrans and fscale from It at pt and st .
	//A- Extract translation sample
	int nDims = 0;
	cv::Mat *feature_map = get_translation_sample(img, tSetup, nDims);
	//cout<<"After translation sample "<<endl;

	//B- Compute Denominator Translation, Numerator Translation
	cv::Mat *feature_map_fourier = new cv::Mat[nDims];
	cv::Mat *num = new cv::Mat[nDims];

	cv::Mat den(feature_map[0].rows, feature_map[0].cols, CV_64FC2);
	den = cv::Mat::zeros(feature_map[0].rows, feature_map[0].cols, CV_64FC2);

//	cout<<"Feature Map Fourier Translation"<<endl;
	for(int i = 0; i < nDims; i++)
	{
		cv::Mat feature_map_double(feature_map[i].rows, feature_map[i].cols, CV_64FC1);
		feature_map[i].convertTo(feature_map_double, CV_64FC1);
		feature_map_fourier[i] = createFourier(feature_map_double);
		mulSpectrums(tSetup.transFourier, feature_map_fourier[i], num[i], 0, true);

		cv::Mat temp;
		mulSpectrums(feature_map_fourier[i], feature_map_fourier[i], temp, 0, true);
		den = den + temp;
	}
	//cout<<"Current Scale tany: "<<tSetup.current_scale_factor<<endl;

	int nDimsScale;
	cv::Mat *num_scale;
	cv::Mat den_scale;
	cv::Mat feature_map_scale_fourier;
	if(tParams.is_scaling)
	{
		//        std::cout<<"Beginning to get scale sample"<<std::endl;
		feature_map_scale_fourier = get_scale_sample(img, tSetup, tParams, nDimsScale);//I have to convert featuremap to double first
		//        std::cout<<"Finished getting scale sample"<<std::endl;

		num_scale = new cv::Mat[feature_map_scale_fourier.rows];
		den_scale = cv::Mat(1, nDimsScale, CV_64FC2);
		den_scale = cv::Mat::zeros(1, nDimsScale, CV_64FC2);

		for(int i = 0; i < feature_map_scale_fourier.rows; i++)
		{
			cv::Mat temp(1, nDimsScale, CV_64FC2);
			for(int j = 0; j < nDimsScale; j++)
				temp.at<cv::Vec2d>(0, j) = feature_map_scale_fourier.at<cv::Vec2d>(i, j);

			mulSpectrums(tSetup.scaleFourier, temp, num_scale[i], 0, true);

		}

		cv::Mat temp;
		mulSpectrums(feature_map_scale_fourier, feature_map_scale_fourier, temp, 0, true);
		//cout<<"Temp "<<temp.rows<<" "<<temp.cols<<endl;
		/*cout<<"Feature scale Fourier"<<endl;
		for(int i = 0 ; i < feature_map_scale_fourier.rows ; i++){
		for(int j = 0 ; j < feature_map_scale_fourier.cols ; j++)
		cout<<feature_map_scale_fourier.at<cv::Vec2d>(i,j)<<"  ";
		cout<<endl;
		}*/
		for(int i = 0; i < temp.cols; i++)
		{
			for(int j = 0; j < temp.rows; j++)
			{
				den_scale.at<cv::Vec2d>(0, i)[0] += temp.at<cv::Vec2d>(j, i)[0];
				den_scale.at<cv::Vec2d>(0, i)[1] += temp.at<cv::Vec2d>(j, i)[1];
			}
		}
	}
	//cout<<"Updating Model "<<endl;

	int nDimsRot;
	cv::Mat *num_rot;
	cv::Mat den_rot;
	cv::Mat feature_map_rot_fourier;
	if(tParams.is_rotating)
	{
//        std::cout<<"Beginning to get rotation sample"<<std::endl;
		feature_map_rot_fourier = get_rot_sample(img, tSetup, tParams, nDimsRot);//I have to convert featuremap to double first
//        std::cout<<"Finished getting rotation sample"<<std::endl;
		num_rot = new cv::Mat[feature_map_rot_fourier.rows];
		den_rot = cv::Mat(1, nDimsRot, CV_64FC2);
		den_rot = cv::Mat::zeros(1, nDimsRot, CV_64FC2);

		for(int i = 0; i < feature_map_rot_fourier.rows; i++)
		{
			cv::Mat temp(1, nDimsRot, CV_64FC2);
			for(int j = 0; j < nDimsRot; j++)
				temp.at<cv::Vec2d>(0, j) = feature_map_rot_fourier.at<cv::Vec2d>(i, j);

			mulSpectrums(tSetup.rotFourier, temp, num_rot[i], 0, true);

		}

		cv::Mat temp;
		mulSpectrums(feature_map_rot_fourier, feature_map_rot_fourier, temp, 0, true);
		//cout<<"Temp "<<temp.rows<<" "<<temp.cols<<endl;
		/*cout<<"Feature scale Fourier"<<endl;
		for(int i = 0 ; i < feature_map_scale_fourier.rows ; i++){
		for(int j = 0 ; j < feature_map_scale_fourier.cols ; j++)
		cout<<feature_map_scale_fourier.at<cv::Vec2d>(i,j)<<"  ";
		cout<<endl;
		}*/
		for(int i = 0; i < temp.cols; i++)
		{
			for(int j = 0; j < temp.rows; j++)
			{
				den_rot.at<cv::Vec2d>(0, i)[0] += temp.at<cv::Vec2d>(j, i)[0];
				den_rot.at<cv::Vec2d>(0, i)[1] += temp.at<cv::Vec2d>(j, i)[1];
			}
		}
	}
	//Update Our Model
	if(first)
	{
		tSetup.num_trans = num;
		tSetup.nNumTrans = nDims;
		tSetup.den_trans = den;
        prev_tSetup.num_trans= new cv::Mat[nDims];

		if(tParams.is_scaling)
		{
			tSetup.num_scale = num_scale;
			tSetup.nNumScale = nDimsScale;
			tSetup.den_scale = den_scale;
            prev_tSetup.num_scale= new cv::Mat[feature_map_scale_fourier.rows];
		}
		if(tParams.is_rotating)
		{
			tSetup.num_rot = num_rot;
			tSetup.nNumRot = nDimsRot;
			tSetup.den_rot = den_rot;
            prev_tSetup.num_rot= new cv::Mat[feature_map_rot_fourier.rows];
		}
	}

	else
	{
        if (original)
            prev_tSetup.den_trans= tSetup.den_trans;
		for(int i = 0; i < tSetup.nNumTrans; i++){
            if(original)
                prev_tSetup.num_trans[i]= tSetup.num_trans[i];
			tSetup.num_trans[i] = tSetup.num_trans[i].mul(1 - tParams.const_learning_rate) + num[i].mul(tParams.const_learning_rate);
        }
		tSetup.den_trans = tSetup.den_trans.mul(1 - tParams.const_learning_rate) + den.mul(tParams.const_learning_rate);
		delete[] num;

		if(tParams.is_scaling)
		{
            if (original)
                prev_tSetup.den_scale= tSetup.den_scale;
			for(int i = 0; i < feature_map_scale_fourier.rows; i++){
                if (original)
                    prev_tSetup.num_scale[i]= tSetup.num_scale[i];
				tSetup.num_scale[i] = tSetup.num_scale[i].mul(1 - tParams.learning_rate) + num_scale[i].mul(tParams.learning_rate);
            }
			tSetup.den_scale = tSetup.den_scale.mul(1 - tParams.learning_rate) + den_scale.mul(tParams.learning_rate);

			delete[] num_scale;
		}
		if(tParams.is_rotating)
		{
            if (original)
                prev_tSetup.den_rot= tSetup.den_rot;

			for(int i = 0; i < feature_map_rot_fourier.rows; i++){
                if (original)
                    prev_tSetup.num_rot[i]= tSetup.num_rot[i];
				tSetup.num_rot[i] = tSetup.num_rot[i].mul(1 - tParams.learning_rate) + num_rot[i].mul(tParams.learning_rate);
            }
			tSetup.den_rot = tSetup.den_rot.mul(1 - tParams.learning_rate) + den_rot.mul(tParams.learning_rate);

			delete[] num_rot;
		}

	}
	delete[] feature_map;
	delete[] feature_map_fourier;
}

cv::Point DSST::updateCentroid(cv::Point oldC, int w, int h, int imgw, int imgh)
{
	bool outBorder = false;
	int left = oldC.x - w / 2;
	if(left <= 0)
	{
		left = 1;
		outBorder = true;
	}
	int top = oldC.y - h / 2;
	if(top <= 0)
	{
		top = 1;
		outBorder = true;
	}

	if((left + w) >= imgw)
	{
		left = imgw - w - 1;
		outBorder = true;
	}

	if((top + h) >= imgh)
	{
		top = imgh - h - 1;
		outBorder = true;
	}
	cv::Point newPt;
	if(outBorder)
	{
		newPt.x = left + w / 2;
		newPt.y = top + h / 2;
	} else
		newPt = oldC;
	return newPt;
}

cv::RotatedRect DSST::processFrame_bb(cv::Mat img, bool enableScaling, bool enableRotating, cv::RotatedRect bb)
{
    tSetup.centroid = bb.center;//bb.x + bb.width / 2;                          
    cout<<"Inside process Frame"<<endl;
    cout<<"centroid "<<bb.center<<" "<<bb.angle<<" "<<bb.size<<endl;
//	tSetup.centroid = updateCentroid(tSetup.centroid, tSetup.original.width*tSetup.current_scale_factor, tSetup.original.height*tSetup.current_scale_factor, img.cols, img.rows);

//    std::cout<<"before scaling"<<endl;
	if(enableScaling)
	{
//        cout<<"scale before "<<tSetup.current_scale_factor<<endl;
		tSetup.current_scale_factor = bb.size.width/ tSetup.original.width;
		//if(tSetup.current_scale_factor< tSetup.min_scale_factor)
		//	tSetup.current_scale_factor = tSetup.min_scale_factor;
		//if(tSetup.current_scale_factor> tSetup.max_scale_factor)
		//	tSetup.current_scale_factor = tSetup.max_scale_factor;
//        cout<<"scale after "<<tSetup.current_scale_factor<<endl;
	}
    if(enableRotating)
	{
//        cout<<"rot before "<<tSetup.current_rot_factor<<endl;
		tSetup.current_rot_factor = bb.angle- tSetup.original_rot;
//        cout<<"rot after "<<tSetup.current_rot_factor<<endl;
	}

	train(false, img, false);
    cout<<"Scale: "<<tSetup.original<<" "<<tSetup.current_scale_factor<<endl;
    cout<<"Rot: "<<tSetup.current_rot_factor<<endl;
	tSetup.centroid = updateCentroid(tSetup.centroid, tSetup.original.width*tSetup.current_scale_factor, tSetup.original.height*tSetup.current_scale_factor, img.cols, img.rows);
	int left = tSetup.centroid.x - (tSetup.original.width / 2 * tSetup.current_scale_factor);
	int top = tSetup.centroid.y - (tSetup.original.height / 2 * tSetup.current_scale_factor);
	cv::RotatedRect rect(tSetup.centroid, cv::Size2f(tSetup.original.width*tSetup.current_scale_factor, tSetup.original.height*tSetup.current_scale_factor), tSetup.current_rot_factor);
	//fout<<"Updated Centroid "<<tSetup.centroid.x<<" "<<tSetup.centroid.y<<" "<<rect.width<<"  "<<rect.height<<endl;
	//fout.close();

	return rect;
}
cv::RotatedRect DSST::processFrame(cv::Mat img, bool enableScaling, bool enableRotating)
{
	int nDims = 0;
	cv::Mat *feature_map = get_translation_sample(img, tSetup, nDims);
	cv::Mat *feature_map_fourier = new cv::Mat[nDims];

	for(int i = 0; i < nDims; i++)
	{
		cv::Mat feature_map_double(feature_map[i].rows, feature_map[i].cols, CV_64FC1);
		feature_map[i].convertTo(feature_map_double, CV_64FC1);
		feature_map_fourier[i] = createFourier(feature_map_double);
	}

	cv::Mat* temp = new cv::Mat[nDims];
	for(int i = 0; i < nDims; i++)
		mulSpectrums(tSetup.num_trans[i], feature_map_fourier[i], temp[i], 0, false);

	int w = tSetup.num_trans[0].cols, h = tSetup.num_trans[0].rows;

	cv::Mat sumDen(h, w, CV_64F);

	for(int j = 0; j < h; j++)
		for(int k = 0; k < w; k++)
			sumDen.at<double>(j, k) = tSetup.den_trans.at<cv::Vec2d>(j, k)[0] + tParams.lambda;

	cv::Mat sumTemp(h, w, CV_64FC2);
	sumTemp = cv::Mat::zeros(sumTemp.size(), CV_64FC2);
	for(int j = 0; j < h; j++)
		for(int k = 0; k < w; k++)
		{
			for(int i = 0; i < nDims; i++)
				sumTemp.at<cv::Vec2d>(j, k) += temp[i].at<cv::Vec2d>(j, k);

			sumTemp.at<cv::Vec2d>(j, k) /= sumDen.at<double>(j, k);
		}

	cv::Mat trans_response = cv::Mat::zeros(sumTemp.rows, sumTemp.cols, CV_64FC1);
	trans_response = inverseFourier(sumTemp);

	cv::Point maxLoc = ComputeMaxDisplayfl(trans_response);
	//imshow("trans_response", trans_response);
	//waitKey();
	maxLoc.x = maxLoc.x * (tSetup.padded.width / trans_response.cols);
	maxLoc.y = maxLoc.y * (tSetup.padded.height / trans_response.rows);

	tSetup.centroid.x += cvRound((maxLoc.x - tSetup.padded.width / 2 + 1)*tSetup.current_scale_factor);
	tSetup.centroid.y += cvRound((maxLoc.y - tSetup.padded.height / 2 + 1)*tSetup.current_scale_factor);

	tSetup.centroid = updateCentroid(tSetup.centroid, tSetup.original.width*tSetup.current_scale_factor, tSetup.original.height*tSetup.current_scale_factor, img.cols, img.rows);

//    std::cout<<"before scaling"<<endl;
	if(enableScaling)
	{
		int nDimsScale;

		cv::Mat feature_map_scale_fourier = get_scale_sample(img, tSetup, tParams, nDimsScale);//I have to convert featuremap to double first
//        std::cout<<"After feature scale extraction "<<std::endl;
		cv::Mat* tempScale = new cv::Mat[feature_map_scale_fourier.rows];

		for(int i = 0; i < feature_map_scale_fourier.rows; i++)
		{
			cv::Mat temp1(1, feature_map_scale_fourier.cols, CV_64FC2);
			for(int j = 0; j < feature_map_scale_fourier.cols; j++)
				temp1.at<cv::Vec2d>(0, j) = feature_map_scale_fourier.at<cv::Vec2d>(i, j);

			mulSpectrums(tSetup.num_scale[i], temp1, tempScale[i], 0, false);
		}
		w = nDimsScale;
		cv::Mat sumDenScale(1, w, CV_64F);
		for(int k = 0; k < w; k++)
			sumDenScale.at<double>(0, k) = tSetup.den_scale.at<cv::Vec2d>(0, k)[0] + tParams.lambda;
		cv::Mat sumTempScale(1, w, CV_64FC2);
		sumTempScale = cv::Mat::zeros(sumTempScale.size(), CV_64FC2);
		for(int k = 0; k < w; k++)
		{

			for(int i = 0; i < feature_map_scale_fourier.rows; i++)
				sumTempScale.at<cv::Vec2d>(0, k) += tempScale[i].at<cv::Vec2d>(0, k);

			sumTempScale.at<cv::Vec2d>(0, k) /= sumDenScale.at<double>(0, k);
		}

		cv::Mat scale_response = cv::Mat::zeros(1, nDimsScale, CV_64FC1);
		scale_response = inverseFourier(sumTempScale);

		cv::Point maxLocScale = ComputeMaxDisplayfl(scale_response);

		tSetup.current_scale_factor = tSetup.current_scale_factor* tSetup.scaleFactors[maxLocScale.x];
		if(tSetup.current_scale_factor< tSetup.min_scale_factor)
			tSetup.current_scale_factor = tSetup.min_scale_factor;
		if(tSetup.current_scale_factor> tSetup.max_scale_factor)
			tSetup.current_scale_factor = tSetup.max_scale_factor;

		delete[] tempScale;
	}
//    std::cout<<"Before rotation"<<std::endl;
    if(enableRotating)
	{
		int nDimsRot;

		cv::Mat feature_map_rot_fourier = get_rot_sample(img, tSetup, tParams, nDimsRot);//I have to convert featuremap to double first

		cv::Mat* tempRot = new cv::Mat[feature_map_rot_fourier.rows];

		for(int i = 0; i < feature_map_rot_fourier.rows; i++)
		{
			cv::Mat temp1(1, feature_map_rot_fourier.cols, CV_64FC2);
			for(int j = 0; j < feature_map_rot_fourier.cols; j++)
				temp1.at<cv::Vec2d>(0, j) = feature_map_rot_fourier.at<cv::Vec2d>(i, j);

			mulSpectrums(tSetup.num_rot[i], temp1, tempRot[i], 0, false);
		}
		w = nDimsRot;
		cv::Mat sumDenRot(1, w, CV_64F);
		for(int k = 0; k < w; k++)
			sumDenRot.at<double>(0, k) = tSetup.den_rot.at<cv::Vec2d>(0, k)[0] + tParams.lambda;
		cv::Mat sumTempRot(1, w, CV_64FC2);
		sumTempRot = cv::Mat::zeros(sumTempRot.size(), CV_64FC2);
		for(int k = 0; k < w; k++)
		{
			for(int i = 0; i < feature_map_rot_fourier.rows; i++)
				sumTempRot.at<cv::Vec2d>(0, k) += tempRot[i].at<cv::Vec2d>(0, k);

			sumTempRot.at<cv::Vec2d>(0, k) /= sumDenRot.at<double>(0, k);
		}

		cv::Mat rot_response = cv::Mat::zeros(1, nDimsRot, CV_64FC1);
		rot_response = inverseFourier(sumTempRot);

		cv::Point maxLocRot = ComputeMaxDisplayfl(rot_response);

		tSetup.current_rot_factor = tSetup.current_rot_factor+ tSetup.rotFactors[maxLocRot.x];
//        std::cout<<"Rotation Angle is "<<tSetup.current_rot_factor<<std::endl;
		delete[] tempRot;
	}

	train(false, img, true);

	//    std::cout<<"Angle is "<<tSetup.current_rot_factor<<std::endl;
	/*ofstream fout("C://Users//mincosy//Desktop//Tracking//Final_DSST_C++//DSST//logzebala.txt",  std::ofstream::out | std::ofstream::app);
	fout<<"Centroid "<<tSetup.centroid.x<<" "<<tSetup.centroid.y<<endl;
	fout<<"Size "<<tSetup.original.width<<" "<<tSetup.original.height<<endl;
	*/
	tSetup.centroid = updateCentroid(tSetup.centroid, tSetup.original.width*tSetup.current_scale_factor, tSetup.original.height*tSetup.current_scale_factor, img.cols, img.rows);
	int left = tSetup.centroid.x - (tSetup.original.width / 2 * tSetup.current_scale_factor);
	int top = tSetup.centroid.y - (tSetup.original.height / 2 * tSetup.current_scale_factor);
	cv::RotatedRect rect(tSetup.centroid, cv::Size2f(tSetup.original.width*tSetup.current_scale_factor, tSetup.original.height*tSetup.current_scale_factor), tSetup.current_rot_factor);
	//fout<<"Updated Centroid "<<tSetup.centroid.x<<" "<<tSetup.centroid.y<<" "<<rect.width<<"  "<<rect.height<<endl;
	//fout.close();

	delete[] feature_map;
	delete[] feature_map_fourier;
	delete[] temp;
	//fout<<"Finished deleting :D?hzhahahaha "<<tSetup.centroid.x<<" "<<tSetup.centroid.y<<endl;
	//fout.close();

	return rect;
}
/*DSSTTracker::~DSSTTracker()
{
if(tSetup.num_trans!=0)
{
cout<<"here "<<tSetup.num_trans<<endl;
delete[] tSetup.num_trans;
}
if (tSetup.enableScaling)
{
delete[] tSetup.num_scale;
delete[] tSetup.scaleFactors;
}
}*/

void DSST::preprocess(cv::Mat img, cv::RotatedRect bb)
{
	tSetup.centroid = bb.center;//bb.x + bb.width / 2;
	tSetup.original = bb.size;
    tSetup.original_rot= bb.angle;
	//0- Preprocessing
	//A- Create Translation Gaussian Filters
	tSetup.padded.width = floor(tSetup.original.width * (1 + tParams.padding));
	tSetup.padded.height = floor(tSetup.original.height * (1 + tParams.padding));
	int szPadding_w = tSetup.padded.width / tParams.bin_size;
	int szPadding_h = tSetup.padded.height / tParams.bin_size;

	//don't change to float cause it causes huge truncation errors :) .. asdna el transFilter
	float transSigma = sqrt(float(tSetup.original.width*tSetup.original.height))*tParams.output_sigma_factor;
	cv::Mat transFilter(szPadding_h, szPadding_w, CV_64FC1);
	for(int r = -szPadding_h / 2; r < ceil((double)szPadding_h / 2); r++)
		for(int c = -szPadding_w / 2; c < ceil((double)szPadding_w / 2); c++)
			transFilter.at<double>(r + szPadding_h / 2, c + szPadding_w / 2) = exp(-0.5 * ((double)((r + 1)*(r + 1) + (c + 1)*(c + 1)) / (transSigma*transSigma)));

	tSetup.transFourier = createFourier(transFilter);

	//B- Create Scale Gaussian Filters
	//We don't know why this equation?!
	double scaleSigma = tParams.number_scales / sqrtf(tParams.number_scales) * tParams.scale_sigma_factor;
	cv::Mat scaleFilter(1, tParams.number_scales, CV_64FC1);
	for(int r = -tParams.number_scales / 2; r < ceil((double)tParams.number_scales / 2); r++)
		scaleFilter.at<double>(0, r + tParams.number_scales / 2) = exp(-0.5 * ((double)(r*r) / (scaleSigma*scaleSigma)));
	tSetup.scaleFourier = createFourier(scaleFilter);

	//C- Create Rotation Gaussian Filters
	//We don't know why this equation?!
	double rotSigma = tParams.number_rots / sqrtf(tParams.number_rots) * tParams.scale_sigma_factor;
	cv::Mat rotFilter(1, tParams.number_rots, CV_64FC1);
	for(int r = -tParams.number_rots / 2; r < ceil((double)tParams.number_rots / 2); r++)
		rotFilter.at<double>(0, r + tParams.number_rots / 2) = exp(-0.5 * ((double)(r*r) / (rotSigma*rotSigma)));
	//    cv::imshow("rot filter", rotFilter);
	//    cv::waitKey();
	tSetup.rotFourier = createFourier(rotFilter);

	//Create Cosine Windows to give less weight to boundarie
	cv::Mat trans_cosine_win(szPadding_h, szPadding_w, CV_32FC1);
	//cout<<"Size cosine window "<<szPadding_w<<" "<<szPadding_h<<endl;

	cv::Mat cos1 = hann(szPadding_h);
	cv::Mat cos2 = hann(szPadding_w);
	tSetup.trans_cos_win = cos1*cos2.t();
	//	cv::Mat scale_cosine_win(tParams.number_scales, 1, CV_32FC1);
	tSetup.scale_cos_win = hann(tParams.number_scales);
	//	cv::Mat rot_cosine_win(tParams.number_rots, 1, CV_32FC1);
	tSetup.rot_cos_win = hann(tParams.number_rots);

	//Create Scale Factors
	//    std::cout<<"Creating Scale Factors "<<std::endl;
	double *scaleFactors = new double[tParams.number_scales];
	for(int i = 1; i <= tParams.number_scales; i++)
		scaleFactors[i - 1] = pow(tParams.scale_step, (ceil((double)tParams.number_scales / 2) - i));
	tSetup.scaleFactors = scaleFactors;

	double *rotFactors = new double[tParams.number_rots];
	int n = 0;
	for(int i = 0; i <= (tParams.number_rots - 1)*tParams.rot_step; i += tParams.rot_step)
	{
		rotFactors[n] = i - ((tParams.number_rots - 1)*tParams.rot_step) / 2;
		n++;
	}
	tSetup.rotFactors = rotFactors;

	//compute the resize dimensions used for feature extraction in the scale estimation
	float scale_model_factor = 1;
	int area = tSetup.original.width*tSetup.original.height;
	if(area > tParams.scale_model_max_area)
		scale_model_factor = sqrt((double)tParams.scale_model_max_area / area);


	tSetup.scale_model_sz = cv::Size(floor(tSetup.original.width * scale_model_factor), floor(tSetup.original.height*scale_model_factor));
    if (tSetup.scale_model_sz.width>27)
        tSetup.scale_model_sz.width= 27;
    if (tSetup.scale_model_sz.width<23)
        tSetup.scale_model_sz.width= 24;
    if (tSetup.scale_model_sz.height>27)
        tSetup.scale_model_sz.height= 27;
    if (tSetup.scale_model_sz.height<23)
        tSetup.scale_model_sz.height= 24;
    std::cout<<tSetup.scale_model_sz<<std::endl;
	// find maximum and minimum scales
	//why 
	tSetup.min_scale_factor = pow(tParams.scale_step, ceil(log(max(5.0 / szPadding_h, 5.0 / szPadding_w)) / log(tParams.scale_step)));
	tSetup.max_scale_factor = pow(tParams.scale_step, floor(log(min((float)img.rows / tSetup.original.height, (float)img.cols / tSetup.original.width)) / log(tParams.scale_step)));
	tSetup.current_scale_factor = 1;
	tSetup.current_rot_factor = bb.angle;

	hParams.binSize = tParams.bin_size;
	hParams.nOrients = 9;
	hParams.clipHog = 0.2;
	hParams.softBin = -1;

	//	cout<<"before train"<<endl;
	train(true, img, true);
	//	cout<<"after train"<<endl;
}

cv::Mat DSST::visualize(cv::Rect rect, cv::Mat img, cv::Scalar scalar)
{
	cv::Mat retImg = img.clone();
	cv::rectangle(retImg, cvPoint(rect.x, rect.y), cvPoint(rect.x + rect.width, rect.y + rect.height), scalar, 2);
	return retImg;
}

