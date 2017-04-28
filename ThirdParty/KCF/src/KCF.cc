/*
 * KCFTracker.cpp
 *      Author: Sara
 */
#define _USE_MATH_DEFINES
#include <math.h>
#include "mtf/ThirdParty/KCF/KCF.h"
#include <iostream>
#include "stdio.h"
#include "mtf/ThirdParty/DSST//HOG.h"
#include <fstream>
#include <iomanip>
#include "mtf/ThirdParty/KCF/defines.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "mtf/Utilities/miscUtils.h"

#define KCF_PADDING 1
#define KCF_LAMBDA 1e-4
#define KCF_OUTPUT_SIGMA_FACTOR 0.1
#define KCF_INTERP_FACTOR 0.02
#define KCF_KERNEL_SIGMA 0.5
#define KCF_NUMBER_SCALES 33
#define KCF_SCALE_STEP 1.02
#define KCF_SCALE_MODEL_MAX_AREA 512
#define KCF_SCALE_SIGMA_FACTOR 0.25
#define KCF_SCALE_LEARNING_RATE 0.025
#define KCF_ENABLESCALING false
#define KCF_RESIZE_FACTOR 4

#ifdef _WIN32
#pragma warning(disable:4244)
#pragma warning(disable:4101)
#endif

using namespace cv;
using namespace std;

KCFParams::KCFParams(
	double _padding,
	double _lambda,
	double _output_sigma_factor,
	double _interp_factor,
	double _kernel_sigma,
	int _number_scales,
	double _scale_step,
	double _scale_model_max_area,
	double _scale_sigma_factor,
	double _scale_learning_rate,
	bool _enableScaling,
	int _resize_factor):
	padding(_padding),
	lambda(_lambda),
	output_sigma_factor(_output_sigma_factor),
	interp_factor(_interp_factor),
	kernel_sigma(_kernel_sigma),
	number_scales(_number_scales),
	scale_step(_scale_step),
	scale_model_max_area(_scale_model_max_area),
	scale_sigma_factor(_scale_sigma_factor),
	scale_learning_rate(_scale_learning_rate),
	enableScaling(_enableScaling),
	resize_factor(_resize_factor){}

KCFParams::KCFParams(const KCFParams *params) :
padding(KCF_PADDING),
lambda(KCF_LAMBDA),
output_sigma_factor(KCF_OUTPUT_SIGMA_FACTOR),
interp_factor(KCF_INTERP_FACTOR),
kernel_sigma(KCF_KERNEL_SIGMA),
number_scales(KCF_NUMBER_SCALES),
scale_step(KCF_SCALE_STEP),
scale_model_max_area(KCF_SCALE_MODEL_MAX_AREA),
scale_sigma_factor(KCF_SCALE_SIGMA_FACTOR),
scale_learning_rate(KCF_SCALE_LEARNING_RATE),
enableScaling(KCF_ENABLESCALING),
resize_factor(KCF_RESIZE_FACTOR){
	if(params){
		padding = params->padding;
		lambda = params->lambda;
		output_sigma_factor = params->output_sigma_factor;
		interp_factor = params->interp_factor;
		kernel_sigma = params->kernel_sigma;
		number_scales = params->number_scales;
		scale_step = params->scale_step;
		scale_model_max_area = params->scale_model_max_area;
		scale_sigma_factor = params->scale_sigma_factor;
		scale_learning_rate = params->scale_learning_rate;
		enableScaling = params->enableScaling;
		resize_factor = params->resize_factor;
	}

}

KCF::KCF(const KCFParams *kcf_params) :
TrackerBase(), tParams(kcf_params){
	name = "kcf";
	printf("\n");
	printf("Using Kernel Correlation Filter tracker with:\n");
	printf("padding: %f\n", tParams.padding);
	printf("lambda: %f\n", tParams.lambda);
	printf("output_sigma_factor: %f\n", tParams.output_sigma_factor);
	printf("interp_factor: %f\n", tParams.interp_factor);
	printf("kernel_sigma: %f\n", tParams.kernel_sigma);
	printf("number_scales: %d\n", tParams.number_scales);
	printf("scale_step: %f\n", tParams.scale_step);
	printf("scale_model_max_area: %f\n", tParams.scale_model_max_area);
	printf("scale_sigma_factor: %f\n", tParams.scale_sigma_factor);
	printf("scale_learning_rate: %f\n", tParams.scale_learning_rate);
	printf("resize_factor: %d\n", tParams.resize_factor);
	printf("enableScaling: %d\n", tParams.enableScaling);
	printf("\n");
}
cv::Mat KCF::convertFloatImg(cv::Mat &img){
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

void KCF::initialize(const cv::Mat& corners){
	//int width = corners.at<double>(0, 1) - corners.at<double>(0, 0);
	//int height = corners.at<double>(1, 2) - corners.at<double>(1, 0);

	//cout<<"Original width "<<width<<" "<<height<<endl;

	//cv::Rect rect;
	//rect.x = corners.at<double>(0, 0) / tParams.resize_factor;
	//rect.y = corners.at<double>(1, 0) / tParams.resize_factor;
	//rect.width = width / tParams.resize_factor;
	//rect.height = height / tParams.resize_factor;

	mtf::Rectd best_fit_rect = mtf::utils::getBestFitRectangle<double>(corners,
		currFrame.cols, currFrame.rows);
	cv::Rect rect;
	rect.x = static_cast<int>(best_fit_rect.x / tParams.resize_factor);
	rect.y = static_cast<int>(best_fit_rect.y / tParams.resize_factor);
	rect.width = static_cast<int>(best_fit_rect.width / tParams.resize_factor);
	rect.height = static_cast<int>(best_fit_rect.height / tParams.resize_factor);

	cout << "corners:\n" << corners << "\n";
	printf("best_fit_rect: x: %f y:%f width: %f height: %f\n",
		best_fit_rect.x, best_fit_rect.y, best_fit_rect.width, best_fit_rect.height);
	printf("rect: x: %d y: %d width: %d height: %d\n",
		rect.x, rect.y, rect.width, rect.height);

	//cout<<"Original width "<<rect.width<<" "<<rect.height<<endl;

	cv::Mat scaledCurrFrame;
	resize(currFrame, scaledCurrFrame, cv::Size(currFrame.cols / tParams.resize_factor, currFrame.rows / tParams.resize_factor));
	preprocess(scaledCurrFrame, Point(rect.x+ rect.width /2, rect.y+rect.height/2), rect.width, rect.height);

	currCorners = corners;
	cv_corners_mat = corners;
}

void KCF::update()
{
	cv::Mat scaledCurrFrame;
	resize(currFrame, scaledCurrFrame, cv::Size(currFrame.cols / tParams.resize_factor, currFrame.rows / tParams.resize_factor));
	cv::Rect rect = processFrame(scaledCurrFrame);
	/*cv::Mat scaledCurrFrame2= convertFloatImg(scaledCurrFrame);
	cv::rectangle(scaledCurrFrame2, rect, Scalar(255, 0, 0), 2);
	imshow("testing", scaledCurrFrame2);
	waitKey();
*/

	cv::Mat corners(2, 4, CV_64FC1);
	corners.at<double>(0, 0) = rect.x*tParams.resize_factor;
	corners.at<double>(0, 1) = rect.x*tParams.resize_factor + rect.width*tParams.resize_factor;
	corners.at<double>(0, 2) = rect.x*tParams.resize_factor + rect.width*tParams.resize_factor;
	corners.at<double>(0, 3) = rect.x*tParams.resize_factor;
	corners.at<double>(1, 0) = rect.y*tParams.resize_factor;
	corners.at<double>(1, 1) = rect.y*tParams.resize_factor;
	corners.at<double>(1, 2) = rect.y*tParams.resize_factor + rect.height*tParams.resize_factor;
	corners.at<double>(1, 3) = rect.y*tParams.resize_factor + rect.height*tParams.resize_factor;

	currCorners = corners;
	cv_corners_mat = corners;
}

Point KCF::ComputeMaxfl(Mat img)
{
	Mat imgU = img.clone();
	double minVal;
	double maxVal;
	Point minLoc;
	Point maxLoc;
	minMaxLoc(img, &minVal, &maxVal, &minLoc, &maxLoc);

	/*imgU -= minVal;
	 imgU.convertTo(imgU, CV_8U, 255.0 / (maxVal - minVal));
	 imshow("FloatImg", imgU);
	 waitKey();*/
	return maxLoc;
}

Point KCF::updateCentroid(Point oldC, int w, int h, int imgw, int imgh)
{
	bool outBorder = false;
	int left = oldC.x - w / 2;
	if (left <= 0)
	{
		left = 1;
		outBorder = true;
	}
	int top = oldC.y - h / 2;
	if (top <= 0)
	{
		top = 1;
		outBorder = true;
	}

	if ((left + w) >= imgw)
	{
		left = imgw - w - 1;
		outBorder = true;
	}

	if ((top + h) >= imgh)
	{
		top = imgh - h - 1;
		outBorder = true;
	}
	Point newPt;
	if (outBorder)
	{
		newPt.x = left + w / 2;
		newPt.y = top + h / 2;
	}
	else
		newPt = oldC;
	return newPt;
}

Point KCF::displayFloat(Mat img)
{
	Mat imgU = img.clone();
	double minVal;
	double maxVal;
	Point minLoc;
	Point maxLoc;
	minMaxLoc(img, &minVal, &maxVal, &minLoc, &maxLoc);
	imgU -= minVal;
	imgU.convertTo(imgU, CV_8U, 255.0 / (maxVal - minVal));
	imshow("FloatImg", imgU);

	return maxLoc;
}

void KCF::createFourier(cv::Mat original, cv::Mat& complexI, int flag)
{
	Mat planes[] =
	{ Mat_<double>(original), Mat::zeros(original.size(), CV_64F) };

	cv::merge(planes, 2, complexI);
	cv::dft(complexI, complexI, flag);  // Applying DFT without padding
	//cout << "Fourier" << endl;
}
void KCF::gaussian_shaped_labels(double sigma, int sz_w, int sz_h, Mat& shiftedFilter)
{
	cv::Mat transFilter(sz_h, sz_w, CV_64FC1);
	shiftedFilter = Mat(sz_h, sz_w, CV_64FC1);
	printf("sz_w: %d sz_h: %d\n", sz_w, sz_h);
	//TODO slow access
	for (int r = -sz_h / 2; r < ceil((double) sz_h / 2); r++)
		for (int c = -sz_w / 2; c < ceil((double) sz_w / 2); c++)
			transFilter.at<double>(r + sz_h / 2, c + sz_w / 2) = exp(-0.5 * ((double) ((r + 1) * (r + 1) + (c + 1) * (c + 1)) / (sigma * sigma)));

	//labels = circshift(labels, -floor(sz(1:2) / 2) + 1);
	int shiftX = static_cast<int>(-sz_w / 2.0) + 1;
	int shiftY = static_cast<int>(-sz_h / 2.0) + 1;

	for (int i = 0; i < transFilter.rows; i++)
		for (int j = 0; j < transFilter.cols; j++)
		{
			shiftedFilter.at<double>(i, j) = transFilter.at<double>((i - shiftY) % transFilter.rows, (j - shiftX) % transFilter.cols);
		}
	if(shiftedFilter.at<double>(0,0) != 1)
		cout<<"SHIFT ERROR"<<endl;

	//imshow("filter", transFilter);
	//waitKey();
}
void KCF::hann(int size, Mat& arr)
{
	arr = Mat(size, 1, CV_64FC1);
	for (int i = 0; i < size; i++)
	{
		float multiplier = static_cast<float>(0.5 * (1 - cos(2 * M_PI * i / (size - 1))));
		*((double *) (arr.data + i * arr.step[0])) = multiplier;
	}
}
float *KCF::convertTo1DFloatArray(Mat &patch)
{

	float *img = (float*) calloc(patch.rows * patch.cols, sizeof(float));

	int k = 0;
	for (int i = 0; i < patch.cols; i++)
		for (int j = 0; j < patch.rows; j++)
			img[k++] = static_cast<float>(patch.at<unsigned char>(j, i) / 255.0);

	/*
	 imshow("", patch);
	 waitKey();
	 for (int i = 0; i < patch.rows * patch.cols; i++)
	 cout << img[i] << endl;
	 cout << "here" << endl;*/
	return img;
}
double *convertTo1DFloatArrayDouble(Mat &patch)
{

	double *img = (double*) calloc(patch.rows * patch.cols, sizeof(double));

	int k = 0;
	for (int i = 0; i < patch.cols; i++)
		for (int j = 0; j < patch.rows; j++)
			img[k++] = (double) patch.at<unsigned char>(j, i) / 255.0;

	/*
	 imshow("", patch);
	 waitKey();
	 for (int i = 0; i < patch.rows * patch.cols; i++)
	 cout << img[i] << endl;
	 cout << "here" << endl;*/
	return img;
}

Mat *KCF::createFeatureMap(Mat& patch, int &nChns, bool isScaling)
{

	int h = patch.rows, w = patch.cols;
	float* M = (float*) calloc(h * w, sizeof(float));
	float* O = (float*) calloc(h * w, sizeof(float));
	
	float *img = convertTo1DFloatArray(patch);

	gradMag(img, M, O, h, w, 1, 1);

	int binSize = isScaling ? hParams.scaleBinSize : hParams.binSize;
	int hb = h / binSize;
	int wb = w / binSize;

	nChns = hParams.nOrients * 3 + 5;
	float *H = (float*) calloc(hb * wb * nChns, sizeof(float));
	//cout << hb << "  " << wb << endl;
	//cerr << "b4 fhogSSE" << endl;
	fhogSSE(M, O, H, h, w, binSize, hParams.nOrients, hParams.softBin, hParams.clipHog);
	//cerr << "after fhogSSE" << endl;
	Mat roiGrayFlot = patch.clone();
	roiGrayFlot = roiGrayFlot.mul(1.0 / 255);
	roiGrayFlot = roiGrayFlot - 0.5;
	Mat *featureMap;
	
	nChns = 28;
	featureMap = new Mat[nChns];
	for (int i = 0; i<nChns; i++)
		featureMap[i] = cv::Mat(hb, wb, CV_64FC1);

	roiGrayFlot.convertTo(featureMap[0], CV_64FC1);
	for (int j = 0; j<wb; j++)
		for (int i = 0; i<hb; i++)
			for (int k = 0; k<nChns - 1; k++)
				featureMap[k + 1].at<double>(i, j) = H[k*(hb*wb) + j*hb + i];
	/*
	nChns = 31;
	featureMap = new Mat[nChns];
	for (int i = 0; i < nChns; i++)
		featureMap[i] = cv::Mat(hb, wb, CV_64FC1);

	for (int j = 0; j < wb; j++)
		for (int i = 0; i < hb; i++)
			for (int k = 0; k < nChns; k++)
				featureMap[k].at<double>(i, j) = H[k * (hb * wb) + j * hb + i];
*/
	if (!isScaling)
		for (int i = 0; i < nChns; i++)
			featureMap[i] = featureMap[i].mul(tSetup.trans_cos_win);

	/*
	 freopen("log.txt", "wt", stdout);

	 for (int k = 0; k < nChns; k++)
	 {
	 for (int j = 0; j < wb; j++)
	 {
	 for (int i = 0; i < hb; i++)
	 cout <<  fixed << setprecision(4)<<featureMap[k].at<double>(i, j) << endl;
	 }
	 }

	 */

	free(img);
	free(H);
	free(M);
	free(O);

	return featureMap;
}
void KCF::inverseFourier(cv::Mat original, cv::Mat& output, int flag)
{
	cv::idft(original, output, DFT_REAL_OUTPUT | DFT_SCALE);
}
void KCF::gaussian_correlation(Mat* xf, Mat* yf, int nChns, double sigma, Mat & corrF)
{
	//ofstream fout("log.txt");
	int w = xf[0].cols;
	int h = xf[0].rows;
	double xx = 0;
	double yy = 0;
	for (int i = 0; i < nChns; i++)
		for (int j = 0; j < h; j++)
			for (int k = 0; k < w; k++)
			{
				Vec2d bla = xf[i].at<Vec2d>(j, k);
				xx += bla[0] * bla[0] + bla[1] * bla[1];
				bla = yf[i].at<Vec2d>(j, k);
				yy += bla[0] * bla[0] + bla[1] * bla[1];
			}
	xx /= (w * h);
	yy /= (w * h);
	//cout << xx << "  " << yy << endl;
	Mat *xyf = new Mat[nChns];
	Mat corr = cv::Mat::zeros(h, w, CV_64FC1);
	for (int ch = 0; ch < nChns; ch++)
	{
		mulSpectrums(xf[ch], yf[ch], xyf[ch], 0, true);
		inverseFourier(xyf[ch], xyf[ch]);
	}
	/*
	 for (int i = 0; i < nChns; i++)
	 for (int k = 0; k < w; k++)
	 for (int j = 0; j < h; j++)
	 fout << fixed << setprecision(4) << xyf[i].at<double>(j, k) << endl;*/

	for (int i = 0; i < nChns; i++)
		corr += xyf[i];
	/*	for (int k = 0; k < w; k++)
	 for (int j = 0; j < h; j++)
	 fout << fixed << setprecision(4) << corr.at<double>(j, k) << endl;*/

	corr *= -2;
	corr += xx + yy;
	corr /= (w * h * nChns);
	max(corr, 0);
	corr *= (-1 / (sigma * sigma));
	exp(corr, corr);

	/*for (int k = 0; k < w; k++)
	 for (int j = 0; j < h; j++)
	 fout << fixed << setprecision(4) << corr.at<double>(j, k) << endl;*/
	createFourier(corr, corrF);
	/*
	 for (int j = 0; j < corrF.cols; j++)
	 {
	 for (int i = 0; i < corrF.rows; i++)
	 {
	 string complexN = (corrF.at<Vec2d>(i, j)[1] <= 0 ? "" : "+");
	 if (abs(corrF.at<Vec2d>(i, j)[1]) != 0)
	 fout << fixed << setprecision(4) << corrF.at<Vec2d>(i, j)[0] << complexN << fixed << setprecision(4) << corrF.at<
	 Vec2d>(i, j)[1] << "i" << endl;
	 else
	 fout << fixed << setprecision(4) << corrF.at<Vec2d>(i, j)[0] << endl;
	 }
	 }
	 fout.close();*/
	delete[] xyf;
}
void KCF::train(Mat img, bool first)
{
	double trainTime = 0;
//Create Patch
	float pw = tSetup.padded.width * tSetup.current_scale_factor;
	float ph = tSetup.padded.height * tSetup.current_scale_factor;
	int centerX = tSetup.centroid.x + 1;
	int centerY = tSetup.centroid.y + 1;

	int tlX1, tlY1, w1, w2, h1, h2;
	tlX1 = max(0.0, centerX - floor(pw / 2.0));
	int padToX = (int) centerX - pw / 2 < 0 ? (int) ceil(centerX - pw / 2) : 0;
	w1 = (padToX + pw);
	w2 = (tlX1 + w1) >= img.cols ? img.cols - tlX1 : w1;

	tlY1 = max(0, (int) ceil(centerY - ph / 2));
	int padToY =
			(int) ceil(centerY - ph / 2) < 0 ? (int) ceil(centerY - ph / 2) : 0;
	h1 = (padToY + ph);
	h2 = (tlY1 + h1) >= img.rows ? img.rows - tlY1 : h1;

	Rect rect(tlX1, tlY1, w2, h2);
	Mat patch = img(rect);

	Mat roi;
	double subwindow;
	timeOfBlock( copyMakeBorder(patch, roi, abs(padToY), h1 - h2, abs(padToX), w1 - w2, BORDER_REPLICATE);, subwindow);
	trainTime += subwindow;
	int interpolation;
	if (tSetup.padded.width > roi.cols)
		interpolation = INTER_LINEAR;
	else
		interpolation = INTER_AREA;
	resize(roi, roi, cv::Size(tSetup.padded.width, tSetup.padded.height), 0, 0, interpolation);

	int nChns;
	Mat* feature_map;
	double transFeatureMap;
	timeOfBlock( feature_map = createFeatureMap(roi, nChns);, transFeatureMap);
	trainTime += transFeatureMap;

	Mat *feature_map_fourier;
	feature_map_fourier = new Mat[nChns];
//cout<<"Feature Map Fourier Translation"<<endl;
	double trainFourier;
	timeOfBlock( for (int i = 0; i < nChns; i++) createFourier(feature_map[i],feature_map_fourier[i] );, trainFourier);
	trainTime += trainFourier;

	/*freopen("log.txt", "wt", stdout);

	 for (int k = 0; k < nChns; k++)
	 {
	 for (int j = 0; j < feature_map_fourier[k].cols; j++)
	 {
	 for (int i = 0; i < feature_map_fourier[k].rows; i++)
	 {
	 string complexN = (feature_map_fourier[k].at<Vec2d>(i, j)[1] <= 0 ? "" : "+");
	 if (abs(feature_map_fourier[k].at<Vec2d>(i, j)[1]) != 0)
	 cout << fixed << setprecision(4) << feature_map_fourier[k].at<Vec2d>(i, j)[0] << complexN << fixed << setprecision(4) << feature_map_fourier[k].at<
	 Vec2d>(i, j)[1] << "i" << endl;
	 else
	 cout << fixed << setprecision(4) << feature_map_fourier[k].at<Vec2d>(i, j)[0] << endl;
	 }
	 }
	 }*/
	Mat corr;
	double transCorr;
	timeOfBlock( gaussian_correlation(feature_map_fourier, feature_map_fourier, nChns, tParams.kernel_sigma, corr);, transCorr);
	trainTime += transCorr;

	Mat temp, temp2;
	Mat corrLambda = corr + tParams.lambda;
	Mat* alpha = new Mat(corrLambda.size(), corrLambda.type());
	mulSpectrums(tSetup.transFourier, corrLambda, temp, true);
	mulSpectrums(corrLambda, corrLambda, temp2, true);

	for (int i = 0; i < corr.rows; i++)
		for (int j = 0; j < corr.cols; j++)
			alpha->at<Vec2d>(i, j) = temp.at<Vec2d>(i, j) / (
					abs(temp2.at<Vec2d>(i, j)[0]) == 0.0 ? 1 : temp2.at<Vec2d>(i, j)[0]);

	/*	freopen("log.txt", "wt", stdout);
	 for (int j = 0; j < alpha->cols; j++)
	 {
	 for (int i = 0; i < alpha->rows; i++)
	 {
	 string complexN = (alpha->at < Vec2d > (i, j)[1] <= 0 ? "" : "+");
	 if (abs(alpha->at < Vec2d > (i, j)[1]) != 0)
	 cout << fixed << setprecision(4) << alpha->at < Vec2d > (i, j)[0] << complexN << fixed << setprecision(4) << alpha->at < Vec2d > (i, j)[1] << "i" << endl;
	 else
	 cout << fixed << setprecision(4) << alpha->at < Vec2d > (i, j)[0] << endl;
	 }

	 }*/
	Mat *num_scale = 0;
	int nDimsScale;
	Mat den_scale;
	int nPixel;
	if (tParams.enableScaling)
	{
		Mat feature_map_scale_fourier;
		double scaleFeatures;
		timeOfBlock( feature_map_scale_fourier = get_scale_sample(img, nDimsScale);, scaleFeatures);
		trainTime += scaleFeatures;

		nPixel = feature_map_scale_fourier.rows;
		num_scale = new Mat[nPixel];
		den_scale = cv::Mat::zeros(1, nDimsScale, CV_64FC2);

		double scaleNumDen;
		timeOfBlock(
		for (int i = 0; i < feature_map_scale_fourier.rows; i++)
		{
			Mat temp(1, nDimsScale, CV_64FC2);
			for (int j = 0; j < nDimsScale; j++)
				temp.at<Vec2d>(0, j) = feature_map_scale_fourier.at<Vec2d>(i, j);

			mulSpectrums(tSetup.scaleFourier, temp, num_scale[i], 0, true);

		}

		Mat temp3;
		mulSpectrums(feature_map_scale_fourier, feature_map_scale_fourier, temp3, 0, true);

		for (int i = 0; i < temp3.cols; i++)
		{
			for (int j = 0; j < temp3.rows; j++)
			{
				den_scale.at<Vec2d>(0, i)[0] += temp3.at<Vec2d>(j, i)[0];
				den_scale.at<Vec2d>(0, i)[1] += temp3.at<Vec2d>(j, i)[1];
			}
		}
		,scaleNumDen);
		trainTime+=scaleNumDen;
	}
	double updateParam;
	timeOfBlock(
	if (first)
	{
		tSetup.model_alphaf = alpha;
		tSetup.model_xf = feature_map_fourier;

		if (tParams.enableScaling)
		{
			tSetup.num_scale = num_scale;
			tSetup.nNumScale = nDimsScale;
			tSetup.den_scale = den_scale;
		}
	}
	else
	{
		//model_alphaf = (1 - interp_factor) * model_alphaf + interp_factor * alphaf;
		//model_xf = (1 - interp_factor) * model_xf + interp_factor * xf;

		*tSetup.model_alphaf = (1 - tParams.interp_factor) * (*tSetup.model_alphaf) + tParams.interp_factor * (*alpha);

		/*	freopen("alpha.txt", "wt", stdout);
		 for (int j = 0; j < tSetup.model_alphaf->cols; j++)
		 for (int i = 0; i < tSetup.model_alphaf->rows; i++)
		 {
		 string complexN = (tSetup.model_alphaf->at<Vec2d>(i, j)[1] <= 0 ? "" : "+");
		 if (abs(tSetup.model_alphaf->at<Vec2d>(i, j)[1]) != 0)
		 cout << fixed << setprecision(4) << tSetup.model_alphaf->at<Vec2d>(i, j)[0] << complexN << fixed << setprecision(4) << tSetup.model_alphaf->at<
		 Vec2d>(i, j)[1] << "i" << endl;
		 else
		 cout << fixed << setprecision(4) << tSetup.model_alphaf->at<Vec2d>(i, j)[0] << endl;
		 }
		 */
		for (int i = 0; i < nChns; i++)
			tSetup.model_xf[i] = (1 - tParams.interp_factor) * tSetup.model_xf[i] + tParams.interp_factor * feature_map_fourier[i];
		/*

		 freopen("xf.txt", "wt", stdout);
		 for (int k = 0; k < nChns; k++)
		 for (int j = 0; j < tSetup.model_xf[k].cols; j++)
		 {
		 for (int i = 0; i < tSetup.model_xf[k].rows; i++)
		 {
		 string complexN = (tSetup.model_xf[k].at<Vec2d>(i, j)[1] <= 0 ? "" : "+");
		 if (abs(tSetup.model_xf[k].at<Vec2d>(i, j)[1]) != 0)
		 cout << fixed << setprecision(4) << tSetup.model_xf[k].at<Vec2d>(i, j)[0] << complexN << fixed << setprecision(4) << tSetup.model_xf[k].at<
		 Vec2d>(i, j)[1] << "i" << endl;
		 else
		 cout << fixed << setprecision(4) << tSetup.model_xf[k].at<Vec2d>(i, j)[0] << endl;
		 }
		 }
		 */
		if (tParams.enableScaling)
		{
			for (int i = 0; i < nPixel; i++)
				tSetup.num_scale[i] = tSetup.num_scale[i].mul(1 - tParams.scale_learning_rate) + num_scale[i].mul(tParams.scale_learning_rate);
			tSetup.den_scale = tSetup.den_scale.mul(1 - tParams.scale_learning_rate) + den_scale.mul(tParams.scale_learning_rate);

			delete[] num_scale;
		}
		delete[] feature_map_fourier;
		delete alpha;
	},updateParam);
	trainTime += updateParam;
	//cerr << "			Train Time " << trainTime << endl;
	delete[] feature_map;
}
Mat KCF::get_scale_sample(Mat img, int &nDims, bool display)
{
	Mat featureMapScale;
	CvRect patchSize;

	Mat roiGray;
	Mat roiResized;
	Mat feature_map_scale_fourier;
	for (int i = 0; i < tParams.number_scales; i++)
	{
		//Create Patch
		float pw = tSetup.scaleFactors[i] * tSetup.current_scale_factor * tSetup.original.width;
		float ph = tSetup.scaleFactors[i] * tSetup.current_scale_factor * tSetup.original.height;

		int tlX1, tlY1, w1, w2, h1, h2;
		tlX1 = max(0, (int) ceil(tSetup.centroid.x + 1 - pw / 2));
		int padToX =
				(int) ceil(tSetup.centroid.x + 1 - pw / 2) < 0 ? (int) ceil(tSetup.centroid.x + 1 - pw / 2) : 0;
		w1 = (padToX + pw);
		w2 = (tlX1 + w1) >= img.cols ? img.cols - tlX1 : w1;

		tlY1 = max(0, (int) ceil(tSetup.centroid.y + 1 - ph / 2));
		int padToY =
				(int) ceil(tSetup.centroid.y + 1 - ph / 2) < 0 ? (int) ceil(tSetup.centroid.y + 1 - ph / 2) : 0;
		h1 = (padToY + ph);
		h2 = (tlY1 + h1) >= img.rows ? img.rows - tlY1 : h1;
		Rect rect(tlX1, tlY1, w2, h2);
		Mat patch = img(rect);
		Mat roi;

		copyMakeBorder(patch, roi, abs(padToY), h1 - h2, abs(padToX), w1 - w2, BORDER_REPLICATE);

		Rect patchSize = Rect(tlX1, tlY1, roi.cols, roi.rows);

		int interpolation;
		if (tSetup.scale_model_sz.width > patchSize.width)
			interpolation = INTER_LINEAR;
		else
			interpolation = INTER_AREA;
		resize(roi, roiResized, cv::Size(tSetup.scale_model_sz.width, tSetup.scale_model_sz.height), 0, 0, interpolation);
		if (display)
		{
			imshow("roi", roi);
			waitKey();
		}
		//Extract Features
		int nChns;
		Mat *featureMap = createFeatureMap(roiResized, nChns, true);
		float s = tSetup.scale_cos_win.at<double>(i, 0);

		//Multiply by scale window + Save it as 1D array in the big array
		if (featureMapScale.data == NULL) //i==0)
		{
			featureMapScale = Mat(featureMap[0].rows * featureMap[0].cols * nChns, tParams.number_scales, CV_64FC1);
			feature_map_scale_fourier = Mat(featureMap[0].rows * featureMap[0].cols * nChns, tParams.number_scales, CV_64FC1);
		}

		int k = 0;

		for (int j = 0; j < nChns; j++)
		{
			for (int m = 0; m < featureMap[j].cols; m++)
				for (int l = 0; l < featureMap[j].rows; l++)
				{
					featureMapScale.at<double>(k, i) = featureMap[j].at<double>(l, m) * s;
					k++;
				}
		}
		delete[] featureMap;

	}

	Mat feature_map_scale_fourier_temp;
	createFourier(featureMapScale, feature_map_scale_fourier_temp, DFT_ROWS);

	nDims = tParams.number_scales;

	return feature_map_scale_fourier_temp;
}

void KCF::preprocess(Mat img, Point centroid, int w, int h)
{
	double convertGray;
	int rows = img.rows;
	int cols = img.cols;
	tSetup.centroid.x = centroid.x;
	tSetup.centroid.y = centroid.y;
	tSetup.original = Size(w, h);

	tSetup.padded.width = floor(tSetup.original.width * (1.0 + tParams.padding));
	tSetup.padded.height = floor(tSetup.original.height * (1.0 + tParams.padding));

	////////////////Localization Parameters/////////////////

	//output_sigma = sqrt(prod(target_sz)) * output_sigma_factor / cell_size;
	double output_sigma = sqrt((double)tSetup.original.width * tSetup.original.height) * tParams.output_sigma_factor / hParams.binSize;

	int sz_w = tSetup.padded.width / hParams.binSize;
	int sz_h = tSetup.padded.height / hParams.binSize;
	//cout<<sz_w<<"  "<<sz_h<<endl;
	//yf = fft2(gaussian_shaped_labels(output_sigma, floor(window_sz / cell_size)));
	//tSetup.transFourier = cv::Mat::zeros(tSetup.transFourier.size(), tSetup.transFourier.type());
	double filterTrans;
	Mat gauss;
	timeOfBlock( gaussian_shaped_labels(output_sigma, sz_w, sz_h,gauss); createFourier(gauss,tSetup.transFourier);

	//cout << "hereafter" << endl;
	//TODO remove
	/*	ofstream fout("log.txt");
	 for (int j = 0; j < tSetup.transFourier.cols; j++)
	 {
	 for (int i = 0; i < tSetup.transFourier.rows; i++)
	 {
	 string complexN = (tSetup.transFourier.at < Vec2d > (i, j)[1] <= 0 ? "" : "+");
	 if (abs(tSetup.transFourier.at < Vec2d > (i, j)[1]) != 0)
	 fout << fixed << setprecision(4) << tSetup.transFourier.at < Vec2d > (i, j)[0] << complexN << fixed << setprecision(4) << tSetup.transFourier.at < Vec2d > (i, j)[1] << "i" << endl;
	 else
	 fout << fixed << setprecision(4) << tSetup.transFourier.at < Vec2d > (i, j)[0] << endl;
	 }
	 }
	 fout.close();*/

	//cos_window = hann(size(yf,1)) * hann(size(yf,2))';
	cv::Mat trans_cosine_win(sz_h, sz_w, CV_64FC1);Mat cos1; hann(sz_h,cos1); cv::Mat cos2; hann(sz_w,cos2); tSetup.trans_cos_win = cos1 * cos2.t();, filterTrans);
	double scalingParam;
	////////////////Scaling Parameters/////////////////
	timeOfBlock( if (tParams.enableScaling) {
	//B- Create Scale Gaussian Filters

	double scaleSigma = tParams.number_scales / sqrt((double)tParams.number_scales) * tParams.scale_sigma_factor; cv::Mat scaleFilter(1, tParams.number_scales, CV_64FC1); for (int r = -tParams.number_scales / 2; r < ceil((double) tParams.number_scales / 2); r++) scaleFilter.at<double>(0, r + tParams.number_scales / 2) = exp(-0.5 * ((double) (r * r) / (scaleSigma * scaleSigma))); createFourier(scaleFilter, tSetup.scaleFourier);

	cv::Mat scale_cosine_win(tParams.number_scales, 1, CV_64FC1); hann(tParams.number_scales, tSetup.scale_cos_win);

	double *scaleFactors = new double[tParams.number_scales]; for (int i = 1; i <= tParams.number_scales; i++) scaleFactors[i - 1] = pow(tParams.scale_step, (ceil((double) tParams.number_scales / 2) - i));

	tSetup.scaleFactors = scaleFactors;

	//compute the resize dimensions used for feature extraction in the scale estimation
	float scale_model_factor = 1; int area = tSetup.original.width * tSetup.original.height; if (area > tParams.scale_model_max_area) scale_model_factor = sqrt((double) tParams.scale_model_max_area / area);

	tSetup.scale_model_sz = Size(floor(tSetup.original.width * scale_model_factor), floor(tSetup.original.height * scale_model_factor));
	ofstream fout("log.txt");
	fout << "heeere" << endl;
	fout.close();
	// find maximum and minimum scales
	tSetup.min_scale_factor = pow(tParams.scale_step, ceil(log(max(5.0 / tSetup.padded.width, 5.0 / tSetup.padded.height)) / log(tParams.scale_step))); tSetup.max_scale_factor = pow(tParams.scale_step, floor(log(min((float) rows / tSetup.original.height, (float) cols / tSetup.original.width)) / log(tParams.scale_step))); }, scalingParam);
	tSetup.current_scale_factor = 1;
	train(img, true);
}
Rect KCF::processFrame(cv::Mat img)
{

	double totTime = 0;
	double convertGray=0;
	totTime += convertGray;

	//Create Patch
	float pw = tSetup.padded.width * tSetup.current_scale_factor;
	float ph = tSetup.padded.height * tSetup.current_scale_factor;
	int centerX = tSetup.centroid.x + 1;
	int centerY = tSetup.centroid.y + 1;
	//cout << "Centre " << centerX << "  " << centerY << endl;
	int tlX1, tlY1, w1, w2, h1, h2;
	tlX1 = max(0.0, centerX - floor(pw / 2.0));
	int padToX = (int) centerX - pw / 2 < 0 ? (int) ceil(centerX - pw / 2) : 0;
	w1 = (padToX + pw);
	w2 = (tlX1 + w1) >= img.cols ? img.cols - tlX1 : w1;

	tlY1 = max(0, (int) ceil(centerY - ph / 2));
	int padToY =
			(int) ceil(centerY - ph / 2) < 0 ? (int) ceil(centerY - ph / 2) : 0;
	h1 = (padToY + ph);
	h2 = (tlY1 + h1) >= img.rows ? img.rows - tlY1 : h1;

	Rect rect(tlX1, tlY1, w2, h2);
	Mat patch = img(rect);

	Mat roi;
	copyMakeBorder(patch, roi, abs(padToY), h1 - h2, abs(padToX), w1 - w2, BORDER_REPLICATE);
	int interpolation;
	if (tSetup.padded.width > roi.cols)
		interpolation = INTER_LINEAR;
	else
		interpolation = INTER_AREA;
	//imshow("roi",roi);
	resize(roi, roi, cv::Size(tSetup.padded.width, tSetup.padded.height), 0, 0, interpolation);
	//imshow("roiResi",roi);
	//zf = fft2(get_features(patch, features, cell_size, cos_window));

	int nChns;
	Mat* feature_map;
	double getFeatureMap;
	timeOfBlock(feature_map = createFeatureMap(roi, nChns) ;, getFeatureMap);
	totTime += getFeatureMap;

	Mat *feature_map_fourier = new Mat[nChns];
	double featuresFourier;
	//cout<<"Feature Map Fourier Translation"<<endl;
	timeOfBlock(for (int i = 0; i < nChns; i++) createFourier(feature_map[i], feature_map_fourier[i]) ;, featuresFourier);
	totTime += featuresFourier;

	double corrTime;
	timeOfBlock( Mat corr; gaussian_correlation(feature_map_fourier, tSetup.model_xf, nChns, tParams.kernel_sigma, corr); Mat temp; mulSpectrums(corr, *tSetup.model_alphaf, temp, false);

	inverseFourier(temp, temp); double mxVal = -1; Point mxLoc;

	//[vert_delta, horiz_delta] = find(response == max(response(:)), 1);
	Point delta = ComputeMaxfl(temp); int w = temp.cols; int h = temp.rows;
	//Mat tempFloat= convertFloatImg(temp);
	//imshow("Response", tempFloat);
	//waitKey();
	
	//ofstream fout("log.txt", ofstream::app);
	//fout<<delta.x<<","<<delta.y<<","<<fixed<<setprecision(4)<< temp.at<double>(delta)<<endl;
	if (delta.x > w / 2 - 1) delta.x -= w; if (delta.y > h / 2 - 1) delta.y -= h;
	//pos = pos + cell_size * [vert_delta - 1, horiz_delta - 1];
	tSetup.centroid = tSetup.centroid + (hParams.binSize * Point(delta.x, delta.y));, corrTime);
	tSetup.centroid = updateCentroid(tSetup.centroid, tSetup.original.width * tSetup.current_scale_factor, tSetup.original.height * tSetup.current_scale_factor, img.cols, img.rows);
	totTime += corrTime;

	//Scaling estimation

	if (tParams.enableScaling)
	{
		int nDimsScale;

		Mat feature_map_scale_fourier;
		double scaleFeatures;
		timeOfBlock(feature_map_scale_fourier = get_scale_sample(img, nDimsScale);, scaleFeatures);
		totTime += scaleFeatures;

		Mat* tempScale = new Mat[feature_map_scale_fourier.rows];
		double scaleEst;
		timeOfBlock( for (int i = 0; i < feature_map_scale_fourier.rows; i++) { Mat temp1(1, feature_map_scale_fourier.cols, CV_64FC2); for (int j = 0; j < feature_map_scale_fourier.cols; j++) temp1.at<Vec2d>(0, j) = feature_map_scale_fourier.at<Vec2d>(i, j);

		mulSpectrums(tSetup.num_scale[i], temp1, tempScale[i], 0, false); }

		Mat sumDenScale(1, nDimsScale, CV_64F); for (int k = 0; k < nDimsScale; k++) sumDenScale.at<double>(0, k) = tSetup.den_scale.at<Vec2d>(0, k)[0] + tParams.lambda;

		Mat sumTempScale(1, nDimsScale, CV_64FC2); sumTempScale = cv::Mat::zeros(sumTempScale.size(), CV_64FC2); for (int k = 0; k < nDimsScale; k++) { for (int i = 0; i < feature_map_scale_fourier.rows; i++) sumTempScale.at<Vec2d>(0, k) += tempScale[i].at<Vec2d>(0, k);

		sumTempScale.at<Vec2d>(0, k) /= sumDenScale.at<double>(0, k); }

		Mat scale_response = cv::Mat::zeros(1, nDimsScale, CV_64FC1); inverseFourier(sumTempScale, scale_response); Point maxLocScale = ComputeMaxfl(scale_response);

		tSetup.current_scale_factor = tSetup.current_scale_factor * tSetup.scaleFactors[maxLocScale.x];, scaleEst);
		totTime += scaleEst;

		if (tSetup.current_scale_factor < tSetup.min_scale_factor)
			tSetup.current_scale_factor = tSetup.min_scale_factor;
		if (tSetup.current_scale_factor > tSetup.max_scale_factor)
			tSetup.current_scale_factor = tSetup.max_scale_factor;

	}

	//fout << delta.x<<","<<delta.y<<","<<tSetup.centroid.x << "," << tSetup.centroid.y << endl;
	double trainTime;
	timeOfBlock( train(img);, trainTime);
	totTime += trainTime;
	Point centroid = updateCentroid(tSetup.centroid, tSetup.original.width * tSetup.current_scale_factor, tSetup.original.height * tSetup.current_scale_factor, img.cols, img.rows); //to make sure not out of boundary
	int left = centroid.x - (tSetup.original.width / 2 * tSetup.current_scale_factor);
	int top = centroid.y - (tSetup.original.height / 2 * tSetup.current_scale_factor);
	rect = Rect(left, top, tSetup.original.width * tSetup.current_scale_factor, tSetup.original.height * tSetup.current_scale_factor);
	//fout.close();
	//cerr << "			Calc Process Time: " << totTime << endl;
	tSetup.centroid = centroid;
	delete[] feature_map;
	delete[] feature_map_fourier;
	return rect;
}
