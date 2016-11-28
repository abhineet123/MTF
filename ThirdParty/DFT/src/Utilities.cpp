/*
Copyright 2014 Alberto Crivellaro, Ecole Polytechnique Federale de Lausanne (EPFL), Switzerland.
alberto.crivellaro@epfl.ch

terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA
 */

#include "mtf/ThirdParty/DFT/Utilities.hpp"

using namespace std;
using namespace cv;



void ComputeImageIntensityDescriptorFields(Mat &grayscaleImage, vector<Mat> &outDescriptorFields)
{
	outDescriptorFields.clear();
	outDescriptorFields.push_back(grayscaleImage.clone());
}
                                                                                                  
void ComputeGradientBasedDescriptorFields(Mat & grayscaleImage, vector<Mat> &outDescriptorFields)
{	
	outDescriptorFields.clear();
	Mat dx, dy;
	ComputeImageDerivatives(grayscaleImage, dx, dy);
	assert(dx.isContinuous());
	assert(dy.isContinuous());

	Size imSize = grayscaleImage.size();
	Mat dxPos(imSize, CV_32F, Scalar(0));
	Mat dxNeg(imSize, CV_32F, Scalar(0));
	Mat dyPos(imSize, CV_32F, Scalar(0));
	Mat dyNeg(imSize, CV_32F, Scalar(0));

	float dxPixel, dyPixel;

	//TODO: use cv::threshold instead of !
	//TODO: add thresholding for eliminating noise
	for (int iRow(0); iRow<grayscaleImage.rows; ++iRow)
	{
		for (int iCol(0); iCol<grayscaleImage.cols; ++iCol)
		{
			dxPixel = ((float*)dx.data)[dx.cols * iRow + iCol];//dx.at<float>(iRow, iCol);
			dyPixel = ((float*)dy.data)[dx.cols * iRow + iCol];//dy.at<float>(iRow, iCol);

			if(dxPixel>0)
				((float*)dxPos.data)[dx.cols * iRow + iCol] = 10*dxPixel;//10 is just a factor for numerical stability, with no particular meaning
			else
				((float*)dxNeg.data)[dx.cols * iRow + iCol] = -10*dxPixel;

			if(dyPixel>0)
				((float*)dyPos.data)[dx.cols * iRow + iCol] = 10*dyPixel;
			else
				((float*)dyNeg.data)[dx.cols * iRow + iCol] = -10*dyPixel;
		}
	}
	outDescriptorFields.push_back(dxPos);
	outDescriptorFields.push_back(dxNeg);
	outDescriptorFields.push_back(dyPos);
	outDescriptorFields.push_back(dyNeg);

}

void ComputeGradientMagnitudeDescriptorFields(Mat &grayscaleImage, vector<Mat> &outDescriptorFields)
{
	outDescriptorFields.clear();
	Mat dx, dy;
	ComputeImageDerivatives(grayscaleImage, dx, dy);
	assert(dx.isContinuous());
	assert(dy.isContinuous());
	dx = dx.mul(dx);
	dy = dy.mul(dy);
	outDescriptorFields.push_back(dx + dy);
}


StructOfArray2di CreateGridOfControlPoints(Mat & image, uint nPoints, 
	float ulx, float uly, float patch_width, float patch_height)
{
	uint width = image.cols;
	uint height = image.rows;
	assert(width > 0);
	assert(height > 0);

	StructOfArray2di controlPoints;
	float deltau((patch_width-1) / nPoints);
	float deltav((patch_height-1) / nPoints);

	int nPointsi, nPointsj;
	nPointsi = nPointsj = nPoints;

	if(deltau < 1) //avoid repeated points
	{
		deltau = 1.;
		nPointsj = ((patch_width - 1) );
	}

	if(deltav < 1)
	{
		deltav = 1.;
		nPointsi = ((patch_height - 1));
	}

	for(uint iPoint(0); iPoint < nPointsi; ++iPoint)
	{
		for(uint jPoint(0); jPoint < nPointsj; ++jPoint)
			controlPoints.push_back(Point(ulx + jPoint*deltau, uly + iPoint*deltav));
	}
	return controlPoints;
}

StructOfArray2di CreateGridOfControlPoints(Mat & image,uint nPoints, float widthBorderThickness, float heightBorderThickness)
{
	uint width = image.cols;
	uint height = image.rows;
	assert(width > 0);
	assert(height > 0);

	StructOfArray2di controlPoints;
	float deltau(((width-1) -2*widthBorderThickness)/nPoints);
	float deltav(((height-1) -2*heightBorderThickness)/nPoints);

	int nPointsi, nPointsj;
	nPointsi = nPointsj = nPoints;

	if(deltau < 1) //avoid repeated points
	{
		deltau = 1.;
		nPointsj = ((width-1) -2*widthBorderThickness);
	}

	if(deltav < 1)
	{
		deltav = 1.;
		nPointsi = ((height-1) -2*heightBorderThickness);
	}

	for(uint iPoint(0); iPoint < nPointsi; ++iPoint)
	{
		for(uint jPoint(0); jPoint < nPointsj; ++jPoint)
			controlPoints.push_back(Point(widthBorderThickness + jPoint*deltau, heightBorderThickness+ iPoint*deltav));
	}
	return controlPoints;
}

// KMYI: use GFT to initialize controlpoints
StructOfArray2di CreateAnisotropicGridOfControlPoints(Mat & image, uint nPoints, 
	float ulx, float uly, float patch_width, float patch_height)

{
	nPoints = nPoints*nPoints; // square the nPoints for compatibility with grid
	/// Set Image ROI
	int width = image.cols;
	int height = image.rows;
	//	if (widthBorderThickness == 0.)
	//		widthBorderThickness = (float)width/10;
	//	if (heightBorderThickness == 0.)
	//		heightBorderThickness = (float)height/10;
	Mat roiImg(image, Rect(ulx, uly, patch_width, patch_height));

	/// Get descriptor fields
	vector<Mat> descImg;
	ComputeGradientBasedDescriptorFields(roiImg, descImg);
	Mat magImg = roiImg.clone();
	Mat magImg2 = roiImg.clone();
	magImg = descImg[0].mul(descImg[0]) + descImg[1].mul(descImg[1]) + descImg[2].mul(descImg[2]) + descImg[3].mul(descImg[3]);
	cv::sqrt(magImg, magImg2);

	/// Parameters for Shi-Tomasi algorithm
	vector<Point2f> corners;
	double qualityLevel = 0.01;
	double minDistance = 5;
	int blockSize = 3;
	bool useHarrisDetector = false;
	double k = 0.01;//0.04;

	/// Apply corner detection
	goodFeaturesToTrack(magImg2,
		corners,
		nPoints,
		qualityLevel,
		minDistance,
		Mat(),
		blockSize,
		useHarrisDetector,
		k);


	StructOfArray2di controlPoints;
	nPoints = MIN(nPoints, corners.size());
	for(uint idxPoint(0); idxPoint < nPoints; ++idxPoint)
	{
		int cur_j = round(corners[idxPoint].x) + ulx;
		int cur_i = round(corners[idxPoint].y) + uly;
		controlPoints.push_back(Point(cur_j, cur_i));
	}
	return controlPoints;
}

// KMYI: use GFT to initialize controlpoints
StructOfArray2di CreateAnisotropicGridOfControlPoints(Mat & image, uint nPoints, float widthBorderThickness, float heightBorderThickness)

{
	nPoints = nPoints*nPoints; // square the nPoints for compatibility with grid
	/// Set Image ROI
	int width = image.cols;
	int height = image.rows;
	//	if (widthBorderThickness == 0.)
	//		widthBorderThickness = (float)width/10;
	//	if (heightBorderThickness == 0.)
	//		heightBorderThickness = (float)height/10;
	Mat roiImg(image, Rect(widthBorderThickness, heightBorderThickness, width - 2*widthBorderThickness, height - 2*heightBorderThickness));

	/// Get descriptor fields
	vector<Mat> descImg;
	ComputeGradientBasedDescriptorFields(roiImg, descImg);
	Mat magImg = roiImg.clone();
	Mat magImg2 = roiImg.clone();
	magImg = descImg[0].mul(descImg[0])+descImg[1].mul(descImg[1])+descImg[2].mul(descImg[2])+descImg[3].mul(descImg[3]);
	cv::sqrt(magImg,magImg2);

	/// Parameters for Shi-Tomasi algorithm
	vector<Point2f> corners;
	double qualityLevel = 0.01;
	double minDistance = 5;
	int blockSize = 3;
	bool useHarrisDetector = false;
	double k = 0.01;//0.04;

	/// Apply corner detection
	goodFeaturesToTrack( magImg2,
			corners,
			nPoints,
			qualityLevel,
			minDistance,
			Mat(),
			blockSize,
			useHarrisDetector,
			k );


	StructOfArray2di controlPoints;
	nPoints = MIN(nPoints, corners.size());
	for(uint idxPoint(0); idxPoint < nPoints; ++idxPoint)
	{
		int cur_j = round(corners[idxPoint].x) + widthBorderThickness;
		int cur_i = round(corners[idxPoint].y) + heightBorderThickness;
		controlPoints.push_back(Point(cur_j, cur_i));
	}
	return controlPoints;
}

StructOfArray2di CreateDenseGridOfControlPoints(uint width, uint height)
{
	StructOfArray2di controlPoints;
	for(uint jPoint(0); jPoint < width; ++jPoint)
	{
		for(uint iPoint(0); iPoint < height; ++iPoint)
			controlPoints.push_back(Point(jPoint, iPoint));
	}
	return controlPoints;
}


void WarpGridOfControlPoints(const StructOfArray2di & pixelsOnTemplate, StructOfArray2di & pixelsOnTemplateWarped, const vector<float> & parameters, const int width, const int height)
{
	pixelsOnTemplateWarped.clear();
	pixelsOnTemplateWarped.reserve(pixelsOnTemplate.size());//TODO : useless?
	//StructOfArray2di tempPixels(pixelsOnTemplate.size());

	vector<vector<bool> > takeMeOn(height,vector<bool>(width,false));

#pragma omp parallel for ordered
	for (int iPoint = 0; iPoint < pixelsOnTemplate.size(); ++iPoint)
	{
		int x,y;
		Homography::ComputeWarpedPixels(pixelsOnTemplate.x[iPoint], pixelsOnTemplate.y[iPoint], x, y, parameters);
		if (x >=0 && y >= 0 && x < width && y < height)
		{
#pragma omp ordered
			if (!takeMeOn[y][x])
			{
				takeMeOn[y][x] = true;
				pixelsOnTemplateWarped.push_back(x, y);
			}

		}
	}
}

StructOfArray2di GetRectangleCornersForAugmentation(OptimizationParameters* optimizationParameters, int width, int height)
{
	StructOfArray2di  panelCorners(4);
	panelCorners.x[0] = optimizationParameters->borderThicknessHorizontal;
	panelCorners.y[0] = optimizationParameters->borderThicknessVertical;
	panelCorners.x[1] = width-optimizationParameters->borderThicknessHorizontal;
	panelCorners.y[1] = optimizationParameters->borderThicknessVertical;
	panelCorners.x[2] = width-optimizationParameters->borderThicknessHorizontal;
	panelCorners.y[2] = height-optimizationParameters->borderThicknessVertical;
	panelCorners.x[3] = optimizationParameters->borderThicknessHorizontal;
	panelCorners.y[3] = height-optimizationParameters->borderThicknessVertical;
	return panelCorners;
}


void AcquireVGAGrayscaleImage(VideoCapture &capture, Mat &outGrayImage, Mat &outRGBImage)
{
	capture >> outRGBImage;
	resize(outRGBImage, outRGBImage, Size(640,480));
	cvtColor(outRGBImage, outGrayImage,CV_RGB2GRAY, 1);
	ConvertImageToFloat(outGrayImage);
}

void AcquireVGAGrayscaleImage(VideoCapture &capture, Mat &outGrayImage)
{
	Mat tempImg;
	capture >> tempImg;
	resize(tempImg, tempImg, Size(640,480));
	cvtColor(tempImg, outGrayImage,CV_RGB2GRAY, 1);
	ConvertImageToFloat(outGrayImage);
}

void ConvertImageToFloat(Mat & image)
{
	//image.convertTo(image, CV_32F);
	double min,max;
	minMaxLoc(image,&min,&max);
	const float v = 1.0/(max - min);
	image.convertTo(image, CV_32F, v, -min * v);
	assert(image.isContinuous());
}


void ComputeImageDerivatives(const Mat & image, Mat & imageDx, Mat &imageDy)
{
	int ddepth = -1; //same image depth as source
	double scale = 1/32.0;// normalize wrt scharr mask for having exact gradient
	double delta = 0;

	Scharr(image, imageDx, ddepth, 1, 0, scale, delta, BORDER_REFLECT );
	Scharr(image, imageDy, ddepth, 0, 1, scale, delta, BORDER_REFLECT );
}

Mat SmoothImage(const float sigma, const Mat &im)
{
	Mat smoothedImage;
	int s = max(5, 2*int(sigma)+1);
	Size kernelSize(s, s);
	GaussianBlur(im, smoothedImage, kernelSize, sigma, sigma,BORDER_REFLECT);
	return smoothedImage;
}

vector<Mat> SmoothDescriptorFields(const float sigma, const vector<Mat> & descriptorFields)
{
	vector<Mat> smoothedDescriptorFields(descriptorFields.size());

#pragma omp parallel for
	for(int iChannel = 0; iChannel < descriptorFields.size(); ++iChannel){
		smoothedDescriptorFields[iChannel] = SmoothImage(sigma, descriptorFields[iChannel]);}

	return smoothedDescriptorFields;
}



void NormalizeImage(Mat &image)
{
	Scalar mean, stddev;
	meanStdDev(image, mean, stddev);
	image = (image - mean)/stddev[0];
}

vector<Point2f> ReadArrayOf2dPoints(const char* fileName)
{
	ifstream aFileStream;
	aFileStream.open(fileName);
	if(!aFileStream)
	{
		cerr << "FILE COULD NOT BE OPENED !!!!!!"<<endl;
		exit(0);
	}
	vector<Point2f> pointsArray;
	float a, b;
	while (aFileStream >> a >> b)
	{
		Vec2f tempPoint(a,b);
		pointsArray.push_back(tempPoint);
	}
	aFileStream.close();
	return pointsArray;
}

vector<float> ReadArrayOfFloats(const char* fileName)
{
	ifstream aFileStream;
	aFileStream.open(fileName);
	if(!aFileStream)
	{
		cerr << "FILE COULD NOT BE OPENED !!!!!!"<<endl;
		exit(0);
	}
	vector<float> data;
	float a;
	while (aFileStream >> a)
	{
		data.push_back(a);
	}
	aFileStream.close();
	return data;
}

vector<vector<float> > ReadMatrixOfFloats(const char* fileName)
{
	vector<vector<float> > data;
	string lineString;
	double a;

	ifstream aFileStream;
	aFileStream.open(fileName);
	if(!aFileStream)
	{
		cerr << "FILE COULD NOT BE OPENED !!!!!!"<<endl;
		exit(0);
	}

	while(getline(aFileStream, lineString))
	{
		stringstream is(lineString);
		vector<float> aDataRow;
		while (is >> a)
		{
			aDataRow.push_back(a);
		}
		data.push_back(aDataRow);
	}

	aFileStream.close();
	return data;
}



void WritePixelsOnTxtFile(const StructOfArray2di & pixels, const char* fileName)
{
	ofstream myfile;
	myfile.open (fileName);
	if(!myfile)
	{
		cerr << "FILE COULD NOT BE OPENED !!!!!!"<<endl;
		exit(0);
	}
	for ( int i(0); i < pixels.size(); ++i)
		myfile << pixels.x[i] << " " << pixels.y[i]<<endl;;
	myfile.close();
}


void LoadImage(const char* fileName, Mat &image)
{
	image = imread(fileName, CV_LOAD_IMAGE_GRAYSCALE);
	image.convertTo(image, CV_32F);
	//	Size res(cameraInternalParameters.width, cameraInternalParameters.height);
	//	resize(image, image, res);
	double min,max;
	minMaxLoc(image,&min,&max);
	image = (image - min)/(max - min);
}

Mat ReadGrayscaleImageFile(const char* fileName, uint nRows, uint nCols)
{
	Mat image(nRows, nCols, CV_32F);

	ifstream aFileStream;
	aFileStream.open(fileName);
	if(!aFileStream)
	{
		cerr << "FILE COULD NOT BE OPENED !!!!!!"<<endl;
		exit(0);
	}
	aFileStream.seekg(0, ios::beg);
	for(uint iRow(0); iRow < nRows; ++iRow)
	{
		for(uint iCol(0); iCol < nCols; ++iCol)
			aFileStream >>image.at<float>(iRow, iCol);
	}
	return image;
}

OptimizationParameters ReadOptimizationParametersFromXML(const char* fileName)
{
	FileStorage fs2(fileName, FileStorage::READ);
	if (!fs2.isOpened())
	{
		cerr << "Failed to open: !" <<  fileName<<"  . Aborting" <<endl;
		exit(0);
	}
	OptimizationParameters optParam;
	fs2["resTol"] >> optParam.resTol;
	fs2["pTol"] >> optParam.pTol;
	fs2["maxIter"] >> optParam.maxIter;
	fs2["maxIterSingleLevel"] >> optParam.maxIterSingleLevel;

	FileNode n = fs2["pyramidSmoothingVariance"];
	if (n.type() == FileNode::SEQ)
	{
		FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
		for (; it != it_end; ++it)
			optParam.pyramidSmoothingVariance.push_back((float)*it);
	}
	fs2["presmoothingVariance"] >> optParam.presmoothingVariance;
	fs2["nControlPointsOnEdge"] >> optParam.nControlPointsOnEdge;
	fs2["borderThicknessHorizontal"] >> optParam.borderThicknessHorizontal;
	fs2["borderThicknessVertical"] >> optParam.borderThicknessVertical;
	fs2["bAdaptativeChoiceOfPoints"] >> optParam.bAdaptativeChoiceOfPoints;
	fs2["bNormalizeDescriptors"] >> optParam.bNormalizeDescriptors;

	fs2.release();
	return optParam;
}

void ShowDetailedOptimizationResults(const AlignmentResults & results, vector<float> parametersBaseline)
{
	float paramNorm2(0);
	cout<<endl<<endl<< "************  Optimization results  *******************"<<endl;
	cout<< "exit flag  : "<< results.exitFlag<< "       - (1->exceeded maxIter; 0,-1->converged)"<<endl;
	cout<< "Number of iterations: "<< results.nIter<<endl;
	cout<< "estimation of the pose  : "<<endl;
	for(uint iIter(0); iIter < results.poseIntermediateGuess.size();++iIter)
	{
		paramNorm2 = 0;
		for (uint iParam(0); iParam < 8;++iParam)
			paramNorm2 += pow((parametersBaseline[iParam] - (results.poseIntermediateGuess[iIter])[iParam]), 2);

		cout<<" iter:"<<iIter<<" --> res. norm = " << results.residualNorm[iIter]<<";param. error norm =     " << paramNorm2<< endl;
	}
}

void ShowConvergenceResults(Mat & templ, Mat &image, vector<vector<float> > &intermediateGuess)
{
	StructOfArray2di panelCorners(4);
	panelCorners.x[0] = 1;
	panelCorners.y[0] = 1;
	panelCorners.x[1] = templ.cols-1;
	panelCorners.y[1] = 1;
	panelCorners.x[2] = templ.cols-1;
	panelCorners.y[2] = templ.rows-1;
	panelCorners.x[3] = 1;
	panelCorners.y[3] = templ.rows-1;

	ShowConvergenceResults(templ, image, intermediateGuess, panelCorners);
}


void ShowConvergenceResults(Mat & templ, Mat &image, vector<vector<float> > &intermediateGuess, StructOfArray2di panelCorners)
{
	StructOfArray2di warpedPixels(4);
	namedWindow("Shot template", CV_WINDOW_AUTOSIZE );
	AugmentFrameWithQuadrilater("Shot template", panelCorners,templ);
	waitKey(0);
	namedWindow("ConvergenceResults", CV_WINDOW_AUTOSIZE );
	int x,y;
	for(uint i(0); i < 4; ++i)
	{
		Homography::ComputeWarpedPixels(panelCorners[i].x,panelCorners[i].y, x,y, intermediateGuess[intermediateGuess.size()-1]);
		warpedPixels.x[i] = x;
		warpedPixels.y[i] = y;
	}

	//	AugmentFrameWithQuadrilater("ConvergenceResults", warpedPixels, shotImage.image);
	//	waitKey(0);
	for (uint iGuess(0); iGuess < intermediateGuess.size(); ++iGuess)
	{
		int x,y;
		for(uint i(0); i < 4; ++i)
		{
			Homography::ComputeWarpedPixels(panelCorners[i].x,panelCorners[i].y, x,y, intermediateGuess[iGuess]);
			warpedPixels.y[i] = y;
		}

		Mat warpedImage = image.clone();
		AugmentFrameWithQuadrilater("ConvergenceResults", warpedPixels, warpedImage);
		for(int i(0); i< 100000; ++i)
			;
		if (iGuess == intermediateGuess.size()-1)
			waitKey(0);
	}
}

void AugmentFrameWithQuadrilater(string windowName, const StructOfArray2di & warpedPixels, Mat& frame)
{
	Scalar color(1.,1.,1.,0.1);
	int thickness=3;
	int lineType=8;
	int shift=0;
	line(frame, warpedPixels[0], warpedPixels[1], color, thickness, lineType, shift);
	line(frame, warpedPixels[1], warpedPixels[2], color, thickness, lineType, shift);
	line(frame, warpedPixels[2], warpedPixels[3], color, thickness, lineType, shift);
	line(frame, warpedPixels[0], warpedPixels[3], color, thickness, lineType, shift);
	//		// 		fillConvexPoly(image, &panelCorners[0], 4, color);
	imshow(windowName, frame);
}

void AugmentFrameWithQuadrilater(const StructOfArray2di & warpedPixels, Mat& frame)
{
	Scalar color(1.,1.,1.,0.1);
	int thickness=3;
	int lineType=8;
	int shift=0;
	line(frame, warpedPixels[0], warpedPixels[1], color, thickness, lineType, shift);
	line(frame, warpedPixels[1], warpedPixels[2], color, thickness, lineType, shift);
	line(frame, warpedPixels[2], warpedPixels[3], color, thickness, lineType, shift);
	line(frame, warpedPixels[0], warpedPixels[3], color, thickness, lineType, shift);
	//		// 		fillConvexPoly(image, &panelCorners[0], 4, color);
	//imshow(windowName, frame);
}

void WriteResultsOnImage(Mat & image, const AlignmentResults & results, int pixelsNumber, OptimizationType optimizationType)
{
	ostringstream str;
	str << "Optimization : " << optimizationType;
	putText(image, str.str(), Point(10,50), FONT_HERSHEY_TRIPLEX, 1.2, CV_RGB(255,255,255));

	str.str("");
	str << "exit flag:" << results.exitFlag;
	putText(image, str.str(), Point(10,80), FONT_HERSHEY_DUPLEX, 1, CV_RGB(255,255,255));

	str.str("");
	str << "n. iterations:" << results.nIter;
	putText(image, str.str(), Point(10,110), FONT_HERSHEY_DUPLEX,1, CV_RGB(255,255,255));

	str.str("");
	str << "final residual norm:" << results.residualNorm.back();
	putText(image, str.str(), Point(10,140), FONT_HERSHEY_DUPLEX, 1, CV_RGB(255,255,255));

	str.str("");
	str << "number of pixels :" << pixelsNumber;
	putText(image, str.str(), Point(10,170), FONT_HERSHEY_DUPLEX, 1, CV_RGB(255,255,255));
}

void CheckMatrixForNans(Eigen::MatrixXf & aMatrix)
{
	for(uint i(0); i< aMatrix.rows(); ++i)
	{
		for(uint j(0); j< aMatrix.cols(); ++j)
		{
			if(aMatrix(i,j) != aMatrix(i,j))
			{
				cout<<"found nan at "<<i << " "<<j << endl;
				break;
			}
		}
	}
}

void operator<<( std::ostream& os, const OptimizationType& optimizationType )
{
	switch( optimizationType )
	{
	case intensity: os << "intensity"; break;
	case gradientModule: os << "gradientModule"; break;
	case descriptorFields: os << "descriptorFields"; break;
	}
}


