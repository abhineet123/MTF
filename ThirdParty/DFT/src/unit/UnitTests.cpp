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



#include "mtf/ThirdParty/DFT/UnitTests.hpp"
	
using namespace cv;
using namespace std;
using namespace std::chrono;

//#define B_PLOT

void ReadDataFromXMLTest()
{
	OptimizationParameters optParam = ReadOptimizationParametersFromXML("parameter/parametersForTest.yml");

	float bOk = true;

	if(abs(optParam.resTol - 0.005) > 1e-9)	bOk = false;
	if(abs(optParam.pTol - 0.000004) > 1e-9)	bOk = false;
	if(optParam.maxIter != 100)	bOk = false;
	if(optParam.maxIterSingleLevel != 10)	bOk = false;
	if(optParam.pyramidSmoothingVariance[0] != 35)	bOk = false;
	if(optParam.pyramidSmoothingVariance[1] != 17)	bOk = false;
	if(optParam.pyramidSmoothingVariance[2] != 9)	bOk = false;
	if(optParam.pyramidSmoothingVariance[3] != 5)	bOk = false;
	if(optParam.presmoothingVariance != 2)	bOk = false;
	if(optParam.nControlPointsOnEdge != 40)	bOk = false;
	if(optParam.borderThicknessHorizontal != 10)	bOk = false;
	if(abs(optParam.borderThicknessVertical - 10.4) > 1e-6)	bOk = false;
	if(optParam.bAdaptativeChoiceOfPoints != 0)	bOk = false;
	if(optParam.bNormalizeDescriptors != 0)	bOk = false;

	if (bOk ==false)
		throw "ReadDataFromXMLTest failed";

}


void InverseHomographyTest()
{
	Mat otherParameters(8, 1, CV_32F);
	vector<float> parameters(8,0);
	vector<float> parametersInv(8);
	float eps(1);

	for(uint iIter(0); iIter < 1000; ++iIter)
	{
		for(uint i(0); i < 8; ++i)
		{
			parameters[i] = (float)rand()/RAND_MAX;
			otherParameters.at<float>(i,0) = parameters[i];
		}

		Point3f inputPixel(1+(rand()% 1000), 1+(rand()% 1000), 1);
		Point3f outputPixel, outputPixel2;

		Homography::InverseWarpParameters(parameters, parametersInv);
		Homography::ComputeWarpedHomoPoints(inputPixel, outputPixel, parameters);
		float denom((outputPixel.x/inputPixel.x));
		outputPixel.x /= denom;
		outputPixel.y /= denom;
		outputPixel.z /= denom;

		Homography::ComputeWarpedHomoPoints(outputPixel, outputPixel2, parametersInv);
		denom = (outputPixel2.x/outputPixel.x);
		outputPixel2.x /= denom;
		outputPixel2.y /= denom;

		if (((abs(inputPixel.x - outputPixel2.x)) < eps) == 0 || ((abs(inputPixel.y - outputPixel2.y)) < eps) == 0)
		{
			cout << "point  input"<<  inputPixel.x << " " << inputPixel.y<<endl;
			cout << "point output " <<outputPixel2.x << " " << outputPixel2.y<<endl;
		}

		Homography::InverseWarpParameters(otherParameters, parameters);
		for(uint i(0); i < 8; ++i)
			if	(abs(parameters[i]- parametersInv[i]) > 1e-10) throw "InverseHomographyTest failed" ;


		if(abs(inputPixel.x - outputPixel2.x) > eps) throw "InverseHomographyTest failed" ;
		if(abs(inputPixel.y - outputPixel2.y) > eps) throw "InverseHomographyTest failed" ;
	}
}

void CompositionHomographyTest()
{
	vector<float> parameters(8,0);
	vector<float> parametersNew(8);
	vector<float> deltap(8);

	float eps(1e-4);
	for(uint iIter(0); iIter < 1000; ++iIter)
	{
		for(uint i(0); i < 8; ++i)
		{
			parameters[i] = (float)rand()/ RAND_MAX;
			deltap[i] = (float)rand()/ RAND_MAX;
		}

		Point3f inputPixel(1+rand()%1000, 1+rand()%1000, 1);
		Point3f outputPixel, outputPixel2, outputPixelComp;

		Homography::ComputeWarpedHomoPoints(inputPixel, outputPixel,parameters );
		Homography::ComputeWarpedHomoPoints(outputPixel, outputPixel2, deltap);

		vector<float> newP = Homography::ParametersUpdateCompositional(parameters, deltap);
		Homography::ComputeWarpedHomoPoints(inputPixel, outputPixelComp, newP);

		if (((abs(outputPixelComp.x - outputPixel2.x)) < eps) == 0 || ((abs(outputPixelComp.y - outputPixel2.y)) < eps) == 0)
		{
			cout << "point numerical composition " <<outputPixel2.x << " " << outputPixel2.y<<endl;
			cout << "point  composition " <<outputPixelComp.x << " " << outputPixelComp.y<<endl;
			throw "CompositionHomographyTest failed!";
		}
	}
}


void HomographyJacobianTest2()
{
	vector<vector<float> > jacobianBaseline = ReadMatrixOfFloats("data/jacobian.txt");
	Eigen::Matrix<float, 2, 8> warpJacobian;
	Mat templ = imread("data/template.png", CV_LOAD_IMAGE_GRAYSCALE);
	ConvertImageToFloat(templ);
	StructOfArray2di controlPoints = CreateDenseGridOfControlPoints(templ.cols, templ.rows);
	vector<float> parameters(8,0);
	parameters[4] = 8.7021963e+01-20;
	parameters[5] = 3.0940031e+01 - 20;

	for(uint iPoint(0); iPoint <controlPoints.size() ; ++iPoint)
	{
		controlPoints.x[iPoint] += 1;//matlab matrices are 1 based
		controlPoints.y[iPoint] += 1;

		Homography::ComputeWarpJacobian(controlPoints.x[iPoint], controlPoints.y[iPoint], parameters, warpJacobian);
		for (uint i(0); i < 8; ++i)
		{
			if(abs(warpJacobian(0, i) -(jacobianBaseline[2*iPoint])[i]) > 5e-3) throw "HomographyJacobianTest2 failed";
			if(abs(warpJacobian(1, i) -(jacobianBaseline[2*iPoint+1])[i]) > 5e-3) throw "HomographyJacobianTest2 failed";
		}
	}
}


void ComputeHomographyTest()
{
	vector<float> parameters(8,0);
	float eps(1);

	for(uint iIter(0); iIter < 1000; ++iIter)
	{
		for(uint i(0); i < 8; ++i)
			parameters[i] = (float)rand()/RAND_MAX;

		Point3f inputPixel(1+(rand()% 1000), 1+(rand()% 1000), 1);
		Point3f outputPixel, outputPixel2;
		Homography::ComputeWarpedHomoPoints(inputPixel, outputPixel, parameters);

		Matx33f aMatrix = Homography::GetMatrix(parameters);
		outputPixel2 = aMatrix * inputPixel;
		outputPixel2.x /= outputPixel2.z;
		outputPixel2.y /= outputPixel2.z;
		outputPixel2.z /= outputPixel2.z;

		Point inputPixel2d, outputPixel2d;
		inputPixel2d.x = (int)inputPixel.x;
		inputPixel2d.y = (int)inputPixel.y;

		Homography::ComputeWarpedPixels(inputPixel2d.x , inputPixel2d.y, outputPixel2d.x, outputPixel2d.y, parameters);
		if(abs(outputPixel.x - outputPixel2d.x) > eps)throw "ComputeHomographyTest failed";
		if(abs(outputPixel.y - outputPixel2d.y) > eps)throw "ComputeHomographyTest failed";
		if(abs(outputPixel.x - outputPixel2.x) > eps)throw "ComputeHomographyTest failed";
		if(abs(outputPixel.y - outputPixel2.y) > eps)throw "ComputeHomographyTest failed";
	}
}


void WarpedIntensitiesTest()
{

	Mat templ = imread("data/template.png", CV_LOAD_IMAGE_GRAYSCALE);
	ConvertImageToFloat(templ);

	Mat image = imread("data/imageCropped.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	ConvertImageToFloat(image);

	StructOfArray2di controlPoints = CreateDenseGridOfControlPoints(templ.cols, templ.rows);
	for (uint iPoint(0); iPoint < controlPoints.size(); ++iPoint)
	{
		controlPoints.x[iPoint] += 1;//matlab matrices are 1 based
		controlPoints.y[iPoint] += 1;
	}
	vector<float> parametersInitialGuess(8,0);
	parametersInitialGuess[4] = 8.7021963e+01-20;
	parametersInitialGuess[5] = 3.0940031e+01 - 20;
	vector<float> warpedPixelIntensities(controlPoints.size());
	LucasKanade optimization;
	StructOfArray2di warpedPixels;
	optimization.ComputeWarpedPixels(controlPoints, parametersInitialGuess, warpedPixels);

	for(uint iPoint(0); iPoint < controlPoints.size(); ++iPoint)
		warpedPixelIntensities[iPoint] = image.at<float>(round((warpedPixels[iPoint]).y)-1, round((warpedPixels[iPoint]).x)-1);

	vector<float> intensitiesBaseline = ReadArrayOfFloats("data/warpedIntensities.txt");
	float max(0), temp;
	for(uint iPoint(0); iPoint < warpedPixelIntensities.size(); ++iPoint)
	{
		temp = abs(warpedPixelIntensities[iPoint] - intensitiesBaseline[iPoint]);
		if (temp > 4e-2)
			cout <<warpedPixelIntensities[iPoint]<< " vs" << intensitiesBaseline[iPoint]<< "  "<<round((warpedPixels[iPoint]).y)-1 << " " << round((warpedPixels[iPoint]).x)-1<< endl;

		if (temp > max)
			max = temp;
	}
	if (max > 4e-2)
		throw "WarpedIntensitiesTest failed.";
}


void SDImagesTest()
{
	Mat templ = imread("data/template.png", CV_LOAD_IMAGE_GRAYSCALE);
	ConvertImageToFloat(templ);

	Mat image = imread("data/imageCropped.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	ConvertImageToFloat(image);

	StructOfArray2di controlPoints = CreateDenseGridOfControlPoints(templ.cols, templ.rows);

	for (uint iPoint(0); iPoint < controlPoints.size(); ++iPoint)
	{
		controlPoints.x[iPoint] += 1;//matlab matrices are 1 based
		controlPoints.y[iPoint] += 1;
	}

	vector<float> parametersInitialGuess(8,0);
	parametersInitialGuess[4] = 8.7021963e+01-20;
	parametersInitialGuess[5] = 3.0940031e+01 - 20;

	Mat imageDx = ReadGrayscaleImageFile("data/imageDx.txt", 452, 378);
	Mat imageDy = ReadGrayscaleImageFile("data/imageDy.txt", 452, 378);
	StructOfArray2di warpedPixels;
	Eigen::MatrixXf sdImages(controlPoints.size(), 8);
	LucasKanade optimization;
	optimization.ComputeWarpedPixels(controlPoints, parametersInitialGuess, warpedPixels);
	vector<Eigen::Matrix<float, 2, N_PARAM>, Eigen::aligned_allocator<Eigen::Matrix<float, 2, N_PARAM> > > warpJacobians(controlPoints.size());
	for (uint iPoint(0); iPoint < controlPoints.size(); ++iPoint)
		Homography::ComputeWarpJacobian(controlPoints.x[iPoint], controlPoints.y[iPoint], parametersInitialGuess, warpJacobians[iPoint]);

	optimization.AssembleSDImages(parametersInitialGuess, imageDx, imageDy, warpedPixels, warpJacobians, sdImages);
	vector<vector<float> > sdImageBaseline = ReadMatrixOfFloats("data/SDImage.txt");

	for(uint iPoint(0); iPoint < controlPoints.size(); ++iPoint)
	{
		for(uint iParam(0); iParam < 8; ++iParam)
			if(abs((sdImageBaseline[iPoint])[iParam] - sdImages(iPoint, iParam)) > 1e-2)
				throw "SDImagesTest failed";
	}
}

void LucaskanadeSSDTest()
{
	//we set some parameters for the optimization. Don't touch !
	OptimizationParameters optimizationParameters;
	optimizationParameters.pTol = 5e-4;
	optimizationParameters.resTol = 5e-8;
	optimizationParameters.maxIter = 200;
	optimizationParameters.maxIterSingleLevel = 10;

	Mat templ = imread("data/template.png", CV_LOAD_IMAGE_GRAYSCALE);
	ConvertImageToFloat(templ);
	Mat image = imread("data/imageCropped.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	ConvertImageToFloat(image);

	Matx33f homography(1+4.2553191e-02, -6.4516129e-02, 8.7021963e+01,
			4.7872340e-02, 1+1.2096774e-02, 3.0940031e+01,
			0, 0, 1);
	vector<float> parametersBaseline(8);
	parametersBaseline[0] = 4.2553191e-02;
	parametersBaseline[1] = 4.7872340e-02;
	parametersBaseline[2] = -6.4516129e-02;
	parametersBaseline[3] = 1.2096774e-02;
	parametersBaseline[4] = 8.7021963e+01;
	parametersBaseline[5] = 3.0940031e+01;
	parametersBaseline[6] = 0;
	parametersBaseline[7] = 0;

	vector<float> parametersInitialGuess(8,0);
	parametersInitialGuess[4] = 8.7021963e+01-20;
	parametersInitialGuess[5] = 3.0940031e+01 - 20;
	LucasKanade optimization;
	StructOfArray2di controlPoints = CreateDenseGridOfControlPoints(templ.cols, templ.rows);
	clock_t begin = clock();
	AlignmentResults results = optimization.SSDCalibration(controlPoints, templ, image, parametersInitialGuess, optimizationParameters);
	clock_t end = clock();
	cout<< "Elapsed time = "<< (double(end - begin) / CLOCKS_PER_SEC)<<endl;

	float paramNorm2(0);
	for (uint iParam(0); iParam < 8;++iParam)
		paramNorm2 += pow((parametersBaseline[iParam] - (results.poseIntermediateGuess[results.poseIntermediateGuess.size()-1])[iParam]), 2);

#ifdef B_PLOT
	ShowDetailedOptimizationResults(results, parametersBaseline);
	namedWindow("Template", CV_WINDOW_AUTOSIZE );
	imshow("Template", templ);
	waitKey(0);
	namedWindow("image", CV_WINDOW_AUTOSIZE );
	imshow("image", image );
	waitKey(0);
	ShowConvergenceResults(templ, image, results.poseIntermediateGuess);
#endif

	if (paramNorm2 > 2e-2) throw "LucaskanadeSSDTest failed 1 " ;
	if (results.residualNorm[results.residualNorm.size()-1] > 0.35) throw "LucaskanadeSSDTest failed 2 " ;
	//check that parametersInitialGuess is updated with the current guess
	for (uint i(0); i< parametersInitialGuess.size(); ++i)
		if(abs(parametersInitialGuess[i] - (results.poseIntermediateGuess[results.poseIntermediateGuess.size()-1])[i]) > 1e-12) throw "LucaskanadeSSDTest failed 3" ;
}

void LucaskanadeDescriptorFieldsTest()
{
	//we set some parameters for the optimization. Don't touch !
	OptimizationParameters optimizationParameters;
	optimizationParameters.pTol = 5e-8;
	optimizationParameters.resTol = 5e-8;
	optimizationParameters.maxIter = 40;
	optimizationParameters.maxIterSingleLevel = 10;
	optimizationParameters.pyramidSmoothingVariance.push_back(10);
	optimizationParameters.pyramidSmoothingVariance.push_back(1);

	Mat templ = imread("data/template.png", CV_LOAD_IMAGE_GRAYSCALE);
	ConvertImageToFloat(templ);
	Mat image = imread("data/imageCropped.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	ConvertImageToFloat(image);

	vector<float> parametersBaseline(8);
	parametersBaseline[0] = 4.2553191e-02;
	parametersBaseline[1] = 4.7872340e-02;
	parametersBaseline[2] = -6.4516129e-02;
	parametersBaseline[3] = 1.2096774e-02;
	parametersBaseline[4] = 8.7021963e+01;
	parametersBaseline[5] = 3.0940031e+01;
	parametersBaseline[6] = 0;
	parametersBaseline[7] = 0;

	StructOfArray2di controlPoints = CreateDenseGridOfControlPoints(templ.cols, templ.rows);

	vector<float> parametersInitialGuess(8,0);
	parametersInitialGuess[4] = 8.7021963e+01-20;
	parametersInitialGuess[5] = 3.0940031e+01 - 20;

	LucasKanade optimization;
	clock_t begin = clock();
	AlignmentResults results = optimization.DescriptorFieldsCalibration(controlPoints, templ, image, parametersInitialGuess, optimizationParameters);
	clock_t end = clock();
	cout<< "Elapsed time = "<< (double(end - begin) / CLOCKS_PER_SEC)<<endl;

	float paramNorm2(0);
	for (uint iParam(0); iParam < 8;++iParam)
		paramNorm2 += pow((parametersBaseline[iParam] - (results.poseIntermediateGuess[results.poseIntermediateGuess.size()-1])[iParam]), 2);

#ifdef B_PLOT

	ShowDetailedOptimizationResults(results, parametersBaseline);
	namedWindow("Template", CV_WINDOW_AUTOSIZE );
	imshow("Template", templ);
	waitKey(0);
	namedWindow("image", CV_WINDOW_AUTOSIZE );
	imshow("image", image );
	waitKey(0);
	ShowConvergenceResults(templ, image, results.poseIntermediateGuess);
#endif

	if(paramNorm2 > 2e-2) throw "LucaskanadeDescriptorFieldsTest failed 1 " ;
	if(results.residualNorm[results.residualNorm.size()-1] > 0.3) throw "LucaskanadeDescriptorFieldsTest failed 2 " ;
	//check that parametersInitialGuess is updated with the current guess
	for (uint i(0); i< parametersInitialGuess.size(); ++i)
	{
		if(abs(parametersInitialGuess[i] - (results.poseIntermediateGuess[results.poseIntermediateGuess.size()-1])[i]) > 1e-12) throw "LucaskanadeDescriptorFieldsTest failed 3 " ;
	}

}


void LucaskanadeVideoSSDSpeedTest()
{

	VideoCapture capture("data/sample1.mov");
	OptimizationParameters optimizationParameters;
	optimizationParameters.resTol = 1e-5;
	optimizationParameters.pTol = 5e-5;
	optimizationParameters.maxIter =  50;
	optimizationParameters.maxIterSingleLevel = 10;
	optimizationParameters.pyramidSmoothingVariance.push_back(7);
	optimizationParameters.presmoothingVariance = 1;
	optimizationParameters.nControlPointsOnEdge = 60;
	optimizationParameters.borderThicknessHorizontal = 100;
	optimizationParameters.borderThicknessVertical = 50;
	optimizationParameters.bAdaptativeChoiceOfPoints = 0;
	optimizationParameters.bNormalizeDescriptors = 1;

	LucasKanade optimization;
	AlignmentResults results;

	Mat templ, image, imageRGB;
	Mat firstFrame;
	AcquireVGAGrayscaleImage(capture, templ);
	firstFrame = templ.clone();

	StructOfArray2di pixelsOnTemplate;
	pixelsOnTemplate = CreateGridOfControlPoints(templ, optimizationParameters.nControlPointsOnEdge, optimizationParameters.borderThicknessHorizontal, optimizationParameters.borderThicknessVertical);
	StructOfArray2di pixelsOnTemplateWarped = pixelsOnTemplate;

	vector<float> parameters(8,0);
	vector<float> parametersInitialGuess(8,0);

#ifdef B_PLOT
	string window_name = "SSD CALIBRATION";
	namedWindow(window_name, CV_WINDOW_KEEPRATIO);
	StructOfArray2di warpedPixels(4);
	StructOfArray2di  panelCorners = GetRectangleCornersForAugmentation(&optimizationParameters, templ.cols, templ.rows);
#endif

	Matx33f homographyMatrix;
	high_resolution_clock::time_point startTime, endTime;
	float elapsedTime(0.), tim;
	int nFrames = 0;
	while(nFrames < 50)
	{
		AcquireVGAGrayscaleImage(capture, image, imageRGB);

		if (image.empty())
			break;

		startTime = high_resolution_clock::now();
		results = optimization.SSDCalibration(pixelsOnTemplateWarped, templ, image, parametersInitialGuess, optimizationParameters);//parameters are updated with the current guess!
		Homography::CompositionalParametersUpdateWithCheck(parameters, parametersInitialGuess);
		homographyMatrix = Homography::GetUnscaledMatrix(parameters);
		warpPerspective(firstFrame, templ, homographyMatrix, templ.size(), INTER_LINEAR, BORDER_CONSTANT);
		WarpGridOfControlPoints(pixelsOnTemplate, pixelsOnTemplateWarped, parameters, templ.cols, templ.rows);
		fill(parametersInitialGuess.begin(), parametersInitialGuess.end(), 0.);
		nFrames++;
		endTime = high_resolution_clock::now();
		tim = duration_cast<duration<double> >(endTime - startTime).count();
		elapsedTime +=tim;

#ifdef B_PLOT
		for (int iPoint = 0; iPoint< panelCorners.size(); ++iPoint)
			Homography::ComputeWarpedPixels(panelCorners[iPoint].x,panelCorners[iPoint].y, warpedPixels.x[iPoint], warpedPixels.y[iPoint], parameters);
		AugmentFrameWithQuadrilater(warpedPixels, imageRGB);
		Mat frameToDisplay = imageRGB;
		imshow(window_name, frameToDisplay);
		waitKey(50);
#endif
	}
	cout << "average elapsedTime = "<< elapsedTime/nFrames<<endl;
}


void LucaskanadeVideoDesciptorFieldsSpeedTest()
{
	VideoCapture capture("data/sample1.mov");

	OptimizationParameters optimizationParameters;
	optimizationParameters.resTol = 1e-5;
	optimizationParameters.pTol = 5e-5;
	optimizationParameters.maxIter =  50;
	optimizationParameters.maxIterSingleLevel = 10;
	optimizationParameters.pyramidSmoothingVariance.push_back(10);
	optimizationParameters.pyramidSmoothingVariance.push_back(5);
	optimizationParameters.presmoothingVariance = 0;
	optimizationParameters.nControlPointsOnEdge = 60;
	optimizationParameters.borderThicknessHorizontal = 100;
	optimizationParameters.borderThicknessVertical = 50;
	optimizationParameters.bAdaptativeChoiceOfPoints = 0;
	optimizationParameters.bNormalizeDescriptors = 0;

	LucasKanade optimization;
	AlignmentResults results;

	Mat templ, image, imageRGB, firstFrame;

	AcquireVGAGrayscaleImage(capture, templ);
	firstFrame = templ.clone();

	StructOfArray2di pixelsOnTemplate;
	pixelsOnTemplate = CreateGridOfControlPoints(templ, optimizationParameters.nControlPointsOnEdge, optimizationParameters.borderThicknessHorizontal, optimizationParameters.borderThicknessVertical);
	StructOfArray2di pixelsOnTemplateWarped = pixelsOnTemplate;

	vector<float> parameters(8,0);
	vector<float> parametersInitialGuess(8,0);

#ifdef B_PLOT
	string window_name = "ISMAR Demo 2014";
	namedWindow(window_name, CV_WINDOW_KEEPRATIO);
	StructOfArray2di warpedPixels(4);
	StructOfArray2di  panelCorners = GetRectangleCornersForAugmentation(&optimizationParameters, templ.cols, templ.rows);
#endif

	Matx33f homographyMatrix;
	high_resolution_clock::time_point startTime, endTime;
	float elapsedTime(0.), tim;
	int nFrames = 0;
	while(nFrames < 50)
	{
		AcquireVGAGrayscaleImage(capture, image, imageRGB);
		if (image.empty())
			break;

		startTime = high_resolution_clock::now();

		results = optimization.DescriptorFieldsCalibration(pixelsOnTemplateWarped, templ, image, parametersInitialGuess, optimizationParameters);//parameters are updated with the current guess!
		Homography::CompositionalParametersUpdateWithCheck(parameters, parametersInitialGuess);
		homographyMatrix = Homography::GetUnscaledMatrix(parameters);
		warpPerspective(firstFrame, templ, homographyMatrix, templ.size(), INTER_LINEAR, BORDER_CONSTANT);
		WarpGridOfControlPoints(pixelsOnTemplate, pixelsOnTemplateWarped, parameters, templ.cols, templ.rows);
		fill(parametersInitialGuess.begin(), parametersInitialGuess.end(), 0.);
		nFrames++;

		endTime = high_resolution_clock::now();
		tim = duration_cast<duration<double> >(endTime - startTime).count();
		elapsedTime +=tim;

#ifdef B_PLOT
		for (int iPoint = 0; iPoint< panelCorners.size(); ++iPoint)
			Homography::ComputeWarpedPixels(panelCorners[iPoint].x,panelCorners[iPoint].y, warpedPixels.x[iPoint], warpedPixels.y[iPoint], parameters);
		AugmentFrameWithQuadrilater(warpedPixels, imageRGB);
		Mat frameToDisplay = imageRGB;
		imshow(window_name, frameToDisplay);
		waitKey(50);
#endif
	}
	cout << "average elapsedTime = "<< elapsedTime/nFrames<<endl;
}

void LucaskanadeVideoGradientMagnitudeSpeedTest()
{
	VideoCapture capture("data/sample1.mov");
	OptimizationParameters optimizationParameters;
	optimizationParameters.resTol = 1e-5;
	optimizationParameters.pTol = 5e-5;
	optimizationParameters.maxIter =  50;
	optimizationParameters.maxIterSingleLevel = 10;
	optimizationParameters.pyramidSmoothingVariance.push_back(10);
	optimizationParameters.pyramidSmoothingVariance.push_back(5);
	optimizationParameters.presmoothingVariance = 1;
	optimizationParameters.nControlPointsOnEdge = 60;
	optimizationParameters.borderThicknessHorizontal = 100;
	optimizationParameters.borderThicknessVertical = 50;
	optimizationParameters.bAdaptativeChoiceOfPoints = 0;
	optimizationParameters.bNormalizeDescriptors = 0;

	LucasKanade optimization;
	AlignmentResults results;

	Mat templ, image, imageRGB, firstFrame;

	AcquireVGAGrayscaleImage(capture, templ);
	firstFrame = templ.clone();

	StructOfArray2di pixelsOnTemplate;
	pixelsOnTemplate = CreateGridOfControlPoints(templ, optimizationParameters.nControlPointsOnEdge, optimizationParameters.borderThicknessHorizontal, optimizationParameters.borderThicknessVertical);
	StructOfArray2di pixelsOnTemplateWarped = pixelsOnTemplate;

	vector<float> parameters(8,0);
	vector<float> parametersInitialGuess(8,0);

#ifdef B_PLOT
	string window_name = "ISMAR Demo 2014";
	namedWindow(window_name, CV_WINDOW_KEEPRATIO);
	StructOfArray2di warpedPixels(4);
	StructOfArray2di  panelCorners = GetRectangleCornersForAugmentation(&optimizationParameters, templ.cols, templ.rows);
#endif

	Matx33f homographyMatrix;
	high_resolution_clock::time_point startTime, endTime;
	float elapsedTime(0.), tim;
	int nFrames = 0;
	while(nFrames < 50)
	{
		AcquireVGAGrayscaleImage(capture, image, imageRGB);
		if (image.empty())
			break;

		startTime = high_resolution_clock::now();

		results = optimization.GradientModuleCalibration(pixelsOnTemplateWarped, templ, image, parametersInitialGuess, optimizationParameters);//parameters are updated with the current guess!
		Homography::CompositionalParametersUpdateWithCheck(parameters, parametersInitialGuess);
		homographyMatrix = Homography::GetUnscaledMatrix(parameters);
		warpPerspective(firstFrame, templ, homographyMatrix, templ.size(), INTER_LINEAR, BORDER_CONSTANT);
		WarpGridOfControlPoints(pixelsOnTemplate, pixelsOnTemplateWarped, parameters, templ.cols, templ.rows);
		fill(parametersInitialGuess.begin(), parametersInitialGuess.end(), 0.);
		nFrames++;

		endTime = high_resolution_clock::now();
		tim = duration_cast<duration<double> >(endTime - startTime).count();
		elapsedTime +=tim;

#ifdef B_PLOT
		for (int iPoint = 0; iPoint< panelCorners.size(); ++iPoint)
			Homography::ComputeWarpedPixels(panelCorners[iPoint].x,panelCorners[iPoint].y, warpedPixels.x[iPoint], warpedPixels.y[iPoint], parameters);
		AugmentFrameWithQuadrilater(warpedPixels, imageRGB);
		Mat frameToDisplay = imageRGB;
		imshow(window_name, frameToDisplay);
		waitKey(50);
#endif
	}
	cout << "average elapsedTime = "<< elapsedTime/nFrames<<endl;
}



void RunAllTests()
{
	try
	{
		cout<<"ReadDataFromXMLTest()...";
		ReadDataFromXMLTest();
		cout<<".................... ok."<<endl;

		cout<<"ComputeHomographyTest...";
		ComputeHomographyTest();
		cout<<".................... ok."<<endl;

		cout<<"CompositionHomographyTest()...";
		CompositionHomographyTest();
		cout<<".................... ok."<<endl;

		cout<<"HomographyJacobianTest()...";
		HomographyJacobianTest2();
		cout<<".................... ok."<<endl;

		cout<<"InverseHomographyTest()...";
		InverseHomographyTest();
		cout<<".................... ok."<<endl;

		cout<<"WarpedIntensitiesTest()...";
		WarpedIntensitiesTest();
		cout<<".................... ok."<<endl;

		cout<<"SDImagesTest()...";
		SDImagesTest();
		cout<<".................... ok."<<endl;

		cout<<"LucaskanadeSSDTest()...";
		LucaskanadeSSDTest();
		cout<<".................... ok."<<endl;

		cout<<"LucaskanadeDescriptorFieldsTest()...";
		LucaskanadeDescriptorFieldsTest();
		cout<<".................... ok."<<endl;

		cout<<"LucaskanadeVideoSSDSpeedTest()...";
		LucaskanadeVideoSSDSpeedTest();
		cout<<".................... ok."<<endl;

		cout<<"LucaskanadeVideoDesciptorFieldsSpeedTest()...";
		LucaskanadeVideoDesciptorFieldsSpeedTest();
		cout<<".................... ok."<<endl;

		cout<<"LucaskanadeVideoGradientMagnitudeSpeedTest...";
		LucaskanadeVideoGradientMagnitudeSpeedTest();
		cout<<".................... ok."<<endl;

	}
	catch(char const* message)
	{
		cerr<<message<<endl;
		exit(0);
	}
}

int main(int argc, char **argv)
{
	RunAllTests();
	exit(0);
}

