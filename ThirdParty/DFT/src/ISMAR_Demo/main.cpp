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



#if defined(_OPENMP)
#include <omp.h>
#endif

#include <Eigen/Core>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <list>
#include <vector>
#include<stdio.h>
#include <chrono>

#include "mtf/ThirdParty/DFT/HomographyEstimation.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono;


#define B_VERBOSE_TRACKING 0
#define CAM_HEIGHT (480)
#define CAM_WIDTH (640)

void CalibrateVideoStream(VideoCapture capture);

int main(int ac, char ** av)
{
	if (ac != 2)
	{
		cerr <<" Usage : ./homographyDemo <videoName>"<<endl;
		cerr <<" <videoName> can be the video file name or a camera ID (0, 1, etc.) for live demo. "<<endl;
		return 1;
	}

	cout<< " ***********************   Starting demo ... ************************************"<<endl;
	cout<< "press 1 : intensities "<<endl;
	cout<< "press 2 : gradient magnitude"<<endl;
	cout<< "press 3 : descriptor fields"<<endl;
	cout<< "press g : resets tracking"<<endl;
	cout<< "press space : pause tracking"<<endl;
	cout<< " *********************************************************************************"<<endl;

	std::string arg = av[1];
	VideoCapture capture(arg);
	if (!capture.isOpened())
		capture.open(atoi(arg.c_str()));
	if (!capture.isOpened()) {
		cerr << "Failed to open a video device or video file!\n" << endl;
		return 1;
	}
	CalibrateVideoStream(capture);

	return 0;
}



void CalibrateVideoStream(VideoCapture capture)
{

	string window_name = "ISMAR Demo 2014";

	namedWindow(window_name, CV_WINDOW_KEEPRATIO);

	OptimizationType optimizationType = descriptorFields;
	string method = string("Using Our DESCRIPTOR FIELDS!");

	bool bResetOptimization(false);
	OptimizationParameters optimizationParametersDescrFields = ReadOptimizationParametersFromXML("parameter/parametersDescriptorFieldsOptimization.yml");
	OptimizationParameters optimizationParametersSSD = ReadOptimizationParametersFromXML("parameter/parametersSSDOptimization.yml");
	OptimizationParameters optimizationParametersGradientModule = ReadOptimizationParametersFromXML("parameter/parametersGradientMagnitudeOptimization.yml");

	OptimizationParameters* optimizationParameters(&optimizationParametersDescrFields);

	cout<<endl<<endl<<*optimizationParameters<<endl;

	LucasKanade optimization;
	AlignmentResults results;
	
	Mat templ, image, firstFrame, currentFrameRGB;


//--------------------init traking of libalbe start here
	AcquireVGAGrayscaleImage(capture, templ);


	firstFrame = templ.clone();
	StructOfArray2di pixelsOnTemplate;
	if(optimizationParameters->bAdaptativeChoiceOfPoints)
		pixelsOnTemplate = CreateAnisotropicGridOfControlPoints(templ, optimizationParameters->nControlPointsOnEdge, 
		optimizationParameters->borderThicknessHorizontal, optimizationParameters->borderThicknessVertical);
	else
		pixelsOnTemplate = CreateGridOfControlPoints(templ, optimizationParameters->nControlPointsOnEdge, 
		optimizationParameters->borderThicknessHorizontal, optimizationParameters->borderThicknessVertical);

	StructOfArray2di pixelsOnTemplateWarped = pixelsOnTemplate;

	vector<float> parameters(8,0);
	vector<float> parametersInitialGuess(8,0);

	StructOfArray2di warpedPixels(4);
	StructOfArray2di  panelCorners = GetRectangleCornersForAugmentation(optimizationParameters, templ.cols, templ.rows);
//--------------------init traking of libalbe stop here

	Matx33f homographyMatrix;
	high_resolution_clock::time_point startTime, endTime;

	float elapsedTime(0.), tim;
	bool bKeepTracking(true);

	int nFrames = 0;

	while(bKeepTracking)
	{
		AcquireVGAGrayscaleImage(capture, image, currentFrameRGB);

		if (image.empty())
			break;

		startTime = high_resolution_clock::now();

		switch (optimizationType)
		{
		case descriptorFields:
			results = optimization.DescriptorFieldsCalibration(pixelsOnTemplateWarped,
				templ, image, parametersInitialGuess,
				*optimizationParameters); //parameters are updated with the current guess!
			break;
		case gradientModule:
			results = optimization.GradientModuleCalibration(pixelsOnTemplateWarped,
				templ, image, parametersInitialGuess,
				*optimizationParameters); //parameters are updated with the current guess!
			break;
		case intensity:
			results = optimization.SSDCalibration(pixelsOnTemplateWarped,
				templ, image, parametersInitialGuess,
				*optimizationParameters);//parameters are updated with the current guess!
			break;
		default:
			break;
		}

		Homography::CompositionalParametersUpdateWithCheck(parameters, parametersInitialGuess);

		for (int iPoint = 0; iPoint< panelCorners.size(); ++iPoint)
			Homography::ComputeWarpedPixels(panelCorners[iPoint].x,panelCorners[iPoint].y, warpedPixels.x[iPoint], warpedPixels.y[iPoint], parameters);

		Mat ctrlPtImg;
		if (B_VERBOSE_TRACKING)
		{
			WriteResultsOnImage(image, results, pixelsOnTemplateWarped.size(), optimizationType);
			// Draw the control points
			ctrlPtImg = Mat(templ.rows, templ.cols,CV_8UC3,Scalar(0,0,0));
			for (int idxPt = 0; idxPt < pixelsOnTemplateWarped.size(); ++idxPt)
				circle(ctrlPtImg, pixelsOnTemplateWarped[idxPt], 3, Scalar(255,255,255) );
		}

		AugmentFrameWithQuadrilater(warpedPixels, currentFrameRGB);

		Mat frameToDisplay;
		if (ctrlPtImg.rows > 0)
		{
			hconcat(currentFrameRGB, ctrlPtImg, frameToDisplay);
		}else
			frameToDisplay = currentFrameRGB;//quick copy

		imshow(window_name, frameToDisplay);

		endTime = high_resolution_clock::now();
		tim = duration_cast<duration<double> >(endTime - startTime).count();
		elapsedTime +=tim;
		nFrames++;

		char key = (char)waitKey(1);
		switch (key)
		{
		case 27:
			bKeepTracking = false;
			break;
		case ' ':
			key = (char)waitKey(0);
			break;
		case '1':
			optimizationType = intensity;
			optimizationParameters = &optimizationParametersSSD;
			method = string("Using intensity!");
			cout<<method<<endl;
			bResetOptimization = true;
			break;
		case '2':
			optimizationType = gradientModule;
			optimizationParameters = &optimizationParametersGradientModule;
			method = string("Using gradient magnitude!");
			cout<< method <<endl;
			bResetOptimization = true;
			break;
		case '3':
			optimizationType = descriptorFields;
			optimizationParameters = &optimizationParametersDescrFields;
			method = string("Using Our DESCRIPTOR FIELDS!");
			cout<< method <<endl;
			bResetOptimization = true;
			break;
		case 'g':
			bResetOptimization = true;
			break;
		default:
			;
		}

		if (bResetOptimization)
		{
			bResetOptimization = false;
			cout<<"reset template!"<<endl;
			fill(parameters.begin(), parameters.end(), 0.);
			fill(parametersInitialGuess.begin(), parametersInitialGuess.end(), 0.);

			AcquireVGAGrayscaleImage(capture, templ);

			firstFrame = templ.clone();
			if(optimizationParameters->bAdaptativeChoiceOfPoints)
				pixelsOnTemplate = CreateAnisotropicGridOfControlPoints(firstFrame, optimizationParameters->nControlPointsOnEdge, optimizationParameters->borderThicknessHorizontal, optimizationParameters->borderThicknessVertical);
			else
				pixelsOnTemplate = CreateGridOfControlPoints(firstFrame, optimizationParameters->nControlPointsOnEdge, optimizationParameters->borderThicknessHorizontal, optimizationParameters->borderThicknessVertical);

			pixelsOnTemplateWarped = pixelsOnTemplate;
		}

		homographyMatrix = Homography::GetUnscaledMatrix(parameters);
		warpPerspective(firstFrame, templ, homographyMatrix, templ.size(), INTER_LINEAR, BORDER_CONSTANT);
		WarpGridOfControlPoints(pixelsOnTemplate, pixelsOnTemplateWarped, parameters, templ.cols, templ.rows);
		fill(parametersInitialGuess.begin(), parametersInitialGuess.end(), 0.);
	}

	cout << "Calibrated "<<nFrames<< " frames"<<endl;
	cout << "average elapsedTime = "<< elapsedTime/nFrames<<endl;
}
