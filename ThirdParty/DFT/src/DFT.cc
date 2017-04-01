#include "mtf/ThirdParty/DFT/DFT.h"
#include "mtf/Utilities/miscUtils.h"

using namespace cv;

namespace dft{
	DFTParams::DFTParams(
		float _resTol,
		float _pTol,
		int _maxIter,
		int _maxIterSingleLevel,
		vector<float> _pyramidSmoothingVariance,
		float _presmoothingVariance,
		int _nControlPointsOnEdge,
		bool _bAdaptativeChoiceOfPoints,
		bool _bNormalizeDescriptors,
		OptimizationType _optimizationType) :
		resTol(_resTol),
		pTol(_pTol),
		maxIter(_maxIter),
		maxIterSingleLevel(_maxIterSingleLevel),
		pyramidSmoothingVariance(_pyramidSmoothingVariance),
		presmoothingVariance(_presmoothingVariance),
		nControlPointsOnEdge(_nControlPointsOnEdge),
		bAdaptativeChoiceOfPoints(_bAdaptativeChoiceOfPoints),
		bNormalizeDescriptors(_bNormalizeDescriptors),
		optimizationType(_optimizationType){}

	DFTParams::DFTParams(const DFTParams *params) :
		resTol(DFT_RES_TO_L),
		pTol(DFT_P_TO_L),
		maxIter(DFT_MAX_ITER),
		maxIterSingleLevel(DFT_MAX_ITER_SINGLE_LEVEL),
		presmoothingVariance(DFT_PRESMOOTHING_VARIANCE),
		nControlPointsOnEdge(DFT_N_CONTROL_POINTS_ON_EDGE),
		bAdaptativeChoiceOfPoints(DFT_B_ADAPTATIVE_CHOICE_OF_POINTS),
		bNormalizeDescriptors(DFT_B_NORMALIZE_DESCRIPTORS),
		optimizationType(DFT_OPTIMIZATION_TYPE){
		pyramidSmoothingVariance.push_back(DFT_PYRAMID_SMOOTHING_VARIANCE);
		if(params){
			resTol = params->resTol;
			pTol = params->pTol;
			maxIter = params->maxIter;
			maxIterSingleLevel = params->maxIterSingleLevel;
			pyramidSmoothingVariance = params->pyramidSmoothingVariance;
			presmoothingVariance = params->presmoothingVariance;
			nControlPointsOnEdge = params->nControlPointsOnEdge;
			bAdaptativeChoiceOfPoints = params->bAdaptativeChoiceOfPoints;
			bNormalizeDescriptors = params->bNormalizeDescriptors;
			optimizationType = params->optimizationType;
		}
	}
	OptimizationParameters DFTParams::clone(){
		OptimizationParameters opt_params;
		opt_params.resTol = resTol;
		opt_params.pTol = pTol;
		opt_params.maxIter = maxIter;
		opt_params.maxIterSingleLevel = maxIterSingleLevel;
		opt_params.pyramidSmoothingVariance = pyramidSmoothingVariance;
		opt_params.presmoothingVariance = presmoothingVariance;
		opt_params.nControlPointsOnEdge = nControlPointsOnEdge;
		opt_params.bAdaptativeChoiceOfPoints = bAdaptativeChoiceOfPoints;
		opt_params.bNormalizeDescriptors = bNormalizeDescriptors;
		opt_params.borderThicknessHorizontal = 0;
		opt_params.borderThicknessVertical = 0;
		return opt_params;
	}
	DFT::DFT() : 
		panelCorners(4), 
		warpedPixels(4),
		parameters(8, 0),
		parametersInitialGuess(8, 0){
		name = "dft";
		opt_params = params.clone();
	}

	DFT::DFT(const ParamType *dft_params) :
		params(dft_params),
		panelCorners(4),
		warpedPixels(4),
		parameters(8, 0),
		parametersInitialGuess(8, 0){
		name = "dft";
		opt_params = params.clone();
		printf("Using Descriptor Fields tracker with:\n");
		printf("optimizationType: %d\n", params.optimizationType);
		cout << opt_params << endl;
	}
	void DFT::initialize(const cv::Mat& corners){
		//double pos_x = corners.at<double>(0, 0);
		//double pos_y = corners.at<double>(1, 0);
		//double size_x = ((corners.at<double>(0, 1) - corners.at<double>(0, 0)) +
		//	(corners.at<double>(0, 2) - corners.at<double>(0, 3))) / 2;
		//double size_y = ((corners.at<double>(1, 3) - corners.at<double>(1, 0)) +
		//	(corners.at<double>(1, 2) - corners.at<double>(1, 1))) / 2;

		mtf::Rectd best_fit_rect = mtf::utils::getBestFitRectangle<double>(corners,
			curr_img.cols, curr_img.rows);
		printf("best_fit_rect: x: %f y:%f width: %f height: %f\n",
			best_fit_rect.x, best_fit_rect.y, best_fit_rect.width, best_fit_rect.height);

		templ = curr_img.clone();
		ConvertImageToFloat(templ);

		firstFrame = templ.clone();

		if(params.bAdaptativeChoiceOfPoints)
			pixelsOnTemplate = CreateAnisotropicGridOfControlPoints(templ, params.nControlPointsOnEdge,
			best_fit_rect.x, best_fit_rect.y, best_fit_rect.width, best_fit_rect.height);
		else
			pixelsOnTemplate = CreateGridOfControlPoints(templ, params.nControlPointsOnEdge,
			best_fit_rect.x, best_fit_rect.y, best_fit_rect.width, best_fit_rect.height);

		pixelsOnTemplateWarped = pixelsOnTemplate;
		cv_corners_mat.create(2, 4, CV_64FC1);
		cv_corners_mat.at<double>(0, 0) = cv_corners_mat.at<double>(0, 3) = best_fit_rect.x;
		cv_corners_mat.at<double>(1, 0) = cv_corners_mat.at<double>(1, 1) = best_fit_rect.y;
		cv_corners_mat.at<double>(0, 1) = cv_corners_mat.at<double>(0, 2) = best_fit_rect.x + best_fit_rect.width;
		cv_corners_mat.at<double>(1, 2) = cv_corners_mat.at<double>(1, 3) = best_fit_rect.y + best_fit_rect.height;

		for(int iPoint = 0; iPoint < 4; ++iPoint){
			panelCorners.x[iPoint] = cv_corners_mat.at<double>(0, iPoint);
			panelCorners.y[iPoint] = cv_corners_mat.at<double>(1, iPoint);
		}
	}
	void DFT::update(){
		curr_img_scaled = curr_img.clone();
		ConvertImageToFloat(curr_img_scaled);

		switch(params.optimizationType)
		{
		case descriptorFields:
			results = optimization.DescriptorFieldsCalibration(pixelsOnTemplateWarped,
				templ, curr_img_scaled, parametersInitialGuess,
				opt_params); //parameters are updated with the current guess!
			break;
		case gradientModule:
			results = optimization.GradientModuleCalibration(pixelsOnTemplateWarped,
				templ, curr_img_scaled, parametersInitialGuess,
				opt_params); //parameters are updated with the current guess!
			break;
		case intensity:
			results = optimization.SSDCalibration(pixelsOnTemplateWarped,
				templ, curr_img_scaled, parametersInitialGuess, opt_params);//parameters are updated with the current guess!
			break;
		default:
			break;
		}

		Homography::CompositionalParametersUpdateWithCheck(parameters, parametersInitialGuess);

		for(int iPoint = 0; iPoint < panelCorners.size(); ++iPoint)
			Homography::ComputeWarpedPixels(panelCorners[iPoint].x,
			panelCorners[iPoint].y, warpedPixels.x[iPoint],
			warpedPixels.y[iPoint], parameters);
		homographyMatrix = Homography::GetUnscaledMatrix(parameters);
		warpPerspective(firstFrame, templ, homographyMatrix, templ.size(), INTER_LINEAR, BORDER_CONSTANT);
		WarpGridOfControlPoints(pixelsOnTemplate, pixelsOnTemplateWarped, parameters, templ.cols, templ.rows);
		fill(parametersInitialGuess.begin(), parametersInitialGuess.end(), 0.);
		updateCVCorners();
	}
	void DFT::updateCVCorners(){
		cv_corners_mat.at<double>(0, 0) = warpedPixels[0].x;
		cv_corners_mat.at<double>(1, 0) = warpedPixels[0].y;
		cv_corners_mat.at<double>(0, 1) = warpedPixels[1].x;
		cv_corners_mat.at<double>(1, 1) = warpedPixels[1].y;
		cv_corners_mat.at<double>(0, 2) = warpedPixels[2].x;
		cv_corners_mat.at<double>(1, 2) = warpedPixels[2].y;
		cv_corners_mat.at<double>(0, 3) = warpedPixels[3].x;
		cv_corners_mat.at<double>(1, 3) = warpedPixels[3].y;
	}
}

