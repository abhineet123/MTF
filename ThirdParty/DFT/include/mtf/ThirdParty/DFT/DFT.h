#ifndef MTF_DFT_H
#define MTF_DFT_H

#include "opencv2/core/core.hpp"
#include "mtf/TrackerBase.h"
#include "mtf/ThirdParty/DFT/HomographyEstimation.hpp"

#define DFT_RES_TO_L  1e-10
#define DFT_P_TO_L   5e-5
#define DFT_MAX_ITER   50
#define	DFT_MAX_ITER_SINGLE_LEVEL   10
#define DFT_PYRAMID_SMOOTHING_VARIANCE 7
#define DFT_PRESMOOTHING_VARIANCE   1
#define DFT_N_CONTROL_POINTS_ON_EDGE   25
#define DFT_B_ADAPTATIVE_CHOICE_OF_POINTS   0
#define DFT_B_NORMALIZE_DESCRIPTORS   0
#define DFT_OPTIMIZATION_TYPE descriptorFields

namespace dft{
	struct DFTParams{
		float resTol, pTol;
		int maxIter;
		int maxIterSingleLevel;
		vector<float> pyramidSmoothingVariance;
		float presmoothingVariance;
		int nControlPointsOnEdge;
		bool bAdaptativeChoiceOfPoints;
		bool bNormalizeDescriptors;
		OptimizationType optimizationType;

		DFTParams(
			float resTol, 
			float pTol,
			int maxIter,
			int maxIterSingleLevel,
			vector<float> pyramidSmoothingVariance,
			float presmoothingVariance,
			int nControlPointsOnEdge,
			bool bAdaptativeChoiceOfPoints,
			bool bNormalizeDescriptors,
			OptimizationType optimizationType);
		DFTParams(const DFTParams *params = nullptr);
		OptimizationParameters clone();
	};

	class DFT : public mtf::TrackerBase{
	public:
		typedef DFTParams ParamType;

		DFT();
		DFT(const ParamType *dft_params = nullptr);
		void setImage(const cv::Mat &img) override{ curr_img = img; }
		int inputType() const  override{ return CV_32FC1; }
		void initialize(const cv::Mat& corners) override;
		void update() override;
		void updateCVCorners();

	private:
		ParamType params;
		OptimizationParameters opt_params;

		LucasKanade optimization;
		AlignmentResults results;

		StructOfArray2di pixelsOnTemplate, pixelsOnTemplateWarped;
		StructOfArray2di  panelCorners;
		StructOfArray2di warpedPixels;
		cv::Matx33f homographyMatrix;
		vector<float> parameters;
		vector<float> parametersInitialGuess;

		cv::Mat curr_img, curr_img_scaled;
		cv::Mat templ, firstFrame;
	};
}

#endif 
