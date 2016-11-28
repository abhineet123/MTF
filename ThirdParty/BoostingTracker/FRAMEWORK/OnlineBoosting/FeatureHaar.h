#pragma once

#include "EstimatedGaussDistribution.h"
#include "ImageRepresentation.h"
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

class FeatureHaar
{

public:

	FeatureHaar(Size patchSize);
	virtual ~FeatureHaar();

	void getInitialDistribution(EstimatedGaussDistribution *distribution);

	bool eval(ImageRepresentation* image, Rect ROI, float* result); 
	
	float getResponse(){return m_response;};

	int getNumAreas(){return m_numAreas;};
	int* getWeights(){return m_weights;};
	Rect* getAreas(){return m_areas;};
	
private:

	char m_type[20];
	int m_numAreas;
	int* m_weights;
	float m_initMean;
	float m_initSigma;

	void generateRandomFeature(Size imageSize);
	Rect* m_areas;     // areas within the patch over which to compute the feature
	Size m_initSize;   // size of the patch used during training
	Size m_curSize;    // size of the patches currently under investigation
	float m_scaleFactorHeight;  // scaling factor in vertical direction
	float m_scaleFactorWidth;   // scaling factor in horizontal direction
	Rect* m_scaleAreas;// areas after scaling
	float* m_scaleWeights; // weights after scaling
	float m_response;

};
