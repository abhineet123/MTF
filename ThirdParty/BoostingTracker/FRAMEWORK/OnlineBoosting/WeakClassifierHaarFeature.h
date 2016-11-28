#pragma once

#include "WeakClassifier.h"
#include "FeatureHaar.h"
#include "ClassifierThreshold.h"
#include "EstimatedGaussDistribution.h"

class WeakClassifierHaarFeature : public WeakClassifier
{

public:

	WeakClassifierHaarFeature(Size patchSize);
	virtual ~WeakClassifierHaarFeature();

	bool update(ImageRepresentation* image, Rect ROI, int target); 

	int eval(ImageRepresentation* image, Rect ROI); 
	
	float getValue(ImageRepresentation* image, Rect ROI);
	
	int getType(){return 1;};

	EstimatedGaussDistribution* getPosDistribution();
	EstimatedGaussDistribution* getNegDistribution();

	void resetPosDist();
	void initPosDist();

private:

	FeatureHaar* m_feature;
	ClassifierThreshold* m_classifier;

	void generateRandomClassifier();

};

