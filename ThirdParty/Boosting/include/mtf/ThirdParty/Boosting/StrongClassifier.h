#pragma once

#include "ImageRepresentation.h"
#include "BaseClassifier.h"
#include "EstimatedGaussDistribution.h"
#include <stdio.h>
#include "windefs.h"

class StrongClassifier  
{
public:

	StrongClassifier( int numBaseClassifier, 
		              int numWeakClassifier, 
		              Size patchSize, 
					  bool useFeatureExchange = false, 
					  int iterationInit = 0);

	~StrongClassifier();

	virtual float eval(ImageRepresentation *image, Rect ROI); 

	virtual bool update(ImageRepresentation *image, Rect ROI, int target, float importance = 1.0f); 
	virtual bool updateSemi(ImageRepresentation *image, Rect ROI, float priorConfidence);

	Size getPatchSize(){return patchSize;};
	int getNumBaseClassifier(){return numBaseClassifier;};
	int getIdxOfSelectedClassifierOfBaseClassifier (int baseClassifierIdx=0){return baseClassifier[baseClassifierIdx]->getIdxOfSelectedClassifier();};
	virtual float getSumAlpha(int toBaseClassifier = -1);
	float getAlpha(int idx){return alpha[idx];};

	float getFeatureValue(ImageRepresentation *image, Rect ROI, int baseClassifierIdx);
	float getImportance(ImageRepresentation *image, Rect ROI, int traget, int numBaseClassifiers = -1);
	
	WeakClassifier** getReferenceWeakClassifier(){return baseClassifier[0]->getReferenceWeakClassifier();};

	void resetWeightDistribution();

protected:

	int numBaseClassifier;
	int numAllWeakClassifier;

	BaseClassifier** baseClassifier;
	float* alpha;
	Size patchSize;
	
	bool useFeatureExchange;

};
