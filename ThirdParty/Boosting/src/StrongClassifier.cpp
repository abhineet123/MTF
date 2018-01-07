#include "StrongClassifier.h"


StrongClassifier::StrongClassifier(int numBaseClassifier, 
								   int numWeakClassifier, 
								   Size patchSize, 
								   bool useFeatureExchange, 
								   int iterationInit)
{
	this->numBaseClassifier = numBaseClassifier;
	this->numAllWeakClassifier = numWeakClassifier+iterationInit;

	alpha = new float[numBaseClassifier];
	memset(alpha, 0x00, sizeof(float)*numBaseClassifier);

	this->patchSize = patchSize;
	this->useFeatureExchange = useFeatureExchange;
}

StrongClassifier::~StrongClassifier()
{
	for (int curBaseClassifier = 0; curBaseClassifier< numBaseClassifier; curBaseClassifier++)
		delete baseClassifier[curBaseClassifier];
	delete[] baseClassifier;
	delete[] alpha;
}

float StrongClassifier::getFeatureValue(ImageRepresentation *image, Rect ROI, int baseClassifierIdx)
{
	return baseClassifier[baseClassifierIdx]->getValue(image, ROI);
}


float StrongClassifier::eval(ImageRepresentation *image, Rect ROI)
{
	float value = 0.0f;
	int curBaseClassifier=0;

	for (curBaseClassifier = 0; curBaseClassifier<numBaseClassifier; curBaseClassifier++)
		value+= baseClassifier[curBaseClassifier]->eval(image, ROI)*alpha[curBaseClassifier];

	return value;
}

bool StrongClassifier::update(ImageRepresentation *image, Rect ROI, int target, float importance) 
{
	assert (true);
	return false;
}

bool StrongClassifier::updateSemi(ImageRepresentation *image, Rect ROI, float priorConfidence)
{
	assert (true);
	return false;
}


float StrongClassifier::getSumAlpha (int toBaseClassifier)
{
	float sumAlpha = 0;
	if (toBaseClassifier == -1)
		toBaseClassifier = numBaseClassifier;

	for (int curBaseClassifier=0; curBaseClassifier < toBaseClassifier; curBaseClassifier++)
		sumAlpha+= alpha[curBaseClassifier];

	return sumAlpha;
}

float StrongClassifier::getImportance(ImageRepresentation *image, Rect ROI, int target, int numBaseClassifier)
{
	if (numBaseClassifier == -1) 
		numBaseClassifier =  this->numBaseClassifier;

	float importance = 1.0f;

	for (int curBaseClassifier = 0; curBaseClassifier<numBaseClassifier; curBaseClassifier++)
	{
		bool error = (baseClassifier[curBaseClassifier]->eval (image, ROI)!= target);

		if(error)
			importance/=(2*baseClassifier[curBaseClassifier]->getError());
		else
			importance/=(2*(1-baseClassifier[curBaseClassifier]->getError()));
	}

	return importance/numBaseClassifier;
}


void StrongClassifier::resetWeightDistribution()
{
	for (int curBaseClassifier = 0; curBaseClassifier<numBaseClassifier; curBaseClassifier++)
	{
		for(int curWeakClassifier = 0; curWeakClassifier < baseClassifier[curBaseClassifier]->getNumWeakClassifier(); curWeakClassifier++)
		{
			baseClassifier[curBaseClassifier]->setWCorrect(curWeakClassifier, 1.0);
			baseClassifier[curBaseClassifier]->setWWrong(curWeakClassifier, 1.0);
		}
	}
}