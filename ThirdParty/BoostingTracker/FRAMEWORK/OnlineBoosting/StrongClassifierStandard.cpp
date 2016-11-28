#include "StrongClassifierStandard.h"

StrongClassifierStandard::StrongClassifierStandard(int numBaseClassifier, 
												   int numWeakClassifier, 
												   Size patchSize, 
												   bool useFeatureExchange,
												   int iterationInit) 
												   : StrongClassifier(  numBaseClassifier, 
												   numWeakClassifier, 
												   patchSize,
												   useFeatureExchange,
												   iterationInit)
{
	// init Base Classifier
	baseClassifier = new BaseClassifier*[numBaseClassifier];

	for (int curBaseClassifier = 0; curBaseClassifier< numBaseClassifier; curBaseClassifier++)
	{
		baseClassifier[curBaseClassifier] = new BaseClassifier(numWeakClassifier, iterationInit, patchSize);
	}

	m_errorMask = new bool[numAllWeakClassifier];
	m_errors = new float[numAllWeakClassifier];
}

StrongClassifierStandard::~StrongClassifierStandard()
{
	delete[] m_errorMask;
	delete[] m_errors;
}


bool StrongClassifierStandard::update(ImageRepresentation *image, Rect ROI, int target, float importance)
{
	int curBaseClassifier;
	for (curBaseClassifier = 0; curBaseClassifier<numBaseClassifier; curBaseClassifier++)
	{
		memset(m_errorMask, 0x00, numAllWeakClassifier*sizeof(bool));
		memset(m_errors, 0x00, numAllWeakClassifier*sizeof(float));

		int selectedClassifier;
		baseClassifier[curBaseClassifier]->trainClassifier(image, ROI, target, importance, m_errorMask);
		selectedClassifier = baseClassifier[curBaseClassifier]->selectBestClassifier (m_errorMask, importance, m_errors);

    	if (m_errors[selectedClassifier] >= 0.5)
			alpha[curBaseClassifier] = 0;
		else
			alpha[curBaseClassifier] = logf((1.0f-m_errors[selectedClassifier])/m_errors[selectedClassifier]);

		if(m_errorMask[selectedClassifier])
			importance *= (float)sqrt((1.0f-m_errors[selectedClassifier])/m_errors[selectedClassifier]);
		else
			importance *= (float)sqrt(m_errors[selectedClassifier]/(1.0f-m_errors[selectedClassifier]));			
	
		//weight limitation
		//if (importance > 100) importance = 100;

		if (useFeatureExchange)
			baseClassifier[curBaseClassifier]->replaceWeakestClassifier (m_errors, patchSize);
	
	}

	return true;
}

