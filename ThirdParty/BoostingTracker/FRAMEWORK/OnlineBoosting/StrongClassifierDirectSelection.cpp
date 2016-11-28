#include "StrongClassifierDirectSelection.h"

StrongClassifierDirectSelection::StrongClassifierDirectSelection(int numBaseClassifier, int numWeakClassifier, 
																 Size patchSize, bool useFeatureExchange, int iterationInit) 
																 : StrongClassifier(numBaseClassifier, numWeakClassifier, patchSize, useFeatureExchange, iterationInit)
{
	this->useFeatureExchange = useFeatureExchange;
	baseClassifier = new BaseClassifier*[numBaseClassifier];
	baseClassifier[0] = new BaseClassifier(numWeakClassifier, iterationInit, patchSize);

	for (int curBaseClassifier = 1; curBaseClassifier< numBaseClassifier; curBaseClassifier++)
		baseClassifier[curBaseClassifier] = new BaseClassifier(numWeakClassifier, iterationInit, baseClassifier[0]->getReferenceWeakClassifier());

	m_errorMask = new bool[numAllWeakClassifier];
	m_errors = new float[numAllWeakClassifier];
	m_sumErrors = new float[numAllWeakClassifier];
}

StrongClassifierDirectSelection::~StrongClassifierDirectSelection()
{
	delete[] m_errorMask;
	delete[] m_errors;
	delete[] m_sumErrors;
}


bool StrongClassifierDirectSelection::update(ImageRepresentation *image, Rect ROI, int target, float importance)
{
	memset(m_errorMask, 0, numAllWeakClassifier*sizeof(bool));
	memset(m_errors, 0, numAllWeakClassifier*sizeof(float));
	memset(m_sumErrors, 0, numAllWeakClassifier*sizeof(float));

	baseClassifier[0]->trainClassifier(image, ROI, target, importance, m_errorMask);
	for (int curBaseClassifier = 0; curBaseClassifier < numBaseClassifier; curBaseClassifier++)
	{
		int selectedClassifier = baseClassifier[curBaseClassifier]->selectBestClassifier(m_errorMask, importance, m_errors);

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

		//sum up errors
		for (int curWeakClassifier = 0; curWeakClassifier<numAllWeakClassifier; curWeakClassifier++)
		{
			if (m_errors[curWeakClassifier]!=FLOAT_MAX && m_sumErrors[curWeakClassifier]>=0)
				m_sumErrors[curWeakClassifier]+= m_errors[curWeakClassifier];
		}

		//mark feature as used
		m_sumErrors[selectedClassifier] = -1;
		m_errors[selectedClassifier] = FLOAT_MAX;
	}

	if (useFeatureExchange)
	{
		int replacedClassifier = baseClassifier[0]->replaceWeakestClassifier (m_sumErrors, patchSize);
		if (replacedClassifier > 0)
			for (int curBaseClassifier = 1; curBaseClassifier < numBaseClassifier; curBaseClassifier++)
				baseClassifier[curBaseClassifier]->replaceClassifierStatistic(baseClassifier[0]->getIdxOfNewWeakClassifier(), replacedClassifier);
	}

	return true;
}

