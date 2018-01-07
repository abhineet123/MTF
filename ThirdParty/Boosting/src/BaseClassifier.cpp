#include "BaseClassifier.h"
#include <iostream>

BaseClassifier::BaseClassifier(int numWeakClassifier, int iterationInit, Size patchSize)
{
	this->m_numWeakClassifier = numWeakClassifier;
	this->m_iterationInit = iterationInit;

	weakClassifier = new WeakClassifier*[numWeakClassifier+iterationInit];
	m_idxOfNewWeakClassifier = numWeakClassifier;

	generateRandomClassifier(patchSize);

	m_referenceWeakClassifier = false;
	m_selectedClassifier = 0;

	m_wCorrect = new float[numWeakClassifier+iterationInit];
	memset (m_wCorrect, 0, sizeof(float)*numWeakClassifier+iterationInit);

	m_wWrong = new float[numWeakClassifier+iterationInit];
	memset (m_wWrong, 0, sizeof(float)*numWeakClassifier+iterationInit);

	for (int curWeakClassifier = 0; curWeakClassifier < numWeakClassifier+iterationInit; curWeakClassifier++)
		m_wWrong[curWeakClassifier] = m_wCorrect[curWeakClassifier] = 1;
}

BaseClassifier::BaseClassifier(int numWeakClassifier, int iterationInit, WeakClassifier** weakClassifier)
{
	this->m_numWeakClassifier = numWeakClassifier;
	this->m_iterationInit = iterationInit;
	this->weakClassifier = weakClassifier;
	m_referenceWeakClassifier = true;
	m_selectedClassifier = 0;
	m_idxOfNewWeakClassifier = numWeakClassifier;

	m_wCorrect = new float[numWeakClassifier+iterationInit];
	memset (m_wCorrect, 0, sizeof(float)*numWeakClassifier+iterationInit);
	m_wWrong = new float[numWeakClassifier+iterationInit];
	memset (m_wWrong, 0, sizeof(float)*numWeakClassifier+iterationInit);

	for (int curWeakClassifier = 0; curWeakClassifier < numWeakClassifier+iterationInit; curWeakClassifier++)
		m_wWrong[curWeakClassifier] = m_wCorrect[curWeakClassifier] = 1;
}

BaseClassifier::~BaseClassifier()
{
	if (!m_referenceWeakClassifier)
	{
		for (int curWeakClassifier = 0; curWeakClassifier< m_numWeakClassifier+m_iterationInit; curWeakClassifier++)
			delete weakClassifier[curWeakClassifier];

		delete[] weakClassifier;
	}
	if (m_wCorrect!=NULL) delete[] m_wCorrect;
	if (m_wWrong!=NULL) delete[] m_wWrong;

}

void BaseClassifier::generateRandomClassifier (Size patchSize)
{
	for (int curWeakClassifier = 0; curWeakClassifier< m_numWeakClassifier+m_iterationInit; curWeakClassifier++)
	{
		weakClassifier[curWeakClassifier] = new WeakClassifierHaarFeature(patchSize);
	}
}

int BaseClassifier::eval(ImageRepresentation *image, Rect ROI) 
{
	return weakClassifier[m_selectedClassifier]->eval(image, ROI);
}

float BaseClassifier::getValue (ImageRepresentation *image, Rect ROI, int weakClassifierIdx)
{
	if (weakClassifierIdx < 0 || weakClassifierIdx >= m_numWeakClassifier ) 
		return weakClassifier[m_selectedClassifier]->getValue(image, ROI);
	return weakClassifier[weakClassifierIdx]->getValue(image, ROI);
}

void BaseClassifier::trainClassifier(ImageRepresentation* image, Rect ROI, int target, float importance, bool* errorMask) 
{
	//get poisson value
	double A = 1;
	int K = 0;
	int K_max = 10;
	while (1)
	{
		double U_k = (double)rand()/RAND_MAX;
		A*=U_k;
		if (K > K_max || A<exp(-importance))
			break;
		K++;
	}

	for (int curK = 0; curK <= K; curK++)
		for (int curWeakClassifier = 0; curWeakClassifier < m_numWeakClassifier+m_iterationInit; curWeakClassifier++)
			errorMask[curWeakClassifier] = weakClassifier[curWeakClassifier]->update (image, ROI, target); 

}

void BaseClassifier::getErrorMask(ImageRepresentation* image, Rect ROI, int target, bool* errorMask) 
{
	for (int curWeakClassifier = 0; curWeakClassifier < m_numWeakClassifier+m_iterationInit; curWeakClassifier++)
		errorMask[curWeakClassifier] = (weakClassifier[curWeakClassifier]->eval(image, ROI) != target);
}

float BaseClassifier::getError(int curWeakClassifier)
{
	if (curWeakClassifier == -1)
		curWeakClassifier = m_selectedClassifier;
	return m_wWrong[curWeakClassifier]/(m_wWrong[curWeakClassifier]+m_wCorrect[curWeakClassifier]);
}

int BaseClassifier::selectBestClassifier(bool* errorMask, float importance, float* errors) 
{
	float minError = FLOAT_MAX;
	int tmp_selectedClassifier = m_selectedClassifier;

	for (int curWeakClassifier = 0; curWeakClassifier < m_numWeakClassifier+m_iterationInit; curWeakClassifier++)
	{
		if (errorMask[curWeakClassifier])
		{
			m_wWrong[curWeakClassifier] +=  importance;
		}
		else
		{
			m_wCorrect[curWeakClassifier] += importance;
		}

		if (errors[curWeakClassifier]==FLOAT_MAX)
			continue;

		errors[curWeakClassifier] = m_wWrong[curWeakClassifier]/
			(m_wWrong[curWeakClassifier]+m_wCorrect[curWeakClassifier]);

		/*if(errors[curWeakClassifier] < 0.001 || !(errors[curWeakClassifier]>0.0))
		{
			errors[curWeakClassifier] = 0.001;
		}

		if(errors[curWeakClassifier] >= 1.0)
			errors[curWeakClassifier] = 0.999;

		assert (errors[curWeakClassifier] > 0.0);
		assert (errors[curWeakClassifier] < 1.0);*/

		if (curWeakClassifier < m_numWeakClassifier)
		{		
			if (errors[curWeakClassifier] < minError)
			{
				minError = errors[curWeakClassifier];
				tmp_selectedClassifier = curWeakClassifier;
			}
		}
	}

	m_selectedClassifier = tmp_selectedClassifier;
	return m_selectedClassifier;
}

void BaseClassifier::getErrors(float* errors)
{
	for (int curWeakClassifier = 0; curWeakClassifier < m_numWeakClassifier+m_iterationInit; curWeakClassifier++)
	{	
		if (errors[curWeakClassifier]==FLOAT_MAX)
			continue;

		errors[curWeakClassifier] = m_wWrong[curWeakClassifier]/
			(m_wWrong[curWeakClassifier]+m_wCorrect[curWeakClassifier]);

		assert (errors[curWeakClassifier] > 0);
	}
}

int BaseClassifier::replaceWeakestClassifier(float* errors, Size patchSize)
{
	float maxError = 0.0f;
	int index = -1;

	//search the classifier with the largest error
	for (int curWeakClassifier = m_numWeakClassifier-1; curWeakClassifier >= 0; curWeakClassifier--)
	{
		if (errors[curWeakClassifier] > maxError)
		{
			maxError = errors[curWeakClassifier];
			index = curWeakClassifier;
		}
	}

	assert (index > -1);
	assert (index != m_selectedClassifier);

	//replace
	m_idxOfNewWeakClassifier++;
	if (m_idxOfNewWeakClassifier == m_numWeakClassifier+m_iterationInit) 
		m_idxOfNewWeakClassifier = m_numWeakClassifier;

	if (maxError > errors[m_idxOfNewWeakClassifier])
	{
		delete weakClassifier[index];
		weakClassifier[index] = weakClassifier[m_idxOfNewWeakClassifier];
		m_wWrong[index] = m_wWrong[m_idxOfNewWeakClassifier];
		m_wWrong[m_idxOfNewWeakClassifier] = 1;
		m_wCorrect[index] = m_wCorrect[m_idxOfNewWeakClassifier];
		m_wCorrect[m_idxOfNewWeakClassifier] = 1;

		weakClassifier[m_idxOfNewWeakClassifier] = new WeakClassifierHaarFeature (patchSize);
	
		return index;
	}
	else
		return -1;

}

void BaseClassifier::replaceClassifierStatistic(int sourceIndex, int targetIndex)
{
	assert (targetIndex >=0);
	assert (targetIndex != m_selectedClassifier);
	assert (targetIndex < m_numWeakClassifier);

	//replace
	m_wWrong[targetIndex] = m_wWrong[sourceIndex];
	m_wWrong[sourceIndex] = 1.0f;
	m_wCorrect[targetIndex] = m_wCorrect[sourceIndex];
	m_wCorrect[sourceIndex] = 1.0f;
}