#include "StrongClassifierStandardSemi.h"

StrongClassifierStandardSemi::StrongClassifierStandardSemi(int numBaseClassifier, 
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

	m_pseudoTarget = new int[numBaseClassifier];
	m_pseudoLambda = new float[numBaseClassifier];
	
	for (int curBaseClassifier = 0; curBaseClassifier< numBaseClassifier; curBaseClassifier++)
	{
		baseClassifier[curBaseClassifier] = new BaseClassifier(numWeakClassifier, iterationInit, patchSize);
	}

	m_errorMask = new bool[numAllWeakClassifier];
	m_errors = new float[numAllWeakClassifier];
}

StrongClassifierStandardSemi::~StrongClassifierStandardSemi()
{
	delete[] m_errorMask;
	delete[] m_errors;

	delete[] m_pseudoTarget;
	delete[] m_pseudoLambda;
}



bool StrongClassifierStandardSemi::updateSemi(ImageRepresentation *image, Rect ROI, float priorConfidence)
{

	float value = 0.0f, kvalue = 0.0f;

	bool decided = false;
	bool uncertain = false;
	bool used = false;
	float sumAlpha = 0;

	float scaleFactor = 2.0f;

	int curBaseClassifier;
	for (curBaseClassifier = 0; curBaseClassifier<numBaseClassifier; curBaseClassifier++)
	{
		memset(m_errorMask, 0x00, numAllWeakClassifier*sizeof(bool));
		memset(m_errors, 0x00, numAllWeakClassifier*sizeof(float));
	
		int selectedClassifier;
		{
     		//scale
			if (sumAlpha > 0)
				kvalue = value/this->getSumAlpha();
			else
				kvalue = 0;
			
			float combinedDecision = tanh(scaleFactor*priorConfidence)-tanh(scaleFactor*kvalue);
			int myTarget = static_cast<int>(sign(combinedDecision));

			m_pseudoTarget[curBaseClassifier] = myTarget;
			float myImportance = abs(combinedDecision);
			m_pseudoLambda[curBaseClassifier] = myImportance;

			baseClassifier[curBaseClassifier]->trainClassifier(image, ROI, myTarget, myImportance, m_errorMask);
			selectedClassifier = baseClassifier[curBaseClassifier]->selectBestClassifier (m_errorMask, myImportance, m_errors);
		}

		float curValue = baseClassifier[curBaseClassifier]->eval(image, ROI)*alpha[curBaseClassifier];
		value += curValue;
		sumAlpha +=alpha[curBaseClassifier];
		
		if (m_errors[selectedClassifier] >= 0.5)
			alpha[curBaseClassifier] = 0;
		else
			alpha[curBaseClassifier] = logf((1.0f-m_errors[selectedClassifier])/m_errors[selectedClassifier]);
				
		if (useFeatureExchange)
			baseClassifier[curBaseClassifier]->replaceWeakestClassifier (m_errors, patchSize);

	}

	return used;
}
