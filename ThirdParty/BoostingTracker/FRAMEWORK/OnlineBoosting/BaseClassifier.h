#pragma once

#include "WeakClassifier.h"
#include "WeakClassifierHaarFeature.h"
#include "ImageRepresentation.h"

#include <deque>
#include <vector>
#include <windefs.h>

using namespace std;

class BaseClassifier  
{
public:
	
	BaseClassifier(int numWeakClassifier, int iterationInit, Size patchSize); 
	BaseClassifier(int numWeakClassifier, int iterationInit, WeakClassifier** weakClassifier); 

	virtual ~BaseClassifier();

	void trainClassifier(ImageRepresentation* image, Rect ROI, int target, float importance, bool* errorMask); 
	
	void getErrorMask(ImageRepresentation* image, Rect ROI, int target, bool* errorMask); 
	void getErrors(float* errors);
	virtual int selectBestClassifier(bool* errorMask, float importance, float* errors); 
	
	virtual int replaceWeakestClassifier(float* errors, Size patchSize);
	virtual float getError(int curWeakClassifier = -1);

	void replaceClassifierStatistic(int sourceIndex, int targetIndex);

 	int eval(ImageRepresentation* image, Rect ROI); 
	
	float getValue(ImageRepresentation *image, Rect ROI, int weakClassifierIdx = -1);

	WeakClassifier** getReferenceWeakClassifier(){return weakClassifier;};
	void setReferenceWeakClassifier(WeakClassifier** weakClassifier){this->weakClassifier = weakClassifier;};
	
	int getNumWeakClassifier(){return m_numWeakClassifier;};
	int getIterationInit(){return m_iterationInit;};
	float getWCorrect(){return m_wCorrect[m_selectedClassifier];};
	float getWWrong(){return m_wWrong[m_selectedClassifier];};
	void setWCorrect(int idx, float value){ if(idx < m_numWeakClassifier) m_wCorrect[idx] = value; };
	void setWWrong(int idx, float value){ if(idx < m_numWeakClassifier) m_wWrong[idx] = value; };

	int getTypeOfSelectedClassifier(){return weakClassifier[m_selectedClassifier]->getType();};
	int getIdxOfSelectedClassifier(){return m_selectedClassifier;};
	int getIdxOfNewWeakClassifier(){return m_idxOfNewWeakClassifier;};
	
protected:

	WeakClassifier** weakClassifier;
	bool m_referenceWeakClassifier;
	int m_numWeakClassifier;
	int m_selectedClassifier;
	int m_idxOfNewWeakClassifier;
	float* m_wCorrect;
	float* m_wWrong;
	int m_iterationInit;
	void generateRandomClassifier (Size patchSize);

};
