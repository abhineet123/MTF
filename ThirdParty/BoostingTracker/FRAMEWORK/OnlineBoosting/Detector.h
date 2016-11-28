#pragma once

#include "StrongClassifier.h"
#include "ImageRepresentation.h"
#include "Patches.h"

#include "cv.h"
#include "highgui.h"

class Detector
{
public:

	Detector(StrongClassifier* classifier);
	virtual ~Detector(void);

	void classify(ImageRepresentation* image, Patches* patches, float minMargin = 0.0f);
	void classify(ImageRepresentation* image, Patches* patches, float minMargin, float minVariance );

	void classifySmooth(ImageRepresentation* image, Patches* patches, float minMargin = 0);

	int getNumDetections();
	float getConfidence(int patchIdx);
	float getConfidenceOfDetection (int detectionIdx);


	float getConfidenceOfBestDetection (){return m_maxConfidence;};
	int getPatchIdxOfBestDetection();

	int getPatchIdxOfDetection(int detectionIdx);

	int* getIdxDetections(){return m_idxDetections;};
	float* getConfidences(){return m_confidences;};

private:

	void prepareConfidencesMemory(int numPatches);
	void prepareDetectionsMemory(int numDetections);

	StrongClassifier* m_classifier;
	float* m_confidences;
	int m_sizeConfidences;
	int m_numDetections;
	int* m_idxDetections;
	int m_sizeDetections;
	int m_idxBestDetection;
	float m_maxConfidence;
 	CvMat *m_confMatrix;
	CvMat *m_confMatrixSmooth;

};
