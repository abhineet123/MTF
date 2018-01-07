#include "Detector.h"

Detector::Detector(StrongClassifier* classifier)
{
	this->m_classifier = classifier;

	m_confidences = NULL;
	m_sizeConfidences = 0;
	m_maxConfidence = -FLOAT_MAX;
	m_numDetections = 0;
	m_idxDetections = NULL;
	m_idxBestDetection = -1;

  m_confMatrix = cvCreateMat(1,1,CV_32FC1);
	m_confMatrixSmooth = cvCreateMat(1,1,CV_32FC1);

}

Detector::~Detector()
{
	if (m_idxDetections != NULL)
		delete[] m_idxDetections;
	if (m_confidences != NULL)
		delete[] m_confidences;

  cvReleaseMat(&m_confMatrix);
  cvReleaseMat(&m_confMatrixSmooth);

}

void Detector::prepareConfidencesMemory(int numPatches)
{
	if ( numPatches <= m_sizeConfidences )	
		return;							
	
	if ( m_confidences )
		delete[] m_confidences;

	m_sizeConfidences = numPatches;
    m_confidences = new float[numPatches];
}

void Detector::prepareDetectionsMemory(int numDetections)
{
	if ( numDetections <= m_sizeDetections )	
		return;							
	
	if ( m_idxDetections )
		delete[] m_idxDetections;
	m_sizeDetections = numDetections;
    m_idxDetections = new int[numDetections];
}


void Detector::classify(ImageRepresentation* image, Patches* patches, float minMargin)
{
	int numPatches = patches->getNum();

	prepareConfidencesMemory(numPatches);

	m_numDetections = 0;
	m_idxBestDetection = -1;
	m_maxConfidence = -FLOAT_MAX;
	int numBaseClassifiers = m_classifier->getNumBaseClassifier();

	for (int curPatch=0; curPatch < numPatches; curPatch++)
	{
		m_confidences[curPatch] = m_classifier->eval(image, patches->getRect(curPatch));
	
		if (m_confidences[curPatch] > m_maxConfidence)
		{
			m_maxConfidence = m_confidences[curPatch];
			m_idxBestDetection = curPatch;
		}
		if (m_confidences[curPatch] > minMargin)
			m_numDetections++;
	}

	prepareDetectionsMemory(m_numDetections);
	int curDetection = -1;
	for (int curPatch=0; curPatch < numPatches; curPatch++)
	{
		if (m_confidences[curPatch]>minMargin) m_idxDetections[++curDetection]=curPatch;
	}
}


void Detector::classifySmooth(ImageRepresentation* image, Patches* patches, float minMargin)
{
	int numPatches = patches->getNum();

	prepareConfidencesMemory(numPatches);

	m_numDetections = 0;
	m_idxBestDetection = -1;
	m_maxConfidence = -FLOAT_MAX;
	int numBaseClassifiers = m_classifier->getNumBaseClassifier();

	PatchesRegularScan *regPatches = (PatchesRegularScan*)patches;
	Size patchGrid = regPatches->getPatchGrid();

  if((patchGrid.width != m_confMatrix->cols) || (patchGrid.height != m_confMatrix->rows)) {
    cvReleaseMat(&m_confMatrix);
    cvReleaseMat(&m_confMatrixSmooth);

    m_confMatrix = cvCreateMat(patchGrid.height,patchGrid.width,CV_32FC1);
	  m_confMatrixSmooth = cvCreateMat(patchGrid.height,patchGrid.width,CV_32FC1);
  }
	
	int curPatch = 0;
	// Eval and filter
	for(int row = 0; row < patchGrid.height; row++) {
		for( int col = 0; col < patchGrid.width; col++) {
			int returnedInLayer;
			m_confidences[curPatch] = m_classifier->eval(image, patches->getRect(curPatch)); 
		
			// fill matrix
			cvmSet(m_confMatrix,row,col,m_confidences[curPatch]);
			curPatch++;
		}
	}

  // Filter
	cvSmooth(m_confMatrix,m_confMatrixSmooth,CV_GAUSSIAN,3);

	// Get best detection
	curPatch = 0;
	for(int row = 0; row < patchGrid.height; row++) {
		for( int col = 0; col < patchGrid.width; col++) {
			// fill matrix
			m_confidences[curPatch] = cvmGet(m_confMatrixSmooth,row,col);

			if (m_confidences[curPatch] > m_maxConfidence)
			{
				m_maxConfidence = m_confidences[curPatch];
				m_idxBestDetection = curPatch;
			}
			if (m_confidences[curPatch] > minMargin) {
				m_numDetections++;
			}
			curPatch++;
		}
	}

	prepareDetectionsMemory(m_numDetections);
	int curDetection = -1;
	for (int curPatch=0; curPatch < numPatches; curPatch++)
	{
		if (m_confidences[curPatch]>minMargin) m_idxDetections[++curDetection]=curPatch;
	}
}


	
int Detector::getNumDetections()
{
	return m_numDetections; 
}

float Detector::getConfidence(int patchIdx)
{
	return m_confidences[patchIdx];
}

float Detector::getConfidenceOfDetection (int detectionIdx)
{
	return m_confidences[getPatchIdxOfDetection(detectionIdx)];
}

int Detector::getPatchIdxOfBestDetection()
{
	return m_idxBestDetection;
}

int Detector::getPatchIdxOfDetection(int detectionIdx)
{
	return m_idxDetections[detectionIdx];
}
