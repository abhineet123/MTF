#include "BoostingTracker.h"

BoostingTracker::BoostingTracker(ImageRepresentation* image, Rect initPatch, Rect validROI, int numBaseClassifier)
{
	int numWeakClassifier = numBaseClassifier*10;
	bool useFeatureExchange = true;
	int iterationInit = 50;
	Size patchSize;
	patchSize = initPatch;

	this->validROI = validROI;

	classifier = new StrongClassifierDirectSelection(numBaseClassifier, numWeakClassifier, patchSize, useFeatureExchange, iterationInit);

	detector = new Detector (classifier);
	
	trackedPatch = initPatch;
	Rect trackingROI = getTrackingROI(2.0f);
	Size trackedPatchSize;
	trackedPatchSize = trackedPatch;
	Patches* trackingPatches = new PatchesRegularScan(trackingROI, validROI, trackedPatchSize, 0.99f);
	
	 iterationInit = 50;
	for (int curInitStep = 0; curInitStep < iterationInit; curInitStep++)
	{
		printf ("\rinit tracker... %3.0f %% ", ((float)curInitStep)/(iterationInit-1)*100);	

		classifier->update (image, trackingPatches->getSpecialRect ("UpperLeft"), -1);
		classifier->update (image, trackedPatch, 1);
		classifier->update (image, trackingPatches->getSpecialRect ("UpperRight"), -1);
		classifier->update (image, trackedPatch, 1);
		classifier->update (image, trackingPatches->getSpecialRect ("LowerLeft"), -1);
		classifier->update (image, trackedPatch, 1);
		classifier->update (image, trackingPatches->getSpecialRect ("LowerRight"), -1);
		classifier->update (image, trackedPatch, 1);
	}

	confidence = -1;
	delete trackingPatches;

}

BoostingTracker::~BoostingTracker(void)
{
	delete detector;
	delete classifier;
}

bool BoostingTracker::track(ImageRepresentation* image, Patches* patches)
{
	//detector->classify (image, patches);
	detector->classifySmooth (image, patches);
	
	//move to best detection
	if (detector->getNumDetections() <=0)
	{
		confidence = 0;
		return false;
	}

	trackedPatch = patches->getRect (detector->getPatchIdxOfBestDetection ());
	confidence  = detector->getConfidenceOfBestDetection ();
	
	classifier->update (image, patches->getSpecialRect ("UpperLeft"), -1);
	classifier->update (image, trackedPatch, 1);
	classifier->update (image, patches->getSpecialRect ("UpperRight"), -1);
	classifier->update (image, trackedPatch, 1);
	classifier->update (image, patches->getSpecialRect ("UpperLeft"), -1);
	classifier->update (image, trackedPatch, 1);
	classifier->update (image, patches->getSpecialRect ("LowerRight"), -1);
	classifier->update (image, trackedPatch, 1);

	return true;
}

Rect BoostingTracker::getTrackingROI(float searchFactor)
{
	Rect searchRegion;

	searchRegion = trackedPatch*(searchFactor);
	//check
	if (searchRegion.upper+searchRegion.height > validROI.height)
		searchRegion.height = validROI.height-searchRegion.upper;
	if (searchRegion.left+searchRegion.width > validROI.width)
		searchRegion.width = validROI.width-searchRegion.left;

	return searchRegion;
}

float BoostingTracker::getConfidence()
{
	return confidence/classifier->getSumAlpha();
}

Rect BoostingTracker::getTrackedPatch()
{
	return trackedPatch;
}

Point2D BoostingTracker::getCenter()
{
	Point2D center;
	center.row = trackedPatch.upper + trackedPatch.height/2 ;
	center.col =  trackedPatch.left +trackedPatch.width/2 ;
	return center;
}
