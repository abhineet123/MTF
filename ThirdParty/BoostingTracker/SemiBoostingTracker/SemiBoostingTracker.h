#pragma once

#include "ImageRepresentation.h"
#include "Patches.h"
#include "StrongClassifier.h"
#include "StrongClassifierDirectSelection.h"
#include "StrongClassifierStandardSemi.h"
#include "Detector.h"

class SemiBoostingTracker
{
public:
	SemiBoostingTracker(ImageRepresentation* image, Rect initPatch, Rect validROI, int numBaseClassifier);
	virtual ~SemiBoostingTracker();

	bool track(ImageRepresentation* image, Patches* patches);

	Rect getTrackingROI(float searchFactor);
	float getConfidence();
	float getPriorConfidence();
	Rect getTrackedPatch();
	Point2D getCenter();
	
private:
	StrongClassifier* classifierOff;
	StrongClassifierStandardSemi* classifier;
	Detector* detector;
    Rect trackedPatch;
	Rect validROI;
	float confidence;
	float priorConfidence;
};
