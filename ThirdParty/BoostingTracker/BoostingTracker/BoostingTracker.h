#pragma once

#include "ImageRepresentation.h"
#include "Patches.h"
#include "StrongClassifier.h"
#include "StrongClassifierDirectSelection.h"
#include "Detector.h"

class BoostingTracker
{
public:
	BoostingTracker(ImageRepresentation* image, Rect initPatch, Rect validROI, int numBaseClassifier);
	virtual ~BoostingTracker();

	bool track(ImageRepresentation* image, Patches* patches);

	Rect getTrackingROI(float searchFactor);
	float getConfidence();
	Rect getTrackedPatch();
	Point2D getCenter();
	
private:
	StrongClassifier* classifier;
	Detector* detector;
	Rect validROI;
    Rect trackedPatch;
	float confidence;
	Point2D dxDy;
};
