#pragma once

#include "ImageRepresentation.h"
#include "Patches.h"
#include "StrongClassifier.h"
#include "StrongClassifierStandard.h"
#include "StrongClassifierDirectSelection.h"
#include "StrongClassifierStandardSemi.h"
#include "Detector.h"
#include "ImageSource.h"
#include "OS_specific.h"

// For OpenCV detector
#include "cv.h"
//#include "cvcam.h"
#include "highgui.h"


class BeyondSemiBoostingTracker
{
public:
        BeyondSemiBoostingTracker(ImageRepresentation* image, ImageRepresentation* backgroundImage, Rect initPatch, Rect validROI, int numBaseClassifier);
	virtual ~BeyondSemiBoostingTracker();

	bool track(ImageRepresentation* image, ImageRepresentation* BGMRep, Patches* patches);

	void initClassifier(Rect initPatch, ImageRepresentation* image, Patches* patches, bool offUpdate, bool onUpdate, bool classifierUpdate);
	void initClassifierOff(Rect initPatch, ImageRepresentation* image, ImageRepresentation* BGM, Patches* patches);
	
	bool updateOn(ImageRepresentation* image, ImageRepresentation* BGMRep, Patches* patches);
	bool update(ImageRepresentation* image, ImageRepresentation* BGMRep, Patches* patches);
	
	Rect getTrackingROI(float searchFactor);
	float getConfidence();
	Rect getTrackedPatch();
	Point2D getCenter();
	
private:
	StrongClassifierStandardSemi* classifierOff;
	StrongClassifierStandardSemi* classifierOn;
	StrongClassifierStandardSemi* classifier;

	int numBaseClassifier;
	Rect validROI;

	Rect trackedPatch;
	float confidence;
	float offThreshold;
	
};
