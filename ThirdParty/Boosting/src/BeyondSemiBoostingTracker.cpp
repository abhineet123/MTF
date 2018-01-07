#include "BeyondSemiBoostingTracker.h"

BeyondSemiBoostingTracker::BeyondSemiBoostingTracker(ImageRepresentation* image, ImageRepresentation* backgroundImage, Rect initPatch, Rect validROI, int numBaseClassifier)
{
	offThreshold=-100;

	this->validROI = validROI;
	this->numBaseClassifier = numBaseClassifier;
	int numWeakClassifier = 100;
	bool useFeatureExchange = true;
	int iterationInit = 50;
	Size patchSize;
	patchSize = initPatch;

	classifierOff = new StrongClassifierStandardSemi(numBaseClassifier, numWeakClassifier, patchSize, useFeatureExchange, iterationInit);
	classifierOn = new StrongClassifierStandardSemi(numBaseClassifier, numWeakClassifier, patchSize, useFeatureExchange, iterationInit);  
	classifier = new StrongClassifierStandardSemi(numBaseClassifier, numWeakClassifier, patchSize, useFeatureExchange, iterationInit);  


	trackedPatch = initPatch;

	//train the off-line "detector"
	Rect trackingROI = getTrackingROI(2.0f);
	Size trackedPatchSize;
	trackedPatchSize = trackedPatch;
	Patches* trackingPatches = new PatchesRegularScan(trackingROI, validROI, trackedPatchSize, 0.99f);
	iterationInit =  25;
	for (int curInitStep = 0; curInitStep < iterationInit; curInitStep++)
	{
		printf ("\rinit tracker... %3.0f %% ", ((float)curInitStep)/(iterationInit-1)*100);	
		initClassifier(initPatch, image, trackingPatches, true, true, true);
	}
        delete trackingPatches;
	printf (" done.\ninit detector... ");
	//also scan on the whole image to make a "better" detector
	Patches* patches = new PatchesRegularScan(validROI, validROI, patchSize, 0.95f);
	initClassifierOff(initPatch, image, backgroundImage, patches);
        delete patches;

	confidence = 1;

}

BeyondSemiBoostingTracker::~BeyondSemiBoostingTracker(void)
{
	delete classifier;
}

void BeyondSemiBoostingTracker::initClassifier(Rect initPatch, ImageRepresentation* image, Patches* patches, bool offUpdate, bool onUpdate, bool classifierUpdate) {

	if (classifierUpdate==true) {
		classifier->updateSemi (image, patches->getSpecialRect ("UpperLeft"), -1);
		classifier->updateSemi (image, initPatch, 1);
		classifier->updateSemi (image, patches->getSpecialRect ("UpperRight"), -1);
		classifier->updateSemi (image, initPatch, 1);
		classifier->updateSemi (image, patches->getSpecialRect ("LowerLeft"), -1);
		classifier->updateSemi (image, initPatch, 1);
		classifier->updateSemi (image, patches->getSpecialRect ("LowerRight"), -1);
		classifier->updateSemi (image, initPatch, 1);
	}

	if (offUpdate==true) {
		classifierOff->updateSemi(image, patches->getSpecialRect ("UpperLeft"), -1);
		classifierOff->updateSemi(image, initPatch, 1);
		classifierOff->updateSemi(image, patches->getSpecialRect ("UpperRight"), -1);
		classifierOff->updateSemi(image, initPatch, 1);
		classifierOff->updateSemi(image, patches->getSpecialRect ("LowerLeft"), -1);
		classifierOff->updateSemi(image, initPatch, 1);
		classifierOff->updateSemi(image, patches->getSpecialRect ("LowerRight"), -1);
		classifierOff->updateSemi(image, initPatch, 1);
	}

	if (onUpdate==true) {
		classifierOn->updateSemi(image, patches->getSpecialRect ("UpperLeft"), -1);
		classifierOn->updateSemi(image, initPatch, 1);
		classifierOn->updateSemi(image, patches->getSpecialRect ("UpperRight"), -1);
		classifierOn->updateSemi(image, initPatch, 1);
		classifierOn->updateSemi(image, patches->getSpecialRect ("LowerLeft"), -1);
		classifierOn->updateSemi(image, initPatch, 1);
		classifierOn->updateSemi(image, patches->getSpecialRect ("LowerRight"), -1);
		classifierOn->updateSemi(image, initPatch, 1);	
	}
}

void BeyondSemiBoostingTracker::initClassifierOff(Rect initPatch, ImageRepresentation* image, ImageRepresentation* BGMRep, Patches *patches) {
	//train the detector with the most similar patches from the image

	vector <int> posSamples;
	float sumAlpha=classifierOff->getSumAlpha();
	float threshold=0.0;
	float tmpConf=0.0;
	float tmpConfBGM=0.0;
	float tmpConfMax= -1.0;
	float tmpConfBGMMax= -1.0;
	float maxConf=-100;

	while (tmpConfBGMMax>threshold && tmpConfMax>threshold || tmpConfBGMMax==-1.0) {
		threshold=threshold+0.05f;
		posSamples.clear();
		tmpConfBGMMax=-1;
		for (int i=0; i < patches->getNum(); i++){
			Rect tmpRect = patches->getRect(i);

			tmpConfBGM=classifierOff->eval(BGMRep, patches->getRect(i))/sumAlpha;

			if (tmpConfBGM>=threshold) { 
				posSamples.push_back(i);	
			}

			if (tmpConfBGM>tmpConfBGMMax) 
				tmpConfBGMMax=tmpConfBGM;
		}	

		for (int k=0; (k<posSamples.size()); k++) {
			tmpConf=classifierOff->eval(image, initPatch)/classifierOff->getSumAlpha();
			tmpConfBGM=classifierOff->eval(BGMRep,patches->getRect(posSamples[k]))/classifierOff->getSumAlpha();

			classifierOff->updateSemi(image,initPatch,1);
			classifierOff->updateSemi(BGMRep,patches->getRect(posSamples[k]),-1);
		}

		tmpConfBGMMax=-100.0;
		tmpConfMax=-100.0;
		tmpConf=-100.0;

		//verification
		sumAlpha=classifierOff->getSumAlpha();
		maxConf=classifierOff->eval(image, initPatch)/sumAlpha; //might be not well aligned

		for (int i=0; i < patches->getNum(); i++){
			Rect tmpRect = patches->getRect(i);

			tmpConfBGM=classifierOff->eval(BGMRep, patches->getRect(i))/sumAlpha;
			tmpConf=classifierOff->eval(image, patches->getRect(i))/sumAlpha;

			if (tmpConfBGM>tmpConfBGMMax) 
				tmpConfBGMMax=tmpConfBGM;

			if (tmpConf>tmpConfMax)
				tmpConfMax=tmpConf;
		}	

	}

	offThreshold=(tmpConfMax+tmpConfBGMMax)/2;
}

bool BeyondSemiBoostingTracker::updateOn(ImageRepresentation* image, ImageRepresentation* BGMRep, Patches* patches){
	//SUPERVISED UPDATES of the recognizer

	//on image and BG image
	classifierOn->updateSemi(image, patches->getSpecialRect ("UpperLeft"), -1);
	classifierOn->updateSemi(image, trackedPatch, 1);
	classifierOn->updateSemi(BGMRep, trackedPatch,-1);
	classifierOn->updateSemi(image, trackedPatch, 1); 
	classifierOn->updateSemi(image, patches->getSpecialRect ("UpperRight"), -1);
	classifierOn->updateSemi(image, trackedPatch, 1);
	classifierOn->updateSemi(BGMRep, trackedPatch,-1);
	classifierOn->updateSemi(image, trackedPatch, 1); 
	classifierOn->updateSemi(image, patches->getSpecialRect ("LowerLeft"), -1);
	classifierOn->updateSemi(image, trackedPatch, 1);
	classifierOn->updateSemi(BGMRep, trackedPatch,-1);
	classifierOn->updateSemi(image, trackedPatch, 1); 
	classifierOn->updateSemi(image, patches->getSpecialRect ("LowerRight"), -1);
	classifierOn->updateSemi(image, trackedPatch, 1);
	classifierOn->updateSemi(BGMRep, trackedPatch,-1);
	classifierOn->updateSemi(image, trackedPatch, 1); 
		
	//now, we have to make sure that the classifier really distinguished FG from BG
	//e.g. that the tracker does not jump in the BG
	vector <int> posSamples;
	float confOn=-100;
	for (int i=0; i < patches->getNum(); i++){
		//outside the tracking bounding box
		confOn=classifierOn->eval(BGMRep, patches->getRect(i))/classifierOn->getSumAlpha();

		if (confOn>=0.1) {
			posSamples.push_back(i);	
		}
	}	
	for (int k=0; (k<posSamples.size()); k++) {
		classifierOn->updateSemi(image,trackedPatch,1);
		classifierOn->updateSemi(BGMRep,patches->getRect(posSamples[k]),-1);
	}

	return true;
}


bool BeyondSemiBoostingTracker::update(ImageRepresentation* image, ImageRepresentation* BGMRep, Patches* patches){
	//UNSUPERVISED UPDATES of the tracking classifier in the FG
	//SUPERVISED UPDATES of the tracking classifier in the BG
	//ANALYSIS OF THE CONFIDENCE MAP

	vector <int> maxIVector;
	bool checkMaxI=false;
	float tempConf = -100;
	float maxConf=-100;
	int maxI=0;

	//ANALYSIS OF THE CONFIDENCE MAP
	//updates at the position of the supposed maximum, 
	//maximum needs to remain at that position
	int iterCounter=0;
	float confOn=-100;
	while (checkMaxI==false && iterCounter<10 && (maxConf>0.1 || iterCounter==0))
	{ 
		iterCounter++;
		maxConf=-100;

		for (int i=0; i < patches->getNum(); i++){
			tempConf=classifier->eval(image, patches->getRect(i))/classifier->getSumAlpha();
			if (tempConf>maxConf) { 
				maxConf=tempConf;
				maxI=i;
			} 
		}

		//supposed maximum is found, now perform unlabeled updates
		float classifierOnAlpha = 1.0f/classifierOn->getSumAlpha();

		//perform unlabeled updates only if maxConf is higher than threshold
		if (maxConf>0.1) {
			float on, on_trackedPatch;
			confOn = classifierOn->eval(image, patches->getRect(maxI));
			confOn *= classifierOnAlpha;
			classifier->updateSemi(BGMRep, patches->getRect(maxI), -1); 
			classifier->updateSemi(image, patches->getRect(maxI), confOn);
		} 

		maxIVector.push_back(maxI);
		checkMaxI=false;

		if (maxIVector.size()>1) {
			if (maxIVector[maxIVector.size()-2]==maxI) {
				checkMaxI=true;
			}
		}
	}

	//if the confidence of supposed maximum is lower than threshold, 
	//then the track is lost
	if (maxConf<0.1 || iterCounter>=10)  { 

		confidence = 0;
		return false;
	}
	
	maxConf = maxConf;
	confidence=maxConf;
	trackedPatch = patches->getRect(maxI);

	return true;
}

bool BeyondSemiBoostingTracker::track(ImageRepresentation* image, ImageRepresentation* BGMRep, Patches* patches){

	// tracker lost or still tracking?
	if (confidence <= 0)
	{
		//INITIALIZATION FLOW
		float sumAlpha=classifierOff->getSumAlpha();
		float maxConf=-100.0; float tempConf; int maxIdx = -1;
		for (int i=0;i < patches->getNum(); i++){
			tempConf=classifierOff->eval(image, patches->getRect(i));
			if (tempConf>maxConf) { 
				maxConf=tempConf;
				maxIdx=i;
			} 
		}
		maxConf=maxConf/sumAlpha;
		if (maxConf>offThreshold) {
			Rect initRect = patches->getRect(maxIdx);

			//init a new tracker
			Rect trackingROI = getTrackingROI(2.0f);
			Size trackedPatchSize;
			trackedPatchSize = trackedPatch;
			
			Patches* trackingPatches = new PatchesRegularScan(trackingROI, validROI, trackedPatchSize, 0.99f);
			int iterationInit = 25;
			for (int curInitStep = 0; curInitStep < iterationInit; curInitStep++)
			{
				printf ("\rreinit tracker... %3.0f %%            \r", ((float)curInitStep)/(iterationInit-1)*100);
				initClassifier(initRect, image, patches, false, true, true);
			}
			delete[] trackingPatches;
			confidence = 0.5;


















			trackedPatch = initRect;
		} else {
			return false;
		}
	}
	else
	{	
		//TRACKING FLOW
		//look for maximum of classifier with confidence map analysis and update classifier with classifierOn as prior
		update(image, BGMRep, patches);

		//if maximum is stable -> update classifiers
		if (confidence > 0)
		{
			//UPDATE priorClassifier with detection
			Rect tmpRect;
			float sumAlpha=1.0f/classifierOff->getSumAlpha();
			float maxConf=-1.0; float tempConf; int maxIdx = -1;
			for (int i=0;i < patches->getNum(); i++){
				tmpRect=patches->getRect(i);
				if (tmpRect.left > trackedPatch.left-1*trackedPatch.width/5 &&
					tmpRect.upper > trackedPatch.upper-1*trackedPatch.height/5 &&
					tmpRect.left < trackedPatch.left+1*trackedPatch.width/5 &&
					tmpRect.upper < trackedPatch.upper+1*trackedPatch.height/5) {
						tempConf=classifierOff->eval(image, tmpRect)*sumAlpha;
						if (tempConf>maxConf) { 
							//max search
							maxConf=tempConf;
							maxIdx=i;
						} 
				}
			}
			if (maxConf>offThreshold) {
				//update the prior
				this->updateOn(image, BGMRep, patches);
			}
		} else
		{
			//tracker lost -> clean up
			delete classifier;
			delete classifierOn;

			int numWeakClassifier = 100;
			bool useFeatureExchange = true;
			int iterationInit = 50;
			Size patchSize; patchSize = patches->getRect(1);
			classifierOn = new StrongClassifierStandardSemi(numBaseClassifier, numWeakClassifier, patchSize, useFeatureExchange, iterationInit);  
			classifier = new StrongClassifierStandardSemi(numBaseClassifier, numWeakClassifier, patchSize, useFeatureExchange, iterationInit);  

			return false;
		}

	} 

	return true;
}

Rect BeyondSemiBoostingTracker::getTrackingROI(float searchFactor)
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

float BeyondSemiBoostingTracker::getConfidence()
{
	return confidence;
}

Rect BeyondSemiBoostingTracker::getTrackedPatch()
{
	return trackedPatch;
}

Point2D BeyondSemiBoostingTracker::getCenter()
{
	Point2D center;
	center.row = trackedPatch.upper + trackedPatch.height/2 ;
	center.col = trackedPatch.left +trackedPatch.width/2 ;
	return center;
}
