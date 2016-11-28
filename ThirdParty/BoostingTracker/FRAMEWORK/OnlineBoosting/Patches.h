#pragma once

#include "ImageRepresentation.h"
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

class Patches
{
public:

	Patches();
	Patches(int num);
	virtual ~Patches(void);

	virtual Rect getRect(int index);
	virtual Rect getSpecialRect(const char* what);
	virtual Rect getSpecialRect(const char* what, Size patchSize);

	virtual Rect getROI();
	virtual int getNum(void){return num;};

	int checkOverlap(Rect rect);
	
    virtual bool isDetection(Rect eval, unsigned char *labeledImg, int imgWidth);
	virtual int getNumPatchesX(){return numPatchesX;}; 
	virtual int getNumPatchesY(){return numPatchesY;};

protected:

	void setCheckedROI(Rect imageROI, Rect validROI);

	Rect* patches;
	int num;
	Rect ROI;
	int numPatchesX; 
	int numPatchesY;
};

class PatchesRegularScan : public Patches
{
public:

	PatchesRegularScan(Rect imageROI, Size patchSize, float relOverlap);
	PatchesRegularScan(Rect imageROI, Rect validROI, Size patchSize, float relOverlap);
	virtual ~PatchesRegularScan (void);

	Rect getSpecialRect(const char* what);
	Rect getSpecialRect(const char* what, Size patchSize);
	Size getPatchGrid(){return m_patchGrid;};

private:

	void calculatePatches(Rect imageROI, Rect validROI, Size patchSize, float relOverlap);

	Rect m_rectUpperLeft;
	Rect m_rectUpperRight;
	Rect m_rectLowerLeft;
	Rect m_rectLowerRight;
	Size m_patchGrid;

};

class PatchesRegularScaleScan : public Patches
{
public:

	PatchesRegularScaleScan (Rect imageROI, Size patchSize, float relOverlap, float scaleStart, float scaleEnd, float scaleFactor);
	PatchesRegularScaleScan (Rect imageROI, Rect validROI, Size patchSize, float relOverlap, float scaleStart, float scaleEnd, float scaleFactor);
	virtual ~PatchesRegularScaleScan();

	Rect getSpecialRect (const char* what);
	Rect getSpecialRect (const char* what, Size patchSize);
	
private:

	void calculatePatches (Rect imageROI, Rect validROI, Size patchSize, float relOverlap, float scaleStart, float scaleEnd, float scaleFactor);

};

class PatchesFunctionScaleScan : public Patches
{
public:
	
	typedef float (*GetScale)(int, int);

	PatchesFunctionScaleScan (Rect imageROI, Size patchSize, float relOverlap, GetScale getScale);
	PatchesFunctionScaleScan (Rect imageROI, Rect validROI, Size PatchSize, float relOverlap, GetScale getScale);
	PatchesFunctionScaleScan (Rect imageROI, Size patchSize, float relOverlap, float coefY, float coef1, float minScaleFactor=1.0f);
	PatchesFunctionScaleScan (Rect imageROI, Rect validROI, Size patchSize, float relOverlap, float coefY, float coef1, float minScaleFactor = 1.0f);
	virtual ~PatchesFunctionScaleScan();

	Rect getSpecialRect (const char* what);
	Rect getSpecialRect (const char* what, Size patchSize);
	
private:

	void calculatePatches (Rect imageROI, Rect validROI, Size patchSize, float relOverlap, GetScale getScale);
	void calculatePatches (Rect imageROI, Rect validROI, Size patchSize, float relOverlap, float coefY, float coef1, float minScaleFactor);

	Rect rectUpperLeft;
	Rect rectUpperRight;
	Rect rectLowerLeft;
	Rect rectLowerRight;
};

class PatchesManualSet : public Patches
{
public:

	PatchesManualSet(int numPatches, Rect* patches);
	PatchesManualSet(int numPatches, Rect* patches, Rect ROI);
	virtual ~PatchesManualSet (void);

	Rect getSpecialRect (const char* what){return Rect(-1,-1,-1,-1);} ;
	Rect getSpecialRect (const char* what, Size patchSize){return Rect(-1,-1,-1,-1);};

private:


};
