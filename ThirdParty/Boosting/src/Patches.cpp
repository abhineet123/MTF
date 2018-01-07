#include "Patches.h"

Patches::Patches(void)
{
	this->num = 1;
	patches = new Rect;
	ROI.height = 0;
	ROI.width = 0;
	ROI.upper = 0;
	ROI.left = 0;
}

Patches::Patches(int num)
{
	this->num = num;
	patches = new Rect[num];
	ROI.height = 0;
	ROI.width = 0;
	ROI.upper = 0;
	ROI.left = 0;
}

Patches::~Patches(void)
{
	delete[] patches;
}

Rect Patches::getRect(int index)
{
	if (index >= num)
		return Rect(-1, -1, -1, -1);
	if (index < 0) 
		return Rect(-1, -1, -1, -1);

	return patches[index];
}

int Patches::checkOverlap(Rect rect)
{
	//loop over all patches and return the first found overap
	for (int curPatch = 0; curPatch< num; curPatch++)
	{
		Rect curRect = getRect (curPatch);
		int overlap = curRect.checkOverlap(rect);
		if (overlap > 0)
			return overlap;
	}
	return 0;
}


bool Patches::isDetection (Rect eval, unsigned char *labeledImg, int imgWidth)
{
    bool isDet = false;
    Rect curRect;
    
	for (int curPatch = 0; curPatch < num; curPatch++)
	{
        curRect = getRect (curPatch);
		isDet = curRect.isDetection(eval, labeledImg, imgWidth);

        if (isDet)
        {
            break;
        }
	}

	return isDet;
}

Rect Patches::getSpecialRect (const char* what)
{
	Rect r;
	r.height = -1;
	r.width = -1;
	r.upper = -1;
	r.left = -1;
	return r;
}

Rect Patches::getSpecialRect (const char* what, Size patchSize)
{
	Rect r;
	r.height = -1;
	r.width = -1;
	r.upper = -1;
	r.left = -1;
	return r;
}

Rect Patches::getROI()
{
	return ROI;
}

void Patches::setCheckedROI(Rect imageROI, Rect validROI)
{
	int dCol, dRow;
	dCol = imageROI.left - validROI.left;
	dRow = imageROI.upper - validROI.upper;
	ROI.upper = (dRow < 0) ? validROI.upper : imageROI.upper;
	ROI.left = (dCol < 0) ? validROI.left : imageROI.left;
	dCol = imageROI.left+imageROI.width - (validROI.left+validROI.width);
	dRow = imageROI.upper+imageROI.height - (validROI.upper+validROI.height);
	ROI.height = (dRow > 0) ? validROI.height+validROI.upper-ROI.upper : imageROI.height+imageROI.upper-ROI.upper; 
	ROI.width = (dCol > 0) ? validROI.width+validROI.left-ROI.left : imageROI.width+imageROI.left-ROI.left; 
}


//-----------------------------------------------------------------------------
PatchesRegularScan::PatchesRegularScan(Rect imageROI, Size patchSize, float relOverlap)
{
	calculatePatches (imageROI, imageROI, patchSize, relOverlap);
}

PatchesRegularScan::PatchesRegularScan (Rect imageROI, Rect validROI, Size patchSize, float relOverlap)
{
	calculatePatches (imageROI, validROI, patchSize, relOverlap);
}

void PatchesRegularScan::calculatePatches(Rect imageROI, Rect validROI, Size patchSize, float relOverlap)
{
	if ((validROI == imageROI))
		ROI = imageROI;
	else
		setCheckedROI(imageROI, validROI);
	
	int stepCol = floor((1-relOverlap) * patchSize.width+0.5);
	int stepRow = floor((1-relOverlap) * patchSize.height+0.5);
	if (stepCol <= 0) stepCol = 1;
	if (stepRow <= 0) stepRow = 1;
	
	m_patchGrid.height = ((int)((float)(ROI.height-patchSize.height)/stepRow)+1);
	m_patchGrid.width = ((int)((float)(ROI.width-patchSize.width)/stepCol)+1);

	num = m_patchGrid.width * m_patchGrid.height;
	patches = new Rect[num];
	int curPatch = 0;

	m_rectUpperLeft = m_rectUpperRight = m_rectLowerLeft = m_rectLowerRight = patchSize;
	m_rectUpperLeft.upper = ROI.upper;
	m_rectUpperLeft.left = ROI.left;
	m_rectUpperRight.upper = ROI.upper;
	m_rectUpperRight.left = ROI.left+ROI.width-patchSize.width;
	m_rectLowerLeft.upper = ROI.upper+ROI.height-patchSize.height;
	m_rectLowerLeft.left = ROI.left;
	m_rectLowerRight.upper = ROI.upper+ROI.height-patchSize.height;
	m_rectLowerRight.left = ROI.left+ROI.width-patchSize.width;


	numPatchesX=0; numPatchesY=0;
	for (int curRow=0; curRow< ROI.height-patchSize.height+1; curRow+=stepRow)
	{
		numPatchesY++;

		for (int curCol=0; curCol< ROI.width-patchSize.width+1; curCol+=stepCol)
		{
			if(curRow == 0)
				numPatchesX++;

			patches[curPatch].width = patchSize.width;
			patches[curPatch].height = patchSize.height;
			patches[curPatch].upper = curRow+ROI.upper;
			patches[curPatch].left = curCol+ROI.left;
			curPatch++;
		}
	}

	assert (curPatch==num);
}

PatchesRegularScan::~PatchesRegularScan(void)
{
}

Rect PatchesRegularScan::getSpecialRect(const char* what, Size patchSize)
{
	Rect r;
	r.height = -1;
	r.width = -1;
	r.upper = -1;
	r.left = -1;
	return r;
}

Rect PatchesRegularScan::getSpecialRect(const char* what)
{
	if (strcmp(what, "UpperLeft")==0) return m_rectUpperLeft;
	if (strcmp(what, "UpperRight")==0) return m_rectUpperRight;
	if (strcmp(what, "LowerLeft")==0) return m_rectLowerLeft;
	if (strcmp(what, "LowerRight")==0) return m_rectLowerRight;
	if (strcmp(what, "Random")==0)
	{
		int index = (rand()%(num));
		return patches[index];
	}

	// assert (false);
	return Rect(-1, -1, -1, -1); // fixed
}

//-----------------------------------------------------------------------------
PatchesRegularScaleScan::PatchesRegularScaleScan (Rect imageROI, Size patchSize, float relOverlap, float scaleStart, float scaleEnd, float scaleFactor)
{
	calculatePatches (imageROI, imageROI, patchSize, relOverlap, scaleStart, scaleEnd, scaleFactor);
}

PatchesRegularScaleScan::PatchesRegularScaleScan (Rect imageROI, Rect validROI, Size patchSize, float relOverlap, float scaleStart, float scaleEnd, float scaleFactor)
{
	calculatePatches (imageROI, validROI, patchSize, relOverlap, scaleStart, scaleEnd, scaleFactor);
}

PatchesRegularScaleScan::~PatchesRegularScaleScan(void)
{

}


void PatchesRegularScaleScan::calculatePatches(Rect imageROI, Rect validROI, Size patchSize, float relOverlap, float scaleStart, float scaleEnd, float scaleFactor)
{

	if ((validROI == imageROI))
		ROI = imageROI;
	else
		setCheckedROI(imageROI, validROI);

	int numScales = (int)(log(scaleEnd/scaleStart)/log(scaleFactor));
	if (numScales < 0) numScales = 0;
	float curScaleFactor = 1;
	Size curPatchSize;
	int stepCol, stepRow;

	num = 0;
	for (int curScale = 0; curScale <= numScales; curScale++)
	{   
		curPatchSize = patchSize * (scaleStart*curScaleFactor);
		if (curPatchSize.height > ROI.height || curPatchSize.width > ROI.width)
		{
			numScales = curScale-1;
			break;
		}
		curScaleFactor *= scaleFactor;
					
		stepCol = floor((1-relOverlap) * curPatchSize.width+0.5);
		stepRow = floor((1-relOverlap) * curPatchSize.height+0.5);
		if (stepCol <= 0) stepCol = 1;
		if (stepRow <= 0) stepRow = 1;
	
		num += ((int)((float)(ROI.width-curPatchSize.width)/stepCol)+1)*((int)((float)(ROI.height-curPatchSize.height)/stepRow)+1);
	}
	patches = new Rect[num];
	    
	int curPatch = 0;
	curScaleFactor = 1;
	for (int curScale = 0; curScale <= numScales; curScale++)
	{   
		curPatchSize = patchSize * (scaleStart*curScaleFactor);
		curScaleFactor *= scaleFactor;
			
		stepCol = floor((1-relOverlap) * curPatchSize.width+0.5);
		stepRow = floor((1-relOverlap) * curPatchSize.height+0.5);
		if (stepCol <= 0) stepCol = 1;
		if (stepRow <= 0) stepRow = 1;
	

		
		
		for (int curRow=0; curRow< ROI.height-curPatchSize.height+1; curRow+=stepRow)
		{
			for (int curCol=0; curCol<ROI.width-curPatchSize.width+1; curCol+=stepCol)
			{
				patches[curPatch].width = curPatchSize.width;
				patches[curPatch].height = curPatchSize.height;
				patches[curPatch].upper = curRow+ROI.upper;
				patches[curPatch].left = curCol+ROI.left;

				curPatch++;
			}
		}
	}
	assert (curPatch==num);

}

Rect PatchesRegularScaleScan::getSpecialRect (const char* what)
{
	
	if (strcmp(what, "Random")==0)
	{
		int index = (rand()%(num));
		return patches[index];
	}

	Rect r;
	r.height = -1;
	r.width = -1;
	r.upper = -1;
	r.left = -1;
	return r;
}
Rect PatchesRegularScaleScan::getSpecialRect (const char* what, Size patchSize)
{		
	if (strcmp(what, "UpperLeft")==0)
	{
		Rect rectUpperLeft;
		rectUpperLeft =  patchSize;
		rectUpperLeft.upper = ROI.upper;
		rectUpperLeft.left = ROI.left;
		return rectUpperLeft;
	}
	if (strcmp(what, "UpperRight")==0) 
	{
		Rect rectUpperRight;
		rectUpperRight = patchSize;
		rectUpperRight.upper = ROI.upper;
		rectUpperRight.left = ROI.left+ROI.width-patchSize.width;
		return rectUpperRight;
	}
	if (strcmp(what, "LowerLeft")==0)
	{
		Rect rectLowerLeft;
		rectLowerLeft = patchSize;
		rectLowerLeft.upper = ROI.upper+ROI.height-patchSize.height;
		rectLowerLeft.left = ROI.left;
		return rectLowerLeft;
	}
	if (strcmp(what, "LowerRight")==0)
	{
		Rect rectLowerRight;
		rectLowerRight = patchSize;
		rectLowerRight.upper = ROI.upper+ROI.height-patchSize.height;
		rectLowerRight.left = ROI.left+ROI.width-patchSize.width;
		return rectLowerRight;
	}
	if (strcmp(what, "Random")==0)
	{
		int index = (rand()%(num));
		return patches[index];
	}

	return Rect(-1, -1, -1, -1); 
}