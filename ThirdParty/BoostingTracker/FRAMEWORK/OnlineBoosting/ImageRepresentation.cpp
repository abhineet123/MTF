#include "OS_specific.h"
#include "ImageRepresentation.h"
#define M_PI (3.14159265358979323846)

#include <stdio.h>


ImageRepresentation::ImageRepresentation(unsigned char* image, Size imageSize)
{  
	// call the default initialization
	this->defaultInit(image, imageSize);
	return;
}

ImageRepresentation::ImageRepresentation(unsigned char* image, Size imageSize, Rect imageROI)
{
	this->m_imageSize = imageSize;

	m_ROI = imageROI;
	m_offset = m_ROI;

	m_useVariance = false;

	intImage = NULL;
	intSqImage = NULL;
        intImage = new __uint32[(m_ROI.width+1)*(m_ROI.height+1)];
        intSqImage = new __uint64[(m_ROI.width+1)*(m_ROI.height+1)];

	if (image!= NULL)
		this->createIntegralsOfROI(image);
}


void ImageRepresentation::defaultInit(unsigned char* image, Size imageSize)
{  
	this->m_imageSize = imageSize;

	m_useVariance = false;

	m_ROI.height = imageSize.height;
	m_ROI.width = imageSize.width;
	m_ROI.upper = 0;
	m_ROI.left = 0;
	m_offset = m_ROI;

	intImage = NULL;
	intSqImage = NULL;


        intImage = new __uint32[(m_ROI.width+1)*(m_ROI.height+1)];
        intSqImage = new __uint64[(m_ROI.width+1)*(m_ROI.height+1)];

	if (image != NULL)
		this->createIntegralsOfROI(image);

	return;
}

ImageRepresentation::~ImageRepresentation()
{

	delete[] intImage;
	delete[] intSqImage;
}

void ImageRepresentation::setNewImage(unsigned char* image)
{
	createIntegralsOfROI(image);
}

void ImageRepresentation::setNewROI(Rect ROI)
{
	if (this->m_ROI.height*this->m_ROI.width != ROI.height*ROI.width)
	{	
		delete[] intImage;
		delete[] intSqImage;
                intImage = new __uint32[(ROI.width+1)*(ROI.height+1)];
                intSqImage = new __uint64[(ROI.width+1)*(ROI.height+1)];
	}
	this->m_ROI = ROI;
	m_offset = ROI;
	return;
}

void ImageRepresentation::setNewImageSize( Rect ROI )
{
	this->m_imageSize = ROI;
}


void ImageRepresentation::setNewImageAndROI(unsigned char* image, Rect ROI)
{
	this->setNewROI(ROI);
	this->createIntegralsOfROI(image);
}

__uint32 ImageRepresentation::getValue(Point2D imagePosition)
{
	Point2D position = imagePosition-m_offset;
	return intImage[position.row*(this->m_ROI.width+1)+position.col];
}

long ImageRepresentation::getSqSum(Rect imageROI)
{
	// left upper Origin
	int OriginX = imageROI.left-m_offset.col;
	int OriginY = imageROI.upper-m_offset.row;

        __uint64 *OriginPtr = &intSqImage[OriginY * (m_ROI.width+1) + OriginX];

	// Check and fix width and height
	int Width  = imageROI.width;
	int Height = imageROI.height;

	if ( OriginX+Width  >= m_ROI.width  ) Width  = m_ROI.width  - OriginX;
	if ( OriginY+Height >= m_ROI.height ) Height = m_ROI.height  - OriginY;

	unsigned long down  = Height * (m_ROI.width+1);
	unsigned long right = Width;

	__int64 value = OriginPtr[down+right] + OriginPtr[0] - OriginPtr[right] - OriginPtr[down];

	assert (value >= 0);

	OriginPtr = NULL;
	return value;

}

float ImageRepresentation::getVariance(Rect imageROI)
{
	double area = imageROI.height*imageROI.width;
	double mean = (double) getSum(imageROI)/area;
	double sqSum = (double) getSqSum( imageROI );

	double variance = sqSum/area - (mean*mean);

	if( variance >= 0. )
		return (float)sqrt(variance);
	else
		return 1.0f;
}

__int32 ImageRepresentation::getSum(Rect imageROI)
{
	// left upper Origin
	int OriginX = imageROI.left-m_offset.col;
	int OriginY = imageROI.upper-m_offset.row;

        __uint32 *OriginPtr = &intImage[OriginY * (m_ROI.width+1) + OriginX];

	// Check and fix width and height
	int Width  = imageROI.width;
	int Height = imageROI.height;

	if ( OriginX+Width  >= m_ROI.width  ) Width  = m_ROI.width  - OriginX;
	if ( OriginY+Height >= m_ROI.height ) Height = m_ROI.height  - OriginY;

	unsigned long down  = Height * (m_ROI.width+1);
	unsigned long right = Width;

	__int32 value = OriginPtr[down+right] + OriginPtr[0] - OriginPtr[right] - OriginPtr[down];


	OriginPtr = NULL;
	return value;
}

float ImageRepresentation::getMean(Rect imageROI)
{
	// left upper Origin
	int OriginX = imageROI.left-m_offset.col;
	int OriginY = imageROI.upper-m_offset.row;

	// Check and fix width and height
	int Width  = imageROI.width;
	int Height = imageROI.height;

	if ( OriginX+Width  >= m_ROI.width  ) Width  = m_ROI.width  - OriginX;
	if ( OriginY+Height >= m_ROI.height ) Height = m_ROI.height  - OriginY;

	return getSum(imageROI)/static_cast<float>(Width*Height);
}

void ImageRepresentation::createIntegralsOfROI(unsigned char* image)
{
	unsigned long ROIlength = (m_ROI.width+1)*(m_ROI.height+1);
	int columnidx, rowidx;
	unsigned long curPointer;
	unsigned long dptr;


	curPointer = 0;
	dptr = 0;

	memset(intImage, 0x00, ROIlength * sizeof( unsigned int ) );
	memset(intSqImage, 0x00, ROIlength * sizeof( unsigned __int64 ) ); 

	// current sum
	unsigned int value_tmp = 0;
	unsigned int value_tmpSq = 0;

	for (rowidx = 0; rowidx<m_ROI.height; rowidx++)
	{
		// current Image Position
		curPointer = (rowidx+m_ROI.upper)*m_imageSize.width+m_ROI.left;

		// current Integral Image Position
		dptr = (m_ROI.width+1)*(rowidx+1) + 1;

		value_tmp = 0;
		value_tmpSq = 0;

		for (columnidx = 0; columnidx<m_ROI.width; columnidx++)
		{
			// cumulative row sum
			value_tmp += image[curPointer];
			value_tmpSq += image[curPointer]*image[curPointer];

			// update Integral Image
			intImage[dptr] = intImage[ dptr - (m_ROI.width+1) ] + value_tmp;
			intSqImage[dptr] = intSqImage[ dptr - (m_ROI.width+1) ] + value_tmpSq;

			dptr++;
			curPointer++;
		}

	}

	return;
}
