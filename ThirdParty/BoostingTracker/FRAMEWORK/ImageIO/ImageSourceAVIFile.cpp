#include "ImageSourceAVIFile.h"
#include <stdio.h>

ImageSourceAVIFile::ImageSourceAVIFile(const char* aviFilename)
{
	m_capture = cvCaptureFromAVI( aviFilename );

	if (m_capture == NULL)
	{
		printf ("ERROR: AVI file not found!\n");
		m_existImage = false;
		return;
	}

	strcpy(this->aviFilename, aviFilename); 
	curImage = NULL;
   	m_copyImg = NULL;
	m_existImage = true;

	getIplImage();
}


ImageSourceAVIFile::~ImageSourceAVIFile()
{
	if(m_capture!=NULL)
	{
		cvReleaseCapture(&m_capture);
	}
}

void ImageSourceAVIFile::getIplImage()
{
	if (curImage != NULL)
    {
        cvReleaseImage (&curImage);
    }

	cvGrabFrame(m_capture);
	m_copyImg = cvRetrieveFrame(m_capture);

	if (m_copyImg != NULL)
	{
		if(m_copyImg->origin == 1)
		{
 			cvFlip(m_copyImg, m_copyImg, 0);
			m_copyImg->origin = 0;
		}
		m_existImage = true;
		curImage = cvCloneImage(m_copyImg);
	}
	else
		m_existImage = false;


}

void ImageSourceAVIFile::reloadIplImage()
{
	cvCopyImage (m_copyImg, curImage);
}

void ImageSourceAVIFile::viewAVI()
{
	cvNamedWindow("AVI", CV_WINDOW_AUTOSIZE);
    
	while(cvWaitKey(1))
	{
		cvGrabFrame(m_capture);
		curImage = cvRetrieveFrame(m_capture);

		if (curImage == NULL)
		{
			break;
		}
		
		cvShowImage("AVI",curImage);
	}
}

bool ImageSourceAVIFile::isImageAvailable()
{
	return m_existImage;
}

