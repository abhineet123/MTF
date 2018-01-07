#include "ImageSourceUSBCam.h"

ImageSourceUSBCam::ImageSourceUSBCam(void)
{
	m_capture = cvCaptureFromCAM(-1);
	
	if (m_capture == NULL)
	{
		printf ("ERROR: failed to init USB cam!\n");
		m_existImage = false;
		return;
	}

	curImage = NULL;
   	m_copyImg = NULL;
	m_existImage = true;

	getIplImage();
	printf ("init USB cam (%d x %d)\n", m_copyImg->width, m_copyImg->height );
}


ImageSourceUSBCam::~ImageSourceUSBCam(void)
{
	if(m_capture!=NULL)
	{
		cvReleaseCapture(&m_capture);
	}

	curImage = NULL;
    cvReleaseImage(&m_copyImg);
}


void ImageSourceUSBCam::getIplImage()
{
	cvGrabFrame(m_capture);
	curImage = cvRetrieveFrame(m_capture);

	if (curImage->origin == 1)
    {
        cvFlip(curImage, curImage, 0);
        curImage->origin = 0;
    }

    if (m_copyImg != NULL)
    {
        cvReleaseImage(&m_copyImg);
    }

    m_copyImg = cvCloneImage(curImage);
}


void ImageSourceUSBCam::reloadIplImage()
{
	cvCopyImage (m_copyImg, curImage);
}


void ImageSourceUSBCam::viewLiveStream()
{
	cvNamedWindow("USB", CV_WINDOW_AUTOSIZE);
    
	while(cvWaitKey(1))
	{
		cvGrabFrame(m_capture);
		curImage = cvRetrieveFrame(m_capture);

		assert(curImage != NULL);
		cvShowImage("USB", curImage);
	}
}

bool ImageSourceUSBCam::isImageAvailable()
{
	return m_existImage;
}
