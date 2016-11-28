#include "ImageHandler.h"


ImageHandler::ImageHandler(ImageSource* imgSrc)
{
    this->m_imgSrc = imgSrc;
    m_windowName = NULL;
    m_grayImage = NULL;
    m_rgbImage = NULL;
}


ImageHandler::~ImageHandler()
{
    if (m_windowName != NULL)
    {
        cvDestroyWindow(m_windowName);
    }

    if (m_grayImage != NULL)
    {
        cvReleaseImage(&m_grayImage);
    }
    
    if (m_rgbImage != NULL)
    {
        cvReleaseImage(&m_rgbImage);
    }
}


bool ImageHandler::getImage()
{
    m_imgSrc->getIplImage();

    // assign grayImage
    if (m_grayImage != NULL)
    {
        cvReleaseImage(&m_grayImage);
    }

    if (m_rgbImage != NULL)
    {
        cvReleaseImage(&m_rgbImage);
    }

    if (!m_imgSrc->isImageAvailable())
    {
        m_grayImage = NULL;
        m_rgbImage = NULL;
        return false;
    }

    if (m_imgSrc->curImage->nChannels > 1)
    {
        m_grayImage = cvCreateImage(cvGetSize(m_imgSrc->curImage), 8, 1);
        cvCvtColor(m_imgSrc->curImage, m_grayImage, CV_RGB2GRAY);
        m_rgbImage = cvCloneImage(m_imgSrc->curImage);
    }
    else
    {
        m_grayImage = cvCloneImage(m_imgSrc->curImage);
        m_rgbImage = NULL;
    }

    return true;
}


bool ImageHandler::getImage(const std::string& fileName)
{
    m_imgSrc->getIplImage(fileName);
	
    // assign grayImage
    if (m_grayImage != NULL)
    {
        cvReleaseImage(&m_grayImage);
    }

    if (m_rgbImage != NULL)
    {
        cvReleaseImage(&m_rgbImage);
    }

    if (!m_imgSrc->isImageAvailable())
    {
        m_grayImage = NULL;
        m_rgbImage = NULL;
        return false;
    }

    if (m_imgSrc->curImage->nChannels > 1)
    {
        m_grayImage = cvCreateImage(cvGetSize(m_imgSrc->curImage), 8, 1);
        cvCvtColor(m_imgSrc->curImage, m_grayImage, CV_RGB2GRAY);
        m_rgbImage = cvCloneImage(m_imgSrc->curImage);
    }
    else
    {
        m_grayImage = cvCloneImage(m_imgSrc->curImage);
        m_rgbImage = NULL;
    }

    return true;
}


const char* ImageHandler::getFilename(int idx)
{
	return m_imgSrc->getFilename(idx);
}

void ImageHandler::reloadImage()
{
	m_imgSrc->reloadIplImage();
}


void ImageHandler::viewImage(char* name, int autoresize)
{
    //cvNamedWindow(windowName, CV_WINDOW_AUTOSIZE);
    cvNamedWindow(name, autoresize);
    
    if (m_windowName == NULL)
    {
        m_windowName = new char[50];
        sprintf (m_windowName, "%s", name);
    }

    if (getImageSize().width < 75)
    {
        cvResizeWindow(name, 75, 75/getImageSize().width*getImageSize().height);
    }
	cvShowImage(name, m_imgSrc->curImage);
	
}
void ImageHandler::viewImage(char* name, int autoresize, int width, int height)
{
    cvNamedWindow(name, autoresize);

    if (m_windowName == NULL)
    {
        m_windowName = new char[50];
        sprintf (m_windowName, "%s", name);
    }


    cvResizeWindow(name, width, height);
    
    cvShowImage(name, m_imgSrc->curImage);
    
}



void ImageHandler::paintRectangle(CvRect rect, Color color, int thickness, bool filled)
{
	if(m_imgSrc->curImage != NULL)
	{
	    CvPoint pt1 = { rect.x, rect.y };
        CvPoint pt2 = { rect.x + rect.width, rect.y + rect.height };

		if(filled)
			cvRectangle(m_imgSrc->curImage, pt1, pt2, CV_RGB(color.red, color.green, color.blue), CV_FILLED);
		else
		    cvRectangle(m_imgSrc->curImage, pt1, pt2, CV_RGB(color.red, color.green, color.blue), thickness);
	}
}


void ImageHandler::paintRectangle(Rect rect, Color color, int thickness, bool filled)
{
	CvPoint pt1 = { rect.left, rect.upper };
    CvPoint pt2 = { rect.left + rect.width, rect.upper + rect.height };

    if(filled)
		cvRectangle(m_imgSrc->curImage, pt1, pt2, CV_RGB(color.red, color.green, color.blue), CV_FILLED);
	else
	    cvRectangle(m_imgSrc->curImage, pt1, pt2, CV_RGB(color.red, color.green, color.blue), thickness);
}

void ImageHandler::paintCenter(Rect rect, Color color, int thickness)
{
	CvPoint pt = { rect.left + floor(rect.width/2+0.5), rect.upper + floor(rect.height/2+0.5) };

    cvRectangle(m_imgSrc->curImage, pt, pt, CV_RGB(color.red, color.green, color.blue), thickness*2);
}

void ImageHandler::paintLine(Point2D p1, Point2D p2, Color color, int thickness)
{
	CvPoint pt1 = {p1.col, p1.row};
	CvPoint pt2 = {p2.col, p2.row};

	cvLine(m_imgSrc->curImage, pt1, pt2, CV_RGB(color.red, color.green, color.blue), thickness);
}

void ImageHandler::paintCircle(Point2D center, int radius, Color color, int thickness)
{
	CvPoint cvCenter = {center.col, center.row};
	cvCircle( m_imgSrc->curImage, cvCenter, radius, CV_RGB(color.red, color.green, color.blue), thickness);
}

void ImageHandler::paintPoint(Point2D center, Color color, int thickness)
{
	CvPoint pt = { center.col, center.row };
	cvRectangle(m_imgSrc->curImage, pt, pt, CV_RGB(color.red, color.green, color.blue), thickness*2);
}

void ImageHandler::saveImage(char* filename)
{
	if(m_imgSrc->curImage != NULL)
	{
		cvSaveImage(filename, m_imgSrc->curImage);
	}
}



Size ImageHandler::getImageSize()
{
	return m_imgSrc->getImageSize();
}


CvSize ImageHandler::getImageCvSize()
{
	return m_imgSrc->getImageCvSize();
}


IplImage* ImageHandler::getIplGrayImage()
{
    return m_grayImage;
}


IplImage* ImageHandler::getIplImage()
{
    if (m_imgSrc->curImage == NULL)
		return NULL;

	IplImage* get=cvCloneImage(m_imgSrc->curImage);

    return get;
}


unsigned char* ImageHandler::getGrayImage()
{
    if (m_imgSrc->curImage == NULL)
    {
        return NULL;
    }
    int rows = m_grayImage->height;
    int cols = m_grayImage->width;
    int iplCols= m_grayImage->widthStep;
    
    unsigned char *dataCh = new unsigned char[rows*cols];
    unsigned char *buffer = reinterpret_cast<unsigned char*>(m_grayImage->imageData);

	for(int i=0; i<rows; i++)
    {
        memcpy(dataCh+i*cols, buffer+i*iplCols, sizeof(unsigned char) * cols);
    }
    return dataCh;
}

unsigned char* ImageHandler::getB_Channel()
{
   if (m_imgSrc->curImage == NULL){return NULL;}
   int rows = m_rgbImage->height;
   int cols = m_rgbImage->width;
   int Area = rows*cols;
   int iplCols= m_rgbImage->widthStep;
   unsigned char *dataCh = new unsigned char[Area];
   unsigned char *buffer = reinterpret_cast<unsigned char*>(m_rgbImage->imageData);
   int DataPtr = 0;
   for (int x=0; x<rows; x++){
      for (int y=0; y<cols; y++){
         dataCh[DataPtr] = buffer[(x*iplCols) + (3*y)];
         DataPtr++;
      }
   }
   return dataCh;
}

unsigned char* ImageHandler::getG_Channel()
{
   if (m_imgSrc->curImage == NULL){return NULL;}
   int rows = m_rgbImage->height;
   int cols = m_rgbImage->width;
   int Area = rows*cols;
   int iplCols= m_rgbImage->widthStep;
   int Offset = 1; // green channel offset
   unsigned char *dataCh = new unsigned char[Area];
   unsigned char *buffer = reinterpret_cast<unsigned char*>(m_rgbImage->imageData);
   int DataPtr = 0;
   for (int x=0; x<rows; x++) {
      for (int y=0; y<cols; y++){
         dataCh[DataPtr] = buffer[(x*iplCols) + 3*y + Offset];
         DataPtr++;
      }
   }
   return dataCh;
}

unsigned char* ImageHandler::getR_Channel()
{
   if (m_imgSrc->curImage == NULL){return NULL;}
   int rows = m_rgbImage->height;
   int cols = m_rgbImage->width;
   int Area = rows*cols;
   int iplCols= m_rgbImage->widthStep;
   int Offset = 2; // red channel offset
   unsigned char *dataCh = new unsigned char[Area];
   unsigned char *buffer = reinterpret_cast<unsigned char*>(m_rgbImage->imageData);
   int DataPtr = 0;
   for (int x=0; x<rows; x++) {
      for (int y=0; y<cols; y++){
         dataCh[DataPtr] = buffer[(x*iplCols) + 3*y + Offset];
         DataPtr++;
      }
   }
   return dataCh;
}


double* ImageHandler::getGrayImageDb()
{
    if (m_imgSrc->curImage == NULL)
    {
        return NULL;
    }
    
    int rows = m_grayImage->height;
    int cols = m_grayImage->width;
    int iplCols= m_grayImage->widthStep;
    
    unsigned char *tmp = new unsigned char[rows*cols];
    double *data = new double[rows*cols];

    for(int i=0; i<rows; i++)
    {
        memcpy (tmp+i*cols, m_grayImage->imageData+i*iplCols, sizeof(char)*cols);
    }

    for(int i=0; i<rows*cols; i++)
    {
        data[i] = (double)tmp[i];
    }

    delete[] tmp;

    return data;
}


unsigned char* ImageHandler::getRGBImage()
{
    if (m_imgSrc->curImage == NULL)
    {
        return NULL;
    }

    if (m_imgSrc->curImage->nChannels == 1)
    {
        return NULL;
    }
    
    //unsigned char *dataCh;
    int rows, cols, iplCols;
    
    rows = m_imgSrc->curImage->height;
    cols = m_imgSrc->curImage->width;
    iplCols= m_imgSrc->curImage->widthStep;
    
    //dataCh = new unsigned char[rows*cols*3];
    unsigned char *dataCh = new unsigned char[rows*cols*3];

    cvGetRawData(m_imgSrc->curImage, &dataCh);

    return dataCh;
}

void ImageHandler::putTextOnImage(char* text, Point2D org, Color color, float fontSize)
{
	CvFont font;
	cvInitFont( &font, CV_FONT_VECTOR0, fontSize, fontSize, 0, 1, 20);
	cvPutText( m_imgSrc->curImage, text, cvPoint(org.col,org.row), &font, cvScalar(color.blue,color.green,color.red) );
}


void ImageHandler::saveROIofGrayImage(Rect rect, char* filename)
{
    int rowsImg = m_grayImage->height;
    int coslImg = m_grayImage->width;
    int iplColsImg = m_grayImage->widthStep;

    // create patch IPL image
    IplImage *grayROIImage = 
        cvCreateImage(cvSize(rect.width,rect.height), IPL_DEPTH_8U, 1);
    
    int rowsROI = grayROIImage->height;
    int colsROI = grayROIImage->width;
    int iplColsROI = grayROIImage->widthStep;
    

    // copy data to patch image
    int offset = (rect.upper)*iplColsImg+(rect.left);
    
    for(int i=0; i<rect.height; i++)
    {
        memcpy (grayROIImage->imageData+i*iplColsROI, 
            m_grayImage->imageData+i*iplColsImg+offset, sizeof(char)*colsROI);
    }
    
    cvSaveImage(filename,grayROIImage);
    
    cvReleaseImage(&grayROIImage);
}

unsigned char* ImageHandler::getPatch(Rect rect)
{
    int iplCols = m_grayImage->widthStep;
    int rows = rect.height;
    int cols = rect.width;

    // copy data to patch image
    unsigned char *patch = new unsigned char[rows*cols];
    int offset = rect.upper*iplCols+rect.left;
    
    for(int i=0; i<rect.height; i++)
    {
        memcpy (patch+i*cols, m_grayImage->imageData+i*iplCols+offset, sizeof(char)*cols);
    }

    return patch;
}