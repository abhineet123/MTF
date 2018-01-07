#include "ImageSource.h"

ImageSource::ImageSource()
{
    curImage = NULL;
}


ImageSource::~ImageSource()
{
}

void ImageSource::getIplImage()
{
}

void ImageSource::getIplImage(const std::string& fileName)
{
}

void ImageSource::reloadIplImage()
{

}

Size ImageSource::getImageSize()
{
    Size imageSize;

    if (curImage != NULL)
    {
        imageSize.height = curImage->height;
    	imageSize.width = curImage->width;
    }
    else
    {
        imageSize.height = -1;
    	imageSize.width = -1;
    }

    return imageSize;
}


CvSize ImageSource::getImageCvSize()
{
    CvSize imageSize;

    if (curImage != NULL)
    {
        imageSize.height = curImage->height;
    	imageSize.width = curImage->width;
    }
    else
    {
        imageSize.height = -1;
    	imageSize.width = -1;
    }

    return imageSize;
}



bool ImageSource::isImageAvailable()
{
	if(curImage == NULL)
    {
		return false;
    }
	else
    {
		return true;
    }
}

const char* ImageSource::getFilename(int idx)
{
	return "";
}
