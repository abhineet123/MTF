#ifndef IMAGE_SOURCE_H
#define IMAGE_SOURCE_H

#include "OS_specific.h"
#include <string>
#include "highgui.h"
#include "cv.h"
#include "Regions.h"

class ImageSource
{
public:

	enum InputDevice {AVI, USB, DIRECTORY};

	IplImage *curImage;

	ImageSource();
	virtual ~ImageSource();

	virtual void getIplImage();
	virtual void getIplImage(const std::string& fileName);

	virtual void reloadIplImage();
	virtual const char* getFilename(int idx=-1);

	Size getImageSize();
	CvSize getImageCvSize();

	bool isImageAvailable();
	virtual void reset() = 0;

protected:

	char aviFilename[255 + 1];


};

#endif //IMAGE_SOURCE_H
