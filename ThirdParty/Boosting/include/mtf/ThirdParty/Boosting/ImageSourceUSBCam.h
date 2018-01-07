#ifndef IMAGE_SOURCE_USB_CAM_H
#define IMAGE_SOURCE_USB_CAM_H

#include "ImageSource.h"
#include "cvcam.h"

class ImageSourceUSBCam :
	public ImageSource
{
public:

	ImageSourceUSBCam();
	virtual ~ImageSourceUSBCam();

	void getIplImage();
	void reloadIplImage();

	void viewLiveStream();

	bool isImageAvailable();
   virtual inline void reset() { };

protected:

	CvCapture* m_capture;
    IplImage *m_copyImg;
	bool m_existImage;
};

#endif //IMAGE_SOURCE_USB_CAM_H
