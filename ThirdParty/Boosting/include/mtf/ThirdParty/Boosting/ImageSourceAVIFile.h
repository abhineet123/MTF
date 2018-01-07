#ifndef IMAGE_SOURCE_AVI_FILE_H
#define IMAGE_SOURCE_AVI_FILE_H

#include "ImageSource.h"
#include "cxcore.h"

class ImageSourceAVIFile : public ImageSource
{
public:

	ImageSourceAVIFile(const char* aviFilename);
	virtual ~ImageSourceAVIFile();

	void getIplImage();
	void reloadIplImage();

	void viewAVI();

	bool isImageAvailable();
    IplImage* getImage( const double index );
    double getNumFrames() { return cvGetCaptureProperty(m_capture, CV_CAP_PROP_FRAME_COUNT); };
	double getFrameRate() { return cvGetCaptureProperty(m_capture, CV_CAP_PROP_FPS); };
    double getCurrentFrameIndex() { return cvGetCaptureProperty(m_capture, CV_CAP_PROP_POS_FRAMES) - 1; };
    virtual inline void reset() {  if (m_capture) cvSetCaptureProperty(m_capture, CV_CAP_PROP_POS_FRAMES, 0); };

protected:

	CvCapture* m_capture;
	bool m_existImage;
	IplImage *m_copyImg;
};

#endif //IMAGE_SOURCE_AVI_FILE_H
