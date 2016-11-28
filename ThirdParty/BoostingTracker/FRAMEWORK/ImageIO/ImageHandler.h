#ifndef IMAGE_HANDLER_H
#define IMAGE_HANDLER_H

#include <iostream>
#include <string>

#include "cv.h"
#include "highgui.h"

#include "ImageSource.h"
#include "ImageSourceDir.h"
#include "ImageSourceAny.h"
#include "Regions.h"

class ImageHandler
{
public:

	ImageHandler(ImageSource* imgSrc);
	virtual ~ImageHandler(void);

	bool getImage();
	bool getImage(const std::string& fileName);
    
	void reloadImage();
	Size getImageSize();
    CvSize getImageCvSize();

	void viewImage(char* name = "", int autoReSize = CV_WINDOW_AUTOSIZE);
    void viewImage(char* name, int autoresize, int width, int height);
		
	void saveImage(char* filename);

	IplImage* getIplImage();
	IplImage* getIplGrayImage();

    unsigned char* getGrayImage();
    unsigned char* getR_Channel();
    unsigned char* getG_Channel();
    unsigned char* getB_Channel();

    double* getGrayImageDb();
    double* getGrayImageDb2();
    unsigned char* getRGBImage();

	void paintRectangle(CvRect rect, Color color = Color(255,255,0), int thickness = 1, bool filled = false);
    void paintRectangle(Rect rect, Color color = Color(255,255,0), int thickness = 1, bool filled = false);
	void paintCenter(Rect rect, Color color = Color(255,255,0), int thickness = 2);
	void paintLine(Point2D p1, Point2D p2, Color color = Color(255,255,0), int thickness = 2);
	void paintCircle(Point2D center, int radius, Color color = Color(255,255,0), int thickness = 1);
	void paintPoint(Point2D center, Color color = Color(255,255,0), int thickness = 1);
	void putTextOnImage(char* text, Point2D org, Color color = Color(255, 255, 0), float fontSize = 0.5);

	void saveROIofGrayImage(Rect rect, char* filename);
	unsigned char* getPatch(Rect rect);

	const char* getFilename(int idx=-1);

private:

 	ImageSource *m_imgSrc;
    char *m_windowName;

    IplImage *m_grayImage;
    IplImage *m_rgbImage;
};

#endif //IMAGE_HANDLER_H
