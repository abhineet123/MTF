#ifndef IMAGE_SOURCE_ANY
#define IMAGE_SOURCE_ANY

#include "ImageSource.h"

class ImageSourceAny : public ImageSource
{
public:

    ImageSourceAny();
    ImageSourceAny(unsigned char **src, CvSize imageSize, int depth);
    ImageSourceAny(IplImage **src);
    ImageSourceAny(double **src, CvSize imageSize, int depth);
    ImageSourceAny(char **src, CvSize imageSize, int depth);
    ~ImageSourceAny();

    void getIplImage();

	void setData(IplImage *newData);
    void setData(char *newData, CvSize imageSize, int depth);
    void setData(unsigned char *newData, CvSize imageSize, int depth);
    void setData(double *newData, CvSize imageSize, int depth);

    virtual inline void reset() { }; 

private:

    double **m_dbSrc;
    IplImage **m_iplSrc;
    unsigned char **m_ucSrc; 
    unsigned char *m_tmp;
    int m_mode;
    CvSize m_imageSize;
    int m_depth;
};

#endif // IMAGE_SOURCE_ANY
