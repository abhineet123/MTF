#ifndef MTF_XV_COMMON_H
#define MTF_XV_COMMON_H

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#define NCORNERS 4
#define NCHANNELS 3

#define XV_RED 0
#define XV_GREEN 1
#define XV_BLUE 2

#define CV_RED 0
#define CV_GREEN 1
#define CV_BLUE 2

#define FNAME_SIZE 200

#define XV_ROTATE 'r'
#define XV_SCALING 'g'
#define XV_SE2 's'
#define XV_TRANS 't'
#define XV_AFFINE 'a'
#define XV_RT 'u'
#define XV_PYRAMID_ROTATE 'R'
#define XV_PYRAMID_AFFINE 'A'
#define XV_PYRAMID_SE2 'S'
#define XV_PYRAMID_TRANS 'T'
#define XV_PYRAMID_RT 'U'
#define XV_COLOR 'c'
#define XV_EDGE 'e'

#define XVISION_PIPELINE 'x'
#define OPENCV_PIPELINE 'c'

//using namespace cv;
using namespace std;

typedef float PIX_TYPE_GS;
typedef XV_RGB24 PIX_TYPE;

//typedef XV_TRGB24 PIX_TYPE;
//typedef XV_RGBA32 PIX_TYPE;

typedef XVImageRGB<PIX_TYPE> IMAGE_TYPE;
typedef XVImageScalar<PIX_TYPE_GS> IMAGE_TYPE_GS;
//typedef XVInteractWindowX< XV_RGB > WIN_INT;
typedef XV2Vec<double> XVPositionD;

#endif