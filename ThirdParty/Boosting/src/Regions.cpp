#include "Regions.h"
#include <algorithm>
#include <math.h>

Color::Color()
{
	red = 255;
	blue = 255;
	green = 255;
}

Color::Color(int red, int green, int blue)
{
	this->red = red;
	this->green = green;
	this->blue = blue;
}

Color::Color(int idx)
{
	switch (idx%6)
	{
	case 0: red = 255; blue = 0; green = 0; break;
	case 1: red = 0; blue = 255; green = 0; break;
	case 2: red = 0; blue = 0; green = 255; break;
	case 3: red = 255; blue = 255; green = 0; break;
	case 4: red = 0; blue = 255; green = 255; break;
	case 5: red = 255; blue = 255; green = 255; break;
	default: red = 255; blue = 255; green = 255; break;
	}
}

Rect::Rect()
{
}

Rect::Rect (int upper, int left, int height, int width)
{
	this->upper = upper;
	this->left = left;
	this->height = height;
	this->width = width;
}

Rect Rect::operator+ (Point2D p)
{
	Rect r_tmp;
	r_tmp.upper = upper+p.row;
	r_tmp.left = left+p.col;
	r_tmp.height = height;
	r_tmp.width = width;
	return r_tmp;
}

Rect Rect::operator+ (Rect r)
{
	Rect r_tmp;
	r_tmp.upper = std::min(upper, r.upper);
	r_tmp.left = std::min(left, r.left);
	r_tmp.height = std::max(upper+height, r.upper + r.height) - r_tmp.upper;
	r_tmp.width = std::max(left+width, r.left+r.width) - r_tmp.left;
	return r_tmp;
}

Rect Rect::operator- (Point2D p)
{
	Rect r_tmp;
	r_tmp.upper = upper-p.row;
	r_tmp.left = left-p.col;
	r_tmp.height = height;
	r_tmp.width = width;
	return r_tmp;
}

Rect Rect::operator* (float f)
{
	Rect r_tmp;
	r_tmp.upper = (int)(upper-((float)height*f-height)/2);
	if (r_tmp.upper < 0) r_tmp.upper = 0;
	r_tmp.left = (int)(left-((float)width*f-width)/2);
	if (r_tmp.left < 0) r_tmp.left = 0;
	r_tmp.height = (int)(height*f);
	r_tmp.width = (int)(width*f);

	return r_tmp;
}


Rect Rect::operator = (Size s)
{
	height = s.height;
	width = s.width;
	upper = 0;
	left = 0;
	return *this;
}

Rect Rect::operator = (Rect r)
{
	height = r.height;
	width = r.width;
	upper = r.upper;
	left = r.left;
	return *this;
}

bool Rect::operator== (Rect r)
{	
	return ((r.width == width) && (r.height == height) && (r.upper == upper) && (r.left == left));
}

bool Rect::isValid (Rect validROI)
{
	return (upper >= validROI.upper) &&  (upper <= validROI.upper +validROI.height) &&
		(left >= validROI.left) && (left <= validROI.left +validROI.width) &&
		(upper+height >= validROI.upper) && (upper+height <=validROI.upper+validROI.height) &&
		(left+width >= validROI.left) && (left+width <= validROI.left +validROI.width );
	
}

int Rect::checkOverlap(Rect rect)
{	
	int x = (left > rect.left) ? left : rect.left;
	int y = (upper > rect.upper) ? upper : rect.upper;
	int w = (left+width > rect.left+rect.width) ? rect.left+rect.width-x : left+width-x;
	int h = (upper+height > rect.upper+rect.height) ? rect.upper+rect.height-y : upper+height-y;
	if (w > 0 && h > 0)
		return w*h;
	return 0;
}


bool Rect::isDetection(Rect eval, unsigned char *labeledImg, int imgWidth)
{
    bool isDet = false;
    unsigned char labelEval;
    unsigned char labelDet;

    labelEval = labeledImg[(eval.upper)*imgWidth+eval.left];
    labelDet = labeledImg[(upper)*imgWidth+left];

    if ((labelEval == labelDet) && (labelDet != 0))
    {
        isDet = true;
    }
    else
    {
        isDet = false;
    }

    return isDet;
}

CvRect Rect::getCvRect()
{
  return cvRect(left, upper, width, height);
}

Size::Size()
{
}

Size::Size(int height, int width)
{
	this->height = height;
	this->width = width;
}

Size Size::operator= (Rect r)
{
	height = r.height;
	width = r.width;
	return *this;
}

Size Size::operator= (Size s)
{
	height = s.height;
	width = s.width;
	return *this;
}

Size Size::operator* (float f)
{
	Size s_tmp;
	s_tmp.height = (int)(height*f);
	s_tmp.width = (int)(width *f);
	return s_tmp;
}


bool Size::operator== (Size s)
{	
	return ((s.width == width) && (s.height == height));
}

int Size::getArea()
{
	return height*width;
}

Point2D::Point2D ()
{
}

Point2D::Point2D (int row, int col)
{
	this->row = row;
	this->col = col;
}

Point2D Point2D::operator+ (Point2D p)
{
	Point2D p_tmp;
	p_tmp.col = col+p.col;
	p_tmp.row = row+p.row;
	return p_tmp;
}

Point2D Point2D::operator- (Point2D p)
{
	Point2D p_tmp;
	p_tmp.col = col-p.col;
	p_tmp.row = row-p.row;
	return p_tmp;
}

Point2D Point2D::operator= (Point2D p)
{
	row = p.row;
	col = p.col;
	return *this;
}

Point2D Point2D::operator= (Rect r)
{
	row = r.upper;
	col = r.left;
	return *this;
}