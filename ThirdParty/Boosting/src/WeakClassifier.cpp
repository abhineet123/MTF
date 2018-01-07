#include "WeakClassifier.h"

WeakClassifier::WeakClassifier()
{
}

WeakClassifier::~WeakClassifier()
{
}


bool WeakClassifier::update(ImageRepresentation* image, Rect ROI, int target) 
{
	return true;
}

int WeakClassifier::eval(ImageRepresentation* image, Rect  ROI) 
{
	return 0;
}

int WeakClassifier::getType ()
{
	return 0;
}

float WeakClassifier::getValue (ImageRepresentation* image, Rect  ROI)
{
	return 0;
}
