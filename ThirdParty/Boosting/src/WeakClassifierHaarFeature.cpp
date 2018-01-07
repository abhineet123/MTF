#include "WeakClassifierHaarFeature.h"

WeakClassifierHaarFeature::WeakClassifierHaarFeature(Size patchSize)
{
	m_feature = new FeatureHaar(patchSize);
	generateRandomClassifier();
	m_feature->getInitialDistribution((EstimatedGaussDistribution*) m_classifier->getDistribution(-1));
	m_feature->getInitialDistribution((EstimatedGaussDistribution*) m_classifier->getDistribution(1));
}

void WeakClassifierHaarFeature::resetPosDist()
{
	m_feature->getInitialDistribution((EstimatedGaussDistribution*) m_classifier->getDistribution(1));
	m_feature->getInitialDistribution((EstimatedGaussDistribution*) m_classifier->getDistribution(-1));
}

WeakClassifierHaarFeature::~WeakClassifierHaarFeature()
{
	delete m_classifier;
	delete m_feature;

}

void WeakClassifierHaarFeature::generateRandomClassifier()
{
	m_classifier = new ClassifierThreshold();
}

bool WeakClassifierHaarFeature::update(ImageRepresentation *image, Rect ROI, int target) 
{
	float value;
	
	bool valid = m_feature->eval (image, ROI, &value); 
	if (!valid)
		return true;

	m_classifier->update(value, target);
	return (m_classifier->eval(value) != target);
}

int WeakClassifierHaarFeature::eval(ImageRepresentation *image, Rect ROI) 
{
	float value;
	bool valid = m_feature->eval(image, ROI, &value); 
	if (!valid)
		return 0;

	return m_classifier->eval(value);
}

float WeakClassifierHaarFeature::getValue(ImageRepresentation *image, Rect ROI)
{
	float value;
	bool valid = m_feature->eval (image, ROI, &value);
	if (!valid)
		return 0;

	return value;
}

EstimatedGaussDistribution* WeakClassifierHaarFeature::getPosDistribution()
{
  return static_cast<EstimatedGaussDistribution*>(m_classifier->getDistribution(1));
}


EstimatedGaussDistribution* WeakClassifierHaarFeature::getNegDistribution()
{
  return static_cast<EstimatedGaussDistribution*>(m_classifier->getDistribution(-1));
}