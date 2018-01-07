#include "ClassifierThreshold.h"

ClassifierThreshold::ClassifierThreshold()
{
	m_posSamples = new EstimatedGaussDistribution();
	m_negSamples = new EstimatedGaussDistribution();
	m_threshold = 0.0f;
	m_parity = 0;
}

ClassifierThreshold::~ClassifierThreshold()
{
	if (m_posSamples!=NULL) delete m_posSamples;
	if (m_negSamples!=NULL) delete m_negSamples;
}

void* ClassifierThreshold::getDistribution (int target)
{
	if (target == 1)
		return m_posSamples;
	else
		return m_negSamples;
}

void ClassifierThreshold::update(float value, int target)
{
	//update distribution
	if (target == 1)
		m_posSamples->update(value); 
	else
		m_negSamples->update(value); 

	//adapt threshold and parity
	m_threshold = (m_posSamples->getMean()+m_negSamples->getMean())/2.0f;
	m_parity = (m_posSamples->getMean() > m_negSamples->getMean()) ? 1 : -1;
}

int ClassifierThreshold::eval(float value)
{
	return (((m_parity*(value-m_threshold))>0) ? 1 : -1);
}