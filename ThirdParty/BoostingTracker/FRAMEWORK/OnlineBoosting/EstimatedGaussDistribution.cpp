#include "EstimatedGaussDistribution.h"
#include <cmath>

EstimatedGaussDistribution::EstimatedGaussDistribution()
{
	m_mean = 0;
	m_sigma = 1;
	this->m_P_mean = 1000;
	this->m_R_mean = 0.01f;
	this->m_P_sigma = 1000;
	this->m_R_sigma = 0.01f;
}

EstimatedGaussDistribution::EstimatedGaussDistribution(float P_mean, float R_mean, float P_sigma, float R_sigma)
{
	m_mean = 0;
	m_sigma = 1;
	this->m_P_mean = P_mean;
	this->m_R_mean = R_mean;
	this->m_P_sigma = P_sigma;
	this->m_R_sigma = R_sigma;
}


EstimatedGaussDistribution::~EstimatedGaussDistribution()
{
}

void EstimatedGaussDistribution::update(float value) 
{
	//update distribution (mean and sigma) using a kalman filter for each

	float K;
	float minFactor = 0.001f; 

	//mean

	K = m_P_mean/(m_P_mean+m_R_mean);
	if (K < minFactor) K = minFactor;

	m_mean = K*value + (1.0f-K)*m_mean;
	m_P_mean = m_P_mean*m_R_mean/(m_P_mean+m_R_mean);


	K = m_P_sigma/(m_P_sigma+m_R_sigma);
	if (K < minFactor) K = minFactor;

	float tmp_sigma = K*(m_mean-value)*(m_mean-value) + (1.0f-K)*m_sigma*m_sigma;
	m_P_sigma = m_P_sigma*m_R_mean/(m_P_sigma+m_R_sigma);

	m_sigma = static_cast<float>( sqrt(tmp_sigma) );
	if (m_sigma <= 1.0f) m_sigma = 1.0f;

}

void EstimatedGaussDistribution::setValues(float mean, float sigma)
{
	this->m_mean = mean;
	this->m_sigma = sigma;
}
