#pragma once

#include <assert.h>
#include <math.h>
#include "Regions.h"
#include <deque>

using namespace std;

class EstimatedGaussDistribution  
{
public:

	EstimatedGaussDistribution();
	EstimatedGaussDistribution(float P_mean, float R_mean, float P_sigma, float R_sigma);
	virtual ~EstimatedGaussDistribution();

	void update(float value); //, float timeConstant = -1.0);

	float getMean(){return m_mean;};
	float getSigma(){return m_sigma;};
	void setValues(float mean, float sigma);

private:

	float m_mean;
	float m_sigma;
	float m_P_mean;
	float m_P_sigma;
	float m_R_mean;
	float m_R_sigma;
};

