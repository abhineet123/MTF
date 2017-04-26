#ifndef MTF_SAD_H
#define MTF_SAD_H

#include "AppearanceModel.h"

_MTF_BEGIN_NAMESPACE

struct SADDist : AMDist{
	typedef bool is_kdtree_distance;
	typedef double ElementType;
	typedef double ResultType;
	SADDist(const string &_name) : AMDist(_name){}
	double operator()(const double* a, const double* b,
		size_t size, double worst_dist = -1) const override;
	double accum_dist(const double& a, const double& b, int) const{
		return fabs(a - b);
	}
};

/**
Sum of Absolute Differences or L1 norm of raw pixel values
This is not differentiable so the derivative functions are not implemented
*/
class SAD : public AppearanceModel{

public:
	typedef AMParams ParamType;
	typedef SADDist DistType;

	SAD(const AMParams *am_params = nullptr,
		const int _n_channels = 1);
	void initializeSimilarity() override;

	double getLikelihood() const override;
	//-----------------------------------------------------------------------------------//
	//-------------------------------update functions------------------------------------//
	//-----------------------------------------------------------------------------------//
	void updateSimilarity(bool prereq_only = true) override;
	
	/**
	Support for FLANN library
	*/
	const DistType* getDistPtr() override{
		return new DistType(name);
	}
	void updateDistFeat(double* feat_addr) override{
		for(size_t pix = 0; pix < feat_size; ++pix) {
			*feat_addr++ = It(pix);
		}
	}
	void initializeDistFeat() override{}
	void updateDistFeat() override{}
	const double* getDistFeat() override{ return It.data(); }
	unsigned int getDistFeatSize() override{ return feat_size; }

protected:
	double likelihood_alpha;
	unsigned int feat_size;
};

_MTF_END_NAMESPACE

#endif