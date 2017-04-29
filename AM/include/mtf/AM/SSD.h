#ifndef MTF_SSD_H
#define MTF_SSD_H

#include "SSDBase.h"

_MTF_BEGIN_NAMESPACE

struct SSDParams : AMParams{
	bool show_template;
	//! value constructor
	SSDParams(const AMParams *am_params,
		bool _show_template);
	//! default/copy constructor
	SSDParams(const SSDParams *params = nullptr);
};

struct SSDDist : SSDBaseDist{
	typedef double ElementType;
	typedef double ResultType;
	SSDDist(const string &_name, const bool _dist_from_likelihood,
		const double _likelihood_alpha, const unsigned int _patch_size);
	double operator()(const double* a, const double* b,
		size_t size, double worst_dist = -1) const override{
		double dist = SSDBaseDist::operator()(a, b, size, worst_dist);
		return dist_from_likelihood ?
			-exp(-likelihood_alpha * sqrt(dist / (static_cast<double>(patch_size)))) : dist;
	}
private:
	const bool dist_from_likelihood;
	const double likelihood_alpha;
	const unsigned int patch_size;
};

class SSD : public SSDBase{
public:

	typedef SSDParams ParamType;
	typedef SSDDist DistType;

	SSD(const ParamType *ssd_params = nullptr, const int _n_channels = 1);
	double getLikelihood() const override{
		return exp(-params.likelihood_alpha * sqrt(-f / (static_cast<double>(patch_size))));
	}
	void updateModel(const Matrix2Xd& curr_pts) override;

	/*Support for FLANN library*/
	const DistType* getDistFunc() override{
		return new DistType(name, params.dist_from_likelihood, 
			params.likelihood_alpha, patch_size);
	}

protected:
	ParamType params;

	bool use_running_avg;
	double old_pix_wt;

};

_MTF_END_NAMESPACE

#endif