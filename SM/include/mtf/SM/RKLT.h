#ifndef MTF_RKLT_H
#define MTF_RKLT_H

#include "SearchMethod.h"
#include "GridBase.h"
#include "mtf/SM/RKLTParams.h"

_MTF_BEGIN_NAMESPACE

template<class AM, class SSM>
class RKLT : public CompositeBase {

public:
	typedef SearchMethod < AM, SSM > TemplTrackerType;
	typedef RKLTParams ParamType;

	RKLT(const ParamType *rklt_params,
		GridBase *_grid_tracker, TemplTrackerType *_templ_tracker);

	void initialize(const cv::Mat &corners) override;
	void update() override;

	using CompositeBase::initialize;
	using CompositeBase::update;

	void setImage(const cv::Mat &cv_img) override;
	void setRegion(const cv::Mat &corners) override;

private:
	ParamType params;

	TemplTrackerType *templ_tracker;
	GridBase * grid_tracker;
	cv::Mat grid_corners_mat;

	~RKLT();
};
_MTF_END_NAMESPACE

#endif

