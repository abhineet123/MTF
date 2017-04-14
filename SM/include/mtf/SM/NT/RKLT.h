#ifndef MTF_RKLT_NT_H
#define MTF_RKLT_NT_H

#include "SearchMethod.h"
#include "mtf/SM/GridBase.h"
#include "mtf/SM/RKLTParams.h"

_MTF_BEGIN_NAMESPACE

namespace nt{
	class RKLT : public CompositeBase {

	public:
		typedef SearchMethod TemplTrackerType;

		typedef RKLTParams ParamType;

		RKLT(const ParamType *rklt_params,  GridBase *_grid_tracker, 
			TemplTrackerType *_templ_tracker);

		void initialize(const cv::Mat &corners) override;
		void update() override;
		
		using CompositeBase::initialize;
		using CompositeBase::update;
		
		void setImage(const cv::Mat &cv_img) override;
		void setRegion(const cv::Mat &corners) override;

		TemplTrackerType* getTemplateTracker(){ return templ_tracker; }
		GridBase* getGridTracker(){ return grid_tracker; }

	private:
		~RKLT();
		ParamType params;

		TemplTrackerType *templ_tracker;
		GridBase * grid_tracker;
		cv::Mat grid_corners_mat;

		int templ_resx, templ_resy;
		int grid_resx, grid_resy;
		int res_ratio_x, res_ratio_y;
		bool using_expanded_mask;
		vector<uchar*> expanded_mask;
	};
}
_MTF_END_NAMESPACE

#endif

