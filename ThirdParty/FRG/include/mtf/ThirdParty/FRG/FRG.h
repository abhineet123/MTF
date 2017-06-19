#ifndef MTF_FRG_H
#define MTF_FRG_H

#include "Fragments_Tracker.hpp"
#include "mtf/TrackerBase.h"
#include <memory>

struct FRGParams{
	enum class HistComparisonMetric{
		ChiSquare = 1, EMD, KolmogorovSmirnovVariation
	};
	int n_bins;
	int search_margin;
	HistComparisonMetric hist_cmp_metric;
	double resize_factor;
	bool show_window;

	FRGParams(int _n_bins,	int _search_margin,
		HistComparisonMetric _hist_cmp_metric, 
		double _resize_factor, bool _show_window);
	FRGParams(const FRGParams *params = nullptr);
};


class FRG : public mtf::TrackerBase{
public:
	typedef unique_ptr<frg::Fragments_Tracker> Tracker_;
	typedef FRGParams ParamType;

	FRG(const ParamType *_frg_params = nullptr);
	int inputType() const override{ return CV_8UC1; }
	void initialize(const cv::Mat& cv_corners) override;
	void update() override;
	void setImage(const cv::Mat &img)  override;
	void updateCVCorners();
	
private:
	ParamType params;
	frg::Parameters frg_params;

	Tracker_ frg_tracker;
	cv::Mat curr_img_uchar;
	cv::Mat curr_img_uchar_resized;
	CvMat input_img;
};

#endif
