#include "mtf/ThirdParty/FRG/FRG.h"
#include <opencv2/imgproc/imgproc.hpp>
#include "mtf/Utilities/miscUtils.h"

#define FRG_N_BINS 16
#define FRG_SEARCH_MARGIN 7
#define FRG_HIST_CMP_METRIC FRGParams::HistComparisonMetric::KolmogorovSmirnovVariation
#define FRG_RESIZE_FACTOR 0.5
#define FRG_SHOW_WINDOW false

FRGParams::FRGParams(int _n_bins, int _search_margin,
	HistComparisonMetric _hist_cmp_metric, 
	double _resize_factor, bool _show_window) :
	n_bins(_n_bins),
	search_margin(_search_margin),
	hist_cmp_metric(_hist_cmp_metric),
	resize_factor(_resize_factor),
	show_window(_show_window){}

FRGParams::FRGParams(const FRGParams *params) :
n_bins(FRG_N_BINS),
search_margin(FRG_SEARCH_MARGIN),
hist_cmp_metric(FRG_HIST_CMP_METRIC),
resize_factor(FRG_RESIZE_FACTOR),
show_window(FRG_SHOW_WINDOW){
	if(params){
		n_bins = params->n_bins;
		search_margin = params->search_margin;
		hist_cmp_metric = params->hist_cmp_metric;
		resize_factor = params->resize_factor;
		show_window = params->show_window;
	}
}

FRG::FRG() : mtf::TrackerBase(){
	name = "frg";
}
FRG::FRG(const ParamType *_frg_params) : TrackerBase(),
params(_frg_params){
	name = "frg";
	printf("Using FRG tracker with:\n");
	printf("n_bins: %d\n", params.n_bins);
	printf("search_margin: %d\n", params.search_margin);
	printf("hist_cmp_metric: %d\n", params.hist_cmp_metric);
	printf("resize_factor: %f\n", params.resize_factor);
	printf("\n");
}

void FRG::initialize(const cv::Mat& cv_corners){
	frg_params.B = params.n_bins;
	frg_params.search_margin = params.search_margin;
	frg_params.metric_used = static_cast<int>(params.hist_cmp_metric);

	mtf::Rectd best_fit_rect = mtf::utils::getBestFitRectangle<double>(cv_corners,
		curr_img_uchar.cols, curr_img_uchar.rows);
	frg_params.initial_br_x = static_cast<int>((best_fit_rect.x + best_fit_rect.width)*params.resize_factor);
	frg_params.initial_br_y = static_cast<int>((best_fit_rect.y + best_fit_rect.height)*params.resize_factor);
	frg_params.initial_tl_x = static_cast<int>(best_fit_rect.x*params.resize_factor);
	frg_params.initial_tl_y = static_cast<int>(best_fit_rect.y*params.resize_factor);

	printf("best_fit_rect: x: %f y:%f width: %f height: %f\n",
		best_fit_rect.x, best_fit_rect.y, best_fit_rect.width, best_fit_rect.height);
	printf("frg_params: initial_br_x: %d initial_br_y: %d initial_tl_x: %d initial_tl_y: %d\n",
		frg_params.initial_br_x, frg_params.initial_br_y, 
		frg_params.initial_tl_x, frg_params.initial_tl_y);

	cv::resize(curr_img_uchar, curr_img_uchar_resized,
		cv::Size(curr_img_uchar_resized.cols, curr_img_uchar_resized.rows));

	frg_tracker.reset(new frg::Fragments_Tracker(&input_img, frg_params));

	cv_corners_mat.create(2, 4, CV_64FC1);

	updateCVCorners();
}
void FRG::update(){
	cv::resize(curr_img_uchar, curr_img_uchar_resized,
		cv::Size(curr_img_uchar_resized.cols, curr_img_uchar_resized.rows));
	if(params.show_window){
		frg_tracker->Handle_Frame(&input_img, "FRG");
	} else{
		frg_tracker->Handle_Frame(&input_img);
	}	
	updateCVCorners();
}
void FRG::setImage(const cv::Mat &img){
	curr_img_uchar = img;
	curr_img_uchar_resized.create(
		static_cast<int>(curr_img_uchar.rows*params.resize_factor),
		static_cast<int>(curr_img_uchar.cols*params.resize_factor), CV_8UC1);
	input_img = curr_img_uchar_resized;
}
void FRG::updateCVCorners(){
	cv_corners_mat.at<double>(0, 0) = cv_corners_mat.at<double>(0, 3) =
		static_cast<double>(frg_tracker->curr_template_tl_x) / params.resize_factor;
	cv_corners_mat.at<double>(1, 0) = cv_corners_mat.at<double>(1, 1) =
		static_cast<double>(frg_tracker->curr_template_tl_y) / params.resize_factor;
	cv_corners_mat.at<double>(0, 1) = cv_corners_mat.at<double>(0, 2) =
		static_cast<double>(frg_tracker->curr_template_tl_x + frg_tracker->curr_template_width) / params.resize_factor;
	cv_corners_mat.at<double>(1, 2) = cv_corners_mat.at<double>(1, 3) =
		static_cast<double>(frg_tracker->curr_template_tl_y + frg_tracker->curr_template_height) / params.resize_factor;
}


