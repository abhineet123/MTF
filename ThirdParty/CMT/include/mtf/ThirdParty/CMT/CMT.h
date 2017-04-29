#ifndef MTF_CMT_H
#define MTF_CMT_H

#include "common.h"
#include "Consensus.h"
#include "Fusion.h"
#include "Matcher.h"
#include "Tracker.h"

#include <opencv2/features2d/features2d.hpp>
#include "opencv2/core/core.hpp"

#include "mtf/TrackerBase.h"

using cv::FeatureDetector;
using cv::DescriptorExtractor;
using cv::Ptr;
using cv::RotatedRect;
using cv::Size2f;

struct CMTParams{
	bool estimate_scale;
	bool estimate_rotation;
	string feat_detector;
	string desc_extractor;
	double resize_factor;
	CMTParams(bool estimate_scale, bool estimate_rotation,
		const char* feat_detector, const char* desc_extractor,
		double resize_factor);
	CMTParams(const CMTParams *params = nullptr);
};

namespace cmt{

	class CMT : public mtf::TrackerBase{
	public:
		typedef CMTParams ParamType;
		CMT(const CMTParams *cmt_params = nullptr);
		int inputType() const  override{ return CV_8UC1; }
		void initialize(const cv::Mat& cv_corners) override;
		void initialize(const Mat im_gray, const Rect rect);
		void update() override;
		void setImage(const cv::Mat &img)  override{ curr_img_gs = img; }
		void processFrame(const Mat im_gray);
		void updateCVCorners();

	private:
		ParamType params;

		Fusion fusion;
		Matcher matcher;
		Tracker tracker;
		Consensus consensus;

		string str_detector;
		string str_descriptor;

		vector<Point2f> points_active; //public for visualization purposes
		RotatedRect bb_rot;

		Mat curr_img_gs;
		Mat curr_img_gs_resized;
		Ptr<FeatureDetector> detector;
		Ptr<DescriptorExtractor> descriptor;
		Size2f size_initial;
		vector<int> classes_active;
		float theta;
		Mat im_prev;
	};
} /* namespace CMT */

#endif /* end of include guard: CMT_H */
