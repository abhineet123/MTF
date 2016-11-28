#include "mtf/ThirdParty/CMT/CMT.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "mtf/Utilities/miscUtils.h"

CMTParams::CMTParams(double estimate_scale, double estimate_rotation,
	char* feat_detector, char* desc_extractor, double resize_factor){
	this->estimate_scale = estimate_scale;
	this->estimate_rotation = estimate_rotation;
	this->feat_detector = feat_detector;
	this->desc_extractor = desc_extractor;
	this->resize_factor = resize_factor;
}

CMTParams::CMTParams(const CMTParams *params) : estimate_scale(true), estimate_rotation(false),
feat_detector("FAST"), desc_extractor("BRISK"), resize_factor(CMT_RESIZE_FACTOR){
	if(params){
		estimate_scale = params->estimate_scale;
		estimate_rotation = params->estimate_rotation;
		feat_detector = params->feat_detector;
		desc_extractor = params->desc_extractor;
		resize_factor = params->resize_factor;
	}
}

namespace cmt {
	CMT::CMT() : TrackerBase(), str_detector("FAST"), str_descriptor("BRISK"){
		name = "cmt";
	}
	CMT::CMT(const CMTParams *cmt_params) : TrackerBase(),
		params(cmt_params),
		str_detector("FAST"), str_descriptor("BRISK"){
		name = "cmt";
		consensus.estimate_scale = params.estimate_scale;
		consensus.estimate_rotation = params.estimate_rotation;
		str_detector = params.feat_detector;
		str_descriptor = params.desc_extractor;
		printf("Using CMT tracker with:\n");
		printf("estimate_scale: %d\n", consensus.estimate_scale);
		printf("estimate_rotation: %d\n", consensus.estimate_rotation);
		printf("str_detector: %s\n", str_detector.c_str());
		printf("str_descriptor: %s\n", str_descriptor.c_str());
		printf("resize_factor: %f\n", params.resize_factor);
		printf("\n");
	}

	void CMT::initialize(const cv::Mat& cv_corners){
		//double pos_x = (cv_corners.at<double>(0, 0) + cv_corners.at<double>(0, 1) +
		//	cv_corners.at<double>(0, 2) + cv_corners.at<double>(0, 3)) / 4;
		//double pos_y = (cv_corners.at<double>(1, 0) + cv_corners.at<double>(1, 1) +
		//	cv_corners.at<double>(1, 2) + cv_corners.at<double>(1, 3)) / 4;
		//cv::cvtColor(curr_img, curr_img_gs, CV_BGR2GRAY);
		cv::resize(curr_img_gs, curr_img_gs_resized, cv::Size(0, 0), 
			params.resize_factor, params.resize_factor);
		
		//processFrame(curr_img_gs_resized);

		//double pos_x = cv_corners.at<double>(0, 0);
		//double pos_y = cv_corners.at<double>(1, 0);
		//double size_x = ((cv_corners.at<double>(0, 1) - cv_corners.at<double>(0, 0)) +
		//	(cv_corners.at<double>(0, 2) - cv_corners.at<double>(0, 3))) / 2;
		//double size_y = ((cv_corners.at<double>(1, 3) - cv_corners.at<double>(1, 0)) +
		//	(cv_corners.at<double>(1, 2) - cv_corners.at<double>(1, 1))) / 2;

		mtf::Rectd best_fit_rect = mtf::utils::getBestFitRectangle<double>(cv_corners,
			curr_img_gs.cols, curr_img_gs.rows);

		cv::Rect resized_rect;
		resized_rect.x = best_fit_rect.x * params.resize_factor;
		resized_rect.y = best_fit_rect.y * params.resize_factor;
		resized_rect.width = best_fit_rect.width * params.resize_factor;
		resized_rect.height = best_fit_rect.height * params.resize_factor;
		//if(resized_rect.x == 0){
		//	resized_rect.x = 100;
		//}
		printf("best_fit_rect: x: %f y:%f width: %f height: %f\n",
			best_fit_rect.x, best_fit_rect.y, best_fit_rect.width, best_fit_rect.height);
		printf("resized_rect: x: %d y: %d width: %d height: %d\n",
			resized_rect.x, resized_rect.y, resized_rect.width, resized_rect.height);


		initialize(curr_img_gs_resized, resized_rect);
		cv_corners_mat.create(2, 4, CV_64FC1);
		updateCVCorners();
	}
	void CMT::update(){
		//cv::cvtColor(curr_img, curr_img_gs, CV_BGR2GRAY);
		cv::resize(curr_img_gs, curr_img_gs_resized, cv::Size(0, 0), params.resize_factor, params.resize_factor);
		processFrame(curr_img_gs_resized);
		updateCVCorners();
	}
	void CMT::updateCVCorners(){

		Point2f vertices[4];
		bb_rot.points(vertices);

		for(int corner_id = 0; corner_id < 4; corner_id++){
			cv_corners_mat.at<double>(0, corner_id) = vertices[(corner_id+1) % 4].x / params.resize_factor;
			cv_corners_mat.at<double>(1, corner_id) = vertices[(corner_id + 1) % 4].y / params.resize_factor;
		}
		//Rect rect = bb_rot.boundingRect();
		//double min_x = rect.x - rect.width / 2.0;
		//double max_x = rect.x + rect.width / 2.0;
		//double min_y = rect.y - rect.height / 2.0;
		//double max_y = rect.y + rect.height / 2.0;
		//cv_corners[0].x = min_x;
		//cv_corners[0].y = min_y;
		//cv_corners[1].x = max_x;
		//cv_corners[1].y = min_y;
		//cv_corners[2].x = max_x;
		//cv_corners[2].y = max_y;
		//cv_corners[3].x = min_x;
		//cv_corners[3].y = max_y;
	}
	void CMT::initialize(const Mat im_gray, const Rect rect)
	{
		//FILE_LOG(logDEBUG) << "CMT::initialize() call";

		//Remember initial size
		size_initial = rect.size();

		//Remember initial image
		im_prev = im_gray;

		//Compute center of rect
		Point2f center = Point2f(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);

		//Initialize rotated bounding box
		bb_rot = RotatedRect(center, size_initial, 0.0);

		//Initialize detector and descriptor
#if CV_MAJOR_VERSION > 2
		detector = cv::FastFeatureDetector::create();
		descriptor = cv::BRISK::create();
#else
		detector = FeatureDetector::create(str_detector);
		descriptor = DescriptorExtractor::create(str_descriptor);
#endif

		//Get initial keypoints in whole image and compute their descriptors
		vector<KeyPoint> keypoints;
		detector->detect(im_gray, keypoints);

		//Divide keypoints into foreground and background keypoints according to selection
		vector<KeyPoint> keypoints_fg;
		vector<KeyPoint> keypoints_bg;

		for(size_t i = 0; i < keypoints.size(); i++)
		{
			KeyPoint k = keypoints[i];
			Point2f pt = k.pt;

			if(pt.x > rect.x && pt.y > rect.y && pt.x < rect.br().x && pt.y < rect.br().y)
			{
				keypoints_fg.push_back(k);
			}

			else
			{
				keypoints_bg.push_back(k);
			}

		}

		//Create foreground classes
		vector<int> classes_fg;
		classes_fg.reserve(keypoints_fg.size());
		for(size_t i = 0; i < keypoints_fg.size(); i++)
		{
			classes_fg.push_back(i);
		}

		//Compute foreground/background features
		Mat descs_fg;
		Mat descs_bg;
		descriptor->compute(im_gray, keypoints_fg, descs_fg);
		descriptor->compute(im_gray, keypoints_bg, descs_bg);

		//Only now is the right time to convert keypoints to points, as compute() might remove some keypoints
		vector<Point2f> points_fg;
		vector<Point2f> points_bg;

		for(size_t i = 0; i < keypoints_fg.size(); i++)
		{
			points_fg.push_back(keypoints_fg[i].pt);
		}

		//FILE_LOG(logDEBUG) << points_fg.size() << " foreground points.";

		for(size_t i = 0; i < keypoints_bg.size(); i++)
		{
			points_bg.push_back(keypoints_bg[i].pt);
		}

		//Create normalized points
		vector<Point2f> points_normalized;
		for(size_t i = 0; i < points_fg.size(); i++)
		{
			points_normalized.push_back(points_fg[i] - center);
		}

		//Initialize matcher
		matcher.initialize(points_normalized, descs_fg, classes_fg, descs_bg, center);

		//Initialize consensus
		consensus.initialize(points_normalized);

		//Create initial set of active keypoints
		for(size_t i = 0; i < keypoints_fg.size(); i++)
		{
			points_active.push_back(keypoints_fg[i].pt);
			classes_active = classes_fg;
		}

		//FILE_LOG(logDEBUG) << "CMT::initialize() return";
	}

	void CMT::processFrame(Mat im_gray) {

		//FILE_LOG(logDEBUG) << "CMT::processFrame() call";

		//Track keypoints
		vector<Point2f> points_tracked;
		vector<unsigned char> status;
		tracker.track(im_prev, im_gray, points_active, points_tracked, status);

		//FILE_LOG(logDEBUG) << points_tracked.size() << " tracked points.";

		//keep only successful classes
		vector<int> classes_tracked;
		for(size_t i = 0; i < classes_active.size(); i++)
		{
			if(status[i])
			{
				classes_tracked.push_back(classes_active[i]);
			}

		}

		//Detect keypoints, compute descriptors
		vector<KeyPoint> keypoints;
		detector->detect(im_gray, keypoints);

		//FILE_LOG(logDEBUG) << keypoints.size() << " keypoints found.";

		Mat descriptors;
		descriptor->compute(im_gray, keypoints, descriptors);

		//Match keypoints globally
		vector<Point2f> points_matched_global;
		vector<int> classes_matched_global;
		matcher.matchGlobal(keypoints, descriptors, points_matched_global, classes_matched_global);

		//FILE_LOG(logDEBUG) << points_matched_global.size() << " points matched globally.";

		//Fuse tracked and globally matched points
		vector<Point2f> points_fused;
		vector<int> classes_fused;
		fusion.preferFirst(points_tracked, classes_tracked, points_matched_global, classes_matched_global,
			points_fused, classes_fused);

		//FILE_LOG(logDEBUG) << points_fused.size() << " points fused.";

		//Estimate scale and rotation from the fused points
		float scale;
		float rotation;
		consensus.estimateScaleRotation(points_fused, classes_fused, scale, rotation);

		//FILE_LOG(logDEBUG) << "scale " << scale << ", " << "rotation " << rotation;

		//Find inliers and the center of their votes
		Point2f center;
		vector<Point2f> points_inlier;
		vector<int> classes_inlier;
		consensus.findConsensus(points_fused, classes_fused, scale, rotation,
			center, points_inlier, classes_inlier);

		//FILE_LOG(logDEBUG) << points_inlier.size() << " inlier points.";
		//FILE_LOG(logDEBUG) << "center " << center;

		//Match keypoints locally
		vector<Point2f> points_matched_local;
		vector<int> classes_matched_local;
		matcher.matchLocal(keypoints, descriptors, center, scale, rotation, points_matched_local, classes_matched_local);

		//FILE_LOG(logDEBUG) << points_matched_local.size() << " points matched locally.";

		//Clear active points
		points_active.clear();
		classes_active.clear();

		//Fuse locally matched points and inliers
		fusion.preferFirst(points_matched_local, classes_matched_local, points_inlier, classes_inlier, points_active, classes_active);
		//    points_active = points_fused;
		//    classes_active = classes_fused;

		//FILE_LOG(logDEBUG) << points_active.size() << " final fused points.";

		//TODO: Use theta to suppress result
		bb_rot = RotatedRect(center, size_initial * scale, rotation / CV_PI * 180);

		//Remember current image
		im_prev = im_gray;

		//FILE_LOG(logDEBUG) << "CMT::processFrame() return";
	}

} /* namespace CMT */
