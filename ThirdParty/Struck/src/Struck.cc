#include "mtf/ThirdParty/Struck/Struck.h"
#include "mtf/Utilities/miscUtils.h"
#include "opencv2/imgproc/imgproc.hpp"

namespace struck{
	StruckParams::StruckParams(std::string _config_path) :
		config_path(_config_path){}
	StruckParams::StruckParams(const StruckParams *params) :
		config_path(STRUCK_CONFIG_PATH){
		if(params){
			config_path = params->config_path;
		}
	}
	Struck::Struck() :
		conf(params.config_path),
		tracker(conf),
		scaleW(1),
		scaleH(1){
		name = "struck";
	}

	Struck::Struck(const ParamType *struck_params) :
		params(struck_params),
		conf(params.config_path),
		tracker(conf),
		scaleW(1),
		scaleH(1){
		name = "struck";
		printf("Using Struck tracker with:\n");
		std::cout << conf;
	}
	void Struck::setImage(const cv::Mat &img){
		curr_img = img;
		scaleW = (float)conf.frameWidth / curr_img.cols;
		scaleH = (float)conf.frameHeight / curr_img.rows;
	}
	void Struck::initialize(const cv::Mat& corners){
		//double pos_x = corners.at<double>(0, 0);
		//double pos_y = corners.at<double>(1, 0);
		//double size_x = ((corners.at<double>(0, 1) - corners.at<double>(0, 0)) +
		//	(corners.at<double>(0, 2) - corners.at<double>(0, 3))) / 2;
		//double size_y = ((corners.at<double>(1, 3) - corners.at<double>(1, 0)) +
		//	(corners.at<double>(1, 2) - corners.at<double>(1, 1))) / 2;

		cv::resize(curr_img, curr_img_resized, cv::Size(conf.frameWidth, conf.frameHeight));

		mtf::Rectd best_fit_rect = mtf::utils::getBestFitRectangle<double>(corners,
			curr_img.cols, curr_img.rows);
		printf("best_fit_rect: x: %f y:%f width: %f height: %f\n",
			best_fit_rect.x, best_fit_rect.y, best_fit_rect.width, best_fit_rect.height);

		FloatRect init_bb = FloatRect(best_fit_rect.x*scaleW, best_fit_rect.y*scaleH, 
			best_fit_rect.width*scaleW, best_fit_rect.height*scaleH);

		printf("init_bb: xXMin %f YMin:%f width: %f height: %f\n",
			init_bb.XMin(), init_bb.YMin(), init_bb.Width(), init_bb.Height());

		tracker.Initialise(curr_img_resized, init_bb);
		float x_min = init_bb.XMin() / scaleW, y_min = init_bb.YMin() / scaleH;
		float width = init_bb.Width() / scaleW, height = init_bb.Height() / scaleH;

		cv_corners_mat.create(2, 4, CV_64FC1);
		cv_corners_mat.at<double>(0, 0) = cv_corners_mat.at<double>(0, 3) = x_min;
		cv_corners_mat.at<double>(1, 0) = cv_corners_mat.at<double>(1, 1) = y_min;
		cv_corners_mat.at<double>(0, 1) = cv_corners_mat.at<double>(0, 2) = x_min + width;
		cv_corners_mat.at<double>(1, 2) = cv_corners_mat.at<double>(1, 3) = y_min + height;
	}
	void Struck::update(){
		cv::resize(curr_img, curr_img_resized, cv::Size(conf.frameWidth, conf.frameHeight));
		tracker.Track(curr_img_resized);
		updateCVCorners();
	}
	void Struck::updateCVCorners(){
		FloatRect curr_bb = tracker.GetBB();
		float x_min = curr_bb.XMin() / scaleW, y_min = curr_bb.YMin() / scaleH;
		float width = curr_bb.Width() / scaleW, height = curr_bb.Height() / scaleH;
		cv_corners_mat.at<double>(0, 0) = cv_corners_mat.at<double>(0, 3) = x_min;
		cv_corners_mat.at<double>(1, 0) = cv_corners_mat.at<double>(1, 1) = y_min;
		cv_corners_mat.at<double>(0, 1) = cv_corners_mat.at<double>(0, 2) = x_min + width;
		cv_corners_mat.at<double>(1, 2) = cv_corners_mat.at<double>(1, 3) = y_min + height;
	}
}

