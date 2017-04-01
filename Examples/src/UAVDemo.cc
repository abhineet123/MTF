#include <mtf/SM/FCLK.h>
//! multi channel SSD
#include<mtf/AM/MCSSD.h>
#include <mtf/SSM/Translation.h>
#include <mtf/Tools/pipeline.h>
#include<mtf/Utilities/miscUtils.h>

int main(){
	mtf::SSMParams ssm_params(240, 240);
	mtf::TranslationParams trans_params(&ssm_params, 0);
	mtf::AMParams am_params(240, 240);
	mtf::FCLKParams sm_params;
	mtf::FCLK<mtf::MCSSD, mtf::Translation> uav_tracker(&sm_params, &am_params, &trans_params);
	mtf::MCSSD &uav_am = uav_tracker.getAM();	

	InputCV uav_input('j', "uav_sim");
	uav_input.initialize();

	cv::Mat satellite_img = cv::imread("./uav_sim/satellite_img.bmp");
	cv::Mat init_uav_location = mtf::utils::readTrackerLocation("./uav_sim/init_location.txt");

	GaussianSmoothing uav_pre_proc(uav_tracker.inputType());
	uav_pre_proc.initialize(uav_input.getFrame());
	GaussianSmoothing satellite_pre_proc(uav_tracker.inputType());
	satellite_pre_proc.initialize(satellite_img);

	uav_tracker.initialize(satellite_pre_proc.getFrame(), init_uav_location);

	cv::Mat uav_img_corners = mtf::utils::getFrameCorners(uav_input.getFrame());

	std::vector<cv::Point2d> uav_trajectory;
	uav_trajectory.push_back(mtf::utils::getCentroid(init_uav_location));

	cv::Mat curr_uav_location = init_uav_location.clone();

	while(uav_input.update()){		

		uav_pre_proc.update(uav_input.getFrame());
		uav_tracker.initialize(uav_pre_proc.getFrame(), uav_img_corners);

		uav_tracker.setRegion(curr_uav_location);
		uav_tracker.update(satellite_pre_proc.getFrame());		

		curr_uav_location = uav_tracker.getRegion().clone();

		cv::Point2d curr_centroid = mtf::utils::getCentroid(curr_uav_location);
		cv::line(satellite_img, curr_centroid, uav_trajectory.back(), CV_RGB(0, 255, 0), 3);

		cv::Mat satellite_img_annotated = satellite_img.clone();
		mtf::utils::drawRegion(satellite_img_annotated, uav_tracker.getRegion(), CV_RGB(255, 0, 0), 3);
		cv::Mat satellite_img_small(794, 720, satellite_img.type());
		cv::resize(satellite_img_annotated, satellite_img_small, cv::Size(satellite_img_small.cols, satellite_img_small.rows));

		cv::imshow("Satellite Image", satellite_img_small);
		cv::imshow("UAV Image", uav_input.getFrame());
		cv::imshow("Tracker Patch", mtf::utils::reshapePatch(uav_am.getCurrPixVals(),
			uav_am.getResY(), uav_am.getResX(), uav_am.getNChannels()));

		uav_trajectory.push_back(curr_centroid);

		if(cv::waitKey(1) == 27){ break; }
	}
	cv::destroyAllWindows();
	return 0;
}

