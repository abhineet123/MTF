#include <mtf/SM/ESM.h>
#include<mtf/AM/MCNCC.h>
#include <mtf/SSM/Isometry.h>
#include <mtf/Tools/pipeline.h>
#include<mtf/Utilities/miscUtils.h>
#include<mtf/Utilities/imgUtils.h>

using namespace mtf;
int main(int argc, char * argv[]){
	if(!readParams(argc, argv)){ return EXIT_FAILURE; }
	Input_ input(getInput(pipeline));
	if(!input->initialize()){ return EXIT_FAILURE; }
	ESM<MCNCC, mtf::Isometry> tracker;
	cv::Mat init_mos_location(utils::Corners(cv::Rect_<double>(mos_border_width + mos_init_offset_x,
		mos_border_height + mos_init_offset_y, input->getWidth(), input->getHeight())).mat());
	cv::Mat mos_img(input->getHeight() + 2 * mos_border_height, input->getWidth() + 2 * mos_border_width, CV_8UC3);
	cv::Mat mask(mos_img.rows, mos_img.cols, CV_8UC1, cv::Scalar(0));
	utils::writePixelsToImage(mos_img, input->getFrame(), init_mos_location, 3, mask);
	NoFiltering pre_proc(mos_img, tracker.inputType());
	cv::Mat temp_img(input->getHeight() + 2 * mos_border_height, input->getWidth() + 2 * mos_border_width, CV_8UC3);
	cv::Mat mos_location = init_mos_location.clone();
	while(input->update()) {
		cv::Mat temp_mask;
		utils::writePixelsToImage(temp_img, input->getFrame(), mos_location, 3, temp_mask);
		tracker.initialize(pre_proc.getFrame(temp_img), mos_location);
		tracker.update(pre_proc.getFrame(mos_img));
		mos_location = tracker.getRegion().clone();
		utils::writePixelsToImage(mos_img, input->getFrame(), mos_location, 3, mask);	
		cv::Mat disp_mos_img(mos_disp_height, mos_disp_width, CV_8UC3);
		cv::resize(mos_img, disp_mos_img, disp_mos_img.size());
		cv::imshow("Mosaic", disp_mos_img);
		if(cv::waitKey(1) == 27){ break; }
	}
	cv::destroyAllWindows();
	return EXIT_SUCCESS;
}

