#include <mtf/SM/ESM.h>
#include<mtf/AM/MCNCC.h>
#include <mtf/SSM/Homography.h>
#include <mtf/pipeline.h>
#include <mtf/Utilities/miscUtils.h>

using namespace mtf;
int main(){
	ESM<MCNCC, Homography> tracker;
	mtf::utils::InputCV input('u');
	input.initialize();
	mtf::utils::ObjUtils obj_utils;
	obj_utils.selectObjects(&input, 1);
	tracker.initialize(input.getFrame(), obj_utils.getObj().corners);
	while(input.update()){
		tracker.update(input.getFrame());
		utils::drawRegion(input.getFrame(mtf::utils::MUTABLE), tracker.getRegion());
		cv::imshow("Tracker Location", input.getFrame());
		if(cv::waitKey(1) == 27){ break; }
	}
	return 0;
}
