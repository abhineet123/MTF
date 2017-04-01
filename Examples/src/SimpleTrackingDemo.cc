#include <mtf/SM/ESM.h>
#include<mtf/AM/MCNCC.h>
#include <mtf/SSM/Homography.h>
#include <mtf/Tools/pipeline.h>
#include <mtf/Tools/cvUtils.h>
#include <mtf/Utilities/miscUtils.h>

using namespace mtf;
int main(){
	ESM<MCNCC, Homography> tracker;
	InputCV input('u');
	input.initialize();
	CVUtils cv_utils;
	cv_utils.selectObjects(&input, 1);
	tracker.initialize(input.getFrame(), cv_utils.getObj().corners);
	while(input.update()){
		tracker.update(input.getFrame());
		utils::drawRegion(input.getFrame(MUTABLE), tracker.getRegion());
		cv::imshow("Tracker Location", input.getFrame());
		if(cv::waitKey(1) == 27){ break; }
	}
	return 0;
}
