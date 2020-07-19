#include <mtf/SM/ESM.h>
#include <mtf/AM/NCC.h>
#include <mtf/SSM/Homography.h>
#include <mtf/pipeline.h>
#include <mtf/Utilities/miscUtils.h>
//#include "mtf/mtf.h"

using namespace mtf;
using namespace mtf::utils;

int main(){
	ESM<NCC, Homography> tracker;
	InputParams input_params('u');
	InputCV input(&input_params);
	input.initialize();
	ObjUtils obj_utils;
	obj_utils.selectObjects(&input, 1);
	GaussianSmoothing pre_proc(tracker.inputType());
	pre_proc.initialize(input.getFrame());
	tracker.initialize(pre_proc.getFrame(), obj_utils.getObj().corners);
	while(input.update()){
		pre_proc.update(input.getFrame());
		tracker.update(pre_proc.getFrame());
		drawRegion(input.getFrame(MUTABLE), tracker.getRegion());
		cv::imshow("SimpleTrackingDemo", input.getFrame());
		if(cv::waitKey(1) == 27){ break; }
	}
	return 0;
}
