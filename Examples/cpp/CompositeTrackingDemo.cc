#include <mtf/SM/ESM.h>
#include <mtf/AM/RSCV.h>
#include <mtf/SSM/Homography.h>
#include <mtf/SM/PF.h>
#include <mtf/AM/ZNCC.h>
#include <mtf/SSM/SL3.h>
#include <mtf/SM/CascadeTracker.h>
#include <mtf/pipeline.h>
#include <mtf/Utilities/miscUtils.h>

using namespace mtf;
using namespace mtf::utils;

int main(int argc, char * argv[]){
	if(!readParams(argc, argv)){ return EXIT_FAILURE; }
	enable_nt = 0;
	ESM<RSCV, Homography> tracker1;
	PF<ZNCC, SL3> tracker2;
	std::vector<TrackerBase*> trackers = { &tracker1, &tracker2 };
	CascadeTracker tracker(trackers);
	InputCV input('u');
	GaussianSmoothing pre_proc(tracker.inputType());
	input.initialize();
	pre_proc.initialize(input.getFrame());
	tracker.setImage(pre_proc.getFrame());
	ObjUtils obj_utils;
	obj_utils.selectObjects(&input, 1);
	tracker.initialize(obj_utils.getObj().corners);
	while(input.update()){
		pre_proc.update(input.getFrame());
		tracker.update(pre_proc.getFrame());
		utils::drawRegion(input.getFrame(MUTABLE), tracker.getRegion());
		cv::imshow("Tracker Location", input.getFrame());
		if(cv::waitKey(1) == 27){ break; }
	}
	cv::destroyAllWindows();
	return 0;
}
