#include <mtf/SM/ESM.h>
#include <mtf/AM/RSCV.h>
#include <mtf/SSM/Similitude.h>
#include <mtf/SM/ICLK.h>
#include <mtf/AM/ZNCC.h>
#include <mtf/SSM/Affine.h>
#include <mtf/SM/CascadeTracker.h>
#include <mtf/pipeline.h>
#include <mtf/Utilities/miscUtils.h>

using namespace mtf;
using namespace mtf::utils;

int main(int argc, char * argv[]){
	if(!readParams(argc, argv)){ return EXIT_FAILURE; }
	enable_nt = 0;
	ESM<RSCV, Similitude> tracker1;
	ICLK<ZNCC, mtf::Affine> tracker2;
	CascadeTracker tracker(std::vector<TrackerBase*>{&tracker1, & tracker2});

	InputParams input_params('u');
	InputCV input(&input_params);
	GaussianSmoothing pre_proc(tracker.inputType());

	input.initialize();
	pre_proc.initialize(input.getFrame());

	ObjUtils obj_utils;
	obj_utils.selectObjects(&input, 1);

	tracker.initialize(pre_proc.getFrame(), obj_utils.getObj().corners);

	while(input.update()){

		pre_proc.update(input.getFrame());
		tracker.update(pre_proc.getFrame());

		utils::drawRegion(input.getFrame(MUTABLE), tracker.getRegion());
		cv::imshow("CompositeTrackingDemo", input.getFrame());
		if(cv::waitKey(1) == 27){ break; }
	}
	cv::destroyAllWindows();
	return 0;
}
