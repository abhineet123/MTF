//#include <mtf/SM/ESM.h>
//#include<mtf/AM/NCC.h>
//#include <mtf/SSM/Homography.h>
#include <mtf/mtf.h>
#include <mtf/pipeline.h>
#include <mtf/Utilities/miscUtils.h>

// typedef mtf::RKLT<mtf::NCC, mtf::Homography> TrackerType;
typedef mtf::CompositeSM<mtf::NCC, mtf::Homography> TrackerType;

int main(int argc, char * argv[]){
	//if (!readParams(argc, argv)){ return EXIT_FAILURE; }
	//enable_nt = 0;
	//ESM<NCC, Homography> tracker;
	// TrackerType *tracker = static_cast<TrackerType*>(mtf::getTracker("rklt", "ncc", "8", "0"));
  TrackerType *tracker = dynamic_cast<TrackerType*>(mtf::getTracker("nnic", "ncc", "8", "0"));
	//TrackerType *tracker = dynamic_cast<TrackerType*>(mtf::getTracker("nnic", "ncc", "8", "0"));
	//TrackerType *tracker = static_cast<TrackerType*>(mtf::getTracker("pffc", "ncc", "8", "0"));

	if (!tracker){
		printf("Dynamic casting failed.\n");
		return EXIT_FAILURE;
	}

	// mtf::Homography &ssm = tracker->templ_tracker->getSSM();
	mtf::Homography &ssm = tracker->getSSM();
	mtf::utils::InputCV input('u');
	mtf::utils::GaussianSmoothing pre_proc(tracker->inputType());
	input.initialize();
	pre_proc.initialize(input.getFrame());
	tracker->setImage(pre_proc.getFrame());
	mtf::utils::ObjUtils obj_utils;
	obj_utils.selectObjects(&input, 1);
	tracker->initialize(obj_utils.getObj().corners);
	while (input.update()){
		pre_proc.update(input.getFrame());
		tracker->update(pre_proc.getFrame());
		mtf::utils::printMatrix(ssm.getState(), "Homography Params");
		Matrix3d warp_mat;
		ssm.getWarpFromState(warp_mat, ssm.getState());
		mtf::utils::printMatrix(warp_mat, "Homography Matrix");
		mtf::utils::drawRegion(input.getFrame(mtf::utils::MUTABLE), tracker->getRegion());
		cv::imshow("Tracker Location", input.getFrame());
		if (cv::waitKey(1) == 27){ break; }
	}
	cv::destroyAllWindows();
	return 0;
}