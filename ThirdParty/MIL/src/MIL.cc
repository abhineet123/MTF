#include "mtf/ThirdParty/MIL/MIL.h"
#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/excpUtils.h"

#define MIL_ALGORITHM mtf::MILTrackerParams::CV_ONLINEMIL
#define MIL_NUM_CLASSIFIERS 100
#define MIL_OVERLAP 0.99f
#define MIL_SEARCH_FACTOR 2.0f
#define MIL_POS_RADIUS_TRAIN 4.0f
#define MIL_NEG_NUM_TRAIN 65
#define MIL_NUM_FEATURES 250
//! minimum distance of the initializing bounding box from the image corners
//! using no border causes the tracker to segfault on occasions
#define MIL_INIT_BORDER_SIZE 10

MILParams::MILParams(int _algorithm, int _num_classifiers,
	float _overlap, float _search_factor,
	float _pos_radius_train, int _neg_num_train,
	int _num_features) :
	_params(_algorithm, _num_classifiers,
	_overlap, _search_factor, _pos_radius_train,
	_neg_num_train, _num_features){}

MILParams::MILParams(const MILParams *params) :
_params(MIL_ALGORITHM, MIL_NUM_CLASSIFIERS,
MIL_OVERLAP, MIL_SEARCH_FACTOR, MIL_POS_RADIUS_TRAIN,
MIL_NEG_NUM_TRAIN, MIL_NUM_FEATURES){
	if(params){
		_params.algorithm_ = params->get().algorithm_;
		_params.num_classifiers_ = params->get().num_classifiers_;
		_params.overlap_ = params->get().overlap_;
		_params.search_factor_ = params->get().search_factor_;
		_params.pos_radius_train_ = params->get().pos_radius_train_;
		_params.neg_num_train_ = params->get().neg_num_train_;
		_params.num_features_ = params->get().num_features_;
	}
}

MIL::MIL(const ParamType *mil_params) :
params(mil_params),
tracker(nullptr){
	name = "mil";
	printf("Using MIL tracker with:\n");
	printf("algorithm: %d\n", params.get().algorithm_);
	printf("num_classifiers: %d\n", params.get().num_classifiers_);
	printf("overlap: %f\n", params.get().overlap_);
	printf("search_factor: %f\n", params.get().search_factor_);
	printf("pos_radius_train: %f\n", params.get().pos_radius_train_);
	printf("neg_num_train: %d\n", params.get().neg_num_train_);
	printf("num_features: %d\n", params.get().num_features_);
	printf("\n");
	cv_corners_mat.create(2, 4, CV_64FC1);

	tracker = new mtf::MILTracker(params.get());
}
void MIL::initialize(const cv::Mat& corners){
	curr_location = mtf::utils::getBestFitRectangle<int>(corners,
		curr_img.cols, curr_img.rows, MIL_INIT_BORDER_SIZE);
	printf("best_fit_rect: x: %d y: %d width: %d height: %d\n",
		curr_location.x, curr_location.y, curr_location.width, curr_location.height);
	init_bb = cvRect(curr_location.x, curr_location.y, curr_location.width, curr_location.height);
	printf("Using MIL tracker at: %d, %d with region of size %dx%d\n",
		curr_location.x, curr_location.y, curr_location.width, curr_location.height);
	if(!tracker->initialize(curr_img, curr_location)){
		throw mtf::utils::InvalidTrackerState("MIL::Tracker could not be initialized successfully\n");
	}
	updateCVCorners();
}
void MIL::update(){
	if(!tracker->update(curr_img, curr_location)){
		throw mtf::utils::InvalidTrackerState("MIL::Tracker could not be updated successfully\n");
	}
	updateCVCorners();
}
void MIL::updateCVCorners(){
	cv_corners_mat.at<double>(0, 0) = cv_corners_mat.at<double>(0, 3) = curr_location.x;
	cv_corners_mat.at<double>(1, 0) = cv_corners_mat.at<double>(1, 1) = curr_location.y;
	cv_corners_mat.at<double>(0, 1) = cv_corners_mat.at<double>(0, 2) = curr_location.x + curr_location.width;
	cv_corners_mat.at<double>(1, 2) = cv_corners_mat.at<double>(1, 3) = curr_location.y + curr_location.height;
}

