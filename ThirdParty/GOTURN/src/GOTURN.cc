#include "mtf/ThirdParty/GOTURN/GOTURN.h"
#include "mtf/Utilities/miscUtils.h"

GOTURNParams::GOTURNParams(bool _do_train, int _gpu_id,
	bool _show_intermediate_output, const char*_model_file,
	const char* _trained_file) :
	do_train(_do_train),
	gpu_id(_gpu_id),
	show_intermediate_output(_show_intermediate_output),
	model_file(_model_file),
	trained_file(_trained_file){}

GOTURNParams::GOTURNParams(const GOTURNParams *params) :
do_train(GOTURN_DO_TRAIN),
gpu_id(GOTURN_GPU_ID),
show_intermediate_output(GOTURN_SHOW_INTERMEDIATE_OUTPUT),
model_file(GOTURN_MODEL_FILE),
trained_file(GOTURN_TRAINED_FILE){
	if(params){
		do_train = params->do_train;
		gpu_id = params->gpu_id;
		show_intermediate_output = params->show_intermediate_output;
		model_file = params->model_file;
		trained_file = params->trained_file;
	}
}

GOTURN::GOTURN(const GOTURNParams *cmt_params) : TrackerBase(),
params(cmt_params), tracker(nullptr), regressor(nullptr){
	name = "goturn";

	printf("Using GOTURN tracker with:\n");
	printf("do_train: %d\n", params.do_train);
	printf("gpu_id: %d\n", params.gpu_id);
	printf("show_intermediate_output: %d\n", params.show_intermediate_output);
	printf("model_file: %s\n", params.model_file.c_str());
	printf("trained_file: %s\n", params.trained_file.c_str());
	printf("\n");

	regressor = new Regressor(params.model_file, params.trained_file,
		params.gpu_id, params.do_train);
	tracker = new Tracker(params.show_intermediate_output);
	bounding_box_vec.resize(4);
	cv_corners_mat.create(2, 4, CV_64FC1);
}

GOTURN::~GOTURN(){
	if(regressor){ delete(regressor); }
	if(tracker){ delete(tracker); }
}

void GOTURN::initialize(const cv::Mat& cv_corners){

	srandom(time(NULL));

	mtf::Rectd best_fit_rect = mtf::utils::getBestFitRectangle<double>(cv_corners,
		curr_img.cols, curr_img.rows);

	printf("best_fit_rect: x: %f y:%f width: %f height: %f\n",
		best_fit_rect.x, best_fit_rect.y, best_fit_rect.width, best_fit_rect.height);

	bounding_box_vec[0] = best_fit_rect.x;
	bounding_box_vec[1] = best_fit_rect.y;
	bounding_box_vec[2] = best_fit_rect.x + best_fit_rect.width;
	bounding_box_vec[3] = best_fit_rect.y + best_fit_rect.height;

	tracker->Init(curr_img, bounding_box_vec, regressor);
	//cv::imshow("GOTURN", curr_img);
	updateCVCorners();
}
void GOTURN::update(){
	tracker->Track(curr_img, regressor, &bbox_estimate);
	bounding_box_vec[0] = bbox_estimate.x1_;
	bounding_box_vec[1] = bbox_estimate.y1_;
	bounding_box_vec[2] = bbox_estimate.x2_;
	bounding_box_vec[3] = bbox_estimate.y2_;
	updateCVCorners();
	//cv::imshow("GOTURN", curr_img);
}
void GOTURN::updateCVCorners(){

	double min_x = bounding_box_vec[0];
	double min_y = bounding_box_vec[1];
	double max_x = bounding_box_vec[2];
	double max_y = bounding_box_vec[3];

	cv_corners_mat.at<double>(0, 0) = min_x;
	cv_corners_mat.at<double>(1, 0) = min_y;
	cv_corners_mat.at<double>(0, 1) = max_x;
	cv_corners_mat.at<double>(1, 1) = min_y;
	cv_corners_mat.at<double>(0, 2) = max_x;
	cv_corners_mat.at<double>(1, 2) = max_y;
	cv_corners_mat.at<double>(0, 3) = min_x;
	cv_corners_mat.at<double>(1, 3) = max_y;
}
