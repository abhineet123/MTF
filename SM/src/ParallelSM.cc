#include "mtf/SM/ParallelSM.h"
#ifdef ENABLE_TBB
#include "tbb/tbb.h" 
#endif
#include "mtf/Utilities/miscUtils.h"

_MTF_BEGIN_NAMESPACE

template<class AM, class SSM>
ParallelSM<AM, SSM>::ParallelSM(const vector<SM*> _trackers,
	const ParamType *parl_params,
	const SSMParams *ssm_params) :
	CompositeSM<AM, SSM>(_trackers), 
	ssm(ssm_params), params(parl_params),
	failure_detected(false), buffer_id(0),
	buffer_filled(false) {

	printf("\n");
	printf("Using Parallel Search Methods with:\n");
	printf("estimation_method: %d :: %s\n", params.estimation_method,
		ParamType::toString(params.estimation_method));
	printf("reset_to_mean: %d\n", params.reset_to_mean);
	printf("auto_reinit: %d\n", params.auto_reinit);
	printf("reinit_err_thresh: %f\n", params.reinit_err_thresh);
	printf("reinit_frame_gap: %d\n", params.reinit_frame_gap);

	printf("n_trackers: %d\n", n_trackers);
	printf("Search methods: ");
	name = "prl: ";
	for(int tracker_id = 0; tracker_id < n_trackers; tracker_id++) {
		name = name + trackers[tracker_id]->name + " ";
		printf("%d: %s ", tracker_id + 1, trackers[tracker_id]->name.c_str());
	}
	printf("\n");
	printf("appearance model: %s\n", trackers[0]->getAM().name.c_str());
	printf("state space model: %s\n", trackers[0]->getSSM().name.c_str());

	switch(params.estimation_method) {
	case PrlEstMethod::MeanOfCorners:
		mean_corners_cv.create(2, 4, CV_64FC1);
		break;
	case PrlEstMethod::MeanOfState:
		ssm_state_size = ssm.getStateSize();
		ssm_states.resize(n_trackers);
		for(int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
			ssm_states[tracker_id].resize(ssm_state_size);
		}
		mean_state.resize(ssm_state_size);
		break;
	}
	failure_detected = false;
	buffer_id = 0;
	buffer_filled = false;

	if(params.auto_reinit){
		if(input_type == HETEROGENEOUS_INPUT){
			printf("Reinitialization is currently not supported for trackers requiring heterogeneous inputs so disabling it...\n");
			params.auto_reinit = false;
		} else{
			img_buffer.resize(params.reinit_frame_gap);
			corners_buffer.resize(params.reinit_frame_gap);
		}
	}
}
template<class AM, class SSM>
void ParallelSM<AM, SSM>::setImage(const cv::Mat &img){
	curr_img = img;
	for(int tracker_id = 0; tracker_id < n_trackers; ++tracker_id){
		if(input_type != HETEROGENEOUS_INPUT ||
			img.type() == trackers[tracker_id]->inputType()){
			trackers[tracker_id]->setImage(img);
		}
	}
}
template<class AM, class SSM>
void ParallelSM<AM, SSM>::initialize(const cv::Mat &corners)  {
	for(int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
		trackers[tracker_id]->initialize(corners);
	}
	ssm.initialize(corners);
	ssm.getCorners(cv_corners_mat);
	if(params.auto_reinit){
		curr_img.copyTo(img_buffer[buffer_id]);
		corners.copyTo(corners_buffer[buffer_id]);
		if(buffer_id == params.reinit_frame_gap - 1){
			buffer_filled = true;
		}
		buffer_id = (buffer_id + 1) % params.reinit_frame_gap;
	}
}
template<class AM, class SSM>
void ParallelSM<AM, SSM>::update()  {
	mean_corners_cv.setTo(cv::Scalar(0));
#ifdef ENABLE_TBB
	parallel_for(tbb::blocked_range<size_t>(0, n_trackers),
		[&](const tbb::blocked_range<size_t>& r) {
		for(size_t tracker_id = r.begin(); tracker_id != r.end(); ++tracker_id) {
#else
	for(int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
#endif
		trackers[tracker_id]->update();
		switch(params.estimation_method) {
		case PrlEstMethod::MeanOfCorners:
			mean_corners_cv += (trackers[tracker_id]->getRegion() - mean_corners_cv) / (tracker_id + 1);
			break;
		case PrlEstMethod::MeanOfState:
			ssm_states[tracker_id] = trackers[tracker_id]->getSSM().getState();
			break;
		}
	}
#ifdef ENABLE_TBB
		});
#endif
		switch(params.estimation_method) {
		case PrlEstMethod::MeanOfCorners:
			ssm.setCorners(mean_corners_cv);
			break;
		case PrlEstMethod::MeanOfState:
			ssm.estimateMeanOfSamples(mean_state, ssm_states, n_trackers);
			ssm.setState(mean_state);
			ssm.getCorners(mean_corners_cv);
			break;
		}
		if(params.auto_reinit){
			if(failure_detected){
				failure_detected = false;
			} else{
				ssm.getCorners(cv_corners_mat);
				for(int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
					double corner_error = utils::getTrackingError<utils::TrackErrT::MCD>(
						trackers[tracker_id]->getRegion(), cv_corners_mat);
					if(corner_error > params.reinit_err_thresh){
						failure_detected = true;
						break;
					}
				}
				if(failure_detected){
					printf("Reinitializing trackers...\n");
					int reinit_buffer_id = buffer_filled ? buffer_id : 0;
					for(int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
						// initialize tracker on the oldest image in the buffer;
						trackers[tracker_id]->initialize(img_buffer[reinit_buffer_id],
							corners_buffer[reinit_buffer_id]);
						// update tracker on the remaining images in the buffer;
						int current_buffer_id = (reinit_buffer_id + 1) % params.reinit_frame_gap;
						while(current_buffer_id != buffer_id){
							trackers[tracker_id]->update(img_buffer[current_buffer_id]);
							current_buffer_id = (current_buffer_id + 1) % params.reinit_frame_gap;
						}
						// restore the current image
						trackers[tracker_id]->setImage(curr_img);
					}
					update();
					return;
				}
			}
		}
		if(params.reset_to_mean) {
			for(int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
				trackers[tracker_id]->setRegion(mean_corners_cv);
			}
		}
		ssm.getCorners(cv_corners_mat);
		if(params.auto_reinit){
			curr_img.copyTo(img_buffer[buffer_id]);
			cv_corners_mat.copyTo(corners_buffer[buffer_id]);
			if(buffer_id == params.reinit_frame_gap - 1){
				buffer_filled = true;
			}
			buffer_id = (buffer_id + 1) % params.reinit_frame_gap;
		}
}

template<class AM, class SSM>
void ParallelSM<AM, SSM>::setRegion(const cv::Mat& corners)   {
	for(int tracker_id = 1; tracker_id < n_trackers; ++tracker_id) {
		trackers[tracker_id]->setRegion(corners);
	}
	ssm.setCorners(corners);
	corners.copyTo(corners_buffer[buffer_id]);
}

_MTF_END_NAMESPACE

#ifndef HEADER_ONLY_MODE
#include "mtf/Macros/register.h"
_REGISTER_TRACKERS(ParallelSM);
#endif