#include "mtf/SM/ParallelTracker.h"
#include "mtf/Utilities/miscUtils.h"

#ifdef ENABLE_TBB
#include "tbb/tbb.h" 
#endif

_MTF_BEGIN_NAMESPACE

ParallelTracker::ParallelTracker(const vector<TrackerBase*> _trackers,
 const ParamType *parl_params) : CompositeBase(_trackers),
	params(parl_params), failure_detected(false), 
	buffer_id(0), buffer_filled(false){
	printf("\n");
	printf("Using generalized Parallel tracker with:\n");
	printf("reset_to_mean: %d\n", params.reset_to_mean);
	printf("auto_reinit: %d\n", params.auto_reinit);
	printf("reinit_err_thresh: %f\n", params.reinit_err_thresh);
	printf("reinit_frame_gap: %d\n", params.reinit_frame_gap);
	printf("n_trackers: %d\n", n_trackers);
	printf("trackers: ");
	name = "prl: ";
	for(int i = 0; i < n_trackers; i++) {
		name = name + trackers[i]->name + " ";
		printf("%d: %s ", i + 1, trackers[i]->name.c_str());
	}
	printf("\n");

	mean_corners_cv.create(2, 4, CV_64FC1);
	if(params.auto_reinit){
		if(input_type==HETEROGENEOUS_INPUT){
			printf("Reinitialization is currently not supported for trackers requiring heterogeneous inputs so disabling it...\n");
			params.auto_reinit = false;
		}else{
			img_buffer.resize(params.reinit_frame_gap);
			corners_buffer.resize(params.reinit_frame_gap);
		}
	}
}

void ParallelTracker::setImage(const cv::Mat &img){
	curr_img = img;
	for(int tracker_id = 0; tracker_id < n_trackers; ++tracker_id){
		if(img.type() == trackers[tracker_id]->inputType()){
			trackers[tracker_id]->setImage(img);
		}
	}
}

void ParallelTracker::initialize(const cv::Mat &corners)  {
	for(int tracker_id = 0; tracker_id < n_trackers; tracker_id++) {
		trackers[tracker_id]->initialize(corners);
	}
	cv_corners_mat = corners;
	if(params.auto_reinit){
		curr_img.copyTo(img_buffer[buffer_id]);
		corners.copyTo(corners_buffer[buffer_id]);
		if(buffer_id == params.reinit_frame_gap - 1){
			buffer_filled = true;
		}
		buffer_id = (buffer_id + 1) % params.reinit_frame_gap;
	}
}

void ParallelTracker::update()  {
	mean_corners_cv.setTo(cv::Scalar(0));
#ifdef ENABLE_TBB
	parallel_for(tbb::blocked_range<size_t>(0, n_trackers),
		[&](const tbb::blocked_range<size_t>& r) {
		for(size_t tracker_id = r.begin(); tracker_id != r.end(); ++tracker_id) {
#else
	for(int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
#endif
		trackers[tracker_id]->update();
		mean_corners_cv += (trackers[tracker_id]->getRegion() - mean_corners_cv) / (tracker_id + 1);
	}
#ifdef ENABLE_TBB
		});
#endif
		if(params.auto_reinit){
			if(failure_detected){
				failure_detected = false;
			} else{
				for(int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
					double corner_error = utils::getTrackingError<utils::TrackErrT::MCD>(
						trackers[tracker_id]->getRegion(), mean_corners_cv);
					if(corner_error > params.reinit_err_thresh){
						failure_detected = true;
						break;
					}
				}
				if(failure_detected){
					printf("Reinitializing trackers...\n");

					int reinit_buffer_id = buffer_filled ? buffer_id  : 0;
					for(int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
						// initialize tracker on the oldest image in the buffer;
						trackers[tracker_id]->setImage(img_buffer[reinit_buffer_id]);
						trackers[tracker_id]->initialize(corners_buffer[reinit_buffer_id]);

						// update tracker on the remaining images in the buffer;
						int current_buffer_id = (reinit_buffer_id + 1) % params.reinit_frame_gap;
						while(current_buffer_id != buffer_id){
							trackers[tracker_id]->setImage(img_buffer[current_buffer_id]);
							trackers[tracker_id]->update();
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
			for(int tracker_id = 0; tracker_id < n_trackers; tracker_id++) {
				trackers[tracker_id]->setRegion(mean_corners_cv);
			}
		}
		if(params.auto_reinit){
			curr_img.copyTo(img_buffer[buffer_id]);
			mean_corners_cv.copyTo(corners_buffer[buffer_id]);
			if(buffer_id == params.reinit_frame_gap - 1){
				buffer_filled = true;
			}
			buffer_id = (buffer_id + 1) % params.reinit_frame_gap;
		}
}
void ParallelTracker::setRegion(const cv::Mat& corners)   {
	for(int tracker_id = 1; tracker_id < n_trackers; tracker_id++) {
		trackers[tracker_id]->setRegion(corners);
	}
	corners.copyTo(mean_corners_cv);
	corners.copyTo(corners_buffer[buffer_id]);
}

_MTF_END_NAMESPACE
