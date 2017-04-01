#include "mtf/SM/CascadeTracker.h"
#include "mtf/Utilities/miscUtils.h"

_MTF_BEGIN_NAMESPACE

CascadeTracker::CascadeTracker(const vector<TrackerBase*> _trackers, const ParamType *casc_params) :
CompositeBase(_trackers), params(casc_params), failure_detected(false),
buffer_id(0), buffer_filled(false){
	printf("\n");
	printf("Using Cascade tracker with:\n");
	printf("n_trackers: %d\n", n_trackers);
	printf("trackers: ");
	name = "casc: ";
	for(int tracker_id = 0; tracker_id < n_trackers; tracker_id++){
		if(!trackers[tracker_id]){
			throw std::invalid_argument(cv::format("CascadeTracker :: tracker %d is invalid\n", tracker_id));
		}
		name = name + trackers[tracker_id]->name + " ";
		printf("%d: %s ", tracker_id + 1, trackers[tracker_id]->name.c_str());
	}
	printf("\n");
	if(params.enable_feedback){
		printf("Feedback is enabled\n");
	}
	if(params.auto_reinit){
		if(inputType() == HETEROGENEOUS_INPUT){
			printf("Auto reinitialization with heterogeneous trackers is currently not supported so disabling it.\n");
			params.auto_reinit = false;
		} else{
			if(params.reinit_frame_gap <= 0){
				params.reinit_frame_gap = 1;
			}
			printf("Tracker auto reinitialization is enabled with:\n");
			printf("reinit_err_thresh: %f\n", params.reinit_err_thresh);
			printf("reinit_frame_gap: %d\n", params.reinit_frame_gap);
			img_buffer.resize(params.reinit_frame_gap);
			corners_buffer.resize(params.reinit_frame_gap);
		}
	}
}

void CascadeTracker::initialize(const cv::Mat &corners){
	for(int tracker_id = 0; tracker_id < n_trackers; tracker_id++){
		//printf("Using tracker %d\n", tracker_id);
		trackers[tracker_id]->initialize(corners);
	}
	if(params.auto_reinit){
		curr_img.copyTo(img_buffer[buffer_id]);
		corners.copyTo(corners_buffer[buffer_id]);
		if(buffer_id == params.reinit_frame_gap - 1){
			buffer_filled = true;
		}
		buffer_id = (buffer_id + 1) % params.reinit_frame_gap;
	}
	//cv_corners = trackers[n_trackers - 1]->cv_corners;
}
void CascadeTracker::update(){
	trackers[0]->update();
	for(int tracker_id = 1; tracker_id < n_trackers; tracker_id++){
		//printf("tracker: %d ", tracker_id - 1);
		//utils::printMatrix<double>(trackers[tracker_id - 1]->getRegion(),
		//	"region is: ");
		trackers[tracker_id]->setRegion(trackers[tracker_id - 1]->getRegion());
		//printf("tracker: %d ", tracker_id);
		//utils::printMatrix<double>(trackers[tracker_id]->getRegion(),
		//	"region set to: ");
		trackers[tracker_id]->update();

		if(params.auto_reinit){
			if(!utils::isFinite<double>(trackers[tracker_id]->getRegion())){
				printf("Failure detected with non finite values in tracker %d corners\n", tracker_id);
				failure_detected = true;
				break;
			}
			double tracker_region_err = utils::getTrackingError<utils::TrackErrT::MCD>(
				trackers[tracker_id]->getRegion(), trackers[tracker_id - 1]->getRegion());
			if(tracker_region_err > params.reinit_err_thresh){
				printf("Failure detected with corner change norm: %f from tracker %d to %d\n",
					tracker_region_err, tracker_id - 1, tracker_id);
				failure_detected = true;
				break;
			}
		}
	}
	if(params.auto_reinit && failure_detected){
		printf("Reinitializing trackers...\n");
		int reinit_buffer_id = buffer_filled ? buffer_id : 0;
		for(int tracker_id = 0; tracker_id < n_trackers; ++tracker_id) {
			//! initialize tracker on the oldest image in the buffer;
			trackers[tracker_id]->initialize(img_buffer[reinit_buffer_id], corners_buffer[reinit_buffer_id]);
		}
		//! update tracker on the remaining images in the buffer;
		int current_buffer_id = (reinit_buffer_id + 1) % params.reinit_frame_gap;
		while(current_buffer_id != buffer_id){
			updateTrackers(img_buffer[current_buffer_id]);
			//! update corners in the buffer assuming of course that the trackers have not 
			//! failed again and that the existing corners are faulty as they led to failure 
			getRegion().copyTo(corners_buffer[current_buffer_id]);
			current_buffer_id = (current_buffer_id + 1) % params.reinit_frame_gap;
		}
		//! update once more on the current image where failure was detected
		updateTrackers(curr_img);
		failure_detected = false;	
	} else if(params.enable_feedback){
		trackers[0]->setRegion(trackers[n_trackers - 1]->getRegion());
	}
	if(params.auto_reinit){
		curr_img.copyTo(img_buffer[buffer_id]);
		getRegion().copyTo(corners_buffer[buffer_id]);
		if(buffer_id == params.reinit_frame_gap - 1){
			buffer_filled = true;
		}
		buffer_id = (buffer_id + 1) % params.reinit_frame_gap;
	}
	//cv_corners = trackers[n_trackers - 1]->cv_corners;
}
void CascadeTracker::updateTrackers(const cv::Mat &img){
	trackers[0]->update(img);
	for(int tracker_id = 1; tracker_id < n_trackers; tracker_id++){
		trackers[tracker_id]->setImage(img);
		trackers[tracker_id]->setRegion(trackers[tracker_id - 1]->getRegion());
		trackers[tracker_id]->update();
	}
	if(params.enable_feedback){
		trackers[0]->setRegion(trackers[n_trackers - 1]->getRegion());
	}
}
void CascadeTracker::setRegion(const cv::Mat& corners) {
	for(int tracker_id = 0; tracker_id < n_trackers; tracker_id++){
		trackers[tracker_id]->setRegion(corners);
	}
	if(params.auto_reinit){
		corners.copyTo(corners_buffer[buffer_id]);
	}
}
void CascadeTracker::setImage(const cv::Mat &img){
	curr_img = img;
	for(int tracker_id = 0; tracker_id < n_trackers; ++tracker_id){
		if(inputType() != HETEROGENEOUS_INPUT || 
			img.type() == trackers[tracker_id]->inputType()){
			trackers[tracker_id]->setImage(img);
		}
	}
}
_MTF_END_NAMESPACE


