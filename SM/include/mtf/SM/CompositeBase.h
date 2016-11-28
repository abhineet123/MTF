#ifndef MTF_COMPOSITE_BASE_H
#define MTF_COMPOSITE_BASE_H

#include "mtf/TrackerBase.h"
#include "mtf/Macros/common.h"

_MTF_BEGIN_NAMESPACE

// base class for all composite trackers
class CompositeBase : public TrackerBase{
public:
	const vector<TrackerBase*> trackers;
	int n_trackers;
	int input_type;
	CompositeBase() : TrackerBase(), n_trackers(0), input_type(0){}
	CompositeBase(const vector<TrackerBase*> _trackers):
		TrackerBase(), trackers(_trackers) {
		n_trackers = trackers.size();
		input_type = trackers[0]->inputType();
		for(int tracker_id = 1; tracker_id < n_trackers; tracker_id++){
			if(input_type != trackers[tracker_id]->inputType()){
				input_type = HETEROGENEOUS_INPUT;
			}
			break;
		}
	}
	virtual ~CompositeBase(){}
	void setImage(const cv::Mat &img) override{
		for(int tracker_id = 0; tracker_id < n_trackers; ++tracker_id){
			if(img.type() == trackers[tracker_id]->inputType()){
				trackers[tracker_id]->setImage(img);
			}
		}
	}
	int inputType() const override{ return input_type; }
};
_MTF_END_NAMESPACE

#endif
