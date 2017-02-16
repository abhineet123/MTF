#ifndef MTF_COMPOSITE_SM_H
#define MTF_COMPOSITE_SM_H

#include "SearchMethod.h"
#include "mtf/Macros/common.h"

_MTF_BEGIN_NAMESPACE

//! base class for all composite search methods
template<class AM, class SSM>
class CompositeSM : public TrackerBase{
public:
	typedef SearchMethod<AM, SSM> SM;

	CompositeSM() : TrackerBase(), n_trackers(0), input_type(0){}
	CompositeSM(const vector<SM*> _trackers) :
		TrackerBase(), trackers(_trackers) {
		n_trackers = trackers.size();
		//! since all SMs have the same AM, we can assume that they also have the same input type
		input_type = trackers[0]->inputType();
	}
	virtual ~CompositeSM(){}
	void setImage(const cv::Mat &img) override{
		for(int tracker_id = 0; tracker_id < n_trackers; ++tracker_id){
			if(img.type() == trackers[tracker_id]->inputType()){
				trackers[tracker_id]->setImage(img);
			}
		}
	}

	int inputType() const override{ return input_type; }

	void setRegion(const cv::Mat& corners) override   {
		for(int tracker_id = 1; tracker_id < n_trackers; ++tracker_id) {
			trackers[tracker_id]->setRegion(corners);
		}
	}
	virtual void setSPIMask(const bool *_spi_mask)  {
		for(int tracker_id = 0; tracker_id < n_trackers; ++tracker_id){
			trackers[tracker_id]->setSPIMask(_spi_mask);
		}
	}
	virtual void clearSPIMask()  {
		for(int tracker_id = 0; tracker_id < n_trackers; ++tracker_id){
			trackers[tracker_id]->clearSPIMask();
		}
	}
	virtual void setInitStatus()  {
		for(int tracker_id = 0; tracker_id < n_trackers; ++tracker_id){
			trackers[tracker_id]->setInitStatus();
		}
	}
	virtual void clearInitStatus()  {
		for(int tracker_id = 0; tracker_id < n_trackers; ++tracker_id){
			trackers[tracker_id]->clearInitStatus();
		}
	}
	virtual bool supportsSPI() {
		for(int tracker_id = 0; tracker_id < n_trackers; ++tracker_id){
			if(!trackers[tracker_id]->supportsSPI())
				return false;
		}
		return true;
	}
	
	virtual AM& getAM() { return trackers.back()->getAM(); }
	virtual SSM& getSSM() { return trackers.back()->getSSM(); }

protected:
	const vector<SM*> trackers;
	int n_trackers;
	int input_type;

};
_MTF_END_NAMESPACE

#endif
