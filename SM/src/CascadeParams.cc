#include "mtf/SM/CascadeParams.h"

_MTF_BEGIN_NAMESPACE

CascadeParams::CascadeParams(bool _enable_feedback, bool _auto_reinit,
double _reinit_err_thresh, int _reinit_frame_gap) :
enable_feedback(_enable_feedback),
auto_reinit(_auto_reinit),
reinit_err_thresh(_reinit_err_thresh),
reinit_frame_gap(_reinit_frame_gap){}

CascadeParams::CascadeParams(const CascadeParams *params) :
enable_feedback(CASC_ENABLE_FEEDBACK),
auto_reinit(CASC_AUTO_REINIT),
reinit_err_thresh(CASC_REINIT_ERR_THRESH),
reinit_frame_gap(CASC_REINIT_FRAME_GAP){
	if(params){
		enable_feedback = params->enable_feedback;
		auto_reinit = params->auto_reinit;
		reinit_err_thresh = params->reinit_err_thresh;
		reinit_frame_gap = params->reinit_frame_gap;
	}
}

_MTF_END_NAMESPACE


