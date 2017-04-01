#include "mtf/SM/ParallelTracker.h"

_MTF_BEGIN_NAMESPACE

const char* ParallelParams::toString(PrlEstMethod _estimation_method){
	switch(_estimation_method) {
	case PrlEstMethod::MeanOfCorners:
		return "MeanOfCorners";
	case PrlEstMethod::MeanOfState:
		return "MeanOfState";
	default:
		throw std::invalid_argument("Invalid dynamic model provided");
	}
}
ParallelParams::ParallelParams(PrlEstMethod _estimation_method, bool _reset_to_mean,
	bool _auto_reinit, double _reinit_err_thresh, int _reinit_frame_gap) {
	estimation_method = _estimation_method;
	reset_to_mean = _reset_to_mean;
	auto_reinit = _auto_reinit;
	reinit_err_thresh = _reinit_err_thresh;
	reinit_frame_gap = _reinit_frame_gap;
}
ParallelParams::ParallelParams(const ParallelParams *params) :
estimation_method(static_cast<PrlEstMethod>(PARL_ESTIMATION_METHOD)),
reset_to_mean(PARL_RESET_TO_MEAN),
auto_reinit(PARL_AUTO_REINIT),
reinit_err_thresh(PARL_REINIT_ERR_THRESH),
reinit_frame_gap(PRL_SM_REINIT_FRAME_GAP){
	if(params) {
		estimation_method = params->estimation_method;
		reset_to_mean = params->reset_to_mean;
		auto_reinit = params->auto_reinit;
		reinit_err_thresh = params->reinit_err_thresh;
		reinit_frame_gap = params->reinit_frame_gap;
	}
}

_MTF_END_NAMESPACE
