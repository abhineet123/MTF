#ifndef MTF_CASCADE_PARAMS_H
#define MTF_CASCADE_PARAMS_H

#include "mtf/Macros/common.h"

#define CASC_ENABLE_FEEDBACK true
#define CASC_AUTO_REINIT false
#define CASC_REINIT_ERR_THRESH 10
#define CASC_REINIT_FRAME_GAP 1


_MTF_BEGIN_NAMESPACE

struct CascadeParams{
	/**
	use the position of the last tracker in the cascade to update that
	of the first tracker before updating the cascade with the next frame
	*/
	bool enable_feedback;
	/**
	automatically reinitialize all trackers when the change in corners between
	consecutive any two trackers in the cascade exceeds a threshold
	*/
	bool auto_reinit;
	/**
	threshold of corner change norm above which failure is assumed
	and trackers are reinitialized
	*/
	double reinit_err_thresh;
	/**
	gap between the frame in which the failure is detected and the one 
	where trackers are reinitialized
	*/
	int reinit_frame_gap;
	CascadeParams(bool _enable_feedback, bool _auto_reinit, 
		double _reinit_err_thresh, int _reinit_frame_gap);
	CascadeParams(const CascadeParams *params = nullptr);
};
_MTF_END_NAMESPACE

#endif

