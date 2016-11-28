#ifndef MTF_PARALLEL_PARAMS_H
#define MTF_PARALLEL_PARAMS_H

#include "mtf/Macros/common.h"

#define PARL_ESTIMATION_METHOD 0
#define PARL_RESET_TO_MEAN 0
#define PARL_AUTO_REINIT 0
#define PARL_REINIT_ERR_THRESH 1
#define PRL_SM_REINIT_FRAME_GAP 1

_MTF_BEGIN_NAMESPACE

struct ParallelParams {
	enum class PrlEstMethod {
		MeanOfCorners,
		MeanOfState
	};
	PrlEstMethod estimation_method;
	bool reset_to_mean;
	bool auto_reinit;
	double reinit_err_thresh;
	int reinit_frame_gap;
	static const char* toString(PrlEstMethod _estimation_method);
	ParallelParams(PrlEstMethod _estimation_method, bool _reset_to_mean,
		bool _auto_reinit, double _reinit_err_thresh, int _reinit_frame_gap);
	ParallelParams(const ParallelParams *params = nullptr);
};

_MTF_END_NAMESPACE

#endif

