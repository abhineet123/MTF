#ifndef MTF_IALK_PARAMS_H
#define MTF_IALK_PARAMS_H

#include "mtf/Macros/common.h"

_MTF_BEGIN_NAMESPACE

struct IALKParams{
	enum class HessType{ InitialSelf, CurrentSelf, Std };
	int max_iters; //! maximum iterations of the IALK algorithm to run for each frame
	double epsilon; //! maximum L1 norm of the state update vector at which to stop the iterations
	HessType hess_type;
	bool sec_ord_hess;
	bool leven_marq;
	double lm_delta_init;
	double lm_delta_update;
	bool debug_mode; //! decides whether logging data will be printed for debugging purposes; 
	//! only matters if logging is enabled at compile time

	IALKParams(int _max_iters, double _epsilon,
		HessType _hess_type, bool _sec_ord_hess,
		bool _leven_marq, double _lm_delta_init, 
		double _lm_delta_update, bool _debug_mode);
	IALKParams(const IALKParams *params = nullptr);

	static const char* toString(HessType hess_type);

};

_MTF_END_NAMESPACE

#endif

