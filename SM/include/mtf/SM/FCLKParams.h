#ifndef MTF_FCLK_PARAMS_H
#define MTF_FCLK_PARAMS_H

#include "mtf/Macros/common.h"

_MTF_BEGIN_NAMESPACE
struct FCLKParams{
	enum class HessType{ InitialSelf, CurrentSelf, Std };

	int max_iters; //! maximum iterations of the FCLK algorithm to run for each frame
	double epsilon; //! maximum L1 norm of the state update vector at which to stop the iterations
	HessType hess_type;
	bool sec_ord_hess;
	bool chained_warp;
	bool leven_marq;
	double lm_delta_init;
	double lm_delta_update;	
	bool enable_learning;
	bool write_ssm_updates;
	bool show_grid;
	bool show_patch;
	double patch_resize_factor;
	bool debug_mode; //! decides whether logging data will be printed for debugging purposes; 
	//! only matters if logging is enabled at compile time

	FCLKParams(int _max_iters, double _epsilon,
		HessType _hess_type, bool _sec_ord_hess,
		bool _chained_warp,	bool _leven_marq, 
		double _lm_delta_init, double _lm_delta_update,
		bool _enable_learning, bool _write_ssm_updates,
		bool _show_grid, bool _show_patch,
		double _patch_resize_factor, bool _debug_mode);
	FCLKParams(const FCLKParams *params = nullptr);
	static const char* toString(HessType hess_type);
};


_MTF_END_NAMESPACE

#endif

