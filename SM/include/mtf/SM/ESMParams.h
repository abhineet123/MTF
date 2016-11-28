#ifndef MTF_ESM_PARAMS_H
#define MTF_ESM_PARAMS_H

#include "mtf/Macros/common.h"

_MTF_BEGIN_NAMESPACE

struct ESMParams{

	enum class JacType{ Original, DiffOfJacs };
	enum class HessType {
		InitialSelf, CurrentSelf, SumOfSelf,
		Original, SumOfStd, Std
	};

	int max_iters; //! maximum iterations of the ESM algorithm to run for each frame
	double epsilon; //! maximum L1 norm of the state update vector at which to stop the iterations

	JacType jac_type;
	HessType hess_type;
	bool sec_ord_hess;
	bool chained_warp;

	bool leven_marq;
	double lm_delta_init;
	double lm_delta_update;

	bool enable_spi;
	double spi_thresh;
	bool debug_mode; //! decides whether logging data will be printed for debugging purposes; 
	//! only matters if logging is enabled at compile time

	// value constructor
	ESMParams(int _max_iters, double _epsilon,
		JacType _jac_type, HessType _hess_type, bool _sec_ord_hess,
		bool _chained_warp,	bool _leven_marq, double _lm_delta_init,
		double _lm_delta_update, bool _enable_spi, double _spi_thresh,
		bool _debug_mode);
	// default and copy constructor
	ESMParams(const ESMParams *params = nullptr);
	static const char* toString(JacType _jac_type);
	static const char* toString(HessType _hess_type);

};

_MTF_END_NAMESPACE

#endif

