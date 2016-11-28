#ifndef MTF_PYRAMIDAL_PARAMS_H
#define MTF_PYRAMIDAL_PARAMS_H

#include "mtf/Macros/common.h"

#define PYRL_NO_OF_LEVELS 3
#define PYRL_SCALE_FACTOR 0
#define PYRL_AUTO_REINIT 0
#define PYRL_REINIT_ERR_THRESH 1
#define PYRL_SM_REINIT_FRAME_GAP 1

_MTF_BEGIN_NAMESPACE

struct PyramidalParams {
	int no_of_levels;
	double scale_factor;
	bool show_levels;
	PyramidalParams(int no_of_levels, double scale_factor, 
		bool show_levels);
	PyramidalParams(const PyramidalParams *params = nullptr);
};

_MTF_END_NAMESPACE

#endif

