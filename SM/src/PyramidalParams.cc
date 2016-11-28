#include "mtf/SM/PyramidalParams.h"

_MTF_BEGIN_NAMESPACE

PyramidalParams::PyramidalParams(int _no_of_levels, double _scale_factor,
bool _show_levels) {
	no_of_levels = _no_of_levels;
	scale_factor = _scale_factor;
	show_levels = _show_levels;
}

PyramidalParams::PyramidalParams(const PyramidalParams *params) :
no_of_levels(PYRL_NO_OF_LEVELS),
scale_factor(PYRL_SCALE_FACTOR){
	if(params) {
		no_of_levels = params->no_of_levels;
		scale_factor = params->scale_factor;
		show_levels = params->show_levels;
	}
}
_MTF_END_NAMESPACE
