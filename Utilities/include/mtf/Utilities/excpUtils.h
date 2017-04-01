#ifndef MTF_EXCP_UTILS_H
#define MTF_EXCP_UTILS_H

#include<stdexcept>
#include "mtf/Macros/common.h"

_MTF_BEGIN_NAMESPACE
namespace utils{
	class InvalidTrackerState : public std::runtime_error {
	public:
		InvalidTrackerState(const char* error = "Invalid Tracker State found") :
			std::runtime_error(error){}
		InvalidTrackerState(const std::string  error = "Invalid Tracker State found") :
			std::runtime_error(error){}
	};

	class FunctonNotImplemented : public std::logic_error {
	public:
		FunctonNotImplemented(const char* error = "Function has not been implemented yet") :
			std::logic_error(error){}
		FunctonNotImplemented(const std::string error = "Function has not been implemented yet") :
			std::logic_error(error){}
	};
}
_MTF_END_NAMESPACE
#endif
