#ifndef MTF_EXCP_UTILS_H
#define MTF_EXCP_UTILS_H

#include<stdexcept>

namespace mtf {
	namespace utils{
		class Exception : public std::exception {
		public:
			Exception(const char* _error = "Exception encountered") :
				std::exception(), error(_error){}
			Exception(const std::string  _error = "Exception encountered") :
				std::exception(), error(_error){}
			const char* what() const noexcept override{ return error.c_str(); }
			virtual	const char* type() const noexcept = 0;
		private:
			const std::string  error;
		};

		class InvalidTrackerState : public Exception {
		public:
			InvalidTrackerState(const char* error = "Invalid Tracker State found") :
				Exception(error){}
			InvalidTrackerState(const std::string  error = "Invalid Tracker State found") :
				Exception(error){}
			const char* type() const noexcept override{ return "InvalidTrackerState"; }
		};

		class FunctonNotImplemented : public Exception {
		public:
			FunctonNotImplemented(const char* error = "Function has not been implemented yet") :
				Exception(error){}
			FunctonNotImplemented(const std::string error = "Function has not been implemented yet") :
				Exception(error){}
			const char* type() const noexcept override{ return "FunctonNotImplemented"; }
		};

		class InvalidArgument : public Exception {
		public:
			InvalidArgument(const char* error = "Invalid argument provided") :
				Exception(error){}
			InvalidArgument(const std::string error = "Invalid argument provided") :
				Exception(error){}
			const char* type() const noexcept override{ return "InvalidArgument"; }
		};

		class LogicError : public Exception {
		public:
			LogicError(const char* error = "Logical error encountered") :
				Exception(error){}
			LogicError(const std::string error = "Logical error encountered") :
				Exception(error){}
			const char* type() const noexcept override{ return "LogicError"; }
		};
	}
}
#endif
