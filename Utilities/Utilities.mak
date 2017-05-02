# ----------------------------------------------------------------------------- #
# --------------------------------- Utilities --------------------------------- #
# ----------------------------------------------------------------------------- #
UTILITIES_INCLUDE_DIR = Utilities/include
UTILITIES_SRC_DIR = Utilities/src
UTILITIES_HEADER_DIR = ${UTILITIES_INCLUDE_DIR}/mtf/Utilities

MTF_INCLUDE_DIRS += ${UTILITIES_INCLUDE_DIR}

UTILITIES = imgUtils warpUtils histUtils miscUtils spiUtils
UTILITIES_HEADER_ONLY = excpUtils
MTF_UTIL_OBJS = $(addprefix ${BUILD_DIR}/,$(addsuffix .o, ${UTILITIES}))
MTF_UTIL_HEADERS = $(addprefix ${UTILITIES_HEADER_DIR}/, $(addsuffix .h, ${UTILITIES} ${UTILITIES_HEADER_ONLY}))

MTF_HEADERS += ${MTF_UTIL_HEADERS}
MTF_OBJS += ${MTF_UTIL_OBJS}

HIST_FLAGS = 

htbb ?= 0
pip ?= -1
gip ?= -1
hip ?= -1
pbr ?= -1
graph_utils ?= 1

ifeq (${htbb}, 1)
HIST_FLAGS += -D ENABLE_HIST_TBB
MTF_RUNTIME_FLAGS += -D ENABLE_PARALLEL
endif

ifeq (${regnet}, 1)
UTILITIES += netUtils
endif
ifeq (${graph_utils}, 1)
UTILITIES += graphUtils
else
MTF_COMPILETIME_FLAGS += -D DISABLE_GRAPH_UTILS
MTF_RUNTIME_FLAGS += -D DISABLE_GRAPH_UTILS
endif

ifeq (${pip}, 0)
MTF_COMPILETIME_FLAGS += -D PIX_INTERP_TYPE=utils::InterpType::Nearest
else ifeq (${pip}, 1)
MTF_COMPILETIME_FLAGS += -D PIX_INTERP_TYPE=utils::InterpType::Linear
else ifeq (${pip}, 2)
MTF_COMPILETIME_FLAGS += -D PIX_INTERP_TYPE=utils::InterpType::Cubic
else ifeq (${pip}, 3)
MTF_COMPILETIME_FLAGS += -D PIX_INTERP_TYPE=utils::InterpType::Cubic2
else ifeq (${pip}, 4)
MTF_COMPILETIME_FLAGS += -D PIX_INTERP_TYPE=utils::InterpType::CubicBSpl
endif

ifeq (${gip}, 0)
MTF_COMPILETIME_FLAGS += -D GRAD_INTERP_TYPE=utils::InterpType::Nearest
else ifeq (${gip}, 1)
MTF_COMPILETIME_FLAGS += -D GRAD_INTERP_TYPE=utils::InterpType::Linear
else ifeq (${gip}, 2)
MTF_COMPILETIME_FLAGS += -D GRAD_INTERP_TYPE=utils::InterpType::Cubic
else ifeq (${gip}, 3)
MTF_COMPILETIME_FLAGS += -D GRAD_INTERP_TYPE=utils::InterpType::Cubic2
else ifeq (${gip}, 4)
MTF_COMPILETIME_FLAGS += -D GRAD_INTERP_TYPE=utils::InterpType::CubicBSpl
endif

ifeq (${hip}, 0)
MTF_COMPILETIME_FLAGS += -D HESS_INTERP_TYPE=utils::InterpType::Nearest
else ifeq (${hip}, 1)
MTF_COMPILETIME_FLAGS += -D HESS_INTERP_TYPE=utils::InterpType::Linear
else ifeq (${hip}, 2)
MTF_COMPILETIME_FLAGS += -D HESS_INTERP_TYPE=utils::InterpType::Cubic
else ifeq (${hip}, 3)
MTF_COMPILETIME_FLAGS += -D HESS_INTERP_TYPE=utils::InterpType::Cubic2
else ifeq (${hip}, 4)
MTF_COMPILETIME_FLAGS += -D HESS_INTERP_TYPE=utils::InterpType::CubicBSpl
endif

${BUILD_DIR}/warpUtils.o: ${UTILITIES_SRC_DIR}/warpUtils.cc ${UTILITIES_HEADER_DIR}/warpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${UTILITIES_HEADER_DIR}/excpUtils.h ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< -o $@
	
${BUILD_DIR}/imgUtils.o: ${UTILITIES_SRC_DIR}/imgUtils.cc ${UTILITIES_HEADER_DIR}/imgUtils.h ${UTILITIES_HEADER_DIR}/warpUtils.h ${UTILITIES_HEADER_DIR}/excpUtils.h ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< -o $@
	
${BUILD_DIR}/histUtils.o: ${UTILITIES_SRC_DIR}/histUtils.cc ${UTILITIES_HEADER_DIR}/histUtils.h ${UTILITIES_HEADER_DIR}/excpUtils.h ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h 
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${HIST_FLAGS} $< -o $@	
	
${BUILD_DIR}/miscUtils.o: ${UTILITIES_SRC_DIR}/miscUtils.cc ${UTILITIES_HEADER_DIR}/miscUtils.h ${UTILITIES_HEADER_DIR}/excpUtils.h ${MACROS_HEADER_DIR}/common.h 
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${HIST_FLAGS} $< -o $@

${BUILD_DIR}/netUtils.o: ${UTILITIES_SRC_DIR}/netUtils.cc  ${UTILITIES_HEADER_DIR}/netUtils.h ${MACROS_HEADER_DIR}/common.h  
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${CAFFE_FLAGS} $< -o $@	

${BUILD_DIR}/graphUtils.o: ${UTILITIES_SRC_DIR}/graphUtils.cc ${UTILITIES_HEADER_DIR}/graphUtils.h ${MACROS_HEADER_DIR}/common.h ${UTILITIES_HEADER_DIR}/miscUtils.h ${UTILITIES_HEADER_DIR}/excpUtils.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< -o $@
	
${BUILD_DIR}/spiUtils.o: ${UTILITIES_SRC_DIR}/spiUtils.cc ${UTILITIES_HEADER_DIR}/spiUtils.h ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< -o $@	