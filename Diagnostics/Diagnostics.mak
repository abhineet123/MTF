# --------------------------------------------------------------------------------- #
# ---------------------------------- Diagnostics ---------------------------------- #
# --------------------------------------------------------------------------------- #

DIAG_INCLUDE_DIR = Diagnostics/include
DIAG_SRC_DIR = Diagnostics/src
DIAG_HEADER_DIR = ${DIAG_INCLUDE_DIR}/mtf/Diagnostics

MTF_INCLUDE_DIRS += ${DIAG_INCLUDE_DIR}

DIAG_BASE_CLASSES += ${DIAG_HEADER_DIR}/DiagBase
DIAG_TOOLS = Diagnostics
DIAG_OBJS = $(addprefix ${BUILD_DIR}/,$(addsuffix .o, ${DIAG_TOOLS})) 
DIAG_BASE_HEADERS = $(addsuffix .h, ${DIAG_BASE_CLASSES})
DIAG_HEADERS = $(addprefix ${DIAG_HEADER_DIR}/,$(addsuffix .h, ${DIAG_TOOLS}))
DIAG_HEADERS += ${DIAG_BASE_HEADERS}

#MTF_HEADERS += ${DIAG_HEADER_DIR}/mtf_diag.h ${DIAG_HEADERS}

ifeq (${o}, 1)
	_MTF_DIAG_LIB_NAME = mtf_diag
else
	_MTF_DIAG_LIB_NAME = mtf_diag_debug
endif

MTF_DIAG_LIB_NAME = $(addsuffix ${LIB_POST_FIX}, ${_MTF_DIAG_LIB_NAME})
MTF_DIAG_LIB_SO =  $(addprefix lib, $(addsuffix ${MTF_LIB_EXT}, ${MTF_DIAG_LIB_NAME}))
MTF_DIAG_LIBS = $(addprefix -l, ${MTF_DIAG_LIB_NAME})

.PHONY: install_diag_lib

install_diag_lib: ${MTF_LIB_INSTALL_DIR}/${MTF_DIAG_LIB_SO}
${MTF_LIB_INSTALL_DIR}/${MTF_DIAG_LIB_SO}: ${BUILD_DIR}/${MTF_DIAG_LIB_SO}
	${MTF_LIB_INSTALL_CMD_PREFIX} ${CP_CMD} ${BUILD_DIR}/${MTF_DIAG_LIB_SO} $@
	
${BUILD_DIR}/Diagnostics.o: ${DIAG_SRC_DIR}/Diagnostics.cc ${DIAG_SRC_DIR}/DiagAnalytic.cc ${DIAG_SRC_DIR}/DiagInvAnalytic.cc ${DIAG_SRC_DIR}/DiagNumeric.cc ${DIAG_SRC_DIR}/DiagHelper.cc ${DIAG_HEADERS} ${AM_BASE_HEADERS} ${SSM_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/miscUtils.h ${UTILITIES_HEADER_DIR}/excpUtils.h ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/${MTF_DIAG_LIB_SO}: | ${BUILD_DIR}
${BUILD_DIR}/${MTF_DIAG_LIB_SO}:  ${DIAG_OBJS}
	${CXX} -shared -o $@  ${DIAG_OBJS} ${MTF_RUNTIME_FLAGS} ${MTF_LIB_LINK} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${OPENCV_LIBS} ${BOOST_LIBS}