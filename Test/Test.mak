# --------------------------------------------------------------------------------- #
# ---------------------------------- Diagnostics ---------------------------------- #
# --------------------------------------------------------------------------------- #

TEST_INCLUDE_DIR = Test/include
TEST_SRC_DIR = Test/src
TEST_HEADER_DIR = ${TEST_INCLUDE_DIR}/mtf/Test

MTF_INCLUDE_DIRS += ${TEST_INCLUDE_DIR}

TEST_BASE_CLASSES += ${TEST_HEADER_DIR}/DiagBase
TEST_TOOLS = Diagnostics
TEST_OBJS = $(addprefix ${BUILD_DIR}/,$(addsuffix .o, ${TEST_TOOLS})) 
TEST_BASE_HEADERS = $(addsuffix .h, ${TEST_BASE_CLASSES})
TEST_HEADERS = $(addprefix ${TEST_HEADER_DIR}/,$(addsuffix .h, ${TEST_TOOLS}))
TEST_HEADERS += ${TEST_BASE_HEADERS}

#MTF_HEADERS += ${TEST_HEADER_DIR}/mtf_test.h ${TEST_HEADERS}

ifeq (${o}, 1)
	_MTF_TEST_LIB_NAME = mtf_test
else
	_MTF_TEST_LIB_NAME = mtf_test_debug
endif

MTF_TEST_LIB_NAME = $(addsuffix ${LIB_POST_FIX}, ${_MTF_TEST_LIB_NAME})
MTF_TEST_LIB_SO =  $(addprefix lib, $(addsuffix ${MTF_LIB_EXT}, ${MTF_TEST_LIB_NAME}))
MTF_TEST_LIBS = $(addprefix -l, ${MTF_TEST_LIB_NAME})

.PHONY: install_test_lib

install_test_lib: ${MTF_LIB_INSTALL_DIR}/${MTF_TEST_LIB_SO}
${MTF_LIB_INSTALL_DIR}/${MTF_TEST_LIB_SO}: ${BUILD_DIR}/${MTF_TEST_LIB_SO}
	${MTF_LIB_INSTALL_CMD_PREFIX} ${CP_CMD} ${BUILD_DIR}/${MTF_TEST_LIB_SO} $@
	
${BUILD_DIR}/Diagnostics.o: ${TEST_SRC_DIR}/Diagnostics.cc ${TEST_SRC_DIR}/DiagAnalytic.cc ${TEST_SRC_DIR}/DiagInvAnalytic.cc ${TEST_SRC_DIR}/DiagNumeric.cc ${TEST_SRC_DIR}/DiagHelper.cc ${TEST_HEADERS} ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS} ${UTILITIES_HEADER_DIR}/miscUtils.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/${MTF_TEST_LIB_SO}: | ${BUILD_DIR}
${BUILD_DIR}/${MTF_TEST_LIB_SO}:  ${TEST_OBJS}
	${CXX} -shared -o $@  ${TEST_OBJS} ${MTF_RUNTIME_FLAGS} ${MTF_LIB_LINK} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${OPENCV_LIBS} ${BOOST_LIBS}