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
MTF_TEST_LIB_NAME = libmtf_test.so
MTF_TEST_LIBS =-lmtf_test
else
MTF_TEST_LIB_NAME = libmtf_test_debug.so
MTF_TEST_LIBS = -lmtf_test_debug
endif

.PHONY: install_test_lib

install_test_lib: ${MTF_LIB_INSTALL_DIR}/${MTF_TEST_LIB_NAME}
${MTF_LIB_INSTALL_DIR}/${MTF_TEST_LIB_NAME}: ${BUILD_DIR}/${MTF_TEST_LIB_NAME}
	sudo cp -f ${BUILD_DIR}/${MTF_TEST_LIB_NAME} $@
	
${BUILD_DIR}/Diagnostics.o: ${TEST_SRC_DIR}/Diagnostics.cc ${TEST_SRC_DIR}/DiagAnalytic.cc ${TEST_SRC_DIR}/DiagInvAnalytic.cc ${TEST_SRC_DIR}/DiagNumeric.cc ${TEST_SRC_DIR}/DiagHelper.cc ${TEST_HEADERS} ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS} ${UTILITIES_HEADER_DIR}/miscUtils.h
	${CXX} -c -fPIC ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${FLAGS64} ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/${MTF_TEST_LIB_NAME}: | ${BUILD_DIR}
${BUILD_DIR}/${MTF_TEST_LIB_NAME}:  ${TEST_OBJS}
	${CXX} -shared -o $@  ${TEST_OBJS} ${MTF_LIB_LINK}	