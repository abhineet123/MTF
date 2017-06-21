CMT_ROOT_DIR = ThirdParty/CMT
CMT_SRC_DIR = ${CMT_ROOT_DIR}/src
CMT_INCLUDE_DIR = ${CMT_ROOT_DIR}/include
CMT_HEADER_DIR = ${CMT_INCLUDE_DIR}/mtf/${CMT_ROOT_DIR}
CMT_HEADERS = $(addprefix  ${CMT_HEADER_DIR}/, CMT.h)
CMT_LIB_NAME = $(addsuffix ${LIB_POST_FIX}, cmt)
CMT_LIB_SO =  $(addprefix lib, $(addsuffix .so, ${CMT_LIB_NAME}))

CMT_LIB_MODULES = common Consensus Fusion gui Matcher Tracker getopt/getopt fastcluster/fastcluster
CMT_LIB_INCLUDES = logging/log
CMT_LIB_HEADERS = $(addprefix ${CMT_HEADER_DIR}/,$(addsuffix .h, ${CMT_LIB_MODULES} ${CMT_LIB_INCLUDES}))
CMT_LIB_SRC = $(addprefix ${CMT_SRC_DIR}/,$(addsuffix .cpp, ${CMT_LIB_MODULES}))

ifneq ($(OS),Windows_NT)
THIRD_PARTY_TRACKERS += CMT
_THIRD_PARTY_TRACKERS_SO += ${CMT_LIB_NAME} 
THIRD_PARTY_TRACKERS_SO_LOCAL += ${CMT_ROOT_DIR}/${CMT_LIB_SO}
THIRD_PARTY_LIBS_DIRS += -L${CMT_ROOT_DIR}
THIRD_PARTY_INCLUDE_DIRS += ${CMT_INCLUDE_DIR}
THIRD_PARTY_HEADERS += ${CMT_HEADERS} ${CMT_LIB_HEADERS}
endif

${BUILD_DIR}/CMT.o: ${CMT_SRC_DIR}/CMT.cc ${CMT_HEADERS} ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${ROOT_HEADER_DIR}/TrackerBase.h
	${CXX} ${MTF_PIC_FLAG} -c ${WARNING_FLAGS} ${OPT_FLAGS} $< -std=c++11 ${OPENCV_FLAGS} ${MTF_COMPILETIME_FLAGS} -I${CMT_INCLUDE_DIR} -I${UTILITIES_INCLUDE_DIR} -I${MACROS_INCLUDE_DIR} -I${ROOT_INCLUDE_DIR} -o  $@

${MTF_LIB_INSTALL_DIR}/${CMT_LIB_SO}: ${CMT_ROOT_DIR}/${CMT_LIB_SO}
	${MTF_LIB_INSTALL_CMD_PREFIX} cp -f $< $@	
${CMT_ROOT_DIR}/${CMT_LIB_SO}: ${CMT_LIB_SRC} ${CMT_LIB_HEADERS}
	@echo ${CMT_LIB_SO}
	cd ${CMT_ROOT_DIR}; rm -rf Build; mkdir Build; cd Build; cmake -D CMT_LIB_NAME=${CMT_LIB_NAME} ..
	$(MAKE) -C ${CMT_ROOT_DIR}/Build --no-print-directory
	mv ${CMT_ROOT_DIR}/Build/${CMT_LIB_SO} ${CMT_ROOT_DIR}/