MIL_ROOT_DIR = ThirdParty/MIL
MIL_SRC_DIR = ${MIL_ROOT_DIR}/src
MIL_INCLUDE_DIR = ${MIL_ROOT_DIR}/include
MIL_HEADER_DIR = ${MIL_INCLUDE_DIR}/mtf/${MIL_ROOT_DIR}
MIL_HEADERS = $(addprefix  ${MIL_HEADER_DIR}/, MIL.h)
MIL_LIB_NAME = $(addsuffix ${LIB_POST_FIX}, mil)
MIL_LIB_SO =  $(addprefix lib, $(addsuffix .so, ${MIL_LIB_NAME}))

mil ?= 0

ifeq (${mil}, 0)
THIRD_PARTY_RUNTIME_FLAGS += -D DISABLE_MIL
else
THIRD_PARTY_TRACKERS += MIL
_THIRD_PARTY_TRACKERS_SO += ${MIL_LIB_NAME} 
THIRD_PARTY_TRACKERS_SO_LOCAL += ${MIL_ROOT_DIR}/${MIL_LIB_SO}
THIRD_PARTY_LIBS_DIRS += -L${MIL_ROOT_DIR}
THIRD_PARTY_INCLUDE_DIRS += ${MIL_INCLUDE_DIR}
THIRD_PARTY_HEADERS += ${MIL_HEADERS} ${MIL_LIB_HEADERS}
endif

MIL_LIB_MODULES = cv_onlineboosting cv_onlinemil object_tracker
MIL_LIB_HEADERS = $(addprefix ${MIL_HEADER_DIR}/,$(addsuffix .h, ${MIL_LIB_MODULES}))
MIL_LIB_SRC = $(addprefix ${MIL_SRC_DIR}/,$(addsuffix .cpp, ${MIL_LIB_MODULES}))
MIL_LIB_SRC += $(addprefix ${MIL_ROOT_DIR}/samples/,$(addsuffix .cpp, object_tracker_app))


${BUILD_DIR}/MIL.o: ${MIL_SRC_DIR}/MIL.cc ${MIL_HEADERS} ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${ROOT_HEADER_DIR}/TrackerBase.h
	${CXX} ${MTF_PIC_FLAG} -c ${WARNING_FLAGS} ${OPT_FLAGS} $< -std=c++11 ${OPENCV_FLAGS} ${MTF_COMPILETIME_FLAGS} -I${MIL_INCLUDE_DIR} -I${UTILITIES_INCLUDE_DIR} -I${MACROS_INCLUDE_DIR} -I${ROOT_INCLUDE_DIR} -o  $@

${MTF_LIB_INSTALL_DIR}/${MIL_LIB_SO}: ${MIL_ROOT_DIR}/${MIL_LIB_SO}
	${MTF_LIB_INSTALL_CMD_PREFIX} cp -f $< $@	
${MIL_ROOT_DIR}/${MIL_LIB_SO}: ${MIL_LIB_SRC} ${MIL_LIB_HEADERS}
	cd ${MIL_ROOT_DIR}; rm -rf Build; mkdir Build; cd Build; cmake -D LIB_NAME=${MIL_LIB_NAME} ..
	$(MAKE) -C ${MIL_ROOT_DIR}/Build --no-print-directory
	mv ${MIL_ROOT_DIR}/Build/${MIL_LIB_SO} ${MIL_ROOT_DIR}/