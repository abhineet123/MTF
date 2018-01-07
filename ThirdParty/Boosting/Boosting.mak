Boosting_ROOT_DIR = ThirdParty/Boosting
Boosting_SRC_DIR = ${Boosting_ROOT_DIR}/src
Boosting_INCLUDE_DIR = ${Boosting_ROOT_DIR}/include
Boosting_HEADER_DIR = ${Boosting_INCLUDE_DIR}/mtf/${Boosting_ROOT_DIR}
Boosting_HEADERS = $(addprefix  ${Boosting_HEADER_DIR}/, Boosting.h)
Boosting_LIB_NAME = $(addsuffix ${LIB_POST_FIX}, mil)
Boosting_LIB_SO =  $(addprefix lib, $(addsuffix .so, ${Boosting_LIB_NAME}))

mil ?= 1
ifeq ($(OS),Windows_NT)
THIRD_PARTY_RUNTIME_FLAGS += -D DISABLE_Boosting
else
ifeq (${mil}, 0)
THIRD_PARTY_RUNTIME_FLAGS += -D DISABLE_Boosting
else
THIRD_PARTY_TRACKERS += Boosting
_THIRD_PARTY_TRACKERS_SO += ${Boosting_LIB_NAME} 
THIRD_PARTY_TRACKERS_SO_LOCAL += ${Boosting_ROOT_DIR}/${Boosting_LIB_SO}
THIRD_PARTY_LIBS_DIRS += -L${Boosting_ROOT_DIR}
THIRD_PARTY_INCLUDE_DIRS += ${Boosting_INCLUDE_DIR}
THIRD_PARTY_HEADERS += ${Boosting_HEADERS} ${Boosting_LIB_HEADERS}
endif
endif

Boosting_LIB_MODULES = cv_onlineboosting cv_onlinemil object_tracker
Boosting_LIB_HEADERS = $(addprefix ${Boosting_HEADER_DIR}/,$(addsuffix .h, ${Boosting_LIB_MODULES}))
Boosting_LIB_SRC = $(addprefix ${Boosting_SRC_DIR}/,$(addsuffix .cpp, ${Boosting_LIB_MODULES}))
Boosting_LIB_SRC += $(addprefix ${Boosting_ROOT_DIR}/samples/,$(addsuffix .cpp, object_tracker_app))


${BUILD_DIR}/Boosting.o: ${Boosting_SRC_DIR}/Boosting.cc ${Boosting_HEADERS} ${UTILITIES_HEADER_DIR}/miscUtils.h  ${UTILITIES_HEADER_DIR}/excpUtils.h  ${MACROS_HEADER_DIR}/common.h ${ROOT_HEADER_DIR}/TrackerBase.h
	${CXX} ${MTF_PIC_FLAG} -c ${WARNING_FLAGS} ${OPT_FLAGS} $< -std=c++11 ${OPENCV_FLAGS} ${MTF_COMPILETIME_FLAGS} -I${Boosting_INCLUDE_DIR} -I${UTILITIES_INCLUDE_DIR} -I${MACROS_INCLUDE_DIR} -I${ROOT_INCLUDE_DIR} -o  $@

${MTF_LIB_INSTALL_DIR}/${Boosting_LIB_SO}: ${Boosting_ROOT_DIR}/${Boosting_LIB_SO}
	${MTF_LIB_INSTALL_CMD_PREFIX} cp -f $< $@	
${Boosting_ROOT_DIR}/${Boosting_LIB_SO}: ${Boosting_LIB_SRC} ${Boosting_LIB_HEADERS}
	cd ${Boosting_ROOT_DIR}; rm -rf Build; mkdir Build; cd Build; cmake -D Boosting_LIB_NAME=${Boosting_LIB_NAME} ..
	$(MAKE) -C ${Boosting_ROOT_DIR}/Build --no-print-directory
	mv ${Boosting_ROOT_DIR}/Build/${Boosting_LIB_SO} ${Boosting_ROOT_DIR}/