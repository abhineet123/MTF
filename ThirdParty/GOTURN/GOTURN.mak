CAFFE_INSTALL_DIR ?= ~/caffe/build/install

GOTURN_ROOT_DIR = ThirdParty/GOTURN
GOTURN_SRC_DIR = ${GOTURN_ROOT_DIR}/src
GOTURN_INCLUDE_DIR = ${GOTURN_ROOT_DIR}/include
GOTURN_HEADER_DIR = ${GOTURN_INCLUDE_DIR}/mtf/${GOTURN_ROOT_DIR}
GOTURN_HEADERS = $(addprefix  ${GOTURN_HEADER_DIR}/, GOTURN.h)
GOTURN_LIB_NAME = $(addsuffix ${LIB_POST_FIX}, goturn)
GOTURN_LIB_SO =  $(addprefix lib, $(addsuffix .so, ${GOTURN_LIB_NAME}))
GOTURN_LIB_DEFS=

CAFFE_INCLUDE_DIR = ${CAFFE_INSTALL_DIR}/include
CAFFE_LIB_DIR = ${CAFFE_INSTALL_DIR}/lib

gtrn ?= 0
gtrn_exe ?= 0

ifeq ($(OS),Windows_NT)
THIRD_PARTY_RUNTIME_FLAGS += -D DISABLE_GOTURN
else
ifeq (${gtrn}, 0)
THIRD_PARTY_RUNTIME_FLAGS += -D DISABLE_GOTURN
else
use_caffe = 1
THIRD_PARTY_TRACKERS += GOTURN
THIRD_PARTY_HEADERS += ${GOTURN_HEADERS} ${GOTURN_LIB_HEADERS}
THIRD_PARTY_INCLUDE_DIRS += ${GOTURN_INCLUDE_DIR}
THIRD_PARTY_LINK_LIBS += -lcaffe -ltinyxml -L${CAFFE_LIB_DIR}
_THIRD_PARTY_TRACKERS_SO += ${GOTURN_LIB_NAME}
THIRD_PARTY_TRACKERS_SO_LOCAL += ${GOTURN_ROOT_DIR}/${GOTURN_LIB_SO}
THIRD_PARTY_RUNTIME_FLAGS += -I${CAFFE_INCLUDE_DIR}
#THIRD_PARTY_RUNTIME_FLAGS += ${CAFFE_FLAGS}
LIBS_BOOST += -lboost_regex
endif
endif

ifeq (${caffe_cpu}, 1)
GOTURN_LIB_DEFS=CPU_ONLY
endif

GOTURN_LIB_MODULES = helper/image_proc helper/bounding_box helper/helper helper/high_res_timer network/regressor_train_base network/regressor network/regressor_base network/regressor_train tracker/tracker_manager tracker/tracker train/example_generator train/tracker_trainer loader/video_loader loader/loader_alov loader/loader_imagenet_det loader/loader_vot loader/video native/vot
GOTURN_EXE_MODULES = 
ifeq (${gtrn_exe}, 1)
GOTURN_EXE_MODULES += visualizer/show_tracker_vot visualizer/show_alov visualizer/show_imagenet visualizer/show_tracker_alov train/train train/pretrain
endif

GOTURN_LIB_HEADERS = $(addprefix ${GOTURN_HEADER_DIR}/,$(addsuffix .h, ${GOTURN_LIB_MODULES}))
GOTURN_LIB_SRC = $(addprefix ${GOTURN_SRC_DIR}/,$(addsuffix .cpp, ${GOTURN_LIB_MODULES}))
GOTURN_EXE_SRC = $(addprefix ${GOTURN_SRC_DIR}/,$(addsuffix .cpp, ${GOTURN_EXE_MODULES}))

${BUILD_DIR}/GOTURN.o: ${GOTURN_SRC_DIR}/GOTURN.cc ${GOTURN_HEADERS} ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${ROOT_HEADER_DIR}/TrackerBase.h
	${CXX} ${MTF_PIC_FLAG} -c ${WARNING_FLAGS} ${OPT_FLAGS} $< -std=c++11 ${OPENCV_FLAGS} ${MTF_COMPILETIME_FLAGS} ${CAFFE_FLAGS} -I${GOTURN_INCLUDE_DIR} -I${CAFFE_INCLUDE_DIR} -I${UTILITIES_INCLUDE_DIR} -I${MACROS_INCLUDE_DIR} -I${ROOT_INCLUDE_DIR} -o  $@

${MTF_LIB_INSTALL_DIR}/${GOTURN_LIB_SO}: ${GOTURN_ROOT_DIR}/${GOTURN_LIB_SO}
	${MTF_LIB_INSTALL_CMD_PREFIX} cp -f $< $@	
${GOTURN_ROOT_DIR}/${GOTURN_LIB_SO}: ${GOTURN_LIB_SRC} ${GOTURN_EXE_SRC} ${GOTURN_LIB_HEADERS}
	cd ${GOTURN_ROOT_DIR}; rm -rf Build; mkdir Build; cd Build; cmake -D LIB_NAME=${GOTURN_LIB_NAME} -D LIB_DEFS=${GOTURN_LIB_DEFS} ..
	$(MAKE) -C ${GOTURN_ROOT_DIR}/Build --no-print-directory
	mv ${GOTURN_ROOT_DIR}/Build/${GOTURN_LIB_SO} ${GOTURN_ROOT_DIR}/