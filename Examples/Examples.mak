ifeq ($(OS),Windows_NT)
	MTF_EXEC_INSTALL_DIR ?= C:/MTF/bin
	MTF_TEST_INSTALL_DIR ?= C:/MTF/bin
	MTF_PY_INSTALL_DIR ?= C:/Python27/Lib/site-packages
	PYTHON_INCLUDE_DIR ?= C:/Python27/include 
	PYTHON_LIBS_DIR ?= C:/Python27/libs 
	NUMPY_INCLUDE_DIR ?= C:/Python27/Lib/site-packages/numpy/core/include
	MTF_PY_LIB_NAME ?= pyMTF.pyd
	PYTHON_LIB_NAME ?= python27
else
	MTF_EXEC_INSTALL_DIR ?= /usr/local/bin
	MTF_TEST_INSTALL_DIR ?= /usr/local/bin
	MTF_PY_INSTALL_DIR ?= /usr/local/lib/python2.7/dist-packages/
	PYTHON_INCLUDE_DIR ?= /usr/include/python2.7
	PYTHON_LIBS_DIR ?= 
	NUMPY_INCLUDE_DIR ?= ${PYTHON_INCLUDE_DIR}/numpy
	MTF_PY_LIB_NAME ?= pyMTF.so
	PYTHON_LIB_NAME ?= python2.7
	# add "sudo" to the cp/mkdir commands if the executable installation folder needs administrative access
	ifneq (,$(findstring /usr,$(MTF_EXEC_INSTALL_DIR)))
		MTF_EXE_INSTALL_CMD_PREFIX = sudo
	endif
	ifneq (,$(findstring /usr,$(MTF_PY_INSTALL_DIR)))
		MTF_PY_INSTALL_CMD_PREFIX = sudo
	endif
	ifneq (,$(findstring /usr,$(MTF_TEST_INSTALL_DIR)))
		MTF_TEST_INSTALL_CMD_PREFIX = sudo
	endif
	ifneq (,$(findstring /usr,$(MTF_APP_INSTALL_DIR)))
		MTF_APP_INSTALL_CMD_PREFIX = sudo
	endif
	FLAGS_TBB +=  -L/opt/intel/composer_xe_2015/tbb/lib/intel64/gcc4.4
endif


MTF_APP_INSTALL_DIR ?= .
MTF_APP_SRC_DIR ?= ${EXAMPLES_ROOT_DIR}/src
PYTHON_LIBS = $(addprefix -l, ${PYTHON_LIB_NAME})
PYTHON_LIBS_FLAGS = $(addprefix -L, ${PYTHON_LIBS_DIR})

header_only ?= 0
utt ?= 0
uavold ?= 0
am ?= ssd
ssm ?= 4
app_name ?= mtfApp

EXAMPLES_ROOT_DIR = Examples
EXAMPLES_SRC_DIR = ${EXAMPLES_ROOT_DIR}/src
EXAMPLES_INCLUDE_DIR = ${EXAMPLES_ROOT_DIR}/include
EXAMPLES_HEADER_DIR = ${EXAMPLES_INCLUDE_DIR}/mtf/Tools
EXAMPLES_INCLUDE_FLAGS = -I${EXAMPLES_INCLUDE_DIR}
EXAMPLES_TOOLS = PreProc cvUtils inputBase inputCV pipeline

EXAMPLES_TOOLS_HEADERS = $(addprefix ${EXAMPLES_HEADER_DIR}/,$(addsuffix .h, ${EXAMPLES_TOOLS}))
MTF_HEADERS += ${EXAMPLES_TOOLS_HEADERS}
MTF_INCLUDE_DIRS += ${EXAMPLES_INCLUDE_DIR}

UAV_FLAGS =
MTF_AM = SSD
MTF_SSM = Similitude

ifeq (${use_caffe}, 1)
	MTF_LIBS += -lstdc++ -lglog -lcaffe
	MTF_LIBS_DIRS +=  -L${CAFFE_LIBRARY_PATH}
	MTF_RUNTIME_FLAGS += ${CAFFE_FLAGS}
endif

ifeq (${utt}, 1)
	UAV_FLAGS += -DUSE_TEMPLATED_SM -DMTF_AM=${MTF_AM} -DMTF_SSM=${MTF_SSM}
endif

ifeq (${uavold}, 1)
	UAV_FLAGS += -DUSE_OLD_METHOD
endif
ifeq (${am}, ssd)
	MTF_AM= SSD
else ifeq (${am}, mi)
	MTF_AM= MI
else ifeq (${am}, ncc)
	MTF_AM= NCC
endif

ifeq (${header_only}, 1)
	MTF_RUNTIME_FLAGS += -DHEADER_ONLY_MODE -DDISABLE_THIRD_PARTY_TRACKERS
	MTF_LIB_LINK = 
endif

ifeq (${ssm}, 2)
	MTF_SSM = Translation
else ifeq (${ssm}, 4)
	MTF_SSM = Similitude
else ifeq (${ssm}, 6)
	MTF_SSM = Affine
else ifeq (${ssm}, 8)
	MTF_SSM = Homography
endif

ifeq (${o}, 1)
	# LIBS_PARALLEL += -ltbb
	MTF_RUNTIME_FLAGS += -O3 -D NDEBUG -D EIGEN_NO_DEBUG
	ifeq (${header_only}, 1)
		_MTF_EXE_NAME = runMTFh
	else
		_MTF_EXE_NAME = runMTF
	endif
	_MTF_TEST_EXE_NAME = testMTF
	_MTF_PATCH_EXE_NAME = extractPatch
	_MTF_UAV_EXE_NAME = trackUAVTrajectory
	_MTF_GT_EXE_NAME = showGroundTruth
	_MTF_SYN_EXE_NAME = generateSyntheticSeq
	_MTF_MOS_EXE_NAME = createMosaic
	_MTF_REC_EXE_NAME = recordSeq
	_MTF_APP_EXE_NAME = ${app_name}
else
	MTF_RUNTIME_FLAGS += -g -O0
	ifeq (${header_only}, 1)
		_MTF_EXE_NAME = runMTFdh
	else
		_MTF_EXE_NAME = runMTFd
	endif
	_MTF_TEST_EXE_NAME = testMTFd
	_MTF_PATCH_EXE_NAME = extractPatchd
	_MTF_UAV_EXE_NAME = trackUAVTrajectoryd
	_MTF_GT_EXE_NAME = showGroundTruthd
	_MTF_SYN_EXE_NAME = generateSyntheticSeqd
	_MTF_MOS_EXE_NAME = createMosaicd
	_MTF_REC_EXE_NAME = recordSeqd
	_MTF_APP_EXE_NAME = $(addsuffix d, ${app_name})				
endif

MTF_EXE_NAME = $(addsuffix ${LIB_POST_FIX}${MTF_EXE_EXT}, ${_MTF_EXE_NAME})
MTF_TEST_EXE_NAME = $(addsuffix ${LIB_POST_FIX}${MTF_EXE_EXT}, ${_MTF_TEST_EXE_NAME})
MTF_PATCH_EXE_NAME = $(addsuffix ${LIB_POST_FIX}${MTF_EXE_EXT}, ${_MTF_PATCH_EXE_NAME})
MTF_UAV_EXE_NAME = $(addsuffix ${LIB_POST_FIX}${MTF_EXE_EXT}, ${_MTF_UAV_EXE_NAME})
MTF_GT_EXE_NAME = $(addsuffix ${LIB_POST_FIX}${MTF_EXE_EXT}, ${_MTF_GT_EXE_NAME})
MTF_SYN_EXE_NAME = $(addsuffix ${LIB_POST_FIX}${MTF_EXE_EXT}, ${_MTF_SYN_EXE_NAME})
MTF_MOS_EXE_NAME = $(addsuffix ${LIB_POST_FIX}${MTF_EXE_EXT}, ${_MTF_MOS_EXE_NAME})
MTF_REC_EXE_NAME = $(addsuffix ${LIB_POST_FIX}${MTF_EXE_EXT}, ${_MTF_REC_EXE_NAME})
MTF_APP_EXE_NAME = $(addsuffix ${LIB_POST_FIX}${MTF_EXE_EXT}, ${_MTF_APP_EXE_NAME})

ifeq (${vp}, 1)
	EXAMPLES_TOOLS += inputVP
	MTF_RUNTIME_FLAGS += -lvisp_io -lvisp_sensor
endif

.PHONY: exe uav mos syn py test gt patch app mtfi mtfp mtfc mtfu mtft mtfs mtfm app
.PHONY: install_exe install_uav install_patch install_py install_test
.PHONY: run

exe: ${BUILD_DIR}/${MTF_EXE_NAME}
uav: ${BUILD_DIR}/${MTF_UAV_EXE_NAME}
mos: ${BUILD_DIR}/${MTF_MOS_EXE_NAME}
syn: ${BUILD_DIR}/${MTF_SYN_EXE_NAME}
py: ${BUILD_DIR}/${MTF_PY_LIB_NAME}
test: ${BUILD_DIR}/${MTF_TEST_EXE_NAME}
gt: ${BUILD_DIR}/${MTF_GT_EXE_NAME}
patch: ${BUILD_DIR}/${MTF_PATCH_EXE_NAME}
rec: ${BUILD_DIR}/${MTF_REC_EXE_NAME}
app: ${BUILD_DIR}/${MTF_APP_EXE_NAME}
all: exe uav mos syn py test gt patch rec

install_exe: ${MTF_EXEC_INSTALL_DIR}/${MTF_EXE_NAME}
install_gt: ${MTF_EXEC_INSTALL_DIR}/${MTF_GT_EXE_NAME}
install_uav: ${MTF_EXEC_INSTALL_DIR}/${MTF_UAV_EXE_NAME}
install_mos: ${MTF_EXEC_INSTALL_DIR}/${MTF_MOS_EXE_NAME}
install_patch: ${MTF_EXEC_INSTALL_DIR}/${MTF_PATCH_EXE_NAME}
install_syn: ${MTF_EXEC_INSTALL_DIR}/${MTF_SYN_EXE_NAME}
install_rec: ${MTF_EXEC_INSTALL_DIR}/${MTF_REC_EXE_NAME}
install_py: ${MTF_PY_INSTALL_DIR}/${MTF_PY_LIB_NAME}
install_test: ${MTF_TEST_INSTALL_DIR}/${MTF_TEST_EXE_NAME}
install_app: ${MTF_APP_INSTALL_DIR}/${MTF_APP_EXE_NAME}
install_all: install_exe install_uav install_mos install_syn install_py install_test install_gt install_patch install_rec

mtfi: install install_exe
mtfpa: install install_patch
mtfp: install install_py
mtfu: install install_uav
mtfg: install install_gt
mtfs: install install_syn
mtfm: install install_mos
mtfr: install_rec
mtft: install install_test_lib install_test
mtfall: install install_test_lib install_all
mtfa: install install_app
mtfc: all
	${RM_CMD} ${BUILD_DIR}/${MTF_EXE_NAME}

run: install_exe
	${MTF_EXE_NAME}
${MTF_EXEC_INSTALL_DIR}/${MTF_EXE_NAME}: ${BUILD_DIR}/${MTF_EXE_NAME}
	${MTF_EXE_INSTALL_CMD_PREFIX} ${CP_CMD} $< $@	
${MTF_EXEC_INSTALL_DIR}/${MTF_UAV_EXE_NAME}: ${BUILD_DIR}/${MTF_UAV_EXE_NAME}
	${MTF_EXE_INSTALL_CMD_PREFIX} ${CP_CMD} $< $@
${MTF_EXEC_INSTALL_DIR}/${MTF_PATCH_EXE_NAME}: ${BUILD_DIR}/${MTF_PATCH_EXE_NAME}
	${MTF_EXE_INSTALL_CMD_PREFIX} ${CP_CMD} $< $@
${MTF_EXEC_INSTALL_DIR}/${MTF_SYN_EXE_NAME}: ${BUILD_DIR}/${MTF_SYN_EXE_NAME}
	${MTF_EXE_INSTALL_CMD_PREFIX} ${CP_CMD} $< $@
${MTF_EXEC_INSTALL_DIR}/${MTF_GT_EXE_NAME}: ${BUILD_DIR}/${MTF_GT_EXE_NAME}
	${MTF_EXE_INSTALL_CMD_PREFIX} ${CP_CMD} $< $@
${MTF_EXEC_INSTALL_DIR}/${MTF_MOS_EXE_NAME}: ${BUILD_DIR}/${MTF_MOS_EXE_NAME}
	${MTF_EXE_INSTALL_CMD_PREFIX} ${CP_CMD} $< $@
${MTF_PY_INSTALL_DIR}/${MTF_PY_LIB_NAME}: ${BUILD_DIR}/${MTF_PY_LIB_NAME}
	${MTF_PY_INSTALL_CMD_PREFIX} ${CP_CMD} $< $@
${MTF_TEST_INSTALL_DIR}/${MTF_TEST_EXE_NAME}: ${BUILD_DIR}/${MTF_TEST_EXE_NAME}
	${MTF_TEST_INSTALL_CMD_PREFIX} ${CP_CMD} $< $@
${MTF_TEST_INSTALL_DIR}/${MTF_REC_EXE_NAME}: ${BUILD_DIR}/${MTF_REC_EXE_NAME}
	${MTF_TEST_INSTALL_CMD_PREFIX} ${CP_CMD} $< $@
${MTF_APP_INSTALL_DIR}/${MTF_APP_EXE_NAME}: ${BUILD_DIR}/${MTF_APP_EXE_NAME}
	${MTF_APP_INSTALL_CMD_PREFIX} ${CP_CMD} $< $@	
	

${BUILD_DIR}/${MTF_EXE_NAME}: | ${BUILD_DIR}	
${BUILD_DIR}/${MTF_PY_LIB_NAME}: | ${BUILD_DIR}	
${BUILD_DIR}/${MTF_TEST_EXE_NAME}: | ${BUILD_DIR}
${BUILD_DIR}/${MTF_REC_EXE_NAME}: | ${BUILD_DIR}
${BUILD_DIR}/${MTF_UAV_EXE_NAME}: | ${BUILD_DIR}	
${BUILD_DIR}/${MTF_GT_EXE_NAME}: | ${BUILD_DIR}	
${BUILD_DIR}/${MTF_SYN_EXE_NAME}: | ${BUILD_DIR}	
${BUILD_DIR}/${MTF_MOS_EXE_NAME}: | ${BUILD_DIR}	
${BUILD_DIR}/${MTF_PATCH_EXE_NAME}: | ${BUILD_DIR}	
${BUILD_DIR}/${MTF_APP_EXE_NAME}: | ${BUILD_DIR}	

${BUILD_DIR}/${MTF_EXE_NAME}: ${EXAMPLES_SRC_DIR}/runMTF.cc ${MTF_HEADERS} ${ROOT_HEADER_DIR}/mtf.h
	${CXX}  $< -o $@ -w ${WARNING_FLAGS} ${MTF_RUNTIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${EXAMPLES_INCLUDE_FLAGS} ${OPENCV_FLAGS} ${MTF_LIB_LINK} ${LIBS} ${BOOST_LIBS} ${LIBS_PARALLEL} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${OPENCV_LIBS} 
	
${BUILD_DIR}/${MTF_UAV_EXE_NAME}: ${EXAMPLES_SRC_DIR}/trackUAVTrajectory.cc ${MTF_HEADERS} ${ROOT_HEADER_DIR}/mtf.h
	${CXX} $< -o $@ -w ${UAV_FLAGS} ${WARNING_FLAGS} ${MTF_RUNTIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${EXAMPLES_INCLUDE_FLAGS} ${OPENCV_FLAGS} ${MTF_LIB_LINK} ${LIBS} ${BOOST_LIBS} ${LIBS_PARALLEL} ${MTF_LIBS_DIRS} ${MTF_LIBS}  ${OPENCV_LIBS} 
	
${BUILD_DIR}/${MTF_PATCH_EXE_NAME}: ${EXAMPLES_SRC_DIR}/extractPatch.cc ${MTF_HEADERS} ${ROOT_HEADER_DIR}/mtf.h
	${CXX} $< -o $@ -w ${WARNING_FLAGS} ${MTF_RUNTIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${EXAMPLES_INCLUDE_FLAGS} ${OPENCV_FLAGS} ${MTF_LIB_LINK} ${LIBS} ${BOOST_LIBS} ${LIBS_PARALLEL} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${OPENCV_LIBS} 
	
${BUILD_DIR}/${MTF_PY_LIB_NAME}: ${BUILD_DIR}/pyMTF.o 
	${CXX} -shared $< -o $@ ${MTF_LIB_LINK} ${LIBS} ${LIBS_PARALLEL} ${MTF_RUNTIME_FLAGS} ${MTF_LIB_LINK} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${OPENCV_LIBS} ${BOOST_LIBS} ${PYTHON_LIBS_FLAGS} ${PYTHON_LIBS}
	
${BUILD_DIR}/pyMTF.o: ${EXAMPLES_SRC_DIR}/pyMTF.cc ${MTF_HEADERS} ${ROOT_HEADER_DIR}/mtf.h
	${CXX} -w -c ${MTF_PIC_FLAG} $< ${WARNING_FLAGS} -D_hypot=hypot ${OPENCV_FLAGS} ${MTF_RUNTIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${EXAMPLES_INCLUDE_FLAGS} -I${PYTHON_INCLUDE_DIR} -I${NUMPY_INCLUDE_DIR} -o $@

${BUILD_DIR}/${MTF_GT_EXE_NAME}: ${EXAMPLES_SRC_DIR}/showGroundTruth.cc ${EXAMPLES_TOOLS_HEADERS} ${UTILITIES_HEADER_DIR}/miscUtils.h  ${ROOT_HEADER_DIR}/mtf.h
	${CXX}  $< -o $@ -w ${WARNING_FLAGS} ${MTF_RUNTIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${EXAMPLES_INCLUDE_FLAGS} ${OPENCV_FLAGS} ${LIBS} ${MTF_LIB_LINK} ${BOOST_LIBS} ${LIBS_PARALLEL} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${OPENCV_LIBS} 
	
${BUILD_DIR}/${MTF_REC_EXE_NAME}: ${EXAMPLES_SRC_DIR}/recordSeq.cc ${EXAMPLES_TOOLS_HEADERS}
	${CXX}  $< -o $@ -w ${WARNING_FLAGS} ${MTF_RUNTIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${EXAMPLES_INCLUDE_FLAGS} ${CONFIG_INCLUDE_FLAGS} ${MTF_LIBS_DIRS} ${OPENCV_FLAGS} ${LIBS} ${OPENCV_LIBS} ${BOOST_LIBS} ${LIBS_PARALLEL}
	
${BUILD_DIR}/${MTF_SYN_EXE_NAME}: ${EXAMPLES_SRC_DIR}/generateSyntheticSeq.cc ${MTF_HEADERS} ${ROOT_HEADER_DIR}/mtf.h
	${CXX}  $< -o $@ -w ${WARNING_FLAGS} ${MTF_RUNTIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${EXAMPLES_INCLUDE_FLAGS} ${OPENCV_FLAGS} ${LIBS} ${MTF_LIB_LINK} ${BOOST_LIBS} ${LIBS_PARALLEL} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${OPENCV_LIBS} 
	
${BUILD_DIR}/${MTF_MOS_EXE_NAME}: ${EXAMPLES_SRC_DIR}/createMosaic.cc ${MTF_HEADERS} ${ROOT_HEADER_DIR}/mtf.h
	${CXX}  $< -o $@ -w ${WARNING_FLAGS} ${MTF_RUNTIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${EXAMPLES_INCLUDE_FLAGS} ${OPENCV_FLAGS} ${LIBS} ${MTF_LIB_LINK} ${BOOST_LIBS} ${LIBS_PARALLEL} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${OPENCV_LIBS} 
	
${BUILD_DIR}/${MTF_TEST_EXE_NAME}: ${EXAMPLES_SRC_DIR}/testMTF.cc ${TEST_HEADERS} ${MTF_HEADERS} ${CONFIG_HEADERS}  ${MTF_HEADERS} ${ROOT_HEADER_DIR}/mtf.h
	${CXX} $< -o $@ -w ${WARNING_FLAGS} ${MTF_RUNTIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${EXAMPLES_INCLUDE_FLAGS} ${OPENCV_FLAGS} ${MTF_LIB_LINK} ${LIBS} ${LIBS_PARALLEL} ${BOOST_LIBS} ${MTF_TEST_LIBS} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${OPENCV_LIBS} 
	
${BUILD_DIR}/${MTF_APP_EXE_NAME}: ${MTF_APP_SRC_DIR}/${app_name}.cc ${MTF_HEADERS} ${ROOT_HEADER_DIR}/mtf.h
	${CXX}  $< -o $@ -w ${WARNING_FLAGS} ${MTF_RUNTIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${EXAMPLES_INCLUDE_FLAGS} ${OPENCV_FLAGS} ${MTF_LIB_LINK} ${LIBS} ${BOOST_LIBS} ${LIBS_PARALLEL} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${OPENCV_LIBS} 
