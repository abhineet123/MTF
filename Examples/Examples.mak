MTF_EXEC_INSTALL_DIR ?= $(MTF_INSTALL_DIR)/bin
MTF_DIAG_INSTALL_DIR ?= $(MTF_INSTALL_DIR)/bin

ifeq ($(OS),Windows_NT)
	MTF_PY_INSTALL_DIR ?= C:/Python27/Lib/site-packages
	PYTHON_INCLUDE_DIR ?= C:/Python27/include 
	PYTHON_LIBS_DIR ?= C:/Python27/libs 
	NUMPY_INCLUDE_DIR ?= C:/Python27/Lib/site-packages/numpy/core/include
	MTF_PY_LIB_NAME ?= pyMTF.pyd
	PYTHON_LIB_NAME ?= python27
	MATLAB_DIR ?= E:/Program\ Files/MATLAB/R2013a
	MEX_EXT = mexw64
else
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
	ifneq (,$(findstring /usr,$(MTF_DIAG_INSTALL_DIR)))
		MTF_DIAG_INSTALL_CMD_PREFIX = sudo
	endif
	ifneq (,$(findstring /usr,$(MTF_APP_INSTALL_DIR)))
		MTF_APP_INSTALL_CMD_PREFIX = sudo
	endif
	FLAGS_TBB +=  -L/opt/intel/composer_xe_2015/tbb/lib/intel64/gcc4.4
	MATLAB_DIR ?= /usr/local/MATLAB/MATLAB_Production_Server/R2013a/
	MEX_EXT = mexa64
endif

MTF_MEX_INSTALL_DIR ?= $(MATLAB_DIR)/toolbox/local

EXAMPLE_TARGETS = exe uav mos syn gt patch rec py diag

# check if MATLAB folder exists
ifneq ($(wildcard ${MATLAB_DIR}/.),)
	EXAMPLE_TARGETS += mex
	MEX_EXT = $(shell $(MATLAB_DIR)/bin/mexext)
endif
# if [ -d "${MATLAB_DIR}" ]; then EXAMPLE_TARGETS += mex; elsefi
ifeq (${feat}, 1)
	EXAMPLE_TARGETS += qr
endif
EXAMPLE_INSTALL_TARGETS = $(addprefix install_,${EXAMPLE_TARGETS})

header_only ?= 0
utt ?= 0
uavold ?= 0
am ?= ssd
ssm ?= 4
app ?= mtfApp

EXAMPLES_ROOT_DIR = Examples
EXAMPLES_SRC_DIR = ${EXAMPLES_ROOT_DIR}/cpp
EXAMPLES_INCLUDE_FLAGS = -I${ROOT_HEADER_DIR}
EXAMPLES_TOOLS = mtf pipeline

MTF_APP_INSTALL_DIR ?= .
MTF_APP_SRC_DIR ?= ${EXAMPLES_ROOT_DIR}/cpp
PYTHON_LIBS = $(addprefix -l, ${PYTHON_LIB_NAME})
PYTHON_LIBS_FLAGS = $(addprefix -L, ${PYTHON_LIBS_DIR})

EXAMPLES_HEADERS = $(addprefix ${ROOT_HEADER_DIR}/,$(addsuffix .h, ${EXAMPLES_TOOLS}))
MTF_HEADERS += ${EXAMPLES_HEADERS}
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
	_MTF_DIAG_EXE_NAME = diagnoseMTF
	_MTF_PATCH_EXE_NAME = extractPatch
	_MTF_UAV_EXE_NAME = trackUAVTrajectory
	_MTF_GT_EXE_NAME = showGroundTruth
	_MTF_SYN_EXE_NAME = generateSyntheticSeq
	_MTF_MOS_EXE_NAME = createMosaic
	_MTF_REC_EXE_NAME = recordSeq
	_MTF_QR_EXE_NAME = trackMarkers
	_MTF_MEX_MODULE_NAME = mexMTF
	_MTF_APP_EXE_NAME = ${app}
else ifeq (${o}, 2)
	# LIBS_PARALLEL += -ltbb
	MTF_RUNTIME_FLAGS += -O3 -ffast-math -D NDEBUG -D EIGEN_NO_DEBUG
	ifeq (${header_only}, 1)
		_MTF_EXE_NAME = runMTFh_fast
	else
		_MTF_EXE_NAME = runMTF_fast
	endif
	_MTF_DIAG_EXE_NAME = diagnoseMTF_fast
	_MTF_PATCH_EXE_NAME = extractPatch_fast
	_MTF_UAV_EXE_NAME = trackUAVTrajectory_fast
	_MTF_GT_EXE_NAME = showGroundTruth_fast
	_MTF_SYN_EXE_NAME = generateSyntheticSeq_fast
	_MTF_MOS_EXE_NAME = createMosaic_fast
	_MTF_REC_EXE_NAME = recordSeq_fast
	_MTF_QR_EXE_NAME = trackMarkers_fast
	_MTF_MEX_MODULE_NAME = mexMTF_fast
	_MTF_APP_EXE_NAME = $(addsuffix _fast, ${app})	
else
	MTF_RUNTIME_FLAGS += -g -O0
	ifeq (${header_only}, 1)
		_MTF_EXE_NAME = runMTFh_debug
	else
		_MTF_EXE_NAME = runMTF_debug
	endif
	_MTF_DIAG_EXE_NAME = diagnoseMTF_debug
	_MTF_PATCH_EXE_NAME = extractPatch_debug
	_MTF_UAV_EXE_NAME = trackUAVTrajectory_debug
	_MTF_GT_EXE_NAME = showGroundTruth_debug
	_MTF_SYN_EXE_NAME = generateSyntheticSeq_debug
	_MTF_MOS_EXE_NAME = createMosaic_debug
	_MTF_REC_EXE_NAME = recordSeq_debug
	_MTF_QR_EXE_NAME = trackMarkers_debug
	_MTF_MEX_MODULE_NAME = mexMTF_debug
	_MTF_APP_EXE_NAME = $(addsuffix _debug, ${app})				
endif

MTF_EXE_NAME = $(addsuffix ${LIB_POST_FIX}${MTF_EXE_EXT}, ${_MTF_EXE_NAME})
MTF_DIAG_EXE_NAME = $(addsuffix ${LIB_POST_FIX}${MTF_EXE_EXT}, ${_MTF_DIAG_EXE_NAME})
MTF_PATCH_EXE_NAME = $(addsuffix ${LIB_POST_FIX}${MTF_EXE_EXT}, ${_MTF_PATCH_EXE_NAME})
MTF_UAV_EXE_NAME = $(addsuffix ${LIB_POST_FIX}${MTF_EXE_EXT}, ${_MTF_UAV_EXE_NAME})
MTF_GT_EXE_NAME = $(addsuffix ${LIB_POST_FIX}${MTF_EXE_EXT}, ${_MTF_GT_EXE_NAME})
MTF_SYN_EXE_NAME = $(addsuffix ${LIB_POST_FIX}${MTF_EXE_EXT}, ${_MTF_SYN_EXE_NAME})
MTF_MOS_EXE_NAME = $(addsuffix ${LIB_POST_FIX}${MTF_EXE_EXT}, ${_MTF_MOS_EXE_NAME})
MTF_REC_EXE_NAME = $(addsuffix ${LIB_POST_FIX}${MTF_EXE_EXT}, ${_MTF_REC_EXE_NAME})
MTF_QR_EXE_NAME = $(addsuffix ${LIB_POST_FIX}${MTF_EXE_EXT}, ${_MTF_QR_EXE_NAME})
MTF_APP_EXE_NAME = $(addsuffix ${LIB_POST_FIX}${MTF_EXE_EXT}, ${_MTF_APP_EXE_NAME})

MEX_CFLAGS =  -fPIC ${WARNING_FLAGS} ${OPENCV_FLAGS} ${MTF_RUNTIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${EXAMPLES_INCLUDE_FLAGS} 
MEX = $(MATLAB_DIR)/bin/mex
MEX_OPTION = CC='$(CXX)' CXX='$(CXX)' CFLAGS='$(MEX_CFLAGS)' CXXFLAGS='$(MEX_CFLAGS)'
# MEX_OPTION += -largeArrayDims
MEX_LIBS = ${MTF_LIBS_DIRS} ${MTF_LIB_LINK} ${LIBS} ${BOOST_LIBS} ${LIBS_PARALLEL} ${MTF_LIBS} ${OPENCV_LIBS}
MTF_MEX_MODULE_NAME = $(addsuffix ${LIB_POST_FIX}.${MEX_EXT}, ${_MTF_MEX_MODULE_NAME})

ifeq (${vp}, 1)
	MTF_RUNTIME_FLAGS += -lvisp_io -lvisp_sensor
endif


.PHONY: exe uav mos syn py diag gt patch qr app mtfi mtfp mtfc mtfu mtfd mtfs mtfm
.PHONY: install_exe install_uav install_mos install_patch install_qr install_rec install_syn install_py install_diag install_app install_all
.PHONY: run

exe: ${BUILD_DIR}/${MTF_EXE_NAME}
uav: ${BUILD_DIR}/${MTF_UAV_EXE_NAME}
mos: ${BUILD_DIR}/${MTF_MOS_EXE_NAME}
syn: ${BUILD_DIR}/${MTF_SYN_EXE_NAME}
py: ${BUILD_DIR}/${MTF_PY_LIB_NAME}
mex: ${BUILD_DIR}/${MTF_MEX_MODULE_NAME}
diag: ${BUILD_DIR}/${MTF_DIAG_EXE_NAME}
gt: ${BUILD_DIR}/${MTF_GT_EXE_NAME}
patch: ${BUILD_DIR}/${MTF_PATCH_EXE_NAME}
rec: ${BUILD_DIR}/${MTF_REC_EXE_NAME}
qr: ${BUILD_DIR}/${MTF_QR_EXE_NAME}
app: ${BUILD_DIR}/${MTF_APP_EXE_NAME}
all: ${EXAMPLE_TARGETS}

install_exe: ${MTF_EXEC_INSTALL_DIR}/${MTF_EXE_NAME}
install_gt: ${MTF_EXEC_INSTALL_DIR}/${MTF_GT_EXE_NAME}
install_uav: ${MTF_EXEC_INSTALL_DIR}/${MTF_UAV_EXE_NAME}
install_mos: ${MTF_EXEC_INSTALL_DIR}/${MTF_MOS_EXE_NAME}
install_patch: ${MTF_EXEC_INSTALL_DIR}/${MTF_PATCH_EXE_NAME}
install_syn: ${MTF_EXEC_INSTALL_DIR}/${MTF_SYN_EXE_NAME}
install_rec: ${MTF_EXEC_INSTALL_DIR}/${MTF_REC_EXE_NAME}
install_qr: ${MTF_EXEC_INSTALL_DIR}/${MTF_QR_EXE_NAME}
install_py: ${MTF_PY_INSTALL_DIR}/${MTF_PY_LIB_NAME}
install_mex: ${MTF_MEX_INSTALL_DIR}/${MTF_MEX_MODULE_NAME}
install_diag: ${MTF_DIAG_INSTALL_DIR}/${MTF_DIAG_EXE_NAME}
install_app: ${MTF_APP_INSTALL_DIR}/${MTF_APP_EXE_NAME}
install_all: ${EXAMPLE_INSTALL_TARGETS}

mtfi: install install_exe
mtfpa: install install_patch
mtfp: install install_py
mtfx: install install_mex
mtfu: install install_uav
mtfg: install install_gt
mtfs: install install_syn
mtfm: install install_mos
mtfq: install install_qr
mtfr: install_rec
mtfd: install install_diag_lib install_diag
mtfall: install install_diag_lib install_all
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
${MTF_EXEC_INSTALL_DIR}/${MTF_REC_EXE_NAME}: ${BUILD_DIR}/${MTF_REC_EXE_NAME}
	${MTF_EXE_INSTALL_CMD_PREFIX} ${CP_CMD} $< $@
${MTF_EXEC_INSTALL_DIR}/${MTF_QR_EXE_NAME}: ${BUILD_DIR}/${MTF_QR_EXE_NAME}
	${MTF_EXE_INSTALL_CMD_PREFIX} ${CP_CMD} $< $@
${MTF_MEX_INSTALL_DIR}/${MTF_MEX_MODULE_NAME}: ${BUILD_DIR}/${MTF_MEX_MODULE_NAME}
	${MTF_EXE_INSTALL_CMD_PREFIX} ${CP_CMD} $< $@
${MTF_APP_INSTALL_DIR}/${MTF_APP_EXE_NAME}: ${BUILD_DIR}/${MTF_APP_EXE_NAME}
	${MTF_APP_INSTALL_CMD_PREFIX} ${CP_CMD} $< $@	
${MTF_PY_INSTALL_DIR}/${MTF_PY_LIB_NAME}: ${BUILD_DIR}/${MTF_PY_LIB_NAME}
	${MTF_PY_INSTALL_CMD_PREFIX} ${CP_CMD} $< $@
	
${MTF_DIAG_INSTALL_DIR}/${MTF_DIAG_EXE_NAME}: ${BUILD_DIR}/${MTF_DIAG_EXE_NAME}
	${MTF_DIAG_INSTALL_CMD_PREFIX} ${CP_CMD} $< $@	

${BUILD_DIR}/${MTF_EXE_NAME}: | ${BUILD_DIR}	
${BUILD_DIR}/${MTF_PY_LIB_NAME}: | ${BUILD_DIR}	
${BUILD_DIR}/${MTF_DIAG_EXE_NAME}: | ${BUILD_DIR}
${BUILD_DIR}/${MTF_REC_EXE_NAME}: | ${BUILD_DIR}
${BUILD_DIR}/${MTF_UAV_EXE_NAME}: | ${BUILD_DIR}	
${BUILD_DIR}/${MTF_GT_EXE_NAME}: | ${BUILD_DIR}	
${BUILD_DIR}/${MTF_SYN_EXE_NAME}: | ${BUILD_DIR}	
${BUILD_DIR}/${MTF_MOS_EXE_NAME}: | ${BUILD_DIR}	
${BUILD_DIR}/${MTF_QR_EXE_NAME}: | ${BUILD_DIR}	
${BUILD_DIR}/${MTF_MEX_MODULE_NAME}: | ${BUILD_DIR}	
${BUILD_DIR}/${MTF_PATCH_EXE_NAME}: | ${BUILD_DIR}	
${BUILD_DIR}/${MTF_APP_EXE_NAME}: | ${BUILD_DIR}	

${BUILD_DIR}/${MTF_EXE_NAME}: ${EXAMPLES_SRC_DIR}/runMTF.cc ${MTF_HEADERS}
	${CXX}  $< -o $@ -w ${WARNING_FLAGS} ${MTF_RUNTIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${EXAMPLES_INCLUDE_FLAGS} ${OPENCV_FLAGS} ${MTF_LIB_LINK} ${LIBS} ${BOOST_LIBS} ${LIBS_PARALLEL} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${OPENCV_LIBS} 
	
${BUILD_DIR}/${MTF_UAV_EXE_NAME}: ${EXAMPLES_SRC_DIR}/trackUAVTrajectory.cc ${MTF_HEADERS}
	${CXX} $< -o $@ -w ${UAV_FLAGS} ${WARNING_FLAGS} ${MTF_RUNTIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${EXAMPLES_INCLUDE_FLAGS} ${OPENCV_FLAGS} ${MTF_LIB_LINK} ${LIBS} ${BOOST_LIBS} ${LIBS_PARALLEL} ${MTF_LIBS_DIRS} ${MTF_LIBS}  ${OPENCV_LIBS} 
	
${BUILD_DIR}/${MTF_PATCH_EXE_NAME}: ${EXAMPLES_SRC_DIR}/extractPatch.cc ${MTF_HEADERS}
	${CXX} $< -o $@ -w ${WARNING_FLAGS} ${MTF_RUNTIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${EXAMPLES_INCLUDE_FLAGS} ${OPENCV_FLAGS} ${MTF_LIB_LINK} ${LIBS} ${BOOST_LIBS} ${LIBS_PARALLEL} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${OPENCV_LIBS} 
	
${BUILD_DIR}/${MTF_QR_EXE_NAME}: ${EXAMPLES_SRC_DIR}/trackMarkers.cc ${MTF_HEADERS}
	${CXX} $< -o $@ -w ${WARNING_FLAGS} ${MTF_RUNTIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${EXAMPLES_INCLUDE_FLAGS} ${OPENCV_FLAGS} ${MTF_LIB_LINK} ${LIBS} ${BOOST_LIBS} ${LIBS_PARALLEL} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${OPENCV_LIBS} 
	
${BUILD_DIR}/${MTF_PY_LIB_NAME}: ${BUILD_DIR}/pyMTF.o 
	${CXX} -shared $< -o $@ ${MTF_LIB_LINK} ${LIBS} ${LIBS_PARALLEL} ${MTF_RUNTIME_FLAGS} ${MTF_LIB_LINK} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${OPENCV_LIBS} ${BOOST_LIBS} ${PYTHON_LIBS_FLAGS} ${PYTHON_LIBS}
	
${BUILD_DIR}/pyMTF.o: ${EXAMPLES_SRC_DIR}/pyMTF.cc ${MTF_HEADERS}
	${CXX} -w -c -fPIC $< ${WARNING_FLAGS} -D_hypot=hypot ${OPENCV_FLAGS} ${MTF_RUNTIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${EXAMPLES_INCLUDE_FLAGS} -I${PYTHON_INCLUDE_DIR} -I${NUMPY_INCLUDE_DIR} -o $@

${BUILD_DIR}/${MTF_MEX_MODULE_NAME}: ${EXAMPLES_SRC_DIR}/mexMTF.cc ${MTF_HEADERS}
	$(MEX) $(MEX_OPTION) $(MEX_LIBS) $< -output $@
	
${BUILD_DIR}/${MTF_GT_EXE_NAME}: ${EXAMPLES_SRC_DIR}/showGroundTruth.cc ${EXAMPLES_HEADERS} ${UTILITIES_HEADER_DIR}/miscUtils.h  ${ROOT_HEADER_DIR}/mtf.h
	${CXX}  $< -o $@ -w ${WARNING_FLAGS} ${MTF_RUNTIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${EXAMPLES_INCLUDE_FLAGS} ${OPENCV_FLAGS} ${LIBS} ${MTF_LIB_LINK} ${BOOST_LIBS} ${LIBS_PARALLEL} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${OPENCV_LIBS} 
	
${BUILD_DIR}/${MTF_REC_EXE_NAME}: ${EXAMPLES_SRC_DIR}/recordSeq.cc ${EXAMPLES_HEADERS}
	${CXX}  $< -o $@ -w ${WARNING_FLAGS} ${MTF_RUNTIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${EXAMPLES_INCLUDE_FLAGS} ${CONFIG_INCLUDE_FLAGS} ${MTF_LIBS_DIRS} ${OPENCV_FLAGS} ${LIBS} ${MTF_LIB_LINK} ${BOOST_LIBS} ${LIBS_PARALLEL} ${MTF_LIBS} ${OPENCV_LIBS} 
	
${BUILD_DIR}/${MTF_SYN_EXE_NAME}: ${EXAMPLES_SRC_DIR}/generateSyntheticSeq.cc ${MTF_HEADERS}
	${CXX}  $< -o $@ -w ${WARNING_FLAGS} ${MTF_RUNTIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${EXAMPLES_INCLUDE_FLAGS} ${OPENCV_FLAGS} ${LIBS} ${MTF_LIB_LINK} ${BOOST_LIBS} ${LIBS_PARALLEL} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${OPENCV_LIBS}
	
${BUILD_DIR}/${MTF_MOS_EXE_NAME}: ${EXAMPLES_SRC_DIR}/createMosaic.cc ${MTF_HEADERS}
	${CXX}  $< -o $@ -w ${WARNING_FLAGS} ${MTF_RUNTIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${EXAMPLES_INCLUDE_FLAGS} ${OPENCV_FLAGS} ${LIBS} ${MTF_LIB_LINK} ${BOOST_LIBS} ${LIBS_PARALLEL} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${OPENCV_LIBS} 
	
${BUILD_DIR}/${MTF_DIAG_EXE_NAME}: ${EXAMPLES_SRC_DIR}/diagnoseMTF.cc ${TEST_HEADERS} ${MTF_HEADERS}
	${CXX} $< -o $@ -w ${WARNING_FLAGS} ${MTF_RUNTIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${EXAMPLES_INCLUDE_FLAGS} ${OPENCV_FLAGS} ${MTF_LIB_LINK} ${LIBS} ${LIBS_PARALLEL} ${BOOST_LIBS} ${MTF_DIAG_LIBS} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${OPENCV_LIBS} 
	
${BUILD_DIR}/${MTF_APP_EXE_NAME}: ${MTF_APP_SRC_DIR}/${app}.cc ${MTF_HEADERS}
	${CXX}  $< -o $@ -w ${WARNING_FLAGS} ${MTF_RUNTIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${EXAMPLES_INCLUDE_FLAGS} ${OPENCV_FLAGS} ${MTF_LIB_LINK} ${LIBS} ${BOOST_LIBS} ${LIBS_PARALLEL} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${OPENCV_LIBS} 
	
# ------------------------------------------------------------------------------- #
# ------------------------------------ Tools ------------------------------------ #
# ------------------------------------------------------------------------------- #	
ifeq (${o}, 1)
TOOLS_LIB_NAME=libmtf_tools.so
TOOLS_LIB_LINK=-lmtf_tools
else
TOOLS_LIB_NAME=libmtf_tools_debug.so
TOOLS_LIB_LINK=-lmtf_tools_debug
endif

${BUILD_DIR}/Tools:
		mkdir -p $@
${TOOLS_LIB_NAME}: ${BUILD_DIR}/Tools ${BUILD_DIR}/Tools/inputCV.o ${BUILD_DIR}/Tools/inputBase.o ${BUILD_DIR}/Tools/objUtils.o
	${CXX} -shared -o $@  ${BUILD_DIR}/Tools/parameters.o ${BUILD_DIR}/Tools/inputCV.o ${BUILD_DIR}/Tools/inputBase.o ${BUILD_DIR}/Tools/objUtils.o
	sudo cp -f $@ /usr/lib	
${BUILD_DIR}/Tools/inputCV.o: Tools/inputCV.cc Tools/inputCV.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${MTF_COMPILETIME_FLAGS} ${OPENCV_FLAGS}  ${MTF_FLAGSXV} $< -o $@
${BUILD_DIR}/Tools/inputBase.o: Tools/inputBase.cc Tools/inputBase.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${MTF_COMPILETIME_FLAGS} ${OPENCV_FLAGS}  ${MTF_FLAGSXV} $< -o $@
${BUILD_DIR}/Tools/objUtils.o: Tools/objUtils.cc Tools/objUtils.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${MTF_COMPILETIME_FLAGS} ${OPENCV_FLAGS} $< -o $@	
