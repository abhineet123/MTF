# ------------------------------------------------------------------------------------ #
# ---------------------------------- Search Methods ---------------------------------- #
# ------------------------------------------------------------------------------------ #
WIN_FLANN_LOCATION = E:/Programming/FLANN/flann-1.8.4-src/src/cpp
WIN_HDF5_LOCATION = E:/Programming/FLANN/HDF5/hdf5-1.10.0-patch1/hdf5-1.10.0-patch1/src
# compile only non templated (NT) implementations of SMs
only_nt ?= 0
# alias for only_nt
nt ?= 0
# enable grid tracker (and its OpenCV variants) - only matters if only_nt is 1 otherwise these are enabled anyway
grid ?= 1
# enable feature tracker
feat ?= 1
# enable parts of the feature tracker that require nonfree module of OpenCV to be installed
feat_nf ?= 0
# enable templated FLANN based implementation of NN SM
ifeq ($(OS),Windows_NT)
	# FLANN has compatibility issue with GNU Make in Windows 
	nn ?= 0
else
	nn ?= 1
endif
hdf5 ?= 0
# enable Regression Network (RegNet) based SM
regnet ?= 0
# use CPU only version of Caffe with RegNet; only matters if RegNet is enabled
regnet_cpu ?= 0

# enable parallelization of Particle Filter SM using OpenMP
pfomp ?= 0

SM_INCLUDE_DIR = SM/include
SM_SRC_DIR = SM/src
SM_HEADER_DIR = ${SM_INCLUDE_DIR}/mtf/SM
SM_BASE_HEADERS =  ${SM_HEADER_DIR}/SearchMethod.h ${ROOT_HEADER_DIR}/TrackerBase.h
SM_NT_BASE_HEADERS =  ${SM_HEADER_DIR}/NT/SearchMethod.h ${ROOT_HEADER_DIR}/TrackerBase.h
COMPOSITE_BASE_HEADERS =  ${SM_HEADER_DIR}/CompositeBase.h

MTF_INCLUDE_DIRS += ${SM_INCLUDE_DIR}
BASE_HEADERS += ${SM_HEADER_DIR}/SearchMethod.h

SEARCH_METHODS = 
COMPOSITE = CascadeTracker ParallelTracker PyramidalTracker LineTracker
SEARCH_METHODS_NT = FCLK ICLK FALK IALK ESM PF NN GNN FCSD AESM
SEARCH_PARAMS = FCLK ICLK FALK IALK ESM PF NN GNN Cascade Parallel Pyramidal

SEARCH_OBJS = $(addprefix ${BUILD_DIR}/,$(addsuffix .o, ${SEARCH_METHODS}))
SEARCH_NT_OBJS = $(addprefix ${BUILD_DIR}/,$(addsuffix _NT.o, ${SEARCH_METHODS_NT}))
SEARCH_PARAMS_OBJS = $(addprefix ${BUILD_DIR}/,$(addsuffix Params.o, ${SEARCH_PARAMS}))
COMPOSITE_OBJS = $(addprefix ${BUILD_DIR}/,$(addsuffix .o, ${COMPOSITE}))
SEARCH_HEADERS = $(addprefix ${SM_HEADER_DIR}/, $(addsuffix .h, ${SEARCH_METHODS}))
SEARCH_PARAMS_HEADERS = $(addprefix ${SM_HEADER_DIR}/, $(addsuffix Params.h, ${SEARCH_PARAMS}))
SEARCH_NT_HEADERS = $(addprefix ${SM_HEADER_DIR}/NT/, $(addsuffix .h, ${SEARCH_METHODS_NT}))
COMPOSITE_HEADERS = $(addprefix ${SM_HEADER_DIR}/, $(addsuffix .h, ${COMPOSITE}))

ifeq (${nt}, 1)
only_nt = 1
endif

ifeq (${only_nt}, 0)
	SEARCH_METHODS += FCLK ICLK FALK IALK ESM PF 
	COMPOSITE +=  RKLT CascadeSM  ParallelSM  PyramidalSM
	COMPOSITE_BASE_HEADERS +=  ${SM_HEADER_DIR}/CompositeSM.h
	ifeq (${grid}, 1)
		COMPOSITE +=  GridTrackerFlow
	endif
else
	MTF_RUNTIME_FLAGS += -D DISABLE_TEMPLATED_SM
	MTF_COMPILETIME_FLAGS += -D DISABLE_TEMPLATED_SM
endif
ifeq (${grid}, 1)
	COMPOSITE +=  GridTracker GridTrackerCV	
	COMPOSITE_BASE_HEADERS +=  ${SM_HEADER_DIR}/GridBase.h
	ifeq (${only_nt}, 0)
		COMPOSITE +=  GridTrackerFlow
	endif
	SEARCH_METHODS_NT += GridTrackerFlow RKLT
	SEARCH_PARAMS += GridTrackerFlow RKLT
else
	MTF_RUNTIME_FLAGS += -D DISABLE_GRID
	MTF_COMPILETIME_FLAGS += -D DISABLE_GRID
endif

ifeq (${nn}, 1)
	SEARCH_METHODS += NN GNN FGNN
	SEARCH_PARAMS += FLANN
	MTF_LIBS += -lflann
	ifeq ($(OS),Windows_NT)
		MTF_RUNTIME_FLAGS += -I ${WIN_FLANN_LOCATION}
		MTF_COMPILETIME_FLAGS += -I ${WIN_FLANN_LOCATION}
	endif
else	
	MTF_RUNTIME_FLAGS += -D DISABLE_FLANN -D DISABLE_HDF5
	MTF_COMPILETIME_FLAGS += -D DISABLE_FLANN -D DISABLE_HDF5
endif
ifeq (${hdf5}, 1)
	MTF_LIBS += -lhdf5
	ifeq ($(OS),Windows_NT)
		MTF_RUNTIME_FLAGS += -I ${WIN_HDF5_LOCATION}
		MTF_COMPILETIME_FLAGS += -I ${WIN_HDF5_LOCATION}
	endif
else
	MTF_RUNTIME_FLAGS += -D DISABLE_HDF5
	MTF_COMPILETIME_FLAGS += -D DISABLE_HDF5
endif


ifeq (${feat}, 1)
	COMPOSITE += FeatureTracker	
	COMPOSITE_BASE_HEADERS +=  ${SM_HEADER_DIR}/FeatureBase.h
	SEARCH_PARAMS += FLANNCV
	ifeq (${feat_nf}, 0)
		MTF_RUNTIME_FLAGS += -D FEAT_DISABLE_NONFREE
		MTF_COMPILETIME_FLAGS += -D FEAT_DISABLE_NONFREE
	endif	
else
	MTF_RUNTIME_FLAGS += -D DISABLE_FEAT
	MTF_COMPILETIME_FLAGS += -D DISABLE_FEAT
endif

SEARCH_HEADERS += ${SM_BASE_HEADERS} ${COMPOSITE_BASE_HEADERS}
SEARCH_NT_HEADERS += ${SM_NT_BASE_HEADERS}

MTF_HEADERS += ${SEARCH_HEADERS} ${SEARCH_PARAMS_HEADERS} ${COMPOSITE_HEADERS} ${SEARCH_NT_HEADERS}
MTF_OBJS += ${SEARCH_OBJS} ${SEARCH_PARAMS_OBJS} ${COMPOSITE_OBJS} ${SEARCH_NT_OBJS}

GRID_FLAGS = 
PRL_FLAGS = 
ESM_FLAGS = 
IC_FLAGS =
FC_FLAGS = 
FA_FLAGS = 
NN_FLAGS = 

el ?= 0
et ?= 0
ed ?= 0
emg ?= 1
icd ?= 0
icl ?= 0
ict ?= 0
fcd ?= 0
fct ?= 0
fad ?= 0
fat ?= 0
iat ?= 0
prltbb ?= 0
prlomp ?= 0
gridtbb ?= 0
gridomp ?= 0
efd ?= 0


ifeq (${regnet}, 1)
SEARCH_METHODS_NT += RegNet
SEARCH_PARAMS += RegNet
use_caffe = 1
else
MTF_COMPILETIME_FLAGS += -D DISABLE_REGNET
MTF_RUNTIME_FLAGS += -D DISABLE_REGNET
endif
ifeq (${regnet_cpu}, 1)
RGN_FLAGS += -D CPU_ONLY
endif
ifeq (${prltbb}, 1)
PRL_FLAGS += -D ENABLE_TBB
MTF_RUNTIME_FLAGS += -D ENABLE_PARALLEL
endif

ifeq (${gridtbb}, 1)
GRID_FLAGS += -D ENABLE_TBB 
MTF_RUNTIME_FLAGS += -D ENABLE_PARALLEL
else
ifeq (${gridomp}, 1)
GRID_FLAGS += -D ENABLE_PARALLEL -D ENABLE_OMP -fopenmp 
MTF_RUNTIME_FLAGS += -D ENABLE_PARALLEL -fopenmp
MTF_LIBS += -fopenmp
endif
endif

ifeq (${pfomp}, 1)
PF_FLAGS += -D ENABLE_PARALLEL -D ENABLE_OMP -fopenmp 
MTF_RUNTIME_FLAGS += -D ENABLE_PARALLEL -fopenmp
MTF_LIBS += -fopenmp
endif

ifeq (${et}, 1)
ESM_FLAGS += -D ENABLE_PROFILING
endif
ifeq (${ed}, 1)
ESM_FLAGS += -D LOG_ESM_DATA
endif
ifeq (${el}, 1)
ESM_FLAGS += -D LOG_ESM_DATA -D ENABLE_PROFILING
endif
ifeq (${emg}, 0)
ESM_FLAGS += -D DISABLE_MEAN_GRADIENT
endif

ifeq (${icd}, 1)
IC_FLAGS += -D LOG_ICLK_DATA
endif
ifeq (${ict}, 1)
IC_FLAGS += -D LOG_ICLK_TIMES 
endif

ifeq (${fcd}, 1)
FC_FLAGS += -D LOG_FCLK_DATA
endif
ifeq (${fct}, 1)
FC_FLAGS += -D ENABLE_PROFILING 
endif

ifeq (${fad}, 1)
FA_FLAGS += -D LOG_FALK_DATA
endif
ifeq (${fat}, 1)
FA_FLAGS += -D LOG_FALK_TIMES 
endif
ifeq (${iat}, 1)
IA_FLAGS += -D LOG_IALK_TIMES 
endif
ifeq (${nnt}, 1)
NN_FLAGS += -D ENABLE_PROFILING
endif


${BUILD_DIR}/ICLK.o: ${SM_SRC_DIR}/ICLK.cc ${SM_HEADER_DIR}/ICLK.h ${SM_HEADER_DIR}/ICLKParams.h ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS} ${SM_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${IC_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/FCLK.o: ${SM_SRC_DIR}/FCLK.cc ${SM_HEADER_DIR}/FCLK.h ${SM_HEADER_DIR}/FCLKParams.h ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS} ${SM_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${FC_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/FALK.o: ${SM_SRC_DIR}/FALK.cc ${SM_HEADER_DIR}/FALK.h ${SM_HEADER_DIR}/FALKParams.h ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS} ${SM_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${FA_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/IALK.o: ${SM_SRC_DIR}/IALK.cc ${SM_HEADER_DIR}/IALK.h ${SM_HEADER_DIR}/IALKParams.h ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS} ${SM_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${IA_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/FCSD.o: ${SM_SRC_DIR}/FCSD.cc ${SM_HEADER_DIR}/FCSD.h ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS} ${SM_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${FC_FLAGS} $< ${OPENCV_FLAGS} -o $@

${BUILD_DIR}/AESM.o: ${SM_SRC_DIR}/AESM.cc ${SM_HEADER_DIR}/AESM.h ${SM_HEADER_DIR}/ESM.h ${SM_HEADER_DIR}/ESMParams.h ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS} ${SM_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${ESM_FLAGS} $< ${OPENCV_FLAGS} -o $@

${BUILD_DIR}/ESM.o: ${SM_SRC_DIR}/ESM.cc ${SM_HEADER_DIR}/ESM.h ${SM_HEADER_DIR}/ESMParams.h ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS} ${SM_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${UTILITIES_HEADER_DIR}/spiUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${ESM_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/PF.o: ${SM_SRC_DIR}/PF.cc ${SM_HEADER_DIR}/PF.h ${SM_HEADER_DIR}/PFParams.h ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS} ${SM_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${PF_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/NN.o: ${SM_SRC_DIR}/NN.cc ${SM_HEADER_DIR}/NN.h ${SM_HEADER_DIR}/FGNN.h ${SM_HEADER_DIR}/GNN.h ${SM_HEADER_DIR}/NNParams.h ${SM_HEADER_DIR}/GNNParams.h ${SM_HEADER_DIR}/FLANNParams.h ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS} ${SM_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${NN_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/GNN.o: ${SM_SRC_DIR}/GNN.cc ${SM_HEADER_DIR}/GNN.h ${SM_HEADER_DIR}/GNNParams.h ${APPEARANCE_HEADERS} ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h 
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${GNN_FLAGS} $< ${OPENCV_FLAGS} -o $@	
	
${BUILD_DIR}/FGNN.o: ${SM_SRC_DIR}/FGNN.cc ${SM_HEADER_DIR}/FGNN.h ${SM_HEADER_DIR}/GNN.h ${APPEARANCE_HEADERS} ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h 
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${FGNN_FLAGS} $< ${OPENCV_FLAGS} -o $@	
	
	
# ------------------------------------------------------------------------------- #
# ---------------------------------- Composite ---------------------------------- #
# ------------------------------------------------------------------------------- #

${BUILD_DIR}/CascadeTracker.o: ${SM_SRC_DIR}/CascadeTracker.cc ${SM_HEADER_DIR}/CascadeTracker.h ${SM_HEADER_DIR}/CascadeParams.h ${SM_HEADER_DIR}/CompositeBase.h ${ROOT_HEADER_DIR}/TrackerBase.h  ${UTILITIES_HEADER_DIR}/excpUtils.h ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/CascadeSM.o: ${SM_SRC_DIR}/CascadeSM.cc ${SM_HEADER_DIR}/CascadeSM.h ${SM_HEADER_DIR}/CascadeParams.h ${SM_HEADER_DIR}/CompositeSM.h ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS} ${SM_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/ParallelTracker.o: ${SM_SRC_DIR}/ParallelTracker.cc ${SM_HEADER_DIR}/ParallelTracker.h ${SM_HEADER_DIR}/ParallelParams.h ${SM_HEADER_DIR}/CompositeBase.h ${ROOT_HEADER_DIR}/TrackerBase.h ${SM_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${PRL_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/ParallelSM.o: ${SM_SRC_DIR}/ParallelSM.cc ${SM_HEADER_DIR}/ParallelSM.h ${SM_HEADER_DIR}/ParallelParams.h ${SM_HEADER_DIR}/CompositeSM.h ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS} ${SM_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${PRL_FLAGS} $< ${OPENCV_FLAGS} -o $@	

${BUILD_DIR}/PyramidalTracker.o: ${SM_SRC_DIR}/PyramidalTracker.cc ${SM_HEADER_DIR}/PyramidalTracker.h ${SM_HEADER_DIR}/PyramidalParams.h ${SM_HEADER_DIR}/CompositeBase.h ${ROOT_HEADER_DIR}/TrackerBase.h ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${PRL_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/PyramidalSM.o: ${SM_SRC_DIR}/PyramidalSM.cc ${SM_HEADER_DIR}/PyramidalSM.h ${SM_HEADER_DIR}/PyramidalParams.h  ${SM_HEADER_DIR}/CompositeSM.h ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS} ${SM_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${PRL_FLAGS} $< ${OPENCV_FLAGS} -o $@	
	
${BUILD_DIR}/GridTrackerCV.o: ${SM_SRC_DIR}/GridTrackerCV.cc ${SM_HEADER_DIR}/GridTrackerCV.h ${SM_HEADER_DIR}/GridBase.h ${SM_HEADER_DIR}/CompositeBase.h ${ROOT_HEADER_DIR}/TrackerBase.h ${UTILITIES_HEADER_DIR}/excpUtils.h ${UTILITIES_HEADER_DIR}/miscUtils.h ${UTILITIES_HEADER_DIR}/imgUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h ${STATE_SPACE_HEADERS}
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@

${BUILD_DIR}/GridTrackerFlow.o: ${SM_SRC_DIR}/GridTrackerFlow.cc ${SM_HEADER_DIR}/GridTrackerFlow.h ${SM_HEADER_DIR}/GridTrackerFlowParams.h ${SM_HEADER_DIR}/GridBase.h ${SM_HEADER_DIR}/CompositeBase.h ${ROOT_HEADER_DIR}/TrackerBase.h ${UTILITIES_HEADER_DIR}/excpUtils.h ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS}
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/FeatureTracker.o: ${SM_SRC_DIR}/FeatureTracker.cc ${SM_HEADER_DIR}/FeatureTracker.h ${SM_HEADER_DIR}/FeatureBase.h ${SM_HEADER_DIR}/CompositeBase.h ${ROOT_HEADER_DIR}/TrackerBase.h ${UTILITIES_HEADER_DIR}/excpUtils.h ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h ${STATE_SPACE_HEADERS}
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@	
	
${BUILD_DIR}/RKLT.o: ${SM_SRC_DIR}/RKLT.cc ${SM_HEADER_DIR}/RKLT.h ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS} ${SM_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h ${SM_HEADER_DIR}/GridBase.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${GRID_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/GridTracker.o: ${SM_SRC_DIR}/GridTracker.cc ${SM_HEADER_DIR}/GridTracker.h ${STATE_SPACE_HEADERS} ${SM_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h ${SM_HEADER_DIR}/GridBase.h ${SM_HEADER_DIR}/CompositeBase.h ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h ${SM_HEADER_DIR}/GridBase.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${GRID_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/LineTracker.o: ${SM_SRC_DIR}/LineTracker.cc ${SM_HEADER_DIR}/LineTracker.h ${SM_HEADER_DIR}/CompositeBase.h ${ROOT_HEADER_DIR}/TrackerBase.h ${UTILITIES_HEADER_DIR}/excpUtils.h ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h ${SM_HEADER_DIR}/GridBase.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${LINE_FLAGS} $< ${OPENCV_FLAGS} -o $@
		
# ----------------------------------------------------------------------------------- #
# ---------------------------------- Non Templated ---------------------------------- #
# ----------------------------------------------------------------------------------- #
	
${BUILD_DIR}/FCLK_NT.o: ${SM_SRC_DIR}/NT/FCLK.cc ${SM_HEADER_DIR}/NT/FCLK.h ${SM_HEADER_DIR}/FCLKParams.h ${AM_BASE_HEADERS} ${SSM_BASE_HEADERS} ${SM_NT_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h 
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${FC_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/ICLK_NT.o: ${SM_SRC_DIR}/NT/ICLK.cc ${SM_HEADER_DIR}/NT/ICLK.h ${SM_HEADER_DIR}/ICLKParams.h ${AM_BASE_HEADERS} ${SSM_BASE_HEADERS} ${SM_NT_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h 
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${IC_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/FALK_NT.o: ${SM_SRC_DIR}/NT/FALK.cc ${SM_HEADER_DIR}/NT/FALK.h ${SM_HEADER_DIR}/FALKParams.h ${AM_BASE_HEADERS} ${SSM_BASE_HEADERS} ${SM_NT_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h 
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${FA_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/IALK_NT.o: ${SM_SRC_DIR}/NT/IALK.cc ${SM_HEADER_DIR}/NT/IALK.h ${SM_HEADER_DIR}/IALKParams.h ${AM_BASE_HEADERS} ${SSM_BASE_HEADERS} ${SM_NT_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${IA_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/ESM_NT.o: ${SM_SRC_DIR}/NT/ESM.cc ${SM_HEADER_DIR}/NT/ESM.h ${SM_HEADER_DIR}/ESMParams.h ${AM_BASE_HEADERS} ${SSM_BASE_HEADERS} ${SM_NT_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${UTILITIES_HEADER_DIR}/spiUtils.h ${MACROS_HEADER_DIR}/common.h 
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${ESM_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/AESM_NT.o: ${SM_SRC_DIR}/NT/AESM.cc ${SM_HEADER_DIR}/NT/AESM.h ${SM_HEADER_DIR}/NT/ESM.h ${SM_HEADER_DIR}/ESMParams.h ${AM_BASE_HEADERS} ${SSM_BASE_HEADERS} ${SM_NT_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${UTILITIES_HEADER_DIR}/spiUtils.h ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${ESM_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/FCSD_NT.o: ${SM_SRC_DIR}/NT/FCSD.cc ${SM_HEADER_DIR}/NT/FCSD.h ${AM_BASE_HEADERS} ${SSM_BASE_HEADERS} ${SM_NT_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${FC_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/PF_NT.o: ${SM_SRC_DIR}/NT/PF.cc ${SM_HEADER_DIR}/NT/PF.h ${SM_HEADER_DIR}/PFParams.h ${AM_BASE_HEADERS} ${SSM_BASE_HEADERS} ${SM_NT_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${UTILITIES_HEADER_DIR}/histUtils.h ${UTILITIES_HEADER_DIR}/graphUtils.h ${MACROS_HEADER_DIR}/common.h 
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${PF_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/NN_NT.o: ${SM_SRC_DIR}/NT/NN.cc ${SM_HEADER_DIR}/NT/NN.h ${SM_HEADER_DIR}/GNN.h ${SM_HEADER_DIR}/NNParams.h ${AM_BASE_HEADERS} ${SSM_BASE_HEADERS} ${SM_NT_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h 
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${NN_FLAGS} $< ${OPENCV_FLAGS} -o $@	

${BUILD_DIR}/GNN_NT.o: ${SM_SRC_DIR}/NT/GNN.cc ${SM_HEADER_DIR}/NT/GNN.h ${SM_HEADER_DIR}/GNNParams.h ${AM_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h 
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${GNN_FLAGS} $< ${OPENCV_FLAGS} -o $@	
	
${BUILD_DIR}/RegNet_NT.o: ${SM_SRC_DIR}/NT/RegNet.cc ${SM_HEADER_DIR}/NT/RegNet.h ${SM_HEADER_DIR}/RegNetParams.h ${AM_BASE_HEADERS} ${SSM_BASE_HEADERS} ${SM_NT_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h 
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${RGN_FLAGS} ${CAFFE_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/RKLT_NT.o: ${SM_SRC_DIR}/NT/RKLT.cc ${SM_HEADER_DIR}/NT/RKLT.h ${AM_BASE_HEADERS} ${SSM_BASE_HEADERS} ${SM_NT_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h ${UTILITIES_HEADER_DIR}/miscUtils.h ${UTILITIES_HEADER_DIR}/spiUtils.h ${MACROS_HEADER_DIR}/common.h ${SM_HEADER_DIR}/GridBase.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${GRID_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/GridTrackerFlow_NT.o: ${SM_SRC_DIR}/NT/GridTrackerFlow.cc ${SM_HEADER_DIR}/NT/GridTrackerFlow.h ${SM_HEADER_DIR}/GridTrackerFlowParams.h ${AM_BASE_HEADERS} ${SSM_BASE_HEADERS} ${SM_HEADER_DIR}/GridBase.h ${SM_HEADER_DIR}/CompositeBase.h ${ROOT_HEADER_DIR}/TrackerBase.h ${UTILITIES_HEADER_DIR}/excpUtils.h ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h 
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@	
# ---------------------------------------------------------------------------------- #
# ------------------------------------- Params ------------------------------------- #
# ---------------------------------------------------------------------------------- #
	
${BUILD_DIR}/ICLKParams.o: ${SM_SRC_DIR}/ICLKParams.cc ${SM_HEADER_DIR}/ICLKParams.h ${UTILITIES_HEADER_DIR}/excpUtils.h  ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@		
	
${BUILD_DIR}/FCLKParams.o: ${SM_SRC_DIR}/FCLKParams.cc ${SM_HEADER_DIR}/FCLKParams.h ${UTILITIES_HEADER_DIR}/excpUtils.h  ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@	
	
${BUILD_DIR}/ESMParams.o: ${SM_SRC_DIR}/ESMParams.cc ${SM_HEADER_DIR}/ESMParams.h ${UTILITIES_HEADER_DIR}/excpUtils.h ${UTILITIES_HEADER_DIR}/spiUtils.h ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@	

${BUILD_DIR}/FALKParams.o: ${SM_SRC_DIR}/FALKParams.cc ${SM_HEADER_DIR}/FALKParams.h ${UTILITIES_HEADER_DIR}/excpUtils.h  ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@	

${BUILD_DIR}/IALKParams.o: ${SM_SRC_DIR}/IALKParams.cc ${SM_HEADER_DIR}/IALKParams.h ${UTILITIES_HEADER_DIR}/excpUtils.h  ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@	
	
${BUILD_DIR}/NNParams.o: ${SM_SRC_DIR}/NNParams.cc ${SM_HEADER_DIR}/NNParams.h ${SM_HEADER_DIR}/GNNParams.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${UTILITIES_HEADER_DIR}/excpUtils.h ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/GNNParams.o: ${SM_SRC_DIR}/GNNParams.cc ${SM_HEADER_DIR}/GNNParams.h ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@		
	
${BUILD_DIR}/FLANNParams.o: ${SM_SRC_DIR}/FLANNParams.cc ${SM_HEADER_DIR}/FLANNParams.h ${UTILITIES_HEADER_DIR}/excpUtils.h ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@	
	
${BUILD_DIR}/FLANNCVParams.o: ${SM_SRC_DIR}/FLANNCVParams.cc ${SM_HEADER_DIR}/FLANNCVParams.h ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@	
	
		
${BUILD_DIR}/RegNetParams.o: ${SM_SRC_DIR}/RegNetParams.cc ${SM_HEADER_DIR}/RegNetParams.h  ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@	
	
${BUILD_DIR}/PFParams.o: ${SM_SRC_DIR}/PFParams.cc ${SM_HEADER_DIR}/PFParams.h ${UTILITIES_HEADER_DIR}/miscUtils.h ${UTILITIES_HEADER_DIR}/excpUtils.h ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@	
	
${BUILD_DIR}/CascadeParams.o: ${SM_SRC_DIR}/CascadeParams.cc ${SM_HEADER_DIR}/CascadeParams.h  ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@	
	
${BUILD_DIR}/ParallelParams.o: ${SM_SRC_DIR}/ParallelParams.cc ${SM_HEADER_DIR}/ParallelParams.h  ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@	
	
${BUILD_DIR}/PyramidalParams.o: ${SM_SRC_DIR}/PyramidalParams.cc ${SM_HEADER_DIR}/PyramidalParams.h  ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/RKLTParams.o: ${SM_SRC_DIR}/RKLTParams.cc ${SM_HEADER_DIR}/RKLTParams.h  ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${BUILD_DIR}/GridTrackerFlowParams.o: ${SM_SRC_DIR}/GridTrackerFlowParams.cc ${SM_HEADER_DIR}/GridTrackerFlowParams.h  ${MACROS_HEADER_DIR}/common.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} $< ${OPENCV_FLAGS} -o $@	
	
# ------------------------------------------------------------------------------------ #
# ---------------------------------- Obsolete Stuff ---------------------------------- #
# ------------------------------------------------------------------------------------ #

${BUILD_DIR}/HACLK.o: ${SM_SRC_DIR}/HACLK.cc ${SM_HEADER_DIR}/HACLK.h ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS} ${SM_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${HAC_FLAGS} $< ${OPENCV_FLAGS} -o $@	
	
${BUILD_DIR}/IALK2.o: ${SM_SRC_DIR}/IALK2.cc ${SM_HEADER_DIR}/IALK2.h ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS} ${SM_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${IA_FLAGS} $< ${OPENCV_FLAGS} -o $@	
	
${BUILD_DIR}/HESM.o: ${SM_SRC_DIR}/HESM.cc ${SM_HEADER_DIR}/HESM.h ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS} ${SM_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h
		${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${FC_FLAGS} $< ${OPENCV_FLAGS} -o $@	
		
${BUILD_DIR}/FESMBase.o: ${SM_SRC_DIR}/FESMBase.cc ${SM_HEADER_DIR}/FESMBase.h ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS} ${SM_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${UTILITIES_HEADER_DIR}/excpUtils.h ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${FESMBase_FLAGS} $< ${OPENCV_FLAGS} -o $@	
	
${BUILD_DIR}/FESM.o: ${SM_SRC_DIR}/FESM.cc ${SM_HEADER_DIR}/FESM.h ${SM_HEADER_DIR}/FESMBase.h ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS} ${SM_BASE_HEADERS} ${UTILITIES_HEADER_DIR}/excpUtils.h  ${UTILITIES_HEADER_DIR}/miscUtils.h ${UTILITIES_HEADER_DIR}/excpUtils.h  ${MACROS_HEADER_DIR}/common.h ${MACROS_HEADER_DIR}/register.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${PROF_FLAGS} ${MTF_COMPILETIME_FLAGS} ${MTF_INCLUDE_FLAGS} ${FESM_FLAGS} $< ${OPENCV_FLAGS} -o $@
	
${MTF_LIB_INSTALL_DIR}/libgnn.so: ${SM_HEADER_DIR}/GNN/libgnn.so
	cp -f $< $@
${SM_HEADER_DIR}/GNN/libgnn.so: ${SM_HEADER_DIR}/GNN/build_graph.h ${SM_SRC_DIR}/GNN/build_graph.cc ${SM_HEADER_DIR}/GNN/search_graph_knn.h ${SM_SRC_DIR}/GNN/search_graph_knn.cc ${SM_SRC_DIR}/GNN/utility.cc ${SM_HEADER_DIR}/GNN/utility.h ${SM_SRC_DIR}/GNN/memwatch.cc ${SM_HEADER_DIR}/GNN/memwatch.h 
	$(MAKE) -C ${SM_HEADER_DIR}/GNN --no-print-directory