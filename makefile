ifeq ($(OS),Windows_NT)
	MTF_INSTALL_DIR ?= C:/MTF
	EIGEN_INCLUDE_DIRS ?= C:/Eigen/include
	OPENCV_INCLUDE_DIRS ?= C:/OpenCV/build/include
	OPENCV_LIB_DIRS ?= C:/OpenCV/build/x86/mingw/lib
	OPENCV_LIBS_SUFFIX ?= 2410
	BOOST_INCLUDE_DIRS ?= C:/Boost/include
	BOOST_LIB_DIRS ?= C:/Boost/lib
	BOOST_LIBS_SUFFIX ?= -mgw53-1_58
	MTF_LIB_EXT = .dll
	MTF_EXE_EXT = .exe
	BUILD_ROOT_DIR = build\\windows
else
	MTF_INSTALL_DIR ?= /usr/local
	EIGEN_INCLUDE_DIRS ?= /usr/local/include/eigen3 /usr/include/eigen3
	MTF_LIB_EXT = .so
	MTF_EXE_EXT =
	BUILD_ROOT_DIR = build
endif

# enable optimization (or Release build)
o ?= 1
# enable selective pixel integration - disabled by default for speed
spi ?= 0
# enable profiling
prf ?= 0 
# enable parallelization using OpenMP
omp ?= 0
# enable Caffe
use_caffe ?= 0
# use Caffe in CPU ONLY mode
caffe_cpu ?= 0
# optional version postfix for all compiled files for multiple versions of the library to coexist
ver ?= 0


WARNING_FLAGS = -Wfatal-errors -Wno-write-strings -Wno-unused-result
MTF_LIBS +=  -lstdc++
EIGEN_INCLUDE_FLAGS = $(addprefix -I, ${EIGEN_INCLUDE_DIRS})
MTF_COMPILETIME_FLAGS += -std=c++11 ${EIGEN_INCLUDE_FLAGS}
MTF_RUNTIME_FLAGS += -std=c++11 ${EIGEN_INCLUDE_FLAGS}
PROF_FLAGS =
LIB_POST_FIX =
CAFFE_FLAGS += -L/usr/lib/x86_64-linux-gnu/ -lcaffe -lglog -lprotobuf
MTF_LIB_INSTALL_CMD_PREFIX = 
MTF_HEADER_INSTALL_CMD_PREFIX =
_BOOST_LIBS =  -lboost_random -lboost_filesystem -lboost_system
MTF_LIB_INSTALL_DIR ?= $(MTF_INSTALL_DIR)/lib
MTF_HEADER_INSTALL_DIR ?= $(MTF_INSTALL_DIR)/include

ifeq ($(OS),Windows_NT)
	_OPENCV_LIBS = -lopencv_calib3d -lopencv_contrib -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_ocl -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_video -lopencv_videostab
	OPENCV_LIBS = $(addsuffix ${OPENCV_LIBS_SUFFIX}, ${_OPENCV_LIBS})	
	BOOST_LIBS = $(addsuffix ${BOOST_LIBS_SUFFIX}, ${_BOOST_LIBS})
	OPENCV_FLAGS = -I ${OPENCV_INCLUDE_DIRS}
	MTF_LIBS_DIRS += -L ${OPENCV_LIB_DIRS} -L ${BOOST_LIB_DIRS}
	MTF_COMPILETIME_FLAGS += -I${BOOST_INCLUDE_DIRS} -ftrack-macro-expansion=0
	MTF_RUNTIME_FLAGS += -I${BOOST_INCLUDE_DIRS}	
	MTF_PIC_FLAG = 
	MKDIR_CMD = mkdir
	# RM_CMD = del /F /Q
	# CP_CMD = copy /Y
	PATHSEP2 = \\

else
	OPENCV_FLAGS = `pkg-config --cflags opencv`
	OPENCV_LIBS = `pkg-config --libs opencv`
	BOOST_LIBS = ${_BOOST_LIBS}
	MTF_LIBS_DIRS += -L/usr/local/lib
	MTF_COMPILETIME_FLAGS += -I/usr/include
	MTF_PIC_FLAG = -fPIC
	# add "sudo" to the cp/mkdir commands if the library/header installation folder needs administrative access
	ifneq (,$(findstring /usr,$(MTF_LIB_INSTALL_DIR)))
		MTF_LIB_INSTALL_CMD_PREFIX = sudo
	endif
	ifneq (,$(findstring /usr,$(MTF_HEADER_INSTALL_DIR)))
		MTF_HEADER_INSTALL_CMD_PREFIX = sudo
	endif
	MKDIR_CMD = mkdir -p
	PATHSEP2 = /
endif

RM_CMD = rm -f	
# if Mac OSX, then copy header files with -f instead of -u
ifeq ($(shell uname -s), Darwin)
	COPY_FLAG = -f
else
	COPY_FLAG = -u
endif
CP_CMD = cp ${COPY_FLAG}
# PATHSEP2 = /
PATHSEP=$(strip $(PATHSEP2))
MTF_COMPILETIME_FLAGS += ${OPENCV_FLAGS}
MTF_RUNTIME_FLAGS += ${OPENCV_FLAGS}

ifneq (${ver}, 0)
	LIB_POST_FIX = $(addprefix _v, ${ver})
endif
ifeq (${caffe_cpu}, 1)
	CAFFE_FLAGS += -D CPU_ONLY
endif
# Optimization
ifeq (${o}, 1)
	OPT_FLAGS = -O3 -D NDEBUG -D EIGEN_NO_DEBUG
	_MTF_LIB_NAME = mtf
	MTF_NT_LIB_NAME = mtf_nt
	# Build Version
	ifneq (${ver}, 0)
		BUILD_DIR = ${BUILD_ROOT_DIR}${PATHSEP}release${PATHSEP}${ver}
	else
		BUILD_DIR = ${BUILD_ROOT_DIR}${PATHSEP}release
	endif	
else ifeq (${o}, 2)
	OPT_FLAGS = -O3 -ffast-math -D NDEBUG -D EIGEN_NO_DEBUG
	_MTF_LIB_NAME = mtf_fast
	MTF_NT_LIB_NAME = mtf_nt_fast
	# Build Version
	ifneq (${ver}, 0)
		BUILD_DIR = ${BUILD_ROOT_DIR}${PATHSEP}fast${PATHSEP}${ver}
	else
		BUILD_DIR = ${BUILD_ROOT_DIR}${PATHSEP}fast
	endif	
else
	OPT_FLAGS += -g -O0
	_MTF_LIB_NAME = mtf_debug
	MTF_NT_LIB_NAME = mtf_nt_debug
	# Build Version
	ifneq (${ver}, 0)
		BUILD_DIR = ${BUILD_ROOT_DIR}${PATHSEP}debug${PATHSEP}${ver}
	else
		BUILD_DIR = ${BUILD_ROOT_DIR}${PATHSEP}debug
	endif
endif
# Profiling
ifeq (${prf}, 1)
	PROF_FLAGS += -pg
endif
ifeq (${omp}, 1)
	MTF_COMPILETIME_FLAGS += -D ENABLE_PARALLEL -fopenmp 
	MTF_RUNTIME_FLAGS += -D ENABLE_PARALLEL -fopenmp 
	MTF_LIBS += -fopenmp
endif
# Selective Pixel Integration
ifeq (${spi}, 0)
	MTF_COMPILETIME_FLAGS += -D DISABLE_SPI
	MTF_RUNTIME_FLAGS += -D DISABLE_SPI
endif

ROOT_INCLUDE_DIR = include
ROOT_HEADER_DIR = ${ROOT_INCLUDE_DIR}${PATHSEP}mtf
MTF_INCLUDE_DIRS += ${ROOT_INCLUDE_DIR}

MTF_LIB_NAME = $(addsuffix ${LIB_POST_FIX}, ${_MTF_LIB_NAME})
MTF_LIB_SO =  $(addprefix lib, $(addsuffix ${MTF_LIB_EXT}, ${MTF_LIB_NAME}))
MTF_LIB_LINK +=  $(addprefix -l, ${MTF_LIB_NAME})
MTF_NT_LIB_SO =  $(addprefix lib, $(addsuffix ${MTF_LIB_EXT}, ${MTF_NT_LIB_NAME}))
MTF_NT_LIB_LINK +=  $(addprefix -l, ${MTF_NT_LIB_NAME})

# different sub parts or modules within MTF
MTF_SUB_DIRS = Macros Config Utilities AM SSM SM ThirdParty Diagnostics Examples
include $(foreach SUB_DIR,${MTF_SUB_DIRS},${SUB_DIR}${PATHSEP}${SUB_DIR}.mak)

# MTF_MISC_INCLUDES = SM${PATHSEP}GNN${PATHSEP}build_graph.h SM${PATHSEP}GNN${PATHSEP}utility.h
BASE_HEADERS += ${ROOT_HEADER_DIR}${PATHSEP}TrackerBase.h
BASE_HEADERS += ${ROOT_HEADER_DIR}${PATHSEP}TrackerStruct.h
#MTF_HEADERS= $(addprefix ${MTF_HEADER_DIR}${PATHSEP}, ${BASE_HEADERS} ${SEARCH_HEADERS} ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS} ${MTF_UTIL_HEADERS} ${COMPOSITE_HEADERS} ${TOOL_HEADERS} ${DIAG_BASE_HEADERS} ${DIAG_HEADERS} ${LEARNING_HEADERS})  
MTF_INSTALLED_INCLUDE_DIRS += $(addsuffix ${PATHSEP}., ${MTF_INCLUDE_DIRS})
# read make commands and definitions for each component of MTF

MTF_RUNTIME_FLAGS += -L${MTF_LIB_INSTALL_DIR}
MTF_INCLUDE_FLAGS += $(addprefix -I, ${MTF_INCLUDE_DIRS})

.PHONY: mtf
.PHONY: install install_lib install_header
.PHONY: clean

h1:
	@echo ${MTF_HEADERS}
all: mtf
mtf: ${BUILD_DIR}${PATHSEP}${MTF_LIB_SO}
# mtfn: ${BUILD_DIR}${PATHSEP}${MTF_NT_LIB_SO}
install: install_lib install_header
install_nt: install_lib_nt install_header
install_lib: ${MTF_LIB_INSTALL_DIR} ${MTF_LIB_INSTALL_DIR}${PATHSEP}${MTF_LIB_SO}
install_lib_nt: ${MTF_LIB_INSTALL_DIR} ${MTF_LIB_INSTALL_DIR}${PATHSEP}${MTF_NT_LIB_SO}
install_header: ${MTF_HEADER_INSTALL_DIR}
	@${MTF_HEADER_INSTALL_CMD_PREFIX} cp -a ${COPY_FLAG} -v ${MTF_INSTALLED_INCLUDE_DIRS} ${MTF_HEADER_INSTALL_DIR}${PATHSEP}	
clean: 
	rm -f ${BUILD_DIR}${PATHSEP}*.o ${BUILD_DIR}${PATHSEP}*${MTF_LIB_EXT}

${BUILD_DIR}:
		${MKDIR_CMD} $@
${MTF_LIB_INSTALL_DIR}:
		${MTF_LIB_INSTALL_CMD_PREFIX} ${MKDIR_CMD} $@	
${MTF_HEADER_INSTALL_DIR}:
		${MTF_HEADER_INSTALL_CMD_PREFIX} ${MKDIR_CMD} $@
		
${MTF_LIB_INSTALL_DIR}${PATHSEP}${MTF_LIB_SO}: ${BUILD_DIR}${PATHSEP}${MTF_LIB_SO} ${MTF_SO_INSTALLED}
	${MTF_LIB_INSTALL_CMD_PREFIX} cp -f $< $@
${MTF_LIB_INSTALL_DIR}${PATHSEP}${MTF_NT_LIB_SO}: ${BUILD_DIR}${PATHSEP}${MTF_NT_LIB_SO}
	${MTF_LIB_INSTALL_CMD_PREFIX} cp -f $< $@	
#@find . -type f -name '*.h' | cpio -p -d -v ${MTF_HEADER_INSTALL_DIR}${PATHSEP}		
# ${MTF_HEADER_INSTALL_DIR}${PATHSEP}%: %
	# sudo cp --parents $^ ${MTF_HEADER_INSTALL_DIR}
	
${MTF_OBJS}: | ${BUILD_DIR}
${BUILD_DIR}${PATHSEP}${MTF_LIB_SO}: ${MTF_OBJS} ${MTF_SO}
	${CXX} -shared -o $@  ${MTF_OBJS} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${OPENCV_LIBS} ${BOOST_LIBS}
	
${BUILD_DIR}${PATHSEP}${MTF_NT_LIB_SO}: ${MTF_NT_OBJS} ${MTF_SO}	
	${CXX} -shared -o $@  ${MTF_NT_OBJS} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${OPENCV_LIBS} ${BOOST_LIBS}
