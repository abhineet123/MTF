MTF_LIB_INSTALL_DIR ?= /usr/local/lib
MTF_HEADER_INSTALL_DIR ?= /usr/local/include
EIGEN_INCLUDE_DIRS ?= /usr/local/include/eigen3 /usr/include/eigen3


# enable optimization (or Release build)
o ?= 1
# enable selective pixel integration - disabled by default for speed
spi ?= 0
# enable profiling
prf ?= 0 
# enable parallelization using OpenMP
omp ?= 0
# optional version postfix for all compiled files for multiple versions of the library to coexist
ver ?= 0
# enable Caffe
use_caffe ?= 0
# use Caffe in CPU ONLY mode
caffe_cpu ?= 0

FLAGSCV = `pkg-config --cflags opencv`
FLAGS64 = -I/usr/include
WARNING_FLAGS = -Wfatal-errors -Wno-write-strings -Wno-unused-result
LIBSCV = `pkg-config --libs opencv`
MTF_LIBS +=  -lstdc++ -lboost_random
MTF_LIBS_DIRS = -L/usr/local/lib
EIGEN_INCLUDE_FLAGS=$(addprefix -I, ${EIGEN_INCLUDE_DIRS})
MTF_COMPILETIME_FLAGS = -std=c++11 ${EIGEN_INCLUDE_FLAGS}
MTF_RUNTIME_FLAGS = -std=c++11 ${EIGEN_INCLUDE_FLAGS}
PROF_FLAGS =
LIB_POST_FIX =
CAFFE_FLAGS += -L/usr/lib/x86_64-linux-gnu/ -lcaffe -lglog -lprotobuf
MTF_LIB_INSTALL_CMD_PREFIX = 
MTF_HEADER_INSTALL_CMD_PREFIX =

# add "sudo" to the cp/mkdir commands if the library/header installation folder needs administrative access
ifneq (,$(findstring /usr,$(MTF_LIB_INSTALL_DIR)))
MTF_LIB_INSTALL_CMD_PREFIX = sudo
endif
ifneq (,$(findstring /usr,$(MTF_HEADER_INSTALL_DIR)))
MTF_HEADER_INSTALL_CMD_PREFIX = sudo
endif

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
BUILD_DIR = Build/Release/${ver}
else
BUILD_DIR = Build/Release
endif
else
OPT_FLAGS += -g -O0
_MTF_LIB_NAME = mtf_debug
MTF_NT_LIB_NAME = mtf_nt_debug
# Build Version
ifneq (${ver}, 0)
BUILD_DIR = Build/Debug/${ver}
else
BUILD_DIR = Build/Debug
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
ROOT_HEADER_DIR = ${ROOT_INCLUDE_DIR}/mtf
MTF_INCLUDE_DIRS += ${ROOT_INCLUDE_DIR}

MTF_LIB_NAME = $(addsuffix ${LIB_POST_FIX}, ${_MTF_LIB_NAME})
MTF_LIB_SO =  $(addprefix lib, $(addsuffix .so, ${MTF_LIB_NAME}))
MTF_LIB_LINK +=  $(addprefix -l, ${MTF_LIB_NAME})
MTF_NT_LIB_SO =  $(addprefix lib, $(addsuffix .so, ${MTF_NT_LIB_NAME}))
MTF_NT_LIB_LINK +=  $(addprefix -l, ${MTF_NT_LIB_NAME})

# different sub parts or modules within MTF
MTF_SUB_DIRS = Macros Config Utilities AM SSM SM ThirdParty Test Examples
include $(foreach SUB_DIR,${MTF_SUB_DIRS},${SUB_DIR}/${SUB_DIR}.mak)

# MTF_MISC_INCLUDES = SM/GNN/build_graph.h SM/GNN/utility.h
BASE_HEADERS += ${ROOT_HEADER_DIR}/TrackerBase.h
#MTF_HEADERS= $(addprefix ${MTF_HEADER_DIR}/, ${BASE_HEADERS} ${SEARCH_HEADERS} ${APPEARANCE_HEADERS} ${STATE_SPACE_HEADERS} ${MTF_UTIL_HEADERS} ${COMPOSITE_HEADERS} ${TOOL_HEADERS} ${DIAG_BASE_HEADERS} ${DIAG_HEADERS} ${LEARNING_HEADERS})  
MTF_INSTALLED_INCLUDE_DIRS += $(addsuffix /., ${MTF_INCLUDE_DIRS})
# read make commands and definitions for each component of MTF

MTF_INCLUDE_FLAGS = $(addprefix -I, ${MTF_INCLUDE_DIRS})

.PHONY: mtf
.PHONY: install install_lib install_header
.PHONY: clean

h1:
	@echo ${MTF_HEADERS}
all: mtf
mtf: ${BUILD_DIR}/${MTF_LIB_SO}
# mtfn: ${BUILD_DIR}/${MTF_NT_LIB_SO}
install: install_lib install_header
install_nt: install_lib_nt install_header
install_lib: ${MTF_LIB_INSTALL_DIR} ${MTF_LIB_INSTALL_DIR}/${MTF_LIB_SO}
install_lib_nt: ${MTF_LIB_INSTALL_DIR} ${MTF_LIB_INSTALL_DIR}/${MTF_NT_LIB_SO}
install_header: ${MTF_HEADER_INSTALL_DIR}
	@${MTF_HEADER_INSTALL_CMD_PREFIX} cp -a -u -v ${MTF_INSTALLED_INCLUDE_DIRS} ${MTF_HEADER_INSTALL_DIR}/	
clean: 
	rm -f ${BUILD_DIR}/*.o ${BUILD_DIR}/*.so

${BUILD_DIR}:
		mkdir -p $@
${MTF_LIB_INSTALL_DIR}:
		${MTF_LIB_INSTALL_CMD_PREFIX} mkdir -p $@	
${MTF_HEADER_INSTALL_DIR}:
		${MTF_HEADER_INSTALL_CMD_PREFIX} mkdir -p $@
		
${MTF_LIB_INSTALL_DIR}/${MTF_LIB_SO}: ${BUILD_DIR}/${MTF_LIB_SO} ${MTF_SO_INSTALLED}
	${MTF_LIB_INSTALL_CMD_PREFIX} cp -f $< $@
${MTF_LIB_INSTALL_DIR}/${MTF_NT_LIB_SO}: ${BUILD_DIR}/${MTF_NT_LIB_SO}
	${MTF_LIB_INSTALL_CMD_PREFIX} cp -f $< $@	
#@find . -type f -name '*.h' | cpio -p -d -v ${MTF_HEADER_INSTALL_DIR}/		
# ${MTF_HEADER_INSTALL_DIR}/%: %
	# sudo cp --parents $^ ${MTF_HEADER_INSTALL_DIR}
	
${MTF_OBJS}: | ${BUILD_DIR}
${BUILD_DIR}/${MTF_LIB_SO}: ${MTF_OBJS} ${MTF_SO}
	${CXX} -shared -o $@  ${MTF_OBJS} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${LIBSCV}
	
${BUILD_DIR}/${MTF_NT_LIB_SO}: ${MTF_NT_OBJS} ${MTF_SO}	
	${CXX} -shared -o $@  ${MTF_NT_OBJS} ${MTF_LIBS_DIRS} ${MTF_LIBS} ${LIBSCV}
