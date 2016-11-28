# ------------------------------------------------------------------------------------- #
# ------------------------------------ Tools ------------------------------------ #
# ------------------------------------------------------------------------------------- #	
TOOLS = inputCV inputBase cvUtils PreProc
TOOL_HEADERS =  $(addprefix Examples/Tools/, $(addsuffix .h, ${TOOLS}))

ifeq (${o}, 1)
TOOLS_LIB_NAME=libmtf_tools.so
TOOLS_LIB_LINK=-lmtf_tools
else
TOOLS_LIB_NAME=libmtf_tools_debug.so
TOOLS_LIB_LINK=-lmtf_tools_debug
endif

${BUILD_DIR}/Tools:
		mkdir -p $@
${TOOLS_LIB_NAME}: ${BUILD_DIR}/Tools ${BUILD_DIR}/Tools/inputCV.o ${BUILD_DIR}/Tools/inputBase.o ${BUILD_DIR}/Tools/cvUtils.o
	${CXX} -shared -o $@  ${BUILD_DIR}/Tools/parameters.o ${BUILD_DIR}/Tools/inputCV.o ${BUILD_DIR}/Tools/inputBase.o ${BUILD_DIR}/Tools/cvUtils.o
	sudo cp -f $@ /usr/lib	
${BUILD_DIR}/Tools/inputCV.o: Tools/inputCV.cc Tools/inputCV.h
	${CXX} -c -fPIC ${WARNING_FLAGS} ${OPT_FLAGS} ${MTF_COMPILETIME_FLAGS} ${FLAGS64} ${FLAGSCV}  ${MTF_FLAGSXV} $< -o $@
${BUILD_DIR}/Tools/inputBase.o: Tools/inputBase.cc Tools/inputBase.h
	${CXX} -c -fPIC ${WARNING_FLAGS} ${OPT_FLAGS} ${MTF_COMPILETIME_FLAGS} ${FLAGS64} ${FLAGSCV}  ${MTF_FLAGSXV} $< -o $@
${BUILD_DIR}/Tools/cvUtils.o: Tools/cvUtils.cc Tools/cvUtils.h
	${CXX} -c -fPIC ${WARNING_FLAGS} ${OPT_FLAGS} ${MTF_COMPILETIME_FLAGS} ${FLAGS64} ${FLAGSCV} $< -o $@	