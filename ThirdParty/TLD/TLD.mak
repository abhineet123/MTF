TLD_ROOT_DIR = ThirdParty/TLD
TLD_SRC_DIR = ${TLD_ROOT_DIR}/src
TLD_INCLUDE_DIR = ${TLD_ROOT_DIR}/include
TLD_HEADER_DIR = ${TLD_INCLUDE_DIR}/mtf/${TLD_ROOT_DIR}
TLD_LIB_NAME = $(addsuffix ${LIB_POST_FIX}, opentld)
TLD_LIB_SO =  $(addprefix lib, $(addsuffix .so, ${TLD_LIB_NAME}))
CVBLOBS_LIB_NAME = $(addsuffix ${LIB_POST_FIX}, cvblobs)
CVBLOBS_LIB_SO =  $(addprefix lib, $(addsuffix .so, ${CVBLOBS_LIB_NAME}))

THIRD_PARTY_TRACKERS += TLD
_THIRD_PARTY_TRACKERS_SO += ${TLD_LIB_NAME} ${CVBLOBS_LIB_NAME}
THIRD_PARTY_TRACKERS_SO_LOCAL +=  ${CVBLOBS_ROOT_DIR}/${CVBLOBS_LIB_SO} ${TLD_ROOT_DIR}/${TLD_LIB_SO}
THIRD_PARTY_LIBS_DIRS += -L${TLD_ROOT_DIR} -L${CVBLOBS_ROOT_DIR}

TLD_HEADERS = $(addprefix  ${TLD_HEADER_DIR}/, TLD.h)

TLD_LIB_MODULES = TLDUtil VarianceFilter Clustering DetectionResult DetectorCascade EnsembleClassifier ForegroundDetector MedianFlowTracker NNClassifier mftracker/Median mftracker/BB mftracker/BBPredict mftracker/FBTrack mftracker/Lk  
TLD_LIB_INCLUDES = IntegralImage NormalizedPatch
TLD_LIB_HEADERS = $(addprefix ${TLD_HEADER_DIR}/,$(addsuffix .h, ${TLD_LIB_MODULES} ${TLD_LIB_INCLUDES}))
TLD_LIB_SRC = $(addprefix ${TLD_SRC_DIR}/,$(addsuffix .cpp, ${TLD_LIB_MODULES}))

CVBLOBS_ROOT_DIR = ${TLD_ROOT_DIR}/3rdparty/cvblobs
CVBLOBS_LIB_MODULES = ComponentLabeling blob BlobContour BlobOperators BlobProperties BlobResult
CVBLOBS_LIB_INCLUDES = BlobLibraryConfiguration 
CVBLOBS_LIB_HEADERS = $(addprefix ${CVBLOBS_ROOT_DIR}/,$(addsuffix .h, ${CVBLOBS_LIB_MODULES} ${CVBLOBS_LIB_INCLUDES}))
CVBLOBS_LIB_SRC = $(addprefix ${CVBLOBS_ROOT_DIR}/,$(addsuffix .cpp, ${CVBLOBS_LIB_MODULES}))

THIRD_PARTY_HEADERS += ${TLD_HEADERS} ${TLD_LIB_HEADERS} ${CVBLOBS_LIB_HEADERS} 
THIRD_PARTY_INCLUDE_DIRS += ${TLD_INCLUDE_DIR}

${BUILD_DIR}/TLD.o: ${TLD_SRC_DIR}/TLD.cc ${TLD_HEADERS} ${UTILITIES_HEADER_DIR}/miscUtils.h ${MACROS_HEADER_DIR}/common.h ${ROOT_HEADER_DIR}/TrackerBase.h
	${CXX} -c ${MTF_PIC_FLAG} ${WARNING_FLAGS} ${OPT_FLAGS} ${MTF_COMPILETIME_FLAGS} $< ${OPENCV_FLAGS} -I${TLD_INCLUDE_DIR} -I${UTILITIES_INCLUDE_DIR} -I${MACROS_INCLUDE_DIR} -I${ROOT_INCLUDE_DIR} -o $@
	
${MTF_LIB_INSTALL_DIR}/${TLD_LIB_SO}: ${TLD_ROOT_DIR}/${TLD_LIB_SO}
	${MTF_LIB_INSTALL_CMD_PREFIX} cp -f $< $@
${TLD_ROOT_DIR}/${TLD_LIB_SO}: ${TLD_LIB_SRC} ${TLD_LIB_HEADERS}
	cd ${TLD_ROOT_DIR}; rm -rf Build; mkdir Build; cd Build; cmake -D TLD_LIB_NAME=${TLD_LIB_NAME} -D CV_BLOBS_LIB_NAME=${CVBLOBS_LIB_NAME} ..
	$(MAKE) -C ${TLD_ROOT_DIR}/Build --no-print-directory
	mv ${TLD_ROOT_DIR}/Build/${TLD_LIB_SO} ${TLD_ROOT_DIR}/	

${MTF_LIB_INSTALL_DIR}/${CVBLOBS_LIB_SO}: ${CVBLOBS_ROOT_DIR}/${CVBLOBS_LIB_SO}
	${MTF_LIB_INSTALL_CMD_PREFIX} cp -f $< $@
${CVBLOBS_ROOT_DIR}/${CVBLOBS_LIB_SO}: ${CVBLOBS_LIB_SRC} ${CVBLOBS_LIB_HEADERS}
	cd ${CVBLOBS_ROOT_DIR}; rm -rf Build; mkdir Build; cd Build; cmake  -D CV_BLOBS_LIB_NAME=${CVBLOBS_LIB_NAME} ..
	$(MAKE) -C ${CVBLOBS_ROOT_DIR}/Build --no-print-directory
	mv ${CVBLOBS_ROOT_DIR}/Build/${CVBLOBS_LIB_SO} ${CVBLOBS_ROOT_DIR}/	