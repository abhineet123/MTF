XVISION_ROOT_DIR = ThirdParty/Xvision
XVISION_SRC_DIR = ${XVISION_ROOT_DIR}/src
XVISION_INCLUDE_DIR = ${XVISION_ROOT_DIR}/include
XVISION_HEADER_DIR = ${XVISION_INCLUDE_DIR}/mtf/${XVISION_ROOT_DIR}

LIBSXV= -L/usr/X11R6/lib -lXVTrack -lXVDevs -lXVCons -lXVSeg -lXVTools \
	-lXVImages -ljpeg -lpng -ltiff -L/usr/X11R6/lib64 -lXext -lX11 -lavformat -lavcodec -lavutil -lpthread -lippi -lippcc -lipps  \
	-lraw1394 -ldc1394 -lmpeg /usr/lib/x86_64-linux-gnu/libXxf86dga.so.1 /usr/lib/x86_64-linux-gnu/libXxf86vm.so.1
FLAGSXV= -I/include/XVision2 -I/usr/include/dc1394/

XVISION_TRACKERS = xvSSDAffine xvSSDGrid xvSSDGridLine xvSSDHelper xvSSDMain xvSSDPyramidAffine xvSSDPyramidRotate xvSSDPyramidRT xvSSDPyramidSE2 xvSSDPyramidTrans xvSSDRotate xvSSDRT xvSSDScaling xvSSDSE2 xvSSDTR xvSSDTrans common xvColor xvEdgeTracker
XVISION_HEADERS =  $(addprefix ${XVISION_HEADER_DIR}/, $(addsuffix .h, ${XVISION_TRACKERS}))

xv ?= 0
ifeq ($(OS),Windows_NT)
	THIRD_PARTY_RUNTIME_FLAGS += -D DISABLE_XVISION
else
	ifeq (${xv}, 1)
		THIRD_PARTY_RUNTIME_FLAGS += ${FLAGSXV}
		THIRD_PARTY_LIBS += ${LIBSXV}
		TOOLS += inputXV
		THIRD_PARTY_HEADERS += ${XVISION_HEADERS}
		THIRD_PARTY_INCLUDE_DIRS += ${XVISION_INCLUDE_DIR}
	else
		XVISION_HEADERS =
		THIRD_PARTY_RUNTIME_FLAGS += -D DISABLE_XVISION
	endif
endif