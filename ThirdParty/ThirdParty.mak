# ------------------------------------------------------------------------------------- #
# ------------------------------------ Third Party ------------------------------------ #
# ------------------------------------------------------------------------------------- #

lt ?= 1
ifeq (${lt}, 0)
vp = 0
xv = 0
MTF_RUNTIME_FLAGS += -D DISABLE_THIRD_PARTY_TRACKERS
else
MTF_INCLUDE_DIRS += ${THIRD_PARTY_INCLUDE_DIRS}
MTF_OBJS += ${THIRD_PARTY_OBJS}
MTF_SO_INSTALLED += ${THIRD_PARTY_TRACKERS_SO}
MTF_SO += ${THIRD_PARTY_TRACKERS_SO_LOCAL}
MTF_HEADERS += ${THIRD_PARTY_HEADERS}
MTF_LIBS += ${THIRD_PARTY_LIBS} ${THIRD_PARTY_LINK_LIBS} 
MTF_LIBS_DIRS +=  ${THIRD_PARTY_LIBS_DIRS}

endif
MTF_RUNTIME_FLAGS += ${THIRD_PARTY_RUNTIME_FLAGS}

THIRD_PARTY_TRACKERS_SO_LOCAL =
THIRD_PARTY_LINK_LIBS = 
THIRD_PARTY_DIRS_SUB_DIRS = CMT DSST KCF RCT Struck TLD MIL DFT FRG PFSL3 GOTURN ViSP Xvision

include $(foreach SUB_DIR,${THIRD_PARTY_DIRS_SUB_DIRS},ThirdParty/${SUB_DIR}/${SUB_DIR}.mak)

THIRD_PARTY_TRACKERS_SO =  $(addprefix ${MTF_LIB_INSTALL_DIR}/lib, $(addsuffix .so, ${_THIRD_PARTY_TRACKERS_SO}))
THIRD_PARTY_LIBS =  $(addprefix -l, ${_THIRD_PARTY_TRACKERS_SO})
# THIRD_PARTY_LIBS_DIRS = $(foreach SUB_DIR,${THIRD_PARTY_DIRS_SUB_DIRS},-L ThirdParty/${SUB_DIR})
THIRD_PARTY_OBJS = $(addprefix ${BUILD_DIR}/,$(addsuffix .o, ${THIRD_PARTY_TRACKERS}))


	



	

	

	

