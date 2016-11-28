MODULES = Homography IterativeOptimization Utilities
SOURCES =  $(addprefix src/, $(addsuffix .cpp, ${MODULES}))
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=DFTDemo
LIB_CPP ?= libdft.so

include Makefile.inc

help:           ## Show this help.
	@echo "——————— Available Targets  ——————— \n\n"
	@fgrep -h "##" $(MAKEFILE_LIST) | fgrep -v fgrep | sed -e 's/\\$$//' | sed -e 's/##//'
	@echo "\n\n"


all: ## 		build all in release mode
all: libCppRelease demoRelease unittestsRelease cleanObj

release: ## 	build all in release mode
release: libCppRelease demoRelease unittestsRelease cleanObj

debug: ## 	build all in debug mode
debug: libCppDebug demoDebug unittestsDebug cleanObj 


libCppRelease: ## 	C++ lib - release mode
libCppRelease: CFLAGS += $(CFLAGS_RELEASE)
libCppRelease:	$(LIB_CPP)

libCppDebug: ##		C++ lib - release mode
libCppDebug: CFLAGS += $(CFLAGS_DEBUG)
libCppDebug:	$(LIB_CPP)

$(LIB_CPP): $(SOURCES) $(OBJECTS)
	$(CC) -shared $(CFLAGS) $(OBJECTS) $(LDFLAGS) -o $@

demoRelease: ##		executable - release mode 
demoRelease: CFLAGS += $(CFLAGS_RELEASE)
demoRelease: libCppRelease $(EXECUTABLE)

demoDebug: ##		executable - debug mode 
demoDebug: CFLAGS += $(CFLAGS_DEBUG)
demoDebug: libCppDebug $(EXECUTABLE)

$(EXECUTABLE): $(SOURCES) $(OBJECTS)
	$(CC) src/ISMAR_Demo/main.cpp $(CFLAGS) $(LDFLAGS) -L. -l$(LIB_CPP) -o src/ISMAR_Demo/$@
	# cd ISMAR_Demo && ln -sf ../libdft.so libdft.so

unittestsRelease: ##		unit tests - release mode 
unittestsRelease: CFLAGS += $(CFLAGS_RELEASE)
unittestsRelease: libCppRelease unittests

unittestsDebug: ##		unit tests - debug mode 
unittestsDebug: CFLAGS += $(CFLAGS_DEBUG)
unittestsDebug: libCppDebug unittests

unittests: $(SOURCES) $(OBJECTS)
	$(CC) src/unit/UnitTests.cpp $(CFLAGS) $(LDFLAGS) -L. -l$(LIB_CPP) -o src/unit/$@ 
	# cd unit && ln -sf ../libdft.so libdft.so
.cpp.o:
	$(CC) -c $(CFLAGS) $< -o $@


.PHONY: clean all help

cleanObj:
	rm -rf src/ISMAR_Demo/main.o src/unit/UnitTests.o $(OBJECTS)

clean:
	rm -rf src/ISMAR_Demo/main.o src/ISMAR_Demo/$(LIB_CPP) src/unit/UnitTests.o src/unit/$(LIB_CPP) $(OBJECTS) $(LIB_CPP) src/unit/unittests src/ISMAR_Demo/$(EXECUTABLE)

