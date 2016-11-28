UNAME=$(shell uname)

CC=g++
LDFLAGS = `pkg-config --libs opencv`
CFLAGS = `pkg-config --cflags eigen3` `pkg-config --cflags opencv ` -Iinclude -std=c++11 -Wfatal-errors
CFLAGS_RELEASE= -Wfatal-errors -O3 -DNDEBUG -DEIGEN_NO_DEBUG -ffast-math
CFLAGS_DEBUG= -g -O0
ifeq ($(UNAME),Linux)
	CFLAGS +=  -fopenmp -fPIC
	LDFLAGS += -fopenmp 
endif
