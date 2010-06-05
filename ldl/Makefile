#-------------------------------------------------------------------------------
# Makefile for the LDL library
#-------------------------------------------------------------------------------

default: all

include ./UFconfig.mk

CXXFLAGS += -O2
CXXFLAGS += -fPIC -I./

CXX ?= g++

all: demo

#-------------------------------------------------------------------------------
# the ldl library:
#-------------------------------------------------------------------------------

libldl.a: ldl.cpp ldl.h
	$(CXX) $(CXXFLAGS) -c ldl.cpp -o ldl.o
	$(AR) libldl.a ldl.o 
	$(RANLIB) libldl.a

demo: ldlsimple.cpp libldl.a
	$(CXX) ldlsimple.cpp -o ldldemo -L./ -lldl

clean:
	$(RM) $(CLEAN) libldl.a ldldemo

check:
	echo 'no check for ldl'

distdir:

install: all

distclean: clean

