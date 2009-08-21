# Makefile to compile the unit test library, will end up in $(LIBDIR)

.PHONY: all install clean

all: libCppUnitLite.a

CC=gcc
CXX=gcc
CPP=gcc
CXXFLAGS += -O2

sources = $(shell ls *.cpp)
objects = $(sources:.cpp=.o)

library = libCppUnitLite.a

$(library): $(objects) 
	ar crsv $@ $(objects) 
	ranlib $(library)

clean:
	rm -f *.o *.*~ $(library)

check:
	echo 'no check for CppUnitLite'

distdir:

install:
