# Makefile to compile the unit test library, will end up in $(LIBDIR)

.PHONY: all install clean

all: libcolamd.a

CC ?= gcc
CXX ?= g++
CXXFLAGS += -O2
CXXFLAGS += -fPIC


sources = $(shell ls *.c)

library = libcolamd.a

#Note: hack was added to ensure that flags are acutally used for compilation 
# This should probably be fixed, but will work for 64 bit machines now

$(library):
	$(CXX) $(CXXFLAGS) -c         -o colamd.o   colamd.c
	$(CXX) $(CXXFLAGS) -c -DDLONG -o colamd_l.o colamd.c
	$(CXX) $(CXXFLAGS) -c -o colamd_global.o colamd_global.c
	ar crsv $@ colamd.o colamd_l.o colamd_global.o
	ranlib $(library)

clean:
	rm -f *.o *.*~ $(library)

check:
	echo 'no check for colamd'

distdir:

# Richard: added the following dependencies so that recursive make works:

install: all

distclean: clean

check: all
