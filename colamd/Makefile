# Makefile to compile the unit test library, will end up in $(LIBDIR)

.PHONY: all install clean

all: libcolamd.a

CC=gcc
CXX=gcc
CPP=gcc
CXXFLAGS += -O2
CXXFLAGS += -fPIC


sources = $(shell ls *.c)
objects = $(sources:.c=.o)

library = libcolamd.a

#Note: hack was added to ensure that flags are acutally used for compilation 
# This should probably be fixed, but will work for 64 bit machines now

$(library): $(objects) 
	$(CPP) $(CXXFLAGS) -c -o colamd.o colamd.c
	$(CPP) $(CXXFLAGS) -c -o colamd_global.o colamd_global.c
	ar crsv $@ $(objects) 
	ranlib $(library)

clean:
	rm -f *.o *.*~ $(library)

check:
	echo 'no check for colamd'

distdir:

install:

