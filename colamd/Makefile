# Makefile to compile the unit test library, will end up in $(LIBDIR)

.PHONY: all install clean

all: libcolamd.a

CC ?= gcc
CFLAGS += -O2
CFLAGS += -fPIC


sources = $(shell ls *.c)

library = libcolamd.a

#Note: hack was added to ensure that flags are acutally used for compilation 
# This should probably be fixed, but will work for 64 bit machines now

$(library): colamd.o colamd_global.o ccolamd.o ccolamd_global.o
	$(CC) $(CFLAGS) -c -DDLONG -o colamd_l.o colamd.c
	ar crsv $@ $^ colamd_l.o
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
