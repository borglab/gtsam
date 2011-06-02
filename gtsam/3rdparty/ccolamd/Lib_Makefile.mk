#-------------------------------------------------------------------------------
# CCOLAMD Makefile
#-------------------------------------------------------------------------------

default: libccolamd.a

include ../../UFconfig/UFconfig.mk

I = -I../Include -I../../UFconfig

INC = ../Include/ccolamd.h ../../UFconfig/UFconfig.h

SRC = ../Source/ccolamd.c ../Source/ccolamd_global.c

# creates libccolamd.a, a C-callable COLAMD library
libccolamd.a:  $(SRC) $(INC)
	$(CC) $(CFLAGS) $(I) -c ../Source/ccolamd_global.c
	$(CC) $(CFLAGS) $(I) -c ../Source/ccolamd.c
	$(CC) $(CFLAGS) $(I) -c ../Source/ccolamd.c -DDLONG -o ccolamd_l.o
	$(AR) libccolamd.a ccolamd.o ccolamd_l.o ccolamd_global.o

ccode: libccolamd.a

library: libccolamd.a

clean:
	- $(RM) $(CLEAN)

purge: distclean

distclean: clean
	- $(RM) libccolamd.a
