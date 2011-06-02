# Makefile for UFconfig.c

VERSION = 3.6.0

default: ccode

include UFconfig.mk

ccode: libufconfig.a

all: libufconfig.a

libufconfig.a: UFconfig.c UFconfig.h
	$(CC) $(CFLAGS) -c UFconfig.c
	$(AR) libufconfig.a UFconfig.o
	- $(RM) UFconfig.o

distclean: purge

purge: clean
	- $(RM) *.o *.a

clean:
	- $(RM) -r $(CLEAN)

# install UFconfig
install:
	$(CP) libufconfig.a $(INSTALL_LIB)/libufconfig.$(VERSION).a
	( cd $(INSTALL_LIB) ; ln -sf libufconfig.$(VERSION).a libufconfig.a )
	$(CP) UFconfig.h $(INSTALL_INCLUDE)

# uninstall UFconfig
uninstall:
	$(RM) $(INSTALL_LIB)/libufconfig*.a
	$(RM) $(INSTALL_INCLUDE)/UFconfig.h

