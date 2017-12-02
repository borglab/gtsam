# Makefile for null-output xerbla, both C and Fortran versions.
# By default, the C version (libcerbla.a and *.so) is compiled and installed. 
# Set the USE_FORTRAN option to 1 to create the Fortran instead (libxerbla):
  USE_FORTRAN = 0
# USE_FORTRAN = 1

VERSION = 1.0.3
SO_VERSION = 1

default: library

# compile and install in SuiteSparse/lib
library:
	$(MAKE) install INSTALL=$(SUITESPARSE)

all: library

ifeq ($(USE_FORTRAN),0)
    LIBRARY = libcerbla
else
    LIBRARY = libxerbla
endif

include ../SuiteSparse_config.mk

ifeq ($(USE_FORTRAN),0)
    COMPILE = $(CC) $(CF) -c xerbla.c
    DEPENDS = xerbla.c xerbla.h
else
    COMPILE = $(F77) $(F77FLAGS) -c xerbla.f
    DEPENDS = xerbla.f
endif

ccode: all

fortran: all

static: $(AR_TARGET)

$(AR_TARGET): $(DEPENDS)
	$(COMPILE)
	$(ARCHIVE)  $(AR_TARGET) xerbla.o
	- $(RANLIB) $(AR_TARGET)
	- $(RM) xerbla.o

# install libcerbla / libxerbla
install: $(AR_TARGET) $(INSTALL_LIB)/$(SO_TARGET)

$(INSTALL_LIB)/$(SO_TARGET): $(DEPENDS)
	@mkdir -p $(INSTALL_LIB)
	@mkdir -p $(INSTALL_INCLUDE)
	@mkdir -p $(INSTALL_DOC)
	$(COMPILE)
	$(CC) $(SO_OPTS) xerbla.o -o $@
	- $(RM) xerbla.o
	( cd $(INSTALL_LIB) ; ln -sf $(SO_TARGET) $(SO_PLAIN) )
	( cd $(INSTALL_LIB) ; ln -sf $(SO_TARGET) $(SO_MAIN) )
	$(CP) xerbla.h $(INSTALL_INCLUDE)
	chmod 755 $(INSTALL_LIB)/$(SO_TARGET)
	chmod 644 $(INSTALL_INCLUDE)/xerbla.h

# uninstall libcerbla / libxerbla
uninstall:
	$(RM) $(INSTALL_LIB)/$(SO_TARGET)
	$(RM) $(INSTALL_LIB)/$(SO_PLAIN)
	$(RM) $(INSTALL_INCLUDE)/xerbla.h

distclean: purge

purge: clean
	- $(RM) -r $(PURGE)

clean:
	- $(RM) -r $(CLEAN)

