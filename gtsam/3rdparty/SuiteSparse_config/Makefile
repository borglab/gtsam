#-------------------------------------------------------------------------------
# SuiteSparse_config Makefile
#-------------------------------------------------------------------------------

SUITESPARSE ?= $(realpath $(CURDIR)/..)
export SUITESPARSE

# version of SuiteSparse_config is also version of SuiteSparse meta-package
LIBRARY = libsuitesparseconfig
VERSION = 5.4.0
SO_VERSION = 5

default: library

include SuiteSparse_config.mk

ccode: all

all: library

# compile and install in SuiteSparse/lib
library: $(AR_TARGET)
	$(MAKE) install INSTALL=$(SUITESPARSE)

OBJ = SuiteSparse_config.o

SuiteSparse_config.o: SuiteSparse_config.c SuiteSparse_config.h
	$(CC) $(CF) -c SuiteSparse_config.c

static: $(AR_TARGET)

$(AR_TARGET): $(OBJ)
	$(ARCHIVE) $(AR_TARGET) SuiteSparse_config.o
	$(RANLIB) $(AR_TARGET)

distclean: purge

purge: clean
	( cd xerbla ; $(MAKE) purge )
	- $(RM) -r $(PURGE)

clean:
	( cd xerbla ; $(MAKE) clean )
	- $(RM) -r $(CLEAN)

# install SuiteSparse_config
install: $(AR_TARGET) $(INSTALL_LIB)/$(SO_TARGET)

$(INSTALL_LIB)/$(SO_TARGET): $(OBJ)
	@mkdir -p $(INSTALL_LIB)
	@mkdir -p $(INSTALL_INCLUDE)
	@mkdir -p $(INSTALL_DOC)
	$(CC) $(SO_OPTS) $^ -o $@ $(LDLIBS)
	( cd $(INSTALL_LIB) ; ln -sf $(SO_TARGET) $(SO_PLAIN) )
	( cd $(INSTALL_LIB) ; ln -sf $(SO_TARGET) $(SO_MAIN) )
	$(CP) SuiteSparse_config.h $(INSTALL_INCLUDE)
	$(CP) README.txt $(INSTALL_DOC)/SUITESPARSECONFIG_README.txt
	chmod 755 $(INSTALL_LIB)/$(SO_TARGET)
	chmod 755 $(INSTALL_LIB)/$(SO_PLAIN)
	chmod 644 $(INSTALL_INCLUDE)/SuiteSparse_config.h
	chmod 644 $(INSTALL_DOC)/SUITESPARSECONFIG_README.txt

# uninstall SuiteSparse_config
uninstall:
	$(RM) $(INSTALL_LIB)/$(SO_TARGET)
	$(RM) $(INSTALL_LIB)/$(SO_PLAIN)
	$(RM) $(INSTALL_LIB)/$(SO_MAIN)
	$(RM) $(INSTALL_INCLUDE)/SuiteSparse_config.h
	$(RM) $(INSTALL_DOC)/SUITESPARSECONFIG_README.txt
	( cd xerbla ; $(MAKE) uninstall )


