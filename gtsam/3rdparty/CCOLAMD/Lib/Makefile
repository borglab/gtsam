#-------------------------------------------------------------------------------
# CCOLAMD Lib/Makefile
#-------------------------------------------------------------------------------

LIBRARY = libccolamd
VERSION = 2.9.6
SO_VERSION = 2

default: library

include ../../SuiteSparse_config/SuiteSparse_config.mk

# CCOLAMD depends on SuiteSparse_config
LDLIBS += -lsuitesparseconfig

# compile and install in SuiteSparse/lib
library:
	$(MAKE) install INSTALL=$(SUITESPARSE)

I = -I../Include -I../../SuiteSparse_config

INC = ../Include/ccolamd.h ../../SuiteSparse_config/SuiteSparse_config.h

SRC = ../Source/ccolamd.c

OBJ = ccolamd.o ccolamd_l.o

ccolamd.o: $(SRC) $(INC)
	$(CC) $(CF) $(I) -c ../Source/ccolamd.c

ccolamd_l.o: $(SRC) $(INC)
	$(CC) $(CF) $(I) -c ../Source/ccolamd.c -DDLONG -o ccolamd_l.o

# creates libccolamd.a, a C-callable CCOLAMD library
static: $(AR_TARGET)

$(AR_TARGET):  $(OBJ)
	$(ARCHIVE)  $@ $^
	- $(RANLIB) $@ 

ccode: library

clean:
	- $(RM) -r $(CLEAN)

purge: distclean

distclean: clean
	- $(RM) -r $(PURGE)

# install CCOLAMD
install: $(AR_TARGET) $(INSTALL_LIB)/$(SO_TARGET)

$(INSTALL_LIB)/$(SO_TARGET): $(OBJ)
	@mkdir -p $(INSTALL_LIB)
	@mkdir -p $(INSTALL_INCLUDE)
	@mkdir -p $(INSTALL_DOC)
	$(CC) $(SO_OPTS) $^ -o $@ $(LDLIBS)
	( cd $(INSTALL_LIB) ; ln -sf $(SO_TARGET) $(SO_PLAIN) )
	( cd $(INSTALL_LIB) ; ln -sf $(SO_TARGET) $(SO_MAIN) )
	$(CP) ../Include/ccolamd.h $(INSTALL_INCLUDE)
	$(CP) ../README.txt $(INSTALL_DOC)/CCOLAMD_README.txt
	chmod 755 $(INSTALL_LIB)/$(SO_TARGET)
	chmod 644 $(INSTALL_INCLUDE)/ccolamd.h
	chmod 644 $(INSTALL_DOC)/CCOLAMD_README.txt

uninstall:
	$(RM) $(INSTALL_LIB)/$(SO_TARGET)
	$(RM) $(INSTALL_LIB)/$(SO_PLAIN)
	$(RM) $(INSTALL_LIB)/$(SO_MAIN)
	$(RM) $(INSTALL_INCLUDE)/ccolamd.h
	$(RM) $(INSTALL_DOC)/CCOLAMD_README.txt

