#------------------------------------------------------------------------------
# CCOLAMD Makefile
#------------------------------------------------------------------------------

VERSION = 2.7.3

default: demos

include ../UFconfig/UFconfig.mk

# Compile all C code, including the C-callable routine and the mexFunctions.
# Do not the MATLAB interface.
demos:
	( cd Lib    ; $(MAKE) )
	( cd Demo   ; $(MAKE) )

# Compile all C code, including the C-callable routine and the mexFunctions.
all:
	( cd Lib    ; $(MAKE) )
	( cd Demo   ; $(MAKE) )
	( cd MATLAB ; $(MAKE) )

# compile just the C-callable libraries (not mexFunctions or Demos)
library:
	( cd Lib    ; $(MAKE) )

# remove object files, but keep the compiled programs and library archives
clean:
	( cd Lib    ; $(MAKE) clean )
	( cd Demo   ; $(MAKE) clean )
	( cd MATLAB ; $(MAKE) clean )

# clean, and then remove compiled programs and library archives
purge:
	( cd Lib    ; $(MAKE) purge )
	( cd Demo   ; $(MAKE) purge )
	( cd MATLAB ; $(MAKE) purge )

distclean: purge

# get ready for distribution
dist: purge
	( cd Demo   ; $(MAKE) dist )

ccode: library

lib: library

# compile the MATLAB mexFunction
mex:
	( cd MATLAB ; $(MAKE) )

# install CCOLAMD
install:
	$(CP) Lib/libccolamd.a $(INSTALL_LIB)/libccolamd.$(VERSION).a
	( cd $(INSTALL_LIB) ; ln -sf libccolamd.$(VERSION).a libccolamd.a )
	$(CP) Include/ccolamd.h $(INSTALL_INCLUDE)

# uninstall CCOLAMD
uninstall:
	$(RM) $(INSTALL_LIB)/libccolamd*.a
	$(RM) $(INSTALL_INCLUDE)/ccolamd.h

