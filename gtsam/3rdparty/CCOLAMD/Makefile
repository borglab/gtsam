#------------------------------------------------------------------------------
# CCOLAMD Makefile
#------------------------------------------------------------------------------

SUITESPARSE ?= $(realpath $(CURDIR)/..)
export SUITESPARSE

default: all

include ../SuiteSparse_config/SuiteSparse_config.mk

demos: all

# Compile all C code
all:
	( cd Lib    ; $(MAKE) )
	( cd Demo   ; $(MAKE) )

# compile just the C-callable libraries (not Demos)
library:
	( cd Lib    ; $(MAKE) )

# compile the static libraries only
static:
	( cd Lib    ; $(MAKE) static )

# remove object files, but keep the compiled programs and library archives
clean:
	( cd Lib    ; $(MAKE) clean )
	( cd Demo   ; $(MAKE) clean )
	( cd MATLAB ; $(RM) $(CLEAN) )

# clean, and then remove compiled programs and library archives
purge:
	( cd Lib    ; $(MAKE) purge )
	( cd Demo   ; $(MAKE) purge )
	( cd MATLAB ; $(RM) $(CLEAN) ; $(RM) *.mex* )

distclean: purge

# get ready for distribution
dist: purge
	( cd Demo   ; $(MAKE) dist )

ccode: library

lib: library

# install CCOLAMD
install:
	( cd Lib  ; $(MAKE) install )

# uninstall CCOLAMD
uninstall:
	( cd Lib  ; $(MAKE) uninstall )
