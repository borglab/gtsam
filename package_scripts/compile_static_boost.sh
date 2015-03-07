#!/bin/sh

# Compile boost statically, with -fPIC to allow linking it into the mex
# module (which is a dynamic library).  --disable-icu prevents depending
# on libicu, which is unneeded and would require then linking the mex
# module with it as well.  We just stage instead of install, then the
# toolbox_package_unix.sh script uses the staged boost.
./b2 link=static threading=multi cxxflags=-fPIC cflags=-fPIC --disable-icu -a stage