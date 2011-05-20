# Eigen build/install - just need to copy header files into correct place

# All the sub-directories that need to be built
SUBDIRS = 

# And the corresponding libraries produced
# TODO add sublibs when something in 3rdparty actually needs compilation
SUBLIBS = 

# use nostdinc to turn off -I. and -I.., we do not need them because 
# header files are qualified so they can be included in external projects.
AUTOMAKE_OPTIONS = nostdinc

headers  = 

# plugins:
headers += ArrayCwiseBinaryOps.h
headers += ArrayCwiseUnaryOps.h
headers += BlockMethods.h
headers += CommonCwiseBinaryOps.h
headers += CommonCwiseUnaryOps.h
headers += MatrixCwiseBinaryOps.h
headers += MatrixCwiseUnaryOps.h

pluginsdir = $(pkgincludedir)/3rdparty/Eigen/src/plugins
plugins_includedir = $(includedir)/gtsam/3rdparty/Eigen/src/plugins
plugins_HEADERS = $(headers) 