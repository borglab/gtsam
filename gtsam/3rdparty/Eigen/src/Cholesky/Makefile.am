# Eigen build/install - just need to copy header files into correct place

# All the sub-directories that need to be built
SUBDIRS = 

# And the corresponding libraries produced
# TODO add sublibs when something in 3rdparty actually needs compilation
SUBLIBS = 

# use nostdinc to turn off -I. and -I.., we do not need them because 
# header files are qualified so they can be included in external projects.
AUTOMAKE_OPTIONS = nostdinc

# add headers from src folder for Eigen
#Cholesky:
headers  = LDLT.h
headers += LLT.h

Choleskydir = $(pkgincludedir)/3rdparty/Eigen/src/Cholesky
Cholesky_includedir = $(includedir)/gtsam/3rdparty/Eigen/src/Cholesky
Cholesky_HEADERS = $(headers) 