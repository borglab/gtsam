# Eigen build/install - just need to copy header files into correct place

# All the sub-directories that need to be built
SUBDIRS = 

# And the corresponding libraries produced
# TODO add sublibs when something in 3rdparty actually needs compilation
SUBLIBS = 

# use nostdinc to turn off -I. and -I.., we do not need them because 
# header files are qualified so they can be included in external projects.
AUTOMAKE_OPTIONS = nostdinc

# LU/arch:
headers = Inverse_SSE.h

archdir = $(pkgincludedir)/3rdparty/Eigen/src/LU/arch
arch_includedir = $(includedir)/gtsam/3rdparty/Eigen/src/LU/arch
arch_HEADERS = $(headers) 