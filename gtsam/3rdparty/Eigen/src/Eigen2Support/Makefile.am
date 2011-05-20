# Eigen build/install - just need to copy header files into correct place

# All the sub-directories that need to be built
SUBDIRS = Geometry

# And the corresponding libraries produced
# TODO add sublibs when something in 3rdparty actually needs compilation
SUBLIBS = 

# use nostdinc to turn off -I. and -I.., we do not need them because 
# header files are qualified so they can be included in external projects.
AUTOMAKE_OPTIONS = nostdinc

headers  = 

# Eigen2Support:
headers += Block.h
headers += Cwise.h
headers += CwiseOperators.h
headers += Lazy.h
headers += LeastSquares.h
headers += LU.h
headers += Macros.h
headers += MathFunctions.h
headers += Memory.h
headers += Meta.h
headers += Minor.h
headers += QR.h
headers += SVD.h
headers += TriangularSolver.h
headers += VectorBlock.h

Eigen2Supportdir = $(pkgincludedir)/3rdparty/Eigen/src/Eigen2Support
Eigen2Support_includedir = $(includedir)/gtsam/3rdparty/Eigen/src/Eigen2Support
Eigen2Support_HEADERS = $(headers) 