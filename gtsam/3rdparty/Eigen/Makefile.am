# Eigen build/install - just need to copy header files into correct place

# All the sub-directories that need to be built
SUBDIRS = src

# And the corresponding libraries produced
SUBLIBS = 

# use nostdinc to turn off -I. and -I.., we do not need them because 
# header files are qualified so they can be included in external projects.
AUTOMAKE_OPTIONS = nostdinc

# add primary headers
headers  = Array Cholesky Core Dense Eigen Eigen2Support Eigenvalues 
headers += Geometry Householder Jacobi LeastSquares LU QR QtAlignedMalloc
headers += Sparse StdDeque StdList StdVector SVD

Eigendir = $(pkgincludedir)/3rdparty/Eigen
Eigen_includedir = $(includedir)/gtsam/3rdparty/Eigen
Eigen_HEADERS = $(headers) 