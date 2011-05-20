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

# Core/products:
headers += CoeffBasedProduct.h
headers += GeneralBlockPanelKernel.h
headers += GeneralMatrixMatrix.h
headers += GeneralMatrixMatrixTriangular.h
headers += GeneralMatrixVector.h
headers += Parallelizer.h
headers += SelfadjointMatrixMatrix.h
headers += SelfadjointMatrixVector.h
headers += SelfadjointProduct.h
headers += SelfadjointRank2Update.h
headers += TriangularMatrixMatrix.h
headers += TriangularMatrixVector.h
headers += TriangularSolverMatrix.h
headers += TriangularSolverVector.h

productsdir = $(pkgincludedir)/3rdparty/Eigen/src/Core/products
products_includedir = $(includedir)/gtsam/3rdparty/Eigen/src/Core/products
products_HEADERS = $(headers) 