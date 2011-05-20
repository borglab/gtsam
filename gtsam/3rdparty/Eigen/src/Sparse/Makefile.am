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

# Sparse:
headers += AmbiVector.h
headers += CompressedStorage.h
headers += CoreIterators.h
headers += DynamicSparseMatrix.h
headers += MappedSparseMatrix.h
headers += SparseAssign.h
headers += SparseBlock.h
headers += SparseCwiseBinaryOp.h
headers += SparseCwiseUnaryOp.h
headers += SparseDenseProduct.h
headers += SparseDiagonalProduct.h
headers += SparseDot.h
headers += SparseFuzzy.h
headers += SparseMatrixBase.h
headers += SparseMatrix.h
headers += SparseProduct.h
headers += SparseRedux.h
headers += SparseSelfAdjointView.h
headers += SparseSparseProduct.h
headers += SparseTranspose.h
headers += SparseTriangularView.h
headers += SparseUtil.h
headers += SparseVector.h
headers += SparseView.h
headers += TriangularSolver.h

Sparsedir = $(pkgincludedir)/3rdparty/Eigen/src/Sparse
Sparse_includedir = $(includedir)/gtsam/3rdparty/Eigen/src/Sparse
Sparse_HEADERS = $(headers) 