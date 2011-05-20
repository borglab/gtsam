# Eigen build/install - just need to copy header files into correct place

# All the sub-directories that need to be built
SUBDIRS = arch products util

# And the corresponding libraries produced
# TODO add sublibs when something in 3rdparty actually needs compilation
SUBLIBS = 

# use nostdinc to turn off -I. and -I.., we do not need them because 
# header files are qualified so they can be included in external projects.
AUTOMAKE_OPTIONS = nostdinc

# add headers from src folder for Eigen
headers =

#Core:
headers += ArrayBase.h
headers += Array.h
headers += ArrayWrapper.h
headers += Assign.h
headers += BandMatrix.h
headers += Block.h
headers += BooleanRedux.h
headers += CommaInitializer.h
headers += CwiseBinaryOp.h
headers += CwiseNullaryOp.h
headers += CwiseUnaryOp.h
headers += CwiseUnaryView.h
headers += DenseBase.h
headers += DenseCoeffsBase.h
headers += DenseStorage.h
headers += Diagonal.h
headers += DiagonalMatrix.h
headers += DiagonalProduct.h
headers += Dot.h
headers += EigenBase.h
headers += Flagged.h
headers += ForceAlignedAccess.h
headers += Functors.h
headers += Fuzzy.h
headers += GenericPacketMath.h
headers += GlobalFunctions.h
headers += IO.h
headers += MapBase.h
headers += Map.h
headers += MathFunctions.h
headers += MatrixBase.h
headers += Matrix.h
headers += NestByValue.h
headers += NoAlias.h
headers += NumTraits.h
headers += PermutationMatrix.h
headers += PlainObjectBase.h
headers += ProductBase.h
headers += Product.h
headers += Random.h
headers += Redux.h
headers += Replicate.h
headers += ReturnByValue.h
headers += Reverse.h
headers += Select.h
headers += SelfAdjointView.h
headers += SelfCwiseBinaryOp.h
headers += SolveTriangular.h
headers += StableNorm.h
headers += Stride.h
headers += Swap.h
headers += Transpose.h
headers += Transpositions.h
headers += TriangularMatrix.h
headers += VectorBlock.h
headers += VectorwiseOp.h
headers += Visitor.h

Coredir = $(pkgincludedir)/3rdparty/Eigen/src/Core
Core_includedir = $(includedir)/gtsam/3rdparty/Eigen/src/Core
Core_HEADERS = $(headers) 