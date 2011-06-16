# 3rd Party libraries to be built and installed along with gtsam

# All the sub-directories that need to be built
SUBDIRS = ccolamd

# And the corresponding libraries produced
SUBLIBS = ccolamd/libccolamd.la

# set up the folders for includes
3rdpartydir = $(pkgincludedir)/3rdparty
3rdparty_includedir = $(includedir)/gtsam/3rdparty
nobase_3rdparty_HEADERS  =

# Eigen Installation - just copies the headers
eigen_path = 
eigen_path += Eigen  
nobase_3rdparty_HEADERS += $(eigen_path)/Array         $(eigen_path)/LeastSquares
nobase_3rdparty_HEADERS += $(eigen_path)/Cholesky      $(eigen_path)/LU
nobase_3rdparty_HEADERS += $(eigen_path)/Core          $(eigen_path)/QR
nobase_3rdparty_HEADERS += $(eigen_path)/Dense         $(eigen_path)/QtAlignedMalloc
nobase_3rdparty_HEADERS += $(eigen_path)/Eigen         $(eigen_path)/Sparse
nobase_3rdparty_HEADERS += $(eigen_path)/Eigen2Support $(eigen_path)/StdDeque
nobase_3rdparty_HEADERS += $(eigen_path)/Eigenvalues   $(eigen_path)/StdList
nobase_3rdparty_HEADERS += $(eigen_path)/Geometry      $(eigen_path)/StdVector
nobase_3rdparty_HEADERS += $(eigen_path)/Householder   $(eigen_path)/SVD
nobase_3rdparty_HEADERS += $(eigen_path)/Jacobi

##./src/Cholesky:
nobase_3rdparty_HEADERS += $(eigen_path)/src/Cholesky/LDLT.h  $(eigen_path)/src/Cholesky/LLT.h

##./src/Core:
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/ArrayBase.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/Array.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/ArrayWrapper.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/Assign.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/BandMatrix.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/Block.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/BooleanRedux.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/CommaInitializer.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/CwiseBinaryOp.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/CwiseNullaryOp.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/CwiseUnaryOp.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/CwiseUnaryView.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/DenseBase.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/DenseCoeffsBase.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/DenseStorage.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/Diagonal.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/DiagonalMatrix.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/DiagonalProduct.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/Dot.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/EigenBase.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/Flagged.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/ForceAlignedAccess.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/Functors.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/Fuzzy.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/GenericPacketMath.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/GlobalFunctions.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/IO.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/MapBase.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/Map.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/MathFunctions.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/MatrixBase.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/Matrix.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/NestByValue.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/NoAlias.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/NumTraits.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/PermutationMatrix.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/PlainObjectBase.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/ProductBase.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/Product.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/Random.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/Redux.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/Replicate.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/ReturnByValue.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/Reverse.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/Select.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/SelfAdjointView.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/SelfCwiseBinaryOp.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/SolveTriangular.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/StableNorm.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/Stride.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/Swap.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/Transpose.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/Transpositions.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/TriangularMatrix.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/VectorBlock.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/VectorwiseOp.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/Visitor.h

##./src/Core/arch/AltiVec:
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/arch/AltiVec/Complex.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/arch/AltiVec/PacketMath.h

##./src/Core/arch/Default:
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/arch/Default/Settings.h

##./src/Core/arch/NEON:
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/arch/NEON/Complex.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/arch/NEON/PacketMath.h

##./src/Core/arch/SSE:
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/arch/SSE/Complex.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/arch/SSE/MathFunctions.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/arch/SSE/PacketMath.h

##./src/Core/products:
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/products/CoeffBasedProduct.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/products/GeneralBlockPanelKernel.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/products/GeneralMatrixMatrix.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/products/GeneralMatrixMatrixTriangular.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/products/GeneralMatrixVector.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/products/Parallelizer.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/products/SelfadjointMatrixMatrix.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/products/SelfadjointMatrixVector.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/products/SelfadjointProduct.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/products/SelfadjointRank2Update.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/products/TriangularMatrixMatrix.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/products/TriangularMatrixVector.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/products/TriangularSolverMatrix.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/products/TriangularSolverVector.h

##./src/Core/util:
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/util/BlasUtil.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/util/Constants.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/util/DisableStupidWarnings.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/util/ForwardDeclarations.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/util/Macros.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/util/Memory.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/util/Meta.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/util/ReenableStupidWarnings.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/util/StaticAssert.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Core/util/XprHelper.h

##./src/Eigen2Support:
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/Block.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/Cwise.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/CwiseOperators.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/Lazy.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/LeastSquares.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/LU.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/Macros.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/MathFunctions.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/Memory.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/Meta.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/Minor.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/QR.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/SVD.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/TriangularSolver.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/VectorBlock.h

##./src/Eigen2Support/Geometry:
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/Geometry/AlignedBox.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/Geometry/All.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/Geometry/AngleAxis.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/Geometry/Hyperplane.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/Geometry/ParametrizedLine.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/Geometry/Quaternion.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/Geometry/Rotation2D.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/Geometry/RotationBase.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/Geometry/Scaling.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/Geometry/Transform.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigen2Support/Geometry/Translation.h

##./src/Eigenvalues:
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigenvalues/ComplexEigenSolver.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigenvalues/ComplexSchur.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigenvalues/EigenSolver.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigenvalues/EigenvaluesCommon.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigenvalues/GeneralizedSelfAdjointEigenSolver.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigenvalues/HessenbergDecomposition.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigenvalues/MatrixBaseEigenvalues.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigenvalues/RealSchur.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigenvalues/SelfAdjointEigenSolver.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Eigenvalues/Tridiagonalization.h

##./src/Geometry:
nobase_3rdparty_HEADERS += $(eigen_path)/src/Geometry/AlignedBox.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Geometry/AngleAxis.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Geometry/EulerAngles.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Geometry/Homogeneous.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Geometry/Hyperplane.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Geometry/OrthoMethods.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Geometry/ParametrizedLine.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Geometry/Quaternion.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Geometry/Rotation2D.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Geometry/RotationBase.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Geometry/Scaling.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Geometry/Transform.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Geometry/Translation.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Geometry/Umeyama.h

##./src/Geometry/arch:
nobase_3rdparty_HEADERS += $(eigen_path)/src/Geometry/arch/Geometry_SSE.h

##./src/Householder:
nobase_3rdparty_HEADERS += $(eigen_path)/src/Householder/BlockHouseholder.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Householder/Householder.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Householder/HouseholderSequence.h

##./src/Jacobi:
nobase_3rdparty_HEADERS += $(eigen_path)/src/Jacobi/Jacobi.h

##./src/LU:
nobase_3rdparty_HEADERS += $(eigen_path)/src/LU/Determinant.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/LU/FullPivLU.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/LU/Inverse.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/LU/PartialPivLU.h

##./src/LU/arch:
nobase_3rdparty_HEADERS += $(eigen_path)/src/LU/arch/Inverse_SSE.h

##./src/misc:
nobase_3rdparty_HEADERS += $(eigen_path)/src/misc/Image.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/misc/Kernel.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/misc/Solve.h

##./src/plugins:
nobase_3rdparty_HEADERS += $(eigen_path)/src/plugins/ArrayCwiseBinaryOps.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/plugins/ArrayCwiseUnaryOps.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/plugins/BlockMethods.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/plugins/CommonCwiseBinaryOps.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/plugins/CommonCwiseUnaryOps.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/plugins/MatrixCwiseBinaryOps.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/plugins/MatrixCwiseUnaryOps.h

##./src/QR:
nobase_3rdparty_HEADERS += $(eigen_path)/src/QR/ColPivHouseholderQR.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/QR/FullPivHouseholderQR.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/QR/HouseholderQR.h

##./src/Sparse:
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/AmbiVector.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/CompressedStorage.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/CoreIterators.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/DynamicSparseMatrix.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/MappedSparseMatrix.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/SparseAssign.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/SparseBlock.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/SparseCwiseBinaryOp.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/SparseCwiseUnaryOp.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/SparseDenseProduct.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/SparseDiagonalProduct.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/SparseDot.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/SparseFuzzy.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/SparseMatrixBase.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/SparseMatrix.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/SparseProduct.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/SparseRedux.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/SparseSelfAdjointView.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/SparseSparseProduct.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/SparseTranspose.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/SparseTriangularView.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/SparseUtil.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/SparseVector.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/SparseView.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/Sparse/TriangularSolver.h

##./src/StlSupport:
nobase_3rdparty_HEADERS += $(eigen_path)/src/StlSupport/details.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/StlSupport/StdDeque.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/StlSupport/StdList.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/StlSupport/StdVector.h

##./src/SVD:
nobase_3rdparty_HEADERS += $(eigen_path)/src/SVD/JacobiSVD.h
nobase_3rdparty_HEADERS += $(eigen_path)/src/SVD/UpperBidiagonalization.h



