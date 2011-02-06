// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2006-2009 Benoit Jacob <jacob.benoit.1@gmail.com>
// Copyright (C) 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// Eigen is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// Alternatively, you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
//
// Eigen is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License or the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License and a copy of the GNU General Public License along with
// Eigen. If not, see <http://www.gnu.org/licenses/>.

#ifndef EIGEN_MATRIXBASE_H
#define EIGEN_MATRIXBASE_H

/** \class MatrixBase
  * \ingroup Core_Module
  *
  * \brief Base class for all dense matrices, vectors, and expressions
  *
  * This class is the base that is inherited by all matrix, vector, and related expression
  * types. Most of the Eigen API is contained in this class, and its base classes. Other important
  * classes for the Eigen API are Matrix, and VectorwiseOp.
  *
  * Note that some methods are defined in other modules such as the \ref LU_Module LU module
  * for all functions related to matrix inversions.
  *
  * \param Derived is the derived type, e.g. a matrix type, or an expression, etc.
  *
  * When writing a function taking Eigen objects as argument, if you want your function
  * to take as argument any matrix, vector, or expression, just let it take a
  * MatrixBase argument. As an example, here is a function printFirstRow which, given
  * a matrix, vector, or expression \a x, prints the first row of \a x.
  *
  * \code
    template<typename Derived>
    void printFirstRow(const Eigen::MatrixBase<Derived>& x)
    {
      cout << x.row(0) << endl;
    }
  * \endcode
  *
  * \sa \ref TopicClassHierarchy
  */
template<typename Derived> class MatrixBase
  : public DenseBase<Derived>
{
  public:
#ifndef EIGEN_PARSED_BY_DOXYGEN
    typedef MatrixBase StorageBaseType;
    typedef typename ei_traits<Derived>::StorageKind StorageKind;
    typedef typename ei_traits<Derived>::Index Index;
    typedef typename ei_traits<Derived>::Scalar Scalar;
    typedef typename ei_packet_traits<Scalar>::type PacketScalar;
    typedef typename NumTraits<Scalar>::Real RealScalar;

    typedef DenseBase<Derived> Base;
    using Base::RowsAtCompileTime;
    using Base::ColsAtCompileTime;
    using Base::SizeAtCompileTime;
    using Base::MaxRowsAtCompileTime;
    using Base::MaxColsAtCompileTime;
    using Base::MaxSizeAtCompileTime;
    using Base::IsVectorAtCompileTime;
    using Base::Flags;
    using Base::CoeffReadCost;

    using Base::derived;
    using Base::const_cast_derived;
    using Base::rows;
    using Base::cols;
    using Base::size;
    using Base::coeff;
    using Base::coeffRef;
    using Base::lazyAssign;
    using Base::eval;
    using Base::operator+=;
    using Base::operator-=;
    using Base::operator*=;
    using Base::operator/=;

    typedef typename Base::CoeffReturnType CoeffReturnType;
    typedef typename Base::RowXpr RowXpr;
    typedef typename Base::ColXpr ColXpr;
#endif // not EIGEN_PARSED_BY_DOXYGEN



#ifndef EIGEN_PARSED_BY_DOXYGEN
    /** type of the equivalent square matrix */
    typedef Matrix<Scalar,EIGEN_SIZE_MAX(RowsAtCompileTime,ColsAtCompileTime),
                          EIGEN_SIZE_MAX(RowsAtCompileTime,ColsAtCompileTime)> SquareMatrixType;
#endif // not EIGEN_PARSED_BY_DOXYGEN

    /** \returns the size of the main diagonal, which is min(rows(),cols()).
      * \sa rows(), cols(), SizeAtCompileTime. */
    inline Index diagonalSize() const { return std::min(rows(),cols()); }

    /** \brief The plain matrix type corresponding to this expression.
      *
      * This is not necessarily exactly the return type of eval(). In the case of plain matrices,
      * the return type of eval() is a const reference to a matrix, not a matrix! It is however guaranteed
      * that the return type of eval() is either PlainObject or const PlainObject&.
      */
    typedef Matrix<typename ei_traits<Derived>::Scalar,
                ei_traits<Derived>::RowsAtCompileTime,
                ei_traits<Derived>::ColsAtCompileTime,
                AutoAlign | (ei_traits<Derived>::Flags&RowMajorBit ? RowMajor : ColMajor),
                ei_traits<Derived>::MaxRowsAtCompileTime,
                ei_traits<Derived>::MaxColsAtCompileTime
          > PlainObject;

#ifndef EIGEN_PARSED_BY_DOXYGEN
    /** \internal Represents a matrix with all coefficients equal to one another*/
    typedef CwiseNullaryOp<ei_scalar_constant_op<Scalar>,Derived> ConstantReturnType;
    /** \internal the return type of MatrixBase::adjoint() */
    typedef typename ei_meta_if<NumTraits<Scalar>::IsComplex,
                        CwiseUnaryOp<ei_scalar_conjugate_op<Scalar>, Eigen::Transpose<Derived> >,
                        Transpose<Derived>
                     >::ret AdjointReturnType;
    /** \internal Return type of eigenvalues() */
    typedef Matrix<std::complex<RealScalar>, ei_traits<Derived>::ColsAtCompileTime, 1, ColMajor> EigenvaluesReturnType;
    /** \internal the return type of identity */
    typedef CwiseNullaryOp<ei_scalar_identity_op<Scalar>,Derived> IdentityReturnType;
    /** \internal the return type of unit vectors */
    typedef Block<CwiseNullaryOp<ei_scalar_identity_op<Scalar>, SquareMatrixType>,
                  ei_traits<Derived>::RowsAtCompileTime,
                  ei_traits<Derived>::ColsAtCompileTime> BasisReturnType;
#endif // not EIGEN_PARSED_BY_DOXYGEN

#define EIGEN_CURRENT_STORAGE_BASE_CLASS Eigen::MatrixBase
#   include "../plugins/CommonCwiseUnaryOps.h"
#   include "../plugins/CommonCwiseBinaryOps.h"
#   include "../plugins/MatrixCwiseUnaryOps.h"
#   include "../plugins/MatrixCwiseBinaryOps.h"
#   ifdef EIGEN_MATRIXBASE_PLUGIN
#     include EIGEN_MATRIXBASE_PLUGIN
#   endif
#undef EIGEN_CURRENT_STORAGE_BASE_CLASS

    /** Special case of the template operator=, in order to prevent the compiler
      * from generating a default operator= (issue hit with g++ 4.1)
      */
    Derived& operator=(const MatrixBase& other);

    // We cannot inherit here via Base::operator= since it is causing
    // trouble with MSVC.

    template <typename OtherDerived>
    Derived& operator=(const DenseBase<OtherDerived>& other);

    template <typename OtherDerived>
    Derived& operator=(const EigenBase<OtherDerived>& other);

    template<typename OtherDerived>
    Derived& operator=(const ReturnByValue<OtherDerived>& other);

#ifndef EIGEN_PARSED_BY_DOXYGEN
    template<typename ProductDerived, typename Lhs, typename Rhs>
    Derived& lazyAssign(const ProductBase<ProductDerived, Lhs,Rhs>& other);
#endif // not EIGEN_PARSED_BY_DOXYGEN

    template<typename OtherDerived>
    Derived& operator+=(const MatrixBase<OtherDerived>& other);
    template<typename OtherDerived>
    Derived& operator-=(const MatrixBase<OtherDerived>& other);

    template<typename OtherDerived>
    const typename ProductReturnType<Derived,OtherDerived>::Type
    operator*(const MatrixBase<OtherDerived> &other) const;

    template<typename OtherDerived>
    const typename LazyProductReturnType<Derived,OtherDerived>::Type
    lazyProduct(const MatrixBase<OtherDerived> &other) const;

    template<typename OtherDerived>
    Derived& operator*=(const EigenBase<OtherDerived>& other);

    template<typename OtherDerived>
    void applyOnTheLeft(const EigenBase<OtherDerived>& other);

    template<typename OtherDerived>
    void applyOnTheRight(const EigenBase<OtherDerived>& other);

    template<typename DiagonalDerived>
    const DiagonalProduct<Derived, DiagonalDerived, OnTheRight>
    operator*(const DiagonalBase<DiagonalDerived> &diagonal) const;

    template<typename OtherDerived>
    Scalar dot(const MatrixBase<OtherDerived>& other) const;
    RealScalar squaredNorm() const;
    RealScalar norm() const;
    RealScalar stableNorm() const;
    RealScalar blueNorm() const;
    RealScalar hypotNorm() const;
    const PlainObject normalized() const;
    void normalize();

    const AdjointReturnType adjoint() const;
    void adjointInPlace();

    Diagonal<Derived,0> diagonal();
    const Diagonal<Derived,0> diagonal() const;

    template<int Index> Diagonal<Derived,Index> diagonal();
    template<int Index> const Diagonal<Derived,Index> diagonal() const;

    Diagonal<Derived, Dynamic> diagonal(Index index);
    const Diagonal<Derived, Dynamic> diagonal(Index index) const;

    template<unsigned int Mode> TriangularView<Derived, Mode> part();
    template<unsigned int Mode> const TriangularView<Derived, Mode> part() const;

    template<unsigned int Mode> TriangularView<Derived, Mode> triangularView();
    template<unsigned int Mode> const TriangularView<Derived, Mode> triangularView() const;

    template<unsigned int UpLo> SelfAdjointView<Derived, UpLo> selfadjointView();
    template<unsigned int UpLo> const SelfAdjointView<Derived, UpLo> selfadjointView() const;

    const SparseView<Derived> sparseView(const Scalar& m_reference = Scalar(0),
                                         typename NumTraits<Scalar>::Real m_epsilon = NumTraits<Scalar>::dummy_precision()) const;
    static const IdentityReturnType Identity();
    static const IdentityReturnType Identity(Index rows, Index cols);
    static const BasisReturnType Unit(Index size, Index i);
    static const BasisReturnType Unit(Index i);
    static const BasisReturnType UnitX();
    static const BasisReturnType UnitY();
    static const BasisReturnType UnitZ();
    static const BasisReturnType UnitW();

    const DiagonalWrapper<Derived> asDiagonal() const;

    Derived& setIdentity();
    Derived& setIdentity(Index rows, Index cols);

    bool isIdentity(RealScalar prec = NumTraits<Scalar>::dummy_precision()) const;
    bool isDiagonal(RealScalar prec = NumTraits<Scalar>::dummy_precision()) const;

    bool isUpperTriangular(RealScalar prec = NumTraits<Scalar>::dummy_precision()) const;
    bool isLowerTriangular(RealScalar prec = NumTraits<Scalar>::dummy_precision()) const;

    template<typename OtherDerived>
    bool isOrthogonal(const MatrixBase<OtherDerived>& other,
                      RealScalar prec = NumTraits<Scalar>::dummy_precision()) const;
    bool isUnitary(RealScalar prec = NumTraits<Scalar>::dummy_precision()) const;

    /** \returns true if each coefficients of \c *this and \a other are all exactly equal.
      * \warning When using floating point scalar values you probably should rather use a
      *          fuzzy comparison such as isApprox()
      * \sa isApprox(), operator!= */
    template<typename OtherDerived>
    inline bool operator==(const MatrixBase<OtherDerived>& other) const
    { return cwiseEqual(other).all(); }

    /** \returns true if at least one pair of coefficients of \c *this and \a other are not exactly equal to each other.
      * \warning When using floating point scalar values you probably should rather use a
      *          fuzzy comparison such as isApprox()
      * \sa isApprox(), operator== */
    template<typename OtherDerived>
    inline bool operator!=(const MatrixBase<OtherDerived>& other) const
    { return cwiseNotEqual(other).any(); }

    NoAlias<Derived,Eigen::MatrixBase > noalias();

    inline const ForceAlignedAccess<Derived> forceAlignedAccess() const;
    inline ForceAlignedAccess<Derived> forceAlignedAccess();
    template<bool Enable> inline typename ei_makeconst<typename ei_meta_if<Enable,ForceAlignedAccess<Derived>,Derived&>::ret>::type forceAlignedAccessIf() const;
    template<bool Enable> inline typename ei_meta_if<Enable,ForceAlignedAccess<Derived>,Derived&>::ret forceAlignedAccessIf();

    Scalar trace() const;

/////////// Array module ///////////

    template<int p> RealScalar lpNorm() const;

    MatrixBase<Derived>& matrix() { return *this; }
    const MatrixBase<Derived>& matrix() const { return *this; }

    /** \returns an \link ArrayBase Array \endlink expression of this matrix
      * \sa ArrayBase::matrix() */
    ArrayWrapper<Derived> array() { return derived(); }
    const ArrayWrapper<Derived> array() const { return derived(); }

/////////// LU module ///////////

    const FullPivLU<PlainObject> fullPivLu() const;
    const PartialPivLU<PlainObject> partialPivLu() const;
    const PartialPivLU<PlainObject> lu() const;
    const ei_inverse_impl<Derived> inverse() const;
    template<typename ResultType>
    void computeInverseAndDetWithCheck(
      ResultType& inverse,
      typename ResultType::Scalar& determinant,
      bool& invertible,
      const RealScalar& absDeterminantThreshold = NumTraits<Scalar>::dummy_precision()
    ) const;
    template<typename ResultType>
    void computeInverseWithCheck(
      ResultType& inverse,
      bool& invertible,
      const RealScalar& absDeterminantThreshold = NumTraits<Scalar>::dummy_precision()
    ) const;
    Scalar determinant() const;

/////////// Cholesky module ///////////

    const LLT<PlainObject>  llt() const;
    const LDLT<PlainObject> ldlt() const;

/////////// QR module ///////////

    const HouseholderQR<PlainObject> householderQr() const;
    const ColPivHouseholderQR<PlainObject> colPivHouseholderQr() const;
    const FullPivHouseholderQR<PlainObject> fullPivHouseholderQr() const;

    EigenvaluesReturnType eigenvalues() const;
    RealScalar operatorNorm() const;

/////////// SVD module ///////////

/////////// Geometry module ///////////

    template<typename OtherDerived>
    PlainObject cross(const MatrixBase<OtherDerived>& other) const;
    template<typename OtherDerived>
    PlainObject cross3(const MatrixBase<OtherDerived>& other) const;
    PlainObject unitOrthogonal(void) const;
    Matrix<Scalar,3,1> eulerAngles(Index a0, Index a1, Index a2) const;
    ScalarMultipleReturnType operator*(const UniformScaling<Scalar>& s) const;
    enum {
      SizeMinusOne = SizeAtCompileTime==Dynamic ? Dynamic : SizeAtCompileTime-1
    };
    typedef Block<Derived,
                  ei_traits<Derived>::ColsAtCompileTime==1 ? SizeMinusOne : 1,
                  ei_traits<Derived>::ColsAtCompileTime==1 ? 1 : SizeMinusOne> StartMinusOne;
    typedef CwiseUnaryOp<ei_scalar_quotient1_op<typename ei_traits<Derived>::Scalar>,
                StartMinusOne > HNormalizedReturnType;

    HNormalizedReturnType hnormalized() const;

    // put this as separate enum value to work around possible GCC 4.3 bug (?)
    enum { HomogeneousReturnTypeDirection = ColsAtCompileTime==1?Vertical:Horizontal };
    typedef Homogeneous<Derived, HomogeneousReturnTypeDirection> HomogeneousReturnType;

    HomogeneousReturnType homogeneous() const;

////////// Householder module ///////////

    void makeHouseholderInPlace(Scalar& tau, RealScalar& beta);
    template<typename EssentialPart>
    void makeHouseholder(EssentialPart& essential,
                         Scalar& tau, RealScalar& beta) const;
    template<typename EssentialPart>
    void applyHouseholderOnTheLeft(const EssentialPart& essential,
                                   const Scalar& tau,
                                   Scalar* workspace);
    template<typename EssentialPart>
    void applyHouseholderOnTheRight(const EssentialPart& essential,
                                    const Scalar& tau,
                                    Scalar* workspace);

///////// Jacobi module /////////

    template<typename OtherScalar>
    void applyOnTheLeft(Index p, Index q, const PlanarRotation<OtherScalar>& j);
    template<typename OtherScalar>
    void applyOnTheRight(Index p, Index q, const PlanarRotation<OtherScalar>& j);

///////// MatrixFunctions module /////////

    typedef typename ei_stem_function<Scalar>::type StemFunction;
    const MatrixExponentialReturnValue<Derived> exp() const;
    const MatrixFunctionReturnValue<Derived> matrixFunction(StemFunction f) const;
    const MatrixFunctionReturnValue<Derived> cosh() const;
    const MatrixFunctionReturnValue<Derived> sinh() const;
    const MatrixFunctionReturnValue<Derived> cos() const;
    const MatrixFunctionReturnValue<Derived> sin() const;

#ifdef EIGEN2_SUPPORT
    template<typename ProductDerived, typename Lhs, typename Rhs>
    Derived& operator+=(const Flagged<ProductBase<ProductDerived, Lhs,Rhs>, 0,
                                      EvalBeforeAssigningBit>& other);

    template<typename ProductDerived, typename Lhs, typename Rhs>
    Derived& operator-=(const Flagged<ProductBase<ProductDerived, Lhs,Rhs>, 0,
                                      EvalBeforeAssigningBit>& other);

    /** \deprecated because .lazy() is deprecated
      * Overloaded for cache friendly product evaluation */
    template<typename OtherDerived>
    Derived& lazyAssign(const Flagged<OtherDerived, 0, EvalBeforeAssigningBit>& other)
    { return lazyAssign(other._expression()); }

    template<unsigned int Added>
    const Flagged<Derived, Added, 0> marked() const;
    const Flagged<Derived, 0, EvalBeforeAssigningBit> lazy() const;

    inline const Cwise<Derived> cwise() const;
    inline Cwise<Derived> cwise();

    VectorBlock<Derived> start(Index size);
    const VectorBlock<Derived> start(Index size) const;
    VectorBlock<Derived> end(Index size);
    const VectorBlock<Derived> end(Index size) const;
    template<int Size> VectorBlock<Derived,Size> start();
    template<int Size> const VectorBlock<Derived,Size> start() const;
    template<int Size> VectorBlock<Derived,Size> end();
    template<int Size> const VectorBlock<Derived,Size> end() const;

    Minor<Derived> minor(Index row, Index col);
    const Minor<Derived> minor(Index row, Index col) const;
#endif

  protected:
    MatrixBase() : Base() {}

  private:
    explicit MatrixBase(int);
    MatrixBase(int,int);
    template<typename OtherDerived> explicit MatrixBase(const MatrixBase<OtherDerived>&);
  protected:
    // mixing arrays and matrices is not legal
    template<typename OtherDerived> Derived& operator+=(const ArrayBase<OtherDerived>& array)
    {EIGEN_STATIC_ASSERT(sizeof(typename OtherDerived::Scalar)==-1,YOU_CANNOT_MIX_ARRAYS_AND_MATRICES);}
    // mixing arrays and matrices is not legal
    template<typename OtherDerived> Derived& operator-=(const ArrayBase<OtherDerived>& array)
    {EIGEN_STATIC_ASSERT(sizeof(typename OtherDerived::Scalar)==-1,YOU_CANNOT_MIX_ARRAYS_AND_MATRICES);}
};

#endif // EIGEN_MATRIXBASE_H
