// // This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2006-2008 Benoit Jacob <jacob.benoit.1@gmail.com>
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

#ifndef EIGEN_XPRHELPER_H
#define EIGEN_XPRHELPER_H

// just a workaround because GCC seems to not really like empty structs
// FIXME: gcc 4.3 generates bad code when strict-aliasing is enabled
// so currently we simply disable this optimization for gcc 4.3
#if (defined __GNUG__) && !((__GNUC__==4) && (__GNUC_MINOR__==3))
  #define EIGEN_EMPTY_STRUCT_CTOR(X) \
    EIGEN_STRONG_INLINE X() {} \
    EIGEN_STRONG_INLINE X(const X& ) {}
#else
  #define EIGEN_EMPTY_STRUCT_CTOR(X)
#endif

//classes inheriting ei_no_assignment_operator don't generate a default operator=.
class ei_no_assignment_operator
{
  private:
    ei_no_assignment_operator& operator=(const ei_no_assignment_operator&);
};

typedef EIGEN_DEFAULT_DENSE_INDEX_TYPE DenseIndex;

/** \internal return the index type with the largest number of bits */
template<typename I1, typename I2>
struct ei_promote_index_type
{
  typedef typename ei_meta_if<(sizeof(I1)<sizeof(I2)), I2, I1>::ret type;
};

/** \internal If the template parameter Value is Dynamic, this class is just a wrapper around a T variable that
  * can be accessed using value() and setValue().
  * Otherwise, this class is an empty structure and value() just returns the template parameter Value.
  */
template<typename T, int Value> class ei_variable_if_dynamic
{
  public:
    EIGEN_EMPTY_STRUCT_CTOR(ei_variable_if_dynamic)
    explicit ei_variable_if_dynamic(T v) { EIGEN_ONLY_USED_FOR_DEBUG(v); ei_assert(v == T(Value)); }
    static T value() { return T(Value); }
    void setValue(T) {}
};

template<typename T> class ei_variable_if_dynamic<T, Dynamic>
{
    T m_value;
    ei_variable_if_dynamic() { ei_assert(false); }
  public:
    explicit ei_variable_if_dynamic(T value) : m_value(value) {}
    T value() const { return m_value; }
    void setValue(T value) { m_value = value; }
};

template<typename T> struct ei_functor_traits
{
  enum
  {
    Cost = 10,
    PacketAccess = false
  };
};

template<typename T> struct ei_packet_traits;

template<typename T> struct ei_unpacket_traits
{
  typedef T type;
  enum {size=1};
};

template<typename _Scalar, int _Rows, int _Cols,
         int _Options = AutoAlign |
                          ( (_Rows==1 && _Cols!=1) ? RowMajor
                          : (_Cols==1 && _Rows!=1) ? ColMajor
                          : EIGEN_DEFAULT_MATRIX_STORAGE_ORDER_OPTION ),
         int _MaxRows = _Rows,
         int _MaxCols = _Cols
> class ei_make_proper_matrix_type
{
    enum {
      IsColVector = _Cols==1 && _Rows!=1,
      IsRowVector = _Rows==1 && _Cols!=1,
      Options = IsColVector ? (_Options | ColMajor) & ~RowMajor
              : IsRowVector ? (_Options | RowMajor) & ~ColMajor
              : _Options
    };
  public:
    typedef Matrix<_Scalar, _Rows, _Cols, Options, _MaxRows, _MaxCols> type;
};

template<typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
class ei_compute_matrix_flags
{
    enum {
      row_major_bit = Options&RowMajor ? RowMajorBit : 0,
      is_dynamic_size_storage = MaxRows==Dynamic || MaxCols==Dynamic,

      aligned_bit =
      (
            ((Options&DontAlign)==0)
        &&  ei_packet_traits<Scalar>::Vectorizable
        && (
#if EIGEN_ALIGN_STATICALLY
             ((!is_dynamic_size_storage) && (((MaxCols*MaxRows) % ei_packet_traits<Scalar>::size) == 0))
#else
             0
#endif

          ||

#if EIGEN_ALIGN
             is_dynamic_size_storage
#else
             0
#endif

          )
      ) ? AlignedBit : 0,
      packet_access_bit = ei_packet_traits<Scalar>::Vectorizable && aligned_bit ? PacketAccessBit : 0
    };

  public:
    enum { ret = LinearAccessBit | LvalueBit | DirectAccessBit | NestByRefBit | packet_access_bit | row_major_bit | aligned_bit };
};

template<int _Rows, int _Cols> struct ei_size_at_compile_time
{
  enum { ret = (_Rows==Dynamic || _Cols==Dynamic) ? Dynamic : _Rows * _Cols };
};

/* ei_plain_matrix_type : the difference from ei_eval is that ei_plain_matrix_type is always a plain matrix type,
 * whereas ei_eval is a const reference in the case of a matrix
 */

template<typename T, typename StorageKind = typename ei_traits<T>::StorageKind> struct ei_plain_matrix_type;
template<typename T, typename BaseClassType> struct ei_plain_matrix_type_dense;
template<typename T> struct ei_plain_matrix_type<T,Dense>
{
  typedef typename ei_plain_matrix_type_dense<T,typename ei_traits<T>::XprKind>::type type;
};

template<typename T> struct ei_plain_matrix_type_dense<T,MatrixXpr>
{
  typedef Matrix<typename ei_traits<T>::Scalar,
                ei_traits<T>::RowsAtCompileTime,
                ei_traits<T>::ColsAtCompileTime,
                AutoAlign | (ei_traits<T>::Flags&RowMajorBit ? RowMajor : ColMajor),
                ei_traits<T>::MaxRowsAtCompileTime,
                ei_traits<T>::MaxColsAtCompileTime
          > type;
};

template<typename T> struct ei_plain_matrix_type_dense<T,ArrayXpr>
{
  typedef Array<typename ei_traits<T>::Scalar,
                ei_traits<T>::RowsAtCompileTime,
                ei_traits<T>::ColsAtCompileTime,
                AutoAlign | (ei_traits<T>::Flags&RowMajorBit ? RowMajor : ColMajor),
                ei_traits<T>::MaxRowsAtCompileTime,
                ei_traits<T>::MaxColsAtCompileTime
          > type;
};

/* ei_eval : the return type of eval(). For matrices, this is just a const reference
 * in order to avoid a useless copy
 */

template<typename T, typename StorageKind = typename ei_traits<T>::StorageKind> struct ei_eval;

template<typename T> struct ei_eval<T,Dense>
{
  typedef typename ei_plain_matrix_type<T>::type type;
//   typedef typename T::PlainObject type;
//   typedef T::Matrix<typename ei_traits<T>::Scalar,
//                 ei_traits<T>::RowsAtCompileTime,
//                 ei_traits<T>::ColsAtCompileTime,
//                 AutoAlign | (ei_traits<T>::Flags&RowMajorBit ? RowMajor : ColMajor),
//                 ei_traits<T>::MaxRowsAtCompileTime,
//                 ei_traits<T>::MaxColsAtCompileTime
//           > type;
};

// for matrices, no need to evaluate, just use a const reference to avoid a useless copy
template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
struct ei_eval<Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>, Dense>
{
  typedef const Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& type;
};

template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
struct ei_eval<Array<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>, Dense>
{
  typedef const Array<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& type;
};



/* ei_plain_matrix_type_column_major : same as ei_plain_matrix_type but guaranteed to be column-major
 */
template<typename T> struct ei_plain_matrix_type_column_major
{
  enum { Rows = ei_traits<T>::RowsAtCompileTime,
         Cols = ei_traits<T>::ColsAtCompileTime,
         MaxRows = ei_traits<T>::MaxRowsAtCompileTime,
         MaxCols = ei_traits<T>::MaxColsAtCompileTime
  };
  typedef Matrix<typename ei_traits<T>::Scalar,
                Rows,
                Cols,
                (MaxRows==1&&MaxCols!=1) ? RowMajor : ColMajor,
                MaxRows,
                MaxCols
          > type;
};

/* ei_plain_matrix_type_row_major : same as ei_plain_matrix_type but guaranteed to be row-major
 */
template<typename T> struct ei_plain_matrix_type_row_major
{
  enum { Rows = ei_traits<T>::RowsAtCompileTime,
         Cols = ei_traits<T>::ColsAtCompileTime,
         MaxRows = ei_traits<T>::MaxRowsAtCompileTime,
         MaxCols = ei_traits<T>::MaxColsAtCompileTime
  };
  typedef Matrix<typename ei_traits<T>::Scalar,
                Rows,
                Cols,
                (MaxCols==1&&MaxRows!=1) ? RowMajor : ColMajor,
                MaxRows,
                MaxCols
          > type;
};

// we should be able to get rid of this one too
template<typename T> struct ei_must_nest_by_value { enum { ret = false }; };

template<class T>
struct ei_is_reference
{
  enum { ret = false };
};

template<class T>
struct ei_is_reference<T&>
{
  enum { ret = true };
};

/**
* \internal The reference selector for template expressions. The idea is that we don't
* need to use references for expressions since they are light weight proxy
* objects which should generate no copying overhead.
**/
template <typename T>
struct ei_ref_selector
{
  typedef typename ei_meta_if<
    bool(ei_traits<T>::Flags & NestByRefBit),
    T const&,
    T
  >::ret type;
};

/** \internal Determines how a given expression should be nested into another one.
  * For example, when you do a * (b+c), Eigen will determine how the expression b+c should be
  * nested into the bigger product expression. The choice is between nesting the expression b+c as-is, or
  * evaluating that expression b+c into a temporary variable d, and nest d so that the resulting expression is
  * a*d. Evaluating can be beneficial for example if every coefficient access in the resulting expression causes
  * many coefficient accesses in the nested expressions -- as is the case with matrix product for example.
  *
  * \param T the type of the expression being nested
  * \param n the number of coefficient accesses in the nested expression for each coefficient access in the bigger expression.
  *
  * Example. Suppose that a, b, and c are of type Matrix3d. The user forms the expression a*(b+c).
  * b+c is an expression "sum of matrices", which we will denote by S. In order to determine how to nest it,
  * the Product expression uses: ei_nested<S, 3>::ret, which turns out to be Matrix3d because the internal logic of
  * ei_nested determined that in this case it was better to evaluate the expression b+c into a temporary. On the other hand,
  * since a is of type Matrix3d, the Product expression nests it as ei_nested<Matrix3d, 3>::ret, which turns out to be
  * const Matrix3d&, because the internal logic of ei_nested determined that since a was already a matrix, there was no point
  * in copying it into another matrix.
  */
template<typename T, int n=1, typename PlainObject = typename ei_eval<T>::type> struct ei_nested
{
 // this is a direct port of the logic used when Dynamic was 33331, to make an atomic commit.
  enum {
    _ScalarReadCost = NumTraits<typename ei_traits<T>::Scalar>::ReadCost,
    ScalarReadCost = _ScalarReadCost == Dynamic ? 33331 : int(_ScalarReadCost),
    _CoeffReadCost = int(ei_traits<T>::CoeffReadCost),
    CoeffReadCost = _CoeffReadCost == Dynamic ? 33331 : int(_CoeffReadCost),
    N = n == Dynamic ? 33331 : n,
    CostEval   = (N+1) * int(ScalarReadCost),
    CostNoEval = (N-1) * int(CoeffReadCost)
  };

  typedef typename ei_meta_if<
    ( int(ei_traits<T>::Flags) & EvalBeforeNestingBit ) ||
    ( int(CostEval) <= int(CostNoEval) ),
      PlainObject,
      typename ei_ref_selector<T>::type
  >::ret type;

/* this is what the above logic should be updated to look like:
  enum {
    ScalarReadCost = NumTraits<typename ei_traits<T>::Scalar>::ReadCost,
    CoeffReadCost = ei_traits<T>::CoeffReadCost,
    CostEval   = n == Dynamic || ScalarReadCost == Dynamic ? int(Dynamic) : (n+1) * int(ScalarReadCost),
    CostNoEval = n == Dynamic || (CoeffReadCost == Dynamic && n>1) ? int(Dynamic) : (n-1) * int(CoeffReadCost)
  };

  typedef typename ei_meta_if<
    ( int(ei_traits<T>::Flags) & EvalBeforeNestingBit ) ||
      (  int(CostNoEval) == Dynamic ? true
       : int(CostEval) == Dynamic   ? false
       : int(CostEval) <= int(CostNoEval) ),
      PlainObject,
      typename ei_ref_selector<T>::type
  >::ret type;
*/
};

template<unsigned int Flags> struct ei_are_flags_consistent
{
  enum { ret = EIGEN_IMPLIES(bool(Flags&DirectAccessBit), bool(Flags&LvalueBit)) };
};

template<typename Derived, typename XprKind = typename ei_traits<Derived>::XprKind>
struct ei_dense_xpr_base
{
  /* ei_dense_xpr_base should only ever be used on dense expressions, thus falling either into the MatrixXpr or into the ArrayXpr cases */
};

template<typename Derived>
struct ei_dense_xpr_base<Derived, MatrixXpr>
{
  typedef MatrixBase<Derived> type;
};

template<typename Derived>
struct ei_dense_xpr_base<Derived, ArrayXpr>
{
  typedef ArrayBase<Derived> type;
};

/** \internal Helper base class to add a scalar multiple operator
  * overloads for complex types */
template<typename Derived,typename Scalar,typename OtherScalar,
         bool EnableIt = !ei_is_same_type<Scalar,OtherScalar>::ret >
struct ei_special_scalar_op_base : public DenseCoeffsBase<Derived>
{
  // dummy operator* so that the
  // "using ei_special_scalar_op_base::operator*" compiles
  void operator*() const;
};

template<typename Derived,typename Scalar,typename OtherScalar>
struct ei_special_scalar_op_base<Derived,Scalar,OtherScalar,true>  : public DenseCoeffsBase<Derived>
{
  const CwiseUnaryOp<ei_scalar_multiple2_op<Scalar,OtherScalar>, Derived>
  operator*(const OtherScalar& scalar) const
  {
    return CwiseUnaryOp<ei_scalar_multiple2_op<Scalar,OtherScalar>, Derived>
      (*static_cast<const Derived*>(this), ei_scalar_multiple2_op<Scalar,OtherScalar>(scalar));
  }

  inline friend const CwiseUnaryOp<ei_scalar_multiple2_op<Scalar,OtherScalar>, Derived>
  operator*(const OtherScalar& scalar, const Derived& matrix)
  { return static_cast<const ei_special_scalar_op_base&>(matrix).operator*(scalar); }
};

template<typename ExpressionType> struct HNormalizedReturnType {

  enum {
    SizeAtCompileTime = ExpressionType::SizeAtCompileTime,
    SizeMinusOne = SizeAtCompileTime==Dynamic ? Dynamic : SizeAtCompileTime-1
  };
  typedef Block<ExpressionType,
                ei_traits<ExpressionType>::ColsAtCompileTime==1 ? SizeMinusOne : 1,
                ei_traits<ExpressionType>::ColsAtCompileTime==1 ? 1 : SizeMinusOne> StartMinusOne;
  typedef CwiseUnaryOp<ei_scalar_quotient1_op<typename ei_traits<ExpressionType>::Scalar>,
              StartMinusOne > Type;
};

template<typename XprType, typename CastType> struct ei_cast_return_type
{
  typedef typename XprType::Scalar CurrentScalarType;
  typedef typename ei_cleantype<CastType>::type _CastType;
  typedef typename _CastType::Scalar NewScalarType;
  typedef typename ei_meta_if<ei_is_same_type<CurrentScalarType,NewScalarType>::ret,
                              const XprType&,CastType>::ret type;
};

template <typename A, typename B> struct ei_promote_storage_type;

template <typename A> struct ei_promote_storage_type<A,A>
{
  typedef A ret;
};

/** \internal gives the plain matrix type to store a row/column/diagonal of a matrix type.
  * \param Scalar optional parameter allowing to pass a different scalar type than the one of the MatrixType.
  */
template<typename MatrixType, typename Scalar = typename MatrixType::Scalar>
struct ei_plain_row_type
{
  typedef Matrix<Scalar, 1, MatrixType::ColsAtCompileTime,
                 MatrixType::PlainObject::Options | RowMajor, 1, MatrixType::MaxColsAtCompileTime> type;
};

template<typename MatrixType, typename Scalar = typename MatrixType::Scalar>
struct ei_plain_col_type
{
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, 1,
                 MatrixType::PlainObject::Options & ~RowMajor, MatrixType::MaxRowsAtCompileTime, 1> type;
};

template<typename MatrixType, typename Scalar = typename MatrixType::Scalar>
struct ei_plain_diag_type
{
  enum { diag_size = EIGEN_SIZE_MIN_PREFER_DYNAMIC(MatrixType::RowsAtCompileTime, MatrixType::ColsAtCompileTime),
         max_diag_size = EIGEN_SIZE_MIN_PREFER_FIXED(MatrixType::MaxRowsAtCompileTime, MatrixType::MaxColsAtCompileTime)
  };
  typedef Matrix<Scalar, diag_size, 1, MatrixType::PlainObject::Options & ~RowMajor, max_diag_size, 1> type;
};

#endif // EIGEN_XPRHELPER_H
