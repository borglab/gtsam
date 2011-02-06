// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009-2010 Gael Guennebaud <gael.guennebaud@inria.fr>
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

#ifndef EIGEN_HOMOGENEOUS_H
#define EIGEN_HOMOGENEOUS_H

/** \geometry_module \ingroup Geometry_Module
  *
  * \class Homogeneous
  *
  * \brief Expression of one (or a set of) homogeneous vector(s)
  *
  * \param MatrixType the type of the object in which we are making homogeneous
  *
  * This class represents an expression of one (or a set of) homogeneous vector(s).
  * It is the return type of MatrixBase::homogeneous() and most of the time
  * this is the only way it is used.
  *
  * \sa MatrixBase::homogeneous()
  */
template<typename MatrixType,int Direction>
struct ei_traits<Homogeneous<MatrixType,Direction> >
 : ei_traits<MatrixType>
{
  typedef typename ei_traits<MatrixType>::StorageKind StorageKind;
  typedef typename ei_nested<MatrixType>::type MatrixTypeNested;
  typedef typename ei_unref<MatrixTypeNested>::type _MatrixTypeNested;
  enum {
    RowsPlusOne = (MatrixType::RowsAtCompileTime != Dynamic) ?
                  int(MatrixType::RowsAtCompileTime) + 1 : Dynamic,
    ColsPlusOne = (MatrixType::ColsAtCompileTime != Dynamic) ?
                  int(MatrixType::ColsAtCompileTime) + 1 : Dynamic,
    RowsAtCompileTime = Direction==Vertical  ?  RowsPlusOne : MatrixType::RowsAtCompileTime,
    ColsAtCompileTime = Direction==Horizontal ? ColsPlusOne : MatrixType::ColsAtCompileTime,
    MaxRowsAtCompileTime = RowsAtCompileTime,
    MaxColsAtCompileTime = ColsAtCompileTime,
    TmpFlags = _MatrixTypeNested::Flags & HereditaryBits,
    Flags = ColsAtCompileTime==1 ? (TmpFlags & ~RowMajorBit)
          : RowsAtCompileTime==1 ? (TmpFlags | RowMajorBit)
          : TmpFlags,
    CoeffReadCost = _MatrixTypeNested::CoeffReadCost
  };
};

template<typename MatrixType,typename Lhs> struct ei_homogeneous_left_product_impl;
template<typename MatrixType,typename Rhs> struct ei_homogeneous_right_product_impl;

template<typename MatrixType,int _Direction> class Homogeneous
  : public MatrixBase<Homogeneous<MatrixType,_Direction> >
{
  public:

    enum { Direction = _Direction };

    typedef MatrixBase<Homogeneous> Base;
    EIGEN_DENSE_PUBLIC_INTERFACE(Homogeneous)

    inline Homogeneous(const MatrixType& matrix)
      : m_matrix(matrix)
    {}

    inline Index rows() const { return m_matrix.rows() + (int(Direction)==Vertical   ? 1 : 0); }
    inline Index cols() const { return m_matrix.cols() + (int(Direction)==Horizontal ? 1 : 0); }

    inline Scalar coeff(Index row, Index col) const
    {
      if(  (int(Direction)==Vertical   && row==m_matrix.rows())
        || (int(Direction)==Horizontal && col==m_matrix.cols()))
        return 1;
      return m_matrix.coeff(row, col);
    }

    template<typename Rhs>
    inline const ei_homogeneous_right_product_impl<Homogeneous,Rhs>
    operator* (const MatrixBase<Rhs>& rhs) const
    {
      ei_assert(int(Direction)==Horizontal);
      return ei_homogeneous_right_product_impl<Homogeneous,Rhs>(m_matrix,rhs.derived());
    }

    template<typename Lhs> friend
    inline const ei_homogeneous_left_product_impl<Homogeneous,Lhs>
    operator* (const MatrixBase<Lhs>& lhs, const Homogeneous& rhs)
    {
      ei_assert(int(Direction)==Vertical);
      return ei_homogeneous_left_product_impl<Homogeneous,Lhs>(lhs.derived(),rhs.m_matrix);
    }

    template<typename Scalar, int Dim, int Mode> friend
    inline const ei_homogeneous_left_product_impl<Homogeneous,
      typename Transform<Scalar,Dim,Mode>::AffinePartNested>
    operator* (const Transform<Scalar,Dim,Mode>& tr, const Homogeneous& rhs)
    {
      ei_assert(int(Direction)==Vertical);
      return ei_homogeneous_left_product_impl<Homogeneous,typename Transform<Scalar,Dim,Mode>::AffinePartNested >
        (tr.affine(),rhs.m_matrix);
    }

    template<typename Scalar, int Dim> friend
    inline const ei_homogeneous_left_product_impl<Homogeneous,
      typename Transform<Scalar,Dim,Projective>::MatrixType>
    operator* (const Transform<Scalar,Dim,Projective>& tr, const Homogeneous& rhs)
    {
      ei_assert(int(Direction)==Vertical);
      return ei_homogeneous_left_product_impl<Homogeneous,typename Transform<Scalar,Dim,Projective>::MatrixType>
        (tr.matrix(),rhs.m_matrix);
    }

  protected:
    const typename MatrixType::Nested m_matrix;
};

/** \geometry_module
  *
  * \return an expression of the equivalent homogeneous vector
  *
  * \only_for_vectors
  *
  * Example: \include MatrixBase_homogeneous.cpp
  * Output: \verbinclude MatrixBase_homogeneous.out
  *
  * \sa class Homogeneous
  */
template<typename Derived>
inline typename MatrixBase<Derived>::HomogeneousReturnType
MatrixBase<Derived>::homogeneous() const
{
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
  return derived();
}

/** \geometry_module
  *
  * \returns a matrix expression of homogeneous column (or row) vectors
  *
  * Example: \include VectorwiseOp_homogeneous.cpp
  * Output: \verbinclude VectorwiseOp_homogeneous.out
  *
  * \sa MatrixBase::homogeneous() */
template<typename ExpressionType, int Direction>
inline Homogeneous<ExpressionType,Direction>
VectorwiseOp<ExpressionType,Direction>::homogeneous() const
{
  return _expression();
}

/** \geometry_module
  *
  * \returns an expression of the homogeneous normalized vector of \c *this
  *
  * Example: \include MatrixBase_hnormalized.cpp
  * Output: \verbinclude MatrixBase_hnormalized.out
  *
  * \sa VectorwiseOp::hnormalized() */
template<typename Derived>
inline typename MatrixBase<Derived>::HNormalizedReturnType
MatrixBase<Derived>::hnormalized() const
{
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
  return StartMinusOne(derived(),0,0,
    ColsAtCompileTime==1?size()-1:1,
    ColsAtCompileTime==1?1:size()-1) / coeff(size()-1);
}

/** \geometry_module
  *
  * \returns an expression of the homogeneous normalized vector of \c *this
  *
  * Example: \include DirectionWise_hnormalized.cpp
  * Output: \verbinclude DirectionWise_hnormalized.out
  *
  * \sa MatrixBase::hnormalized() */
template<typename ExpressionType, int Direction>
inline typename VectorwiseOp<ExpressionType,Direction>::HNormalizedReturnType
VectorwiseOp<ExpressionType,Direction>::hnormalized() const
{
  return HNormalized_Block(_expression(),0,0,
      Direction==Vertical   ? _expression().rows()-1 : _expression().rows(),
      Direction==Horizontal ? _expression().cols()-1 : _expression().cols()).cwiseQuotient(
      Replicate<HNormalized_Factors,
                Direction==Vertical   ? HNormalized_SizeMinusOne : 1,
                Direction==Horizontal ? HNormalized_SizeMinusOne : 1>
        (HNormalized_Factors(_expression(),
          Direction==Vertical    ? _expression().rows()-1:0,
          Direction==Horizontal  ? _expression().cols()-1:0,
          Direction==Vertical    ? 1 : _expression().rows(),
          Direction==Horizontal  ? 1 : _expression().cols()),
         Direction==Vertical   ? _expression().rows()-1 : 1,
         Direction==Horizontal ? _expression().cols()-1 : 1));
}

template<typename MatrixType,typename Lhs>
struct ei_traits<ei_homogeneous_left_product_impl<Homogeneous<MatrixType,Vertical>,Lhs> >
{
  typedef typename ei_make_proper_matrix_type<
                 typename ei_traits<MatrixType>::Scalar,
                 Lhs::RowsAtCompileTime,
                 MatrixType::ColsAtCompileTime,
                 MatrixType::PlainObject::Options,
                 Lhs::MaxRowsAtCompileTime,
                 MatrixType::MaxColsAtCompileTime>::type ReturnType;
};

template<typename MatrixType,typename Lhs>
struct ei_homogeneous_left_product_impl<Homogeneous<MatrixType,Vertical>,Lhs>
  : public ReturnByValue<ei_homogeneous_left_product_impl<Homogeneous<MatrixType,Vertical>,Lhs> >
{
  typedef typename ei_cleantype<typename Lhs::Nested>::type LhsNested;
  typedef typename MatrixType::Index Index;
  ei_homogeneous_left_product_impl(const Lhs& lhs, const MatrixType& rhs)
    : m_lhs(lhs), m_rhs(rhs)
  {}

  inline Index rows() const { return m_lhs.rows(); }
  inline Index cols() const { return m_rhs.cols(); }

  template<typename Dest> void evalTo(Dest& dst) const
  {
    // FIXME investigate how to allow lazy evaluation of this product when possible
    dst = Block<LhsNested,
              LhsNested::RowsAtCompileTime,
              LhsNested::ColsAtCompileTime==Dynamic?Dynamic:LhsNested::ColsAtCompileTime-1>
            (m_lhs,0,0,m_lhs.rows(),m_lhs.cols()-1) * m_rhs;
    dst += m_lhs.col(m_lhs.cols()-1).rowwise()
            .template replicate<MatrixType::ColsAtCompileTime>(m_rhs.cols());
  }

  const typename Lhs::Nested m_lhs;
  const typename MatrixType::Nested m_rhs;
};

template<typename MatrixType,typename Rhs>
struct ei_traits<ei_homogeneous_right_product_impl<Homogeneous<MatrixType,Horizontal>,Rhs> >
{
  typedef typename ei_make_proper_matrix_type<typename ei_traits<MatrixType>::Scalar,
                 MatrixType::RowsAtCompileTime,
                 Rhs::ColsAtCompileTime,
                 MatrixType::PlainObject::Options,
                 MatrixType::MaxRowsAtCompileTime,
                 Rhs::MaxColsAtCompileTime>::type ReturnType;
};

template<typename MatrixType,typename Rhs>
struct ei_homogeneous_right_product_impl<Homogeneous<MatrixType,Horizontal>,Rhs>
  : public ReturnByValue<ei_homogeneous_right_product_impl<Homogeneous<MatrixType,Horizontal>,Rhs> >
{
  typedef typename ei_cleantype<typename Rhs::Nested>::type RhsNested;
  typedef typename MatrixType::Index Index;
  ei_homogeneous_right_product_impl(const MatrixType& lhs, const Rhs& rhs)
    : m_lhs(lhs), m_rhs(rhs)
  {}

  inline Index rows() const { return m_lhs.rows(); }
  inline Index cols() const { return m_rhs.cols(); }

  template<typename Dest> void evalTo(Dest& dst) const
  {
    // FIXME investigate how to allow lazy evaluation of this product when possible
    dst = m_lhs * Block<RhsNested,
                        RhsNested::RowsAtCompileTime==Dynamic?Dynamic:RhsNested::RowsAtCompileTime-1,
                        RhsNested::ColsAtCompileTime>
            (m_rhs,0,0,m_rhs.rows()-1,m_rhs.cols());
    dst += m_rhs.row(m_rhs.rows()-1).colwise()
            .template replicate<MatrixType::RowsAtCompileTime>(m_lhs.rows());
  }

  const typename MatrixType::Nested m_lhs;
  const typename Rhs::Nested m_rhs;
};

#endif // EIGEN_HOMOGENEOUS_H
