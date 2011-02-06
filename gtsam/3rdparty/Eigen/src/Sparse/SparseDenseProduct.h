// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2010 Gael Guennebaud <gael.guennebaud@inria.fr>
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

#ifndef EIGEN_SPARSEDENSEPRODUCT_H
#define EIGEN_SPARSEDENSEPRODUCT_H

template<typename Lhs, typename Rhs, int InnerSize> struct SparseDenseProductReturnType
{
  typedef SparseTimeDenseProduct<Lhs,Rhs> Type;
};

template<typename Lhs, typename Rhs> struct SparseDenseProductReturnType<Lhs,Rhs,1>
{
  typedef SparseDenseOuterProduct<Lhs,Rhs,false> Type;
};

template<typename Lhs, typename Rhs, int InnerSize> struct DenseSparseProductReturnType
{
  typedef DenseTimeSparseProduct<Lhs,Rhs> Type;
};

template<typename Lhs, typename Rhs> struct DenseSparseProductReturnType<Lhs,Rhs,1>
{
  typedef SparseDenseOuterProduct<Rhs,Lhs,true> Type;
};

template<typename Lhs, typename Rhs, bool Tr>
struct ei_traits<SparseDenseOuterProduct<Lhs,Rhs,Tr> >
{
  typedef Sparse StorageKind;
  typedef typename ei_scalar_product_traits<typename ei_traits<Lhs>::Scalar,
                                            typename ei_traits<Rhs>::Scalar>::ReturnType Scalar;
  typedef typename Lhs::Index Index;
  typedef typename Lhs::Nested LhsNested;
  typedef typename Rhs::Nested RhsNested;
  typedef typename ei_cleantype<LhsNested>::type _LhsNested;
  typedef typename ei_cleantype<RhsNested>::type _RhsNested;

  enum {
    LhsCoeffReadCost = ei_traits<_LhsNested>::CoeffReadCost,
    RhsCoeffReadCost = ei_traits<_RhsNested>::CoeffReadCost,

    RowsAtCompileTime    = Tr ? int(ei_traits<Rhs>::RowsAtCompileTime)     : int(ei_traits<Lhs>::RowsAtCompileTime),
    ColsAtCompileTime    = Tr ? int(ei_traits<Lhs>::ColsAtCompileTime)     : int(ei_traits<Rhs>::ColsAtCompileTime),
    MaxRowsAtCompileTime = Tr ? int(ei_traits<Rhs>::MaxRowsAtCompileTime)  : int(ei_traits<Lhs>::MaxRowsAtCompileTime),
    MaxColsAtCompileTime = Tr ? int(ei_traits<Lhs>::MaxColsAtCompileTime)  : int(ei_traits<Rhs>::MaxColsAtCompileTime),

    Flags = Tr ? RowMajorBit : 0,

    CoeffReadCost = LhsCoeffReadCost + RhsCoeffReadCost + NumTraits<Scalar>::MulCost
  };
};

template<typename Lhs, typename Rhs, bool Tr>
class SparseDenseOuterProduct
 : public SparseMatrixBase<SparseDenseOuterProduct<Lhs,Rhs,Tr> >
{
  public:

    typedef SparseMatrixBase<SparseDenseOuterProduct> Base;
    EIGEN_DENSE_PUBLIC_INTERFACE(SparseDenseOuterProduct)
    typedef ei_traits<SparseDenseOuterProduct> Traits;

  private:

    typedef typename Traits::LhsNested LhsNested;
    typedef typename Traits::RhsNested RhsNested;
    typedef typename Traits::_LhsNested _LhsNested;
    typedef typename Traits::_RhsNested _RhsNested;

  public:

    class InnerIterator;

    EIGEN_STRONG_INLINE SparseDenseOuterProduct(const Lhs& lhs, const Rhs& rhs)
      : m_lhs(lhs), m_rhs(rhs)
    {
      EIGEN_STATIC_ASSERT(!Tr,YOU_MADE_A_PROGRAMMING_MISTAKE);
    }

    EIGEN_STRONG_INLINE SparseDenseOuterProduct(const Rhs& rhs, const Lhs& lhs)
      : m_lhs(lhs), m_rhs(rhs)
    {
      EIGEN_STATIC_ASSERT(Tr,YOU_MADE_A_PROGRAMMING_MISTAKE);
    }

    EIGEN_STRONG_INLINE Index rows() const { return Tr ? m_rhs.rows() : m_lhs.rows(); }
    EIGEN_STRONG_INLINE Index cols() const { return Tr ? m_lhs.cols() : m_rhs.cols(); }

    EIGEN_STRONG_INLINE const _LhsNested& lhs() const { return m_lhs; }
    EIGEN_STRONG_INLINE const _RhsNested& rhs() const { return m_rhs; }

  protected:
    LhsNested m_lhs;
    RhsNested m_rhs;
};

template<typename Lhs, typename Rhs, bool Transpose>
class SparseDenseOuterProduct<Lhs,Rhs,Transpose>::InnerIterator : public _LhsNested::InnerIterator
{
    typedef typename _LhsNested::InnerIterator Base;
  public:
    EIGEN_STRONG_INLINE InnerIterator(const SparseDenseOuterProduct& prod, Index outer)
      : Base(prod.lhs(), 0), m_outer(outer), m_factor(prod.rhs().coeff(outer))
    {
    }

    inline Index outer() const { return m_outer; }
    inline Index row() const { return Transpose ? Base::row() : m_outer; }
    inline Index col() const { return Transpose ? m_outer : Base::row(); }

    inline Scalar value() const { return Base::value() * m_factor; }

  protected:
    int m_outer;
    Scalar m_factor;
};

template<typename Lhs, typename Rhs>
struct ei_traits<SparseTimeDenseProduct<Lhs,Rhs> >
 : ei_traits<ProductBase<SparseTimeDenseProduct<Lhs,Rhs>, Lhs, Rhs> >
{
  typedef Dense StorageKind;
  typedef MatrixXpr XprKind;
};

template<typename Lhs, typename Rhs>
class SparseTimeDenseProduct
  : public ProductBase<SparseTimeDenseProduct<Lhs,Rhs>, Lhs, Rhs>
{
  public:
    EIGEN_PRODUCT_PUBLIC_INTERFACE(SparseTimeDenseProduct)

    SparseTimeDenseProduct(const Lhs& lhs, const Rhs& rhs) : Base(lhs,rhs)
    {}

    template<typename Dest> void scaleAndAddTo(Dest& dest, Scalar alpha) const
    {
      typedef typename ei_cleantype<Lhs>::type _Lhs;
      typedef typename ei_cleantype<Rhs>::type _Rhs;
      typedef typename _Lhs::InnerIterator LhsInnerIterator;
      enum { LhsIsRowMajor = (_Lhs::Flags&RowMajorBit)==RowMajorBit };
      for(Index j=0; j<m_lhs.outerSize(); ++j)
      {
        typename Rhs::Scalar rhs_j = alpha * m_rhs.coeff(j,0);
        Block<Dest,1,Dest::ColsAtCompileTime> dest_j(dest.row(LhsIsRowMajor ? j : 0));
        for(LhsInnerIterator it(m_lhs,j); it ;++it)
        {
          if(LhsIsRowMajor)                   dest_j += (alpha*it.value()) * m_rhs.row(it.index());
          else if(Rhs::ColsAtCompileTime==1)  dest.coeffRef(it.index()) += it.value() * rhs_j;
          else                                dest.row(it.index()) += (alpha*it.value()) * m_rhs.row(j);
        }
      }
    }

  private:
    SparseTimeDenseProduct& operator=(const SparseTimeDenseProduct&);
};


// dense = dense * sparse
template<typename Lhs, typename Rhs>
struct ei_traits<DenseTimeSparseProduct<Lhs,Rhs> >
 : ei_traits<ProductBase<DenseTimeSparseProduct<Lhs,Rhs>, Lhs, Rhs> >
{
  typedef Dense StorageKind;
};

template<typename Lhs, typename Rhs>
class DenseTimeSparseProduct
  : public ProductBase<DenseTimeSparseProduct<Lhs,Rhs>, Lhs, Rhs>
{
  public:
    EIGEN_PRODUCT_PUBLIC_INTERFACE(DenseTimeSparseProduct)

    DenseTimeSparseProduct(const Lhs& lhs, const Rhs& rhs) : Base(lhs,rhs)
    {}

    template<typename Dest> void scaleAndAddTo(Dest& dest, Scalar alpha) const
    {
      typedef typename ei_cleantype<Rhs>::type _Rhs;
      typedef typename _Rhs::InnerIterator RhsInnerIterator;
      enum { RhsIsRowMajor = (_Rhs::Flags&RowMajorBit)==RowMajorBit };
      for(Index j=0; j<m_rhs.outerSize(); ++j)
        for(RhsInnerIterator i(m_rhs,j); i; ++i)
          dest.col(RhsIsRowMajor ? i.index() : j) += (alpha*i.value()) * m_lhs.col(RhsIsRowMajor ? j : i.index());
    }

  private:
    DenseTimeSparseProduct& operator=(const DenseTimeSparseProduct&);
};

// sparse * dense
template<typename Derived>
template<typename OtherDerived>
inline const typename SparseDenseProductReturnType<Derived,OtherDerived>::Type
SparseMatrixBase<Derived>::operator*(const MatrixBase<OtherDerived> &other) const
{
  return typename SparseDenseProductReturnType<Derived,OtherDerived>::Type(derived(), other.derived());
}

#endif // EIGEN_SPARSEDENSEPRODUCT_H
