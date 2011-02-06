// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009 Gael Guennebaud <gael.guennebaud@inria.fr>
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

#ifndef EIGEN_SPARSE_SELFADJOINTVIEW_H
#define EIGEN_SPARSE_SELFADJOINTVIEW_H

/** \class SparseSelfAdjointView
  *
  *
  * \brief Pseudo expression to manipulate a triangular sparse matrix as a selfadjoint matrix.
  *
  * \param MatrixType the type of the dense matrix storing the coefficients
  * \param UpLo can be either \c Lower or \c Upper
  *
  * This class is an expression of a sefladjoint matrix from a triangular part of a matrix
  * with given dense storage of the coefficients. It is the return type of MatrixBase::selfadjointView()
  * and most of the time this is the only way that it is used.
  *
  * \sa SparseMatrixBase::selfAdjointView()
  */
template<typename Lhs, typename Rhs, int UpLo>
class SparseSelfAdjointTimeDenseProduct;

template<typename Lhs, typename Rhs, int UpLo>
class DenseTimeSparseSelfAdjointProduct;

template<typename MatrixType, unsigned int UpLo> class SparseSelfAdjointView
{
  public:

    typedef typename MatrixType::Scalar Scalar;
    typedef typename MatrixType::Index Index;

    inline SparseSelfAdjointView(const MatrixType& matrix) : m_matrix(matrix)
    {
      ei_assert(ei_are_flags_consistent<UpLo>::ret);
      ei_assert(rows()==cols() && "SelfAdjointView is only for squared matrices");
    }

    inline Index rows() const { return m_matrix.rows(); }
    inline Index cols() const { return m_matrix.cols(); }

    /** \internal \returns a reference to the nested matrix */
    const MatrixType& matrix() const { return m_matrix; }

    /** Efficient sparse self-adjoint matrix times dense vector/matrix product */
    template<typename OtherDerived>
    SparseSelfAdjointTimeDenseProduct<MatrixType,OtherDerived,UpLo>
    operator*(const MatrixBase<OtherDerived>& rhs) const
    {
      return SparseSelfAdjointTimeDenseProduct<MatrixType,OtherDerived,UpLo>(m_matrix, rhs.derived());
    }

    /** Efficient dense vector/matrix times sparse self-adjoint matrix product */
    template<typename OtherDerived> friend
    DenseTimeSparseSelfAdjointProduct<OtherDerived,MatrixType,UpLo>
    operator*(const MatrixBase<OtherDerived>& lhs, const SparseSelfAdjointView& rhs)
    {
      return DenseTimeSparseSelfAdjointProduct<OtherDerived,MatrixType,UpLo>(lhs.derived(), rhs.m_matrix);
    }

    /** Perform a symmetric rank K update of the selfadjoint matrix \c *this:
      * \f$ this = this + \alpha ( u u^* ) \f$ where \a u is a vector or matrix.
      *
      * \returns a reference to \c *this
      *
      * Note that it is faster to set alpha=0 than initializing the matrix to zero
      * and then keep the default value alpha=1.
      *
      * To perform \f$ this = this + \alpha ( u^* u ) \f$ you can simply
      * call this function with u.adjoint().
      */
    template<typename DerivedU>
    SparseSelfAdjointView& rankUpdate(const MatrixBase<DerivedU>& u, Scalar alpha = Scalar(1));

    // const SparseLLT<PlainObject, UpLo> llt() const;
    // const SparseLDLT<PlainObject, UpLo> ldlt() const;

  protected:

    const typename MatrixType::Nested m_matrix;
};

/***************************************************************************
* Implementation of SparseMatrixBase methods
***************************************************************************/

template<typename Derived>
template<unsigned int UpLo>
const SparseSelfAdjointView<Derived, UpLo> SparseMatrixBase<Derived>::selfadjointView() const
{
  return derived();
}

template<typename Derived>
template<unsigned int UpLo>
SparseSelfAdjointView<Derived, UpLo> SparseMatrixBase<Derived>::selfadjointView()
{
  return derived();
}

/***************************************************************************
* Implementation of SparseSelfAdjointView methods
***************************************************************************/

template<typename MatrixType, unsigned int UpLo>
template<typename DerivedU>
SparseSelfAdjointView<MatrixType,UpLo>&
SparseSelfAdjointView<MatrixType,UpLo>::rankUpdate(const MatrixBase<DerivedU>& u, Scalar alpha)
{
  SparseMatrix<Scalar,MatrixType::Flags&RowMajorBit?RowMajor:ColMajor> tmp = u * u.adjoint();
  if(alpha==Scalar(0))
    m_matrix = tmp.template triangularView<UpLo>();
  else
    m_matrix += alpha * tmp.template triangularView<UpLo>();

  return this;
}

/***************************************************************************
* Implementation of sparse self-adjoint time dense matrix
***************************************************************************/

template<typename Lhs, typename Rhs, int UpLo>
struct ei_traits<SparseSelfAdjointTimeDenseProduct<Lhs,Rhs,UpLo> >
 : ei_traits<ProductBase<SparseSelfAdjointTimeDenseProduct<Lhs,Rhs,UpLo>, Lhs, Rhs> >
{
  typedef Dense StorageKind;
};

template<typename Lhs, typename Rhs, int UpLo>
class SparseSelfAdjointTimeDenseProduct
  : public ProductBase<SparseSelfAdjointTimeDenseProduct<Lhs,Rhs,UpLo>, Lhs, Rhs>
{
  public:
    EIGEN_PRODUCT_PUBLIC_INTERFACE(SparseSelfAdjointTimeDenseProduct)

    SparseSelfAdjointTimeDenseProduct(const Lhs& lhs, const Rhs& rhs) : Base(lhs,rhs)
    {}

    template<typename Dest> void scaleAndAddTo(Dest& dest, Scalar alpha) const
    {
      // TODO use alpha
      ei_assert(alpha==Scalar(1) && "alpha != 1 is not implemented yet, sorry");
      typedef typename ei_cleantype<Lhs>::type _Lhs;
      typedef typename ei_cleantype<Rhs>::type _Rhs;
      typedef typename _Lhs::InnerIterator LhsInnerIterator;
      enum {
        LhsIsRowMajor = (_Lhs::Flags&RowMajorBit)==RowMajorBit,
        ProcessFirstHalf =
                 ((UpLo&(Upper|Lower))==(Upper|Lower))
              || ( (UpLo&Upper) && !LhsIsRowMajor)
              || ( (UpLo&Lower) && LhsIsRowMajor),
        ProcessSecondHalf = !ProcessFirstHalf
      };
      for (Index j=0; j<m_lhs.outerSize(); ++j)
      {
        LhsInnerIterator i(m_lhs,j);
        if (ProcessSecondHalf && i && (i.index()==j))
        {
          dest.row(j) += i.value() * m_rhs.row(j);
          ++i;
        }
        Block<Dest,1,Dest::ColsAtCompileTime> dest_j(dest.row(LhsIsRowMajor ? j : 0));
        for(; (ProcessFirstHalf ? i && i.index() < j : i) ; ++i)
        {
          Index a = LhsIsRowMajor ? j : i.index();
          Index b = LhsIsRowMajor ? i.index() : j;
          typename Lhs::Scalar v = i.value();
          dest.row(a) += (v) * m_rhs.row(b);
          dest.row(b) += ei_conj(v) * m_rhs.row(a);
        }
        if (ProcessFirstHalf && i && (i.index()==j))
          dest.row(j) += i.value() * m_rhs.row(j);
      }
    }

  private:
    SparseSelfAdjointTimeDenseProduct& operator=(const SparseSelfAdjointTimeDenseProduct&);
};

template<typename Lhs, typename Rhs, int UpLo>
struct ei_traits<DenseTimeSparseSelfAdjointProduct<Lhs,Rhs,UpLo> >
 : ei_traits<ProductBase<DenseTimeSparseSelfAdjointProduct<Lhs,Rhs,UpLo>, Lhs, Rhs> >
{};

template<typename Lhs, typename Rhs, int UpLo>
class DenseTimeSparseSelfAdjointProduct
  : public ProductBase<DenseTimeSparseSelfAdjointProduct<Lhs,Rhs,UpLo>, Lhs, Rhs>
{
  public:
    EIGEN_PRODUCT_PUBLIC_INTERFACE(DenseTimeSparseSelfAdjointProduct)

    DenseTimeSparseSelfAdjointProduct(const Lhs& lhs, const Rhs& rhs) : Base(lhs,rhs)
    {}

    template<typename Dest> void scaleAndAddTo(Dest& /*dest*/, Scalar /*alpha*/) const
    {
      // TODO
    }

  private:
    DenseTimeSparseSelfAdjointProduct& operator=(const DenseTimeSparseSelfAdjointProduct&);
};
#endif // EIGEN_SPARSE_SELFADJOINTVIEW_H
