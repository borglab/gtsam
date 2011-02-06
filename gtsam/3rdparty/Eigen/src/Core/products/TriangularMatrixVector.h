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

#ifndef EIGEN_TRIANGULARMATRIXVECTOR_H
#define EIGEN_TRIANGULARMATRIXVECTOR_H

template<bool LhsIsTriangular, typename Lhs, typename Rhs, typename Result,
         int Mode, bool ConjLhs, bool ConjRhs, int StorageOrder>
struct ei_product_triangular_vector_selector;

template<typename Lhs, typename Rhs, typename Result, int Mode, bool ConjLhs, bool ConjRhs, int StorageOrder>
struct ei_product_triangular_vector_selector<false,Lhs,Rhs,Result,Mode,ConjLhs,ConjRhs,StorageOrder>
{
  static EIGEN_DONT_INLINE  void run(const Lhs& lhs, const Rhs& rhs, Result& res, typename ei_traits<Lhs>::Scalar alpha)
  {
    typedef Transpose<Rhs>    TrRhs;  TrRhs trRhs(rhs);
    typedef Transpose<Lhs>    TrLhs;  TrLhs trLhs(lhs);
    typedef Transpose<Result> TrRes;  TrRes trRes(res);
    ei_product_triangular_vector_selector<true,TrRhs,TrLhs,TrRes,
      (Mode & UnitDiag) | (Mode & Lower) ? Upper : Lower, ConjRhs, ConjLhs, StorageOrder==RowMajor ? ColMajor : RowMajor>
      ::run(trRhs,trLhs,trRes,alpha);
  }
};

template<typename Lhs, typename Rhs, typename Result, int Mode, bool ConjLhs, bool ConjRhs>
struct ei_product_triangular_vector_selector<true,Lhs,Rhs,Result,Mode,ConjLhs,ConjRhs,ColMajor>
{
  typedef typename Rhs::Scalar Scalar;
  typedef typename Rhs::Index Index;
  enum {
    IsLower = ((Mode&Lower)==Lower),
    HasUnitDiag = (Mode & UnitDiag)==UnitDiag
  };
  static EIGEN_DONT_INLINE  void run(const Lhs& lhs, const Rhs& rhs, Result& res, typename ei_traits<Lhs>::Scalar alpha)
  {
    static const Index PanelWidth = EIGEN_TUNE_TRIANGULAR_PANEL_WIDTH;
    typename ei_conj_expr_if<ConjLhs,Lhs>::ret cjLhs(lhs);
    typename ei_conj_expr_if<ConjRhs,Rhs>::ret cjRhs(rhs);

    Index size = lhs.cols();
    for (Index pi=0; pi<size; pi+=PanelWidth)
    {
      Index actualPanelWidth = std::min(PanelWidth, size-pi);
      for (Index k=0; k<actualPanelWidth; ++k)
      {
        Index i = pi + k;
        Index s = IsLower ? (HasUnitDiag ? i+1 : i ) : pi;
        Index r = IsLower ? actualPanelWidth-k : k+1;
        if ((!HasUnitDiag) || (--r)>0)
          res.segment(s,r) += (alpha * cjRhs.coeff(i)) * cjLhs.col(i).segment(s,r);
        if (HasUnitDiag)
          res.coeffRef(i) += alpha * cjRhs.coeff(i);
      }
      Index r = IsLower ? size - pi - actualPanelWidth : pi;
      if (r>0)
      {
        Index s = IsLower ? pi+actualPanelWidth : 0;
        ei_general_matrix_vector_product<Index,Scalar,ColMajor,ConjLhs,Scalar,ConjRhs>::run(
            r, actualPanelWidth,
            &(lhs.const_cast_derived().coeffRef(s,pi)), lhs.outerStride(),
            &rhs.coeff(pi), rhs.innerStride(),
            &res.coeffRef(s), res.innerStride(), alpha);
      }
    }
  }
};

template<typename Lhs, typename Rhs, typename Result, int Mode, bool ConjLhs, bool ConjRhs>
struct ei_product_triangular_vector_selector<true,Lhs,Rhs,Result,Mode,ConjLhs,ConjRhs,RowMajor>
{
  typedef typename Rhs::Scalar Scalar;
  typedef typename Rhs::Index Index;
  enum {
    IsLower = ((Mode&Lower)==Lower),
    HasUnitDiag = (Mode & UnitDiag)==UnitDiag
  };
  static void run(const Lhs& lhs, const Rhs& rhs, Result& res, typename ei_traits<Lhs>::Scalar alpha)
  {
    static const Index PanelWidth = EIGEN_TUNE_TRIANGULAR_PANEL_WIDTH;
    typename ei_conj_expr_if<ConjLhs,Lhs>::ret cjLhs(lhs);
    typename ei_conj_expr_if<ConjRhs,Rhs>::ret cjRhs(rhs);
    Index size = lhs.cols();
    for (Index pi=0; pi<size; pi+=PanelWidth)
    {
      Index actualPanelWidth = std::min(PanelWidth, size-pi);
      for (Index k=0; k<actualPanelWidth; ++k)
      {
        Index i = pi + k;
        Index s = IsLower ? pi  : (HasUnitDiag ? i+1 : i);
        Index r = IsLower ? k+1 : actualPanelWidth-k;
        if ((!HasUnitDiag) || (--r)>0)
          res.coeffRef(i) += alpha * (cjLhs.row(i).segment(s,r).cwiseProduct(cjRhs.segment(s,r).transpose())).sum();
        if (HasUnitDiag)
          res.coeffRef(i) += alpha * cjRhs.coeff(i);
      }
      Index r = IsLower ? pi : size - pi - actualPanelWidth;
      if (r>0)
      {
        Index s = IsLower ? 0 : pi + actualPanelWidth;
        ei_general_matrix_vector_product<Index,Scalar,RowMajor,ConjLhs,Scalar,ConjRhs>::run(
            actualPanelWidth, r,
            &(lhs.const_cast_derived().coeffRef(pi,s)), lhs.outerStride(),
            &(rhs.const_cast_derived().coeffRef(s)), 1,
            &res.coeffRef(pi,0), res.innerStride(), alpha);
      }
    }
  }
};

/***************************************************************************
* Wrapper to ei_product_triangular_vector
***************************************************************************/

template<int Mode, bool LhsIsTriangular, typename Lhs, typename Rhs>
struct ei_traits<TriangularProduct<Mode,LhsIsTriangular,Lhs,false,Rhs,true> >
 : ei_traits<ProductBase<TriangularProduct<Mode,LhsIsTriangular,Lhs,false,Rhs,true>, Lhs, Rhs> >
{};

template<int Mode, bool LhsIsTriangular, typename Lhs, typename Rhs>
struct ei_traits<TriangularProduct<Mode,LhsIsTriangular,Lhs,true,Rhs,false> >
 : ei_traits<ProductBase<TriangularProduct<Mode,LhsIsTriangular,Lhs,true,Rhs,false>, Lhs, Rhs> >
{};

template<int Mode, typename Lhs, typename Rhs>
struct TriangularProduct<Mode,true,Lhs,false,Rhs,true>
  : public ProductBase<TriangularProduct<Mode,true,Lhs,false,Rhs,true>, Lhs, Rhs >
{
  EIGEN_PRODUCT_PUBLIC_INTERFACE(TriangularProduct)

  TriangularProduct(const Lhs& lhs, const Rhs& rhs) : Base(lhs,rhs) {}

  template<typename Dest> void scaleAndAddTo(Dest& dst, Scalar alpha) const
  {
    ei_assert(dst.rows()==m_lhs.rows() && dst.cols()==m_rhs.cols());

    const ActualLhsType lhs = LhsBlasTraits::extract(m_lhs);
    const ActualRhsType rhs = RhsBlasTraits::extract(m_rhs);

    Scalar actualAlpha = alpha * LhsBlasTraits::extractScalarFactor(m_lhs)
                               * RhsBlasTraits::extractScalarFactor(m_rhs);

    ei_product_triangular_vector_selector
      <true,_ActualLhsType,_ActualRhsType,Dest,
       Mode,
       LhsBlasTraits::NeedToConjugate,
       RhsBlasTraits::NeedToConjugate,
       (int(ei_traits<Lhs>::Flags)&RowMajorBit) ? RowMajor : ColMajor>
      ::run(lhs,rhs,dst,actualAlpha);
  }
};

template<int Mode, typename Lhs, typename Rhs>
struct TriangularProduct<Mode,false,Lhs,true,Rhs,false>
  : public ProductBase<TriangularProduct<Mode,false,Lhs,true,Rhs,false>, Lhs, Rhs >
{
  EIGEN_PRODUCT_PUBLIC_INTERFACE(TriangularProduct)

  TriangularProduct(const Lhs& lhs, const Rhs& rhs) : Base(lhs,rhs) {}

  template<typename Dest> void scaleAndAddTo(Dest& dst, Scalar alpha) const
  {

    ei_assert(dst.rows()==m_lhs.rows() && dst.cols()==m_rhs.cols());

    const ActualLhsType lhs = LhsBlasTraits::extract(m_lhs);
    const ActualRhsType rhs = RhsBlasTraits::extract(m_rhs);

    Scalar actualAlpha = alpha * LhsBlasTraits::extractScalarFactor(m_lhs)
                               * RhsBlasTraits::extractScalarFactor(m_rhs);

    ei_product_triangular_vector_selector
      <false,_ActualLhsType,_ActualRhsType,Dest,
       Mode,
       LhsBlasTraits::NeedToConjugate,
       RhsBlasTraits::NeedToConjugate,
       (int(ei_traits<Rhs>::Flags)&RowMajorBit) ? RowMajor : ColMajor>
      ::run(lhs,rhs,dst,actualAlpha);
  }
};

#endif // EIGEN_TRIANGULARMATRIXVECTOR_H
