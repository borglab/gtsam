// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2009 Gael Guennebaud <gael.guennebaud@inria.fr>
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

#ifndef EIGEN_SOLVETRIANGULAR_H
#define EIGEN_SOLVETRIANGULAR_H

template<typename Lhs, typename Rhs, int Side>
class ei_trsolve_traits
{
  private:
    enum {
      RhsIsVectorAtCompileTime = (Side==OnTheLeft ? Rhs::ColsAtCompileTime : Rhs::RowsAtCompileTime)==1
    };
  public:
    enum {
      Unrolling   = (RhsIsVectorAtCompileTime && Rhs::SizeAtCompileTime != Dynamic && Rhs::SizeAtCompileTime <= 8)
                  ? CompleteUnrolling : NoUnrolling,
      RhsVectors  = RhsIsVectorAtCompileTime ? 1 : Dynamic
    };
};

template<typename Lhs, typename Rhs,
  int Side, // can be OnTheLeft/OnTheRight
  int Mode, // can be Upper/Lower | UnitDiag
  int Unrolling = ei_trsolve_traits<Lhs,Rhs,Side>::Unrolling,
  int StorageOrder = (int(Lhs::Flags) & RowMajorBit) ? RowMajor : ColMajor,
  int RhsVectors = ei_trsolve_traits<Lhs,Rhs,Side>::RhsVectors
  >
struct ei_triangular_solver_selector;

// forward and backward substitution, row-major, rhs is a vector
template<typename Lhs, typename Rhs, int Mode>
struct ei_triangular_solver_selector<Lhs,Rhs,OnTheLeft,Mode,NoUnrolling,RowMajor,1>
{
  typedef typename Lhs::Scalar LhsScalar;
  typedef typename Rhs::Scalar RhsScalar;
  typedef ei_blas_traits<Lhs> LhsProductTraits;
  typedef typename LhsProductTraits::ExtractType ActualLhsType;
  typedef typename Lhs::Index Index;
  enum {
    IsLower = ((Mode&Lower)==Lower)
  };
  static void run(const Lhs& lhs, Rhs& other)
  {
    static const Index PanelWidth = EIGEN_TUNE_TRIANGULAR_PANEL_WIDTH;
    ActualLhsType actualLhs = LhsProductTraits::extract(lhs);

    const Index size = lhs.cols();
    for(Index pi=IsLower ? 0 : size;
        IsLower ? pi<size : pi>0;
        IsLower ? pi+=PanelWidth : pi-=PanelWidth)
    {
      Index actualPanelWidth = std::min(IsLower ? size - pi : pi, PanelWidth);

      Index r = IsLower ? pi : size - pi; // remaining size
      if (r > 0)
      {
        // let's directly call the low level product function because:
        // 1 - it is faster to compile
        // 2 - it is slighlty faster at runtime
        Index startRow = IsLower ? pi : pi-actualPanelWidth;
        Index startCol = IsLower ? 0 : pi;

        ei_general_matrix_vector_product<Index,LhsScalar,RowMajor,LhsProductTraits::NeedToConjugate,RhsScalar,false>::run(
          actualPanelWidth, r,
          &(actualLhs.const_cast_derived().coeffRef(startRow,startCol)), actualLhs.outerStride(),
          &(other.coeffRef(startCol)), other.innerStride(),
          &other.coeffRef(startRow), other.innerStride(),
          RhsScalar(-1));
      }

      for(Index k=0; k<actualPanelWidth; ++k)
      {
        Index i = IsLower ? pi+k : pi-k-1;
        Index s = IsLower ? pi : i+1;
        if (k>0)
          other.coeffRef(i) -= (lhs.row(i).segment(s,k).transpose().cwiseProduct(other.segment(s,k))).sum();

        if(!(Mode & UnitDiag))
          other.coeffRef(i) /= lhs.coeff(i,i);
      }
    }
  }
};

// forward and backward substitution, column-major, rhs is a vector
template<typename Lhs, typename Rhs, int Mode>
struct ei_triangular_solver_selector<Lhs,Rhs,OnTheLeft,Mode,NoUnrolling,ColMajor,1>
{
  typedef typename Lhs::Scalar LhsScalar;
  typedef typename Rhs::Scalar RhsScalar;
  typedef ei_blas_traits<Lhs> LhsProductTraits;
  typedef typename LhsProductTraits::ExtractType ActualLhsType;
  typedef typename Lhs::Index Index;
  enum {
    IsLower = ((Mode&Lower)==Lower)
  };

  static void run(const Lhs& lhs, Rhs& other)
  {
    static const Index PanelWidth = EIGEN_TUNE_TRIANGULAR_PANEL_WIDTH;
    ActualLhsType actualLhs = LhsProductTraits::extract(lhs);

    const Index size = lhs.cols();
    for(Index pi=IsLower ? 0 : size;
        IsLower ? pi<size : pi>0;
        IsLower ? pi+=PanelWidth : pi-=PanelWidth)
    {
      Index actualPanelWidth = std::min(IsLower ? size - pi : pi, PanelWidth);
      Index startBlock = IsLower ? pi : pi-actualPanelWidth;
      Index endBlock = IsLower ? pi + actualPanelWidth : 0;

      for(Index k=0; k<actualPanelWidth; ++k)
      {
        Index i = IsLower ? pi+k : pi-k-1;
        if(!(Mode & UnitDiag))
          other.coeffRef(i) /= lhs.coeff(i,i);

        Index r = actualPanelWidth - k - 1; // remaining size
        Index s = IsLower ? i+1 : i-r;
        if (r>0)
          other.segment(s,r) -= other.coeffRef(i) * Block<Lhs,Dynamic,1>(lhs, s, i, r, 1);
      }
      Index r = IsLower ? size - endBlock : startBlock; // remaining size
      if (r > 0)
      {
        // let's directly call the low level product function because:
        // 1 - it is faster to compile
        // 2 - it is slighlty faster at runtime
        ei_general_matrix_vector_product<Index,LhsScalar,ColMajor,LhsProductTraits::NeedToConjugate,RhsScalar,false>::run(
            r, actualPanelWidth,
            &(actualLhs.const_cast_derived().coeffRef(endBlock,startBlock)), actualLhs.outerStride(),
            &other.coeff(startBlock), other.innerStride(),
            &(other.coeffRef(endBlock, 0)), other.innerStride(), RhsScalar(-1));
      }
    }
  }
};

// transpose OnTheRight cases for vectors
template<typename Lhs, typename Rhs, int Mode, int Unrolling, int StorageOrder>
struct ei_triangular_solver_selector<Lhs,Rhs,OnTheRight,Mode,Unrolling,StorageOrder,1>
{
  static void run(const Lhs& lhs, Rhs& rhs)
  {
    Transpose<Rhs> rhsTr(rhs);
    Transpose<Lhs> lhsTr(lhs);
    ei_triangular_solver_selector<Transpose<Lhs>,Transpose<Rhs>,OnTheLeft,TriangularView<Lhs,Mode>::TransposeMode>::run(lhsTr,rhsTr);
  }
};

template <typename Scalar, typename Index, int Side, int Mode, bool Conjugate, int TriStorageOrder, int OtherStorageOrder>
struct ei_triangular_solve_matrix;

// the rhs is a matrix
template<typename Lhs, typename Rhs, int Side, int Mode, int StorageOrder>
struct ei_triangular_solver_selector<Lhs,Rhs,Side,Mode,NoUnrolling,StorageOrder,Dynamic>
{
  typedef typename Rhs::Scalar Scalar;
  typedef typename Rhs::Index Index;
  typedef ei_blas_traits<Lhs> LhsProductTraits;
  typedef typename LhsProductTraits::DirectLinearAccessType ActualLhsType;
  static void run(const Lhs& lhs, Rhs& rhs)
  {
    const ActualLhsType actualLhs = LhsProductTraits::extract(lhs);
    ei_triangular_solve_matrix<Scalar,Index,Side,Mode,LhsProductTraits::NeedToConjugate,StorageOrder,
                               (Rhs::Flags&RowMajorBit) ? RowMajor : ColMajor>
      ::run(lhs.rows(), Side==OnTheLeft? rhs.cols() : rhs.rows(), &actualLhs.coeff(0,0), actualLhs.outerStride(), &rhs.coeffRef(0,0), rhs.outerStride());
  }
};

/***************************************************************************
* meta-unrolling implementation
***************************************************************************/

template<typename Lhs, typename Rhs, int Mode, int Index, int Size,
         bool Stop = Index==Size>
struct ei_triangular_solver_unroller;

template<typename Lhs, typename Rhs, int Mode, int Index, int Size>
struct ei_triangular_solver_unroller<Lhs,Rhs,Mode,Index,Size,false> {
  enum {
    IsLower = ((Mode&Lower)==Lower),
    I = IsLower ? Index : Size - Index - 1,
    S = IsLower ? 0     : I+1
  };
  static void run(const Lhs& lhs, Rhs& rhs)
  {
    if (Index>0)
      rhs.coeffRef(I) -= lhs.row(I).template segment<Index>(S).transpose().cwiseProduct(rhs.template segment<Index>(S)).sum();

    if(!(Mode & UnitDiag))
      rhs.coeffRef(I) /= lhs.coeff(I,I);

    ei_triangular_solver_unroller<Lhs,Rhs,Mode,Index+1,Size>::run(lhs,rhs);
  }
};

template<typename Lhs, typename Rhs, int Mode, int Index, int Size>
struct ei_triangular_solver_unroller<Lhs,Rhs,Mode,Index,Size,true> {
  static void run(const Lhs&, Rhs&) {}
};

template<typename Lhs, typename Rhs, int Mode, int StorageOrder>
struct ei_triangular_solver_selector<Lhs,Rhs,OnTheLeft,Mode,CompleteUnrolling,StorageOrder,1> {
  static void run(const Lhs& lhs, Rhs& rhs)
  { ei_triangular_solver_unroller<Lhs,Rhs,Mode,0,Rhs::SizeAtCompileTime>::run(lhs,rhs); }
};

/***************************************************************************
* TriangularView methods
***************************************************************************/

/** "in-place" version of TriangularView::solve() where the result is written in \a other
  *
  *
  *
  * \warning The parameter is only marked 'const' to make the C++ compiler accept a temporary expression here.
  * This function will const_cast it, so constness isn't honored here.
  *
  * See TriangularView:solve() for the details.
  */
template<typename MatrixType, unsigned int Mode>
template<int Side, typename OtherDerived>
void TriangularView<MatrixType,Mode>::solveInPlace(const MatrixBase<OtherDerived>& _other) const
{
  OtherDerived& other = _other.const_cast_derived();
  ei_assert(cols() == rows());
  ei_assert( (Side==OnTheLeft && cols() == other.rows()) || (Side==OnTheRight && cols() == other.cols()) );
  ei_assert(!(Mode & ZeroDiag));
  ei_assert(Mode & (Upper|Lower));

  enum { copy = ei_traits<OtherDerived>::Flags & RowMajorBit  && OtherDerived::IsVectorAtCompileTime };
  typedef typename ei_meta_if<copy,
    typename ei_plain_matrix_type_column_major<OtherDerived>::type, OtherDerived&>::ret OtherCopy;
  OtherCopy otherCopy(other);

  ei_triangular_solver_selector<MatrixType, typename ei_unref<OtherCopy>::type,
    Side, Mode>::run(nestedExpression(), otherCopy);

  if (copy)
    other = otherCopy;
}

/** \returns the product of the inverse of \c *this with \a other, \a *this being triangular.
  *
  *
  *
  * This function computes the inverse-matrix matrix product inverse(\c *this) * \a other.
  * The matrix \c *this must be triangular and invertible (i.e., all the coefficients of the
  * diagonal must be non zero). It works as a forward (resp. backward) substitution if \c *this
  * is an upper (resp. lower) triangular matrix.
  *
  * It is required that \c *this be marked as either an upper or a lower triangular matrix, which
  * can be done by marked(), and that is automatically the case with expressions such as those returned
  * by extract().
  *
  * Example: \include MatrixBase_marked.cpp
  * Output: \verbinclude MatrixBase_marked.out
  *
  * This function is essentially a wrapper to the faster solveTriangularInPlace() function creating
  * a temporary copy of \a other, calling solveTriangularInPlace() on the copy and returning it.
  * Therefore, if \a other is not needed anymore, it is quite faster to call solveTriangularInPlace()
  * instead of solveTriangular().
  *
  * For users coming from BLAS, this function (and more specifically solveTriangularInPlace()) offer
  * all the operations supported by the \c *TRSV and \c *TRSM BLAS routines.
  *
  * \b Tips: to perform a \em "right-inverse-multiply" you can simply transpose the operation, e.g.:
  * \code
  * M * T^1  <=>  T.transpose().solveInPlace(M.transpose());
  * \endcode
  *
  * \sa TriangularView::solveInPlace()
  */
template<typename Derived, unsigned int Mode>
template<int Side, typename RhsDerived>
typename ei_plain_matrix_type_column_major<RhsDerived>::type
TriangularView<Derived,Mode>::solve(const MatrixBase<RhsDerived>& rhs) const
{
  typename ei_plain_matrix_type_column_major<RhsDerived>::type res(rhs);
  solveInPlace<Side>(res);
  return res;
}

#endif // EIGEN_SOLVETRIANGULAR_H
