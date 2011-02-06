// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
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

#ifndef EIGEN_SPARSETRIANGULARSOLVER_H
#define EIGEN_SPARSETRIANGULARSOLVER_H

template<typename Lhs, typename Rhs, int Mode,
  int UpLo = (Mode & Lower)
           ? Lower
           : (Mode & Upper)
           ? Upper
           : -1,
  int StorageOrder = int(ei_traits<Lhs>::Flags) & RowMajorBit>
struct ei_sparse_solve_triangular_selector;

// forward substitution, row-major
template<typename Lhs, typename Rhs, int Mode>
struct ei_sparse_solve_triangular_selector<Lhs,Rhs,Mode,Lower,RowMajor>
{
  typedef typename Rhs::Scalar Scalar;
  static void run(const Lhs& lhs, Rhs& other)
  {
    for(int col=0 ; col<other.cols() ; ++col)
    {
      for(int i=0; i<lhs.rows(); ++i)
      {
        Scalar tmp = other.coeff(i,col);
        Scalar lastVal = 0;
        int lastIndex = 0;
        for(typename Lhs::InnerIterator it(lhs, i); it; ++it)
        {
          lastVal = it.value();
          lastIndex = it.index();
          if(lastIndex==i)
            break;
          tmp -= lastVal * other.coeff(lastIndex,col);
        }
        if (Mode & UnitDiag)
          other.coeffRef(i,col) = tmp;
        else
        {
          ei_assert(lastIndex==i);
          other.coeffRef(i,col) = tmp/lastVal;
        }
      }
    }
  }
};

// backward substitution, row-major
template<typename Lhs, typename Rhs, int Mode>
struct ei_sparse_solve_triangular_selector<Lhs,Rhs,Mode,Upper,RowMajor>
{
  typedef typename Rhs::Scalar Scalar;
  static void run(const Lhs& lhs, Rhs& other)
  {
    for(int col=0 ; col<other.cols() ; ++col)
    {
      for(int i=lhs.rows()-1 ; i>=0 ; --i)
      {
        Scalar tmp = other.coeff(i,col);
        typename Lhs::InnerIterator it(lhs, i);
        if (it && it.index() == i)
          ++it;
        for(; it; ++it)
        {
          tmp -= it.value() * other.coeff(it.index(),col);
        }

        if (Mode & UnitDiag)
          other.coeffRef(i,col) = tmp;
        else
        {
          typename Lhs::InnerIterator it(lhs, i);
          ei_assert(it && it.index() == i);
          other.coeffRef(i,col) = tmp/it.value();
        }
      }
    }
  }
};

// forward substitution, col-major
template<typename Lhs, typename Rhs, int Mode>
struct ei_sparse_solve_triangular_selector<Lhs,Rhs,Mode,Lower,ColMajor>
{
  typedef typename Rhs::Scalar Scalar;
  static void run(const Lhs& lhs, Rhs& other)
  {
    for(int col=0 ; col<other.cols() ; ++col)
    {
      for(int i=0; i<lhs.cols(); ++i)
      {
        Scalar& tmp = other.coeffRef(i,col);
        if (tmp!=Scalar(0)) // optimization when other is actually sparse
        {
          typename Lhs::InnerIterator it(lhs, i);
          if(!(Mode & UnitDiag))
          {
            ei_assert(it.index()==i);
            tmp /= it.value();
          }
          if (it && it.index()==i)
            ++it;
          for(; it; ++it)
            other.coeffRef(it.index(), col) -= tmp * it.value();
        }
      }
    }
  }
};

// backward substitution, col-major
template<typename Lhs, typename Rhs, int Mode>
struct ei_sparse_solve_triangular_selector<Lhs,Rhs,Mode,Upper,ColMajor>
{
  typedef typename Rhs::Scalar Scalar;
  static void run(const Lhs& lhs, Rhs& other)
  {
    for(int col=0 ; col<other.cols() ; ++col)
    {
      for(int i=lhs.cols()-1; i>=0; --i)
      {
        Scalar& tmp = other.coeffRef(i,col);
        if (tmp!=Scalar(0)) // optimization when other is actually sparse
        {
          if(!(Mode & UnitDiag))
          {
            // FIXME lhs.coeff(i,i) might not be always efficient while it must simply be the
            // last element of the column !
            other.coeffRef(i,col) /= lhs.innerVector(i).lastCoeff();
          }
          typename Lhs::InnerIterator it(lhs, i);
          for(; it && it.index()<i; ++it)
            other.coeffRef(it.index(), col) -= tmp * it.value();
        }
      }
    }
  }
};

template<typename ExpressionType,int Mode>
template<typename OtherDerived>
void SparseTriangularView<ExpressionType,Mode>::solveInPlace(MatrixBase<OtherDerived>& other) const
{
  ei_assert(m_matrix.cols() == m_matrix.rows());
  ei_assert(m_matrix.cols() == other.rows());
  ei_assert(!(Mode & ZeroDiag));
  ei_assert(Mode & (Upper|Lower));

  enum { copy = ei_traits<OtherDerived>::Flags & RowMajorBit };

  typedef typename ei_meta_if<copy,
    typename ei_plain_matrix_type_column_major<OtherDerived>::type, OtherDerived&>::ret OtherCopy;
  OtherCopy otherCopy(other.derived());

  ei_sparse_solve_triangular_selector<ExpressionType, typename ei_unref<OtherCopy>::type, Mode>::run(m_matrix, otherCopy);

  if (copy)
    other = otherCopy;
}

template<typename ExpressionType,int Mode>
template<typename OtherDerived>
typename ei_plain_matrix_type_column_major<OtherDerived>::type
SparseTriangularView<ExpressionType,Mode>::solve(const MatrixBase<OtherDerived>& other) const
{
  typename ei_plain_matrix_type_column_major<OtherDerived>::type res(other);
  solveInPlace(res);
  return res;
}

// pure sparse path

template<typename Lhs, typename Rhs, int Mode,
  int UpLo = (Mode & Lower)
           ? Lower
           : (Mode & Upper)
           ? Upper
           : -1,
  int StorageOrder = int(Lhs::Flags) & (RowMajorBit)>
struct ei_sparse_solve_triangular_sparse_selector;

// forward substitution, col-major
template<typename Lhs, typename Rhs, int Mode, int UpLo>
struct ei_sparse_solve_triangular_sparse_selector<Lhs,Rhs,Mode,UpLo,ColMajor>
{
  typedef typename Rhs::Scalar Scalar;
  typedef typename ei_promote_index_type<typename ei_traits<Lhs>::Index,
                                         typename ei_traits<Rhs>::Index>::type Index;
  static void run(const Lhs& lhs, Rhs& other)
  {
    const bool IsLower = (UpLo==Lower);
    AmbiVector<Scalar,Index> tempVector(other.rows()*2);
    tempVector.setBounds(0,other.rows());

    Rhs res(other.rows(), other.cols());
    res.reserve(other.nonZeros());

    for(int col=0 ; col<other.cols() ; ++col)
    {
      // FIXME estimate number of non zeros
      tempVector.init(.99/*float(other.col(col).nonZeros())/float(other.rows())*/);
      tempVector.setZero();
      tempVector.restart();
      for (typename Rhs::InnerIterator rhsIt(other, col); rhsIt; ++rhsIt)
      {
        tempVector.coeffRef(rhsIt.index()) = rhsIt.value();
      }

      for(int i=IsLower?0:lhs.cols()-1;
          IsLower?i<lhs.cols():i>=0;
          i+=IsLower?1:-1)
      {
        tempVector.restart();
        Scalar& ci = tempVector.coeffRef(i);
        if (ci!=Scalar(0))
        {
          // find
          typename Lhs::InnerIterator it(lhs, i);
          if(!(Mode & UnitDiag))
          {
            if (IsLower)
            {
              ei_assert(it.index()==i);
              ci /= it.value();
            }
            else
              ci /= lhs.coeff(i,i);
          }
          tempVector.restart();
          if (IsLower)
          {
            if (it.index()==i)
              ++it;
            for(; it; ++it)
              tempVector.coeffRef(it.index()) -= ci * it.value();
          }
          else
          {
            for(; it && it.index()<i; ++it)
              tempVector.coeffRef(it.index()) -= ci * it.value();
          }
        }
      }


      int count = 0;
      // FIXME compute a reference value to filter zeros
      for (typename AmbiVector<Scalar,Index>::Iterator it(tempVector/*,1e-12*/); it; ++it)
      {
        ++ count;
//         std::cerr << "fill " << it.index() << ", " << col << "\n";
//         std::cout << it.value() << "  ";
        // FIXME use insertBack
        res.insert(it.index(), col) = it.value();
      }
//       std::cout << "tempVector.nonZeros() == " << int(count) << " / " << (other.rows()) << "\n";
    }
    res.finalize();
    other = res.markAsRValue();
  }
};

template<typename ExpressionType,int Mode>
template<typename OtherDerived>
void SparseTriangularView<ExpressionType,Mode>::solveInPlace(SparseMatrixBase<OtherDerived>& other) const
{
  ei_assert(m_matrix.cols() == m_matrix.rows());
  ei_assert(m_matrix.cols() == other.rows());
  ei_assert(!(Mode & ZeroDiag));
  ei_assert(Mode & (Upper|Lower));

//   enum { copy = ei_traits<OtherDerived>::Flags & RowMajorBit };

//   typedef typename ei_meta_if<copy,
//     typename ei_plain_matrix_type_column_major<OtherDerived>::type, OtherDerived&>::ret OtherCopy;
//   OtherCopy otherCopy(other.derived());

  ei_sparse_solve_triangular_sparse_selector<ExpressionType, OtherDerived, Mode>::run(m_matrix, other.derived());

//   if (copy)
//     other = otherCopy;
}

#ifdef EIGEN2_SUPPORT

// deprecated stuff:

/** \deprecated */
template<typename Derived>
template<typename OtherDerived>
void SparseMatrixBase<Derived>::solveTriangularInPlace(MatrixBase<OtherDerived>& other) const
{
  this->template triangular<Flags&(Upper|Lower)>().solveInPlace(other);
}

/** \deprecated */
template<typename Derived>
template<typename OtherDerived>
typename ei_plain_matrix_type_column_major<OtherDerived>::type
SparseMatrixBase<Derived>::solveTriangular(const MatrixBase<OtherDerived>& other) const
{
  typename ei_plain_matrix_type_column_major<OtherDerived>::type res(other);
  derived().solveTriangularInPlace(res);
  return res;
}
#endif // EIGEN2_SUPPORT

#endif // EIGEN_SPARSETRIANGULARSOLVER_H
