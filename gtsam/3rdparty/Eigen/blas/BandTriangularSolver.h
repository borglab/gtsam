// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2011 Gael Guennebaud <gael.guennebaud@inria.fr>
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

#ifndef EIGEN_BAND_TRIANGULARSOLVER_H
#define EIGEN_BAND_TRIANGULARSOLVER_H

namespace internal {

 /* \internal
  * Solve Ax=b with A a band triangular matrix
  * TODO: extend it to matrices for x abd b */
template<typename Index, int Mode, typename LhsScalar, bool ConjLhs, typename RhsScalar, int StorageOrder>
struct band_solve_triangular_selector;


template<typename Index, int Mode, typename LhsScalar, bool ConjLhs, typename RhsScalar>
struct band_solve_triangular_selector<Index,Mode,LhsScalar,ConjLhs,RhsScalar,RowMajor>
{
  typedef Map<const Matrix<LhsScalar,Dynamic,Dynamic,RowMajor>, 0, OuterStride<> > LhsMap;
  typedef Map<Matrix<RhsScalar,Dynamic,1> > RhsMap;
  enum { IsLower = (Mode&Lower) ? 1 : 0 };
  static void run(Index size, Index k, const LhsScalar* _lhs, Index lhsStride, RhsScalar* _other)
  {
    const LhsMap lhs(_lhs,size,k+1,OuterStride<>(lhsStride));
    RhsMap other(_other,size,1);
    typename internal::conditional<
                          ConjLhs,
                          const CwiseUnaryOp<typename internal::scalar_conjugate_op<LhsScalar>,LhsMap>,
                          const LhsMap&>
                        ::type cjLhs(lhs);
                        
    for(int col=0 ; col<other.cols() ; ++col)
    {
      for(int ii=0; ii<size; ++ii)
      {
        int i = IsLower ? ii : size-ii-1;
        int actual_k = (std::min)(k,ii);
        int actual_start = IsLower ? k-actual_k : 1;
        
        if(actual_k>0)
          other.coeffRef(i,col) -= cjLhs.row(i).segment(actual_start,actual_k).transpose()
                                  .cwiseProduct(other.col(col).segment(IsLower ? i-actual_k : i+1,actual_k)).sum();

        if((Mode&UnitDiag)==0)
          other.coeffRef(i,col) /= cjLhs(i,IsLower ? k : 0);
      }
    }
  }
  
};

template<typename Index, int Mode, typename LhsScalar, bool ConjLhs, typename RhsScalar>
struct band_solve_triangular_selector<Index,Mode,LhsScalar,ConjLhs,RhsScalar,ColMajor>
{
  typedef Map<const Matrix<LhsScalar,Dynamic,Dynamic,ColMajor>, 0, OuterStride<> > LhsMap;
  typedef Map<Matrix<RhsScalar,Dynamic,1> > RhsMap;
  enum { IsLower = (Mode&Lower) ? 1 : 0 };
  static void run(Index size, Index k, const LhsScalar* _lhs, Index lhsStride, RhsScalar* _other)
  {
    const LhsMap lhs(_lhs,k+1,size,OuterStride<>(lhsStride));
    RhsMap other(_other,size,1);
    typename internal::conditional<
                          ConjLhs,
                          const CwiseUnaryOp<typename internal::scalar_conjugate_op<LhsScalar>,LhsMap>,
                          const LhsMap&>
                        ::type cjLhs(lhs);
                        
    for(int col=0 ; col<other.cols() ; ++col)
    {
      for(int ii=0; ii<size; ++ii)
      {
        int i = IsLower ? ii : size-ii-1;
        int actual_k = (std::min)(k,size-ii-1);
        int actual_start = IsLower ? 1 : k-actual_k;
        
        if((Mode&UnitDiag)==0)
          other.coeffRef(i,col) /= cjLhs(IsLower ? 0 : k, i);

        if(actual_k>0)
          other.col(col).segment(IsLower ? i+1 : i-actual_k, actual_k)
              -= other.coeff(i,col) * cjLhs.col(i).segment(actual_start,actual_k);
        
      }
    }
  }
};


} // end namespace internal

#endif // EIGEN_BAND_TRIANGULARSOLVER_H
