// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008-2012 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2012 Désiré Nuentsa-Wakam <desire.nuentsa_wakam@inria.fr>
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
#include "sparse_solver.h"
#include <Eigen/PaStiXSupport>
#include <unsupported/Eigen/SparseExtra>


template<typename T> void test_pastix_T()
{
  PastixLLT< SparseMatrix<T, ColMajor>, Eigen::Lower > pastix_llt_lower;
  PastixLDLT< SparseMatrix<T, ColMajor>, Eigen::Lower > pastix_ldlt_lower;
  PastixLLT< SparseMatrix<T, ColMajor>, Eigen::Upper > pastix_llt_upper;
  PastixLDLT< SparseMatrix<T, ColMajor>, Eigen::Upper > pastix_ldlt_upper;
  PastixLU< SparseMatrix<T, ColMajor> > pastix_lu;

  check_sparse_spd_solving(pastix_llt_lower);
  check_sparse_spd_solving(pastix_ldlt_lower);
  check_sparse_spd_solving(pastix_llt_upper);
  check_sparse_spd_solving(pastix_ldlt_upper);
  check_sparse_square_solving(pastix_lu);
}

// There is no support for selfadjoint matrices with PaStiX. 
// Complex symmetric matrices should pass though
template<typename T> void test_pastix_T_LU()
{
  PastixLU< SparseMatrix<T, ColMajor> > pastix_lu;
  check_sparse_square_solving(pastix_lu);
}

void test_pastix_support()
{
  CALL_SUBTEST_1(test_pastix_T<float>());
  CALL_SUBTEST_2(test_pastix_T<double>());
  CALL_SUBTEST_3( (test_pastix_T_LU<std::complex<float> >()) );
  CALL_SUBTEST_4(test_pastix_T_LU<std::complex<double> >());
} 