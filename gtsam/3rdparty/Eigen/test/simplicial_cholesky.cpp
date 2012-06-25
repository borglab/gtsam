// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2011 Gael Guennebaud <g.gael@free.fr>
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

template<typename T> void test_simplicial_cholesky_T()
{
  SimplicialCholesky<SparseMatrix<T>, Lower> chol_colmajor_lower;
  SimplicialCholesky<SparseMatrix<T>, Upper> chol_colmajor_upper;
  SimplicialLLT<SparseMatrix<T>, Lower> llt_colmajor_lower;
  SimplicialLDLT<SparseMatrix<T>, Upper> llt_colmajor_upper;
  SimplicialLDLT<SparseMatrix<T>, Lower> ldlt_colmajor_lower;
  SimplicialLDLT<SparseMatrix<T>, Upper> ldlt_colmajor_upper;

  check_sparse_spd_solving(chol_colmajor_lower);
  check_sparse_spd_solving(chol_colmajor_upper);
  check_sparse_spd_solving(llt_colmajor_lower);
  check_sparse_spd_solving(llt_colmajor_upper);
  check_sparse_spd_solving(ldlt_colmajor_lower);
  check_sparse_spd_solving(ldlt_colmajor_upper);
  
  check_sparse_spd_determinant(chol_colmajor_lower);
  check_sparse_spd_determinant(chol_colmajor_upper);
  check_sparse_spd_determinant(llt_colmajor_lower);
  check_sparse_spd_determinant(llt_colmajor_upper);
  check_sparse_spd_determinant(ldlt_colmajor_lower);
  check_sparse_spd_determinant(ldlt_colmajor_upper);
}

void test_simplicial_cholesky()
{
  CALL_SUBTEST_1(test_simplicial_cholesky_T<double>());
  CALL_SUBTEST_2(test_simplicial_cholesky_T<std::complex<double> >());
}
