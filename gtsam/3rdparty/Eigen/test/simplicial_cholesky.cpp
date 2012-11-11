// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2011 Gael Guennebaud <g.gael@free.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

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
