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

#include <Eigen/UmfPackSupport>

template<typename T> void test_umfpack_support_T()
{
  UmfPackLU<SparseMatrix<T, ColMajor> > umfpack_colmajor;
  UmfPackLU<SparseMatrix<T, RowMajor> > umfpack_rowmajor;
  
  check_sparse_square_solving(umfpack_colmajor);
  check_sparse_square_solving(umfpack_rowmajor);
  
  check_sparse_square_determinant(umfpack_colmajor);
  check_sparse_square_determinant(umfpack_rowmajor);
}

void test_umfpack_support()
{
  CALL_SUBTEST_1(test_umfpack_support_T<double>());
  CALL_SUBTEST_2(test_umfpack_support_T<std::complex<double> >());
}

