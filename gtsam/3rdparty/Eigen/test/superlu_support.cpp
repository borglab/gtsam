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

#include <Eigen/SuperLUSupport>

void test_superlu_support()
{
  SuperLU<SparseMatrix<double> > superlu_double_colmajor;
  SuperLU<SparseMatrix<std::complex<double> > > superlu_cplxdouble_colmajor;
  CALL_SUBTEST_1( check_sparse_square_solving(superlu_double_colmajor)      );
  CALL_SUBTEST_2( check_sparse_square_solving(superlu_cplxdouble_colmajor)  );
  CALL_SUBTEST_1( check_sparse_square_determinant(superlu_double_colmajor)      );
  CALL_SUBTEST_2( check_sparse_square_determinant(superlu_cplxdouble_colmajor)  );
}
