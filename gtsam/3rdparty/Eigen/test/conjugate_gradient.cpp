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
#include <Eigen/IterativeLinearSolvers>

template<typename T> void test_conjugate_gradient_T()
{
  ConjugateGradient<SparseMatrix<T>, Lower> cg_colmajor_lower_diag;
  ConjugateGradient<SparseMatrix<T>, Upper> cg_colmajor_upper_diag;
  ConjugateGradient<SparseMatrix<T>, Lower, IdentityPreconditioner> cg_colmajor_lower_I;
  ConjugateGradient<SparseMatrix<T>, Upper, IdentityPreconditioner> cg_colmajor_upper_I;

  CALL_SUBTEST( check_sparse_spd_solving(cg_colmajor_lower_diag)  );
  CALL_SUBTEST( check_sparse_spd_solving(cg_colmajor_upper_diag)  );
  CALL_SUBTEST( check_sparse_spd_solving(cg_colmajor_lower_I)     );
  CALL_SUBTEST( check_sparse_spd_solving(cg_colmajor_upper_I)     );
}

void test_conjugate_gradient()
{
  CALL_SUBTEST_1(test_conjugate_gradient_T<double>());
  CALL_SUBTEST_2(test_conjugate_gradient_T<std::complex<double> >());
}
