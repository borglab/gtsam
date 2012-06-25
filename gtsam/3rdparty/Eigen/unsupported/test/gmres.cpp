// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2011 Gael Guennebaud <g.gael@free.fr>
// Copyright (C) 2012 Kolja Brix <brix@igpm.rwth-aaachen.de>
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

#include "../../test/sparse_solver.h"
#include <Eigen/IterativeSolvers>

template<typename T> void test_gmres_T()
{
  GMRES<SparseMatrix<T>, DiagonalPreconditioner<T> > gmres_colmajor_diag;
  GMRES<SparseMatrix<T>, IdentityPreconditioner    > gmres_colmajor_I;
  GMRES<SparseMatrix<T>, IncompleteLUT<T> >           gmres_colmajor_ilut;
  //GMRES<SparseMatrix<T>, SSORPreconditioner<T> >     gmres_colmajor_ssor;

  CALL_SUBTEST( check_sparse_square_solving(gmres_colmajor_diag)  );
//   CALL_SUBTEST( check_sparse_square_solving(gmres_colmajor_I)     );
  CALL_SUBTEST( check_sparse_square_solving(gmres_colmajor_ilut)     );
  //CALL_SUBTEST( check_sparse_square_solving(gmres_colmajor_ssor)     );
}

void test_gmres()
{
  for(int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(test_gmres_T<double>());
    CALL_SUBTEST_2(test_gmres_T<std::complex<double> >());
  }
}
