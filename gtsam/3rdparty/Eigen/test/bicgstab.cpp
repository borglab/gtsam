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

template<typename T> void test_bicgstab_T()
{
  BiCGSTAB<SparseMatrix<T>, DiagonalPreconditioner<T> > bicgstab_colmajor_diag;
  BiCGSTAB<SparseMatrix<T>, IdentityPreconditioner    > bicgstab_colmajor_I;
  BiCGSTAB<SparseMatrix<T>, IncompleteLUT<T> >           bicgstab_colmajor_ilut;
  //BiCGSTAB<SparseMatrix<T>, SSORPreconditioner<T> >     bicgstab_colmajor_ssor;

  CALL_SUBTEST( check_sparse_square_solving(bicgstab_colmajor_diag)  );
//   CALL_SUBTEST( check_sparse_square_solving(bicgstab_colmajor_I)     );
  CALL_SUBTEST( check_sparse_square_solving(bicgstab_colmajor_ilut)     );
  //CALL_SUBTEST( check_sparse_square_solving(bicgstab_colmajor_ssor)     );
}

void test_bicgstab()
{
  CALL_SUBTEST_1(test_bicgstab_T<double>());
  CALL_SUBTEST_2(test_bicgstab_T<std::complex<double> >());
}
