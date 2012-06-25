// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2011 Jitse Niesen <jitse@maths.leeds.ac.uk>
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

#include "main.h"
#include <unsupported/Eigen/MatrixFunctions>

template <typename MatrixType, int IsComplex = NumTraits<typename internal::traits<MatrixType>::Scalar>::IsComplex>
struct generateTestMatrix;

// for real matrices, make sure none of the eigenvalues are negative
template <typename MatrixType>
struct generateTestMatrix<MatrixType,0>
{
  static void run(MatrixType& result, typename MatrixType::Index size)
  {
    MatrixType mat = MatrixType::Random(size, size);
    EigenSolver<MatrixType> es(mat);
    typename EigenSolver<MatrixType>::EigenvalueType eivals = es.eigenvalues();
    for (typename MatrixType::Index i = 0; i < size; ++i) {
      if (eivals(i).imag() == 0 && eivals(i).real() < 0)
	eivals(i) = -eivals(i);
    }
    result = (es.eigenvectors() * eivals.asDiagonal() * es.eigenvectors().inverse()).real();
  }
};

// for complex matrices, any matrix is fine
template <typename MatrixType>
struct generateTestMatrix<MatrixType,1>
{
  static void run(MatrixType& result, typename MatrixType::Index size)
  {
    result = MatrixType::Random(size, size);
  }
};

template<typename MatrixType>
void testMatrixSqrt(const MatrixType& m)
{
  MatrixType A;
  generateTestMatrix<MatrixType>::run(A, m.rows());
  MatrixType sqrtA = A.sqrt();
  VERIFY_IS_APPROX(sqrtA * sqrtA, A);
}

void test_matrix_square_root()
{
  for (int i = 0; i < g_repeat; i++) {
    CALL_SUBTEST_1(testMatrixSqrt(Matrix3cf()));
    CALL_SUBTEST_2(testMatrixSqrt(MatrixXcd(12,12)));
    CALL_SUBTEST_3(testMatrixSqrt(Matrix4f()));
    CALL_SUBTEST_4(testMatrixSqrt(Matrix<double,Dynamic,Dynamic,RowMajor>(9, 9)));
    CALL_SUBTEST_5(testMatrixSqrt(Matrix<float,1,1>()));
    CALL_SUBTEST_5(testMatrixSqrt(Matrix<std::complex<float>,1,1>()));
  }
}
