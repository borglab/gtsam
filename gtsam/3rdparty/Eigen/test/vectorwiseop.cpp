// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2011 Benoit Jacob <jacob.benoit.1@gmail.com>
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

#define EIGEN_NO_STATIC_ASSERT

#include "main.h"

template<typename ArrayType> void vectorwiseop_array(const ArrayType& m)
{
  typedef typename ArrayType::Index Index;
  typedef typename ArrayType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Array<Scalar, ArrayType::RowsAtCompileTime, 1> ColVectorType;
  typedef Array<Scalar, 1, ArrayType::ColsAtCompileTime> RowVectorType;

  Index rows = m.rows();
  Index cols = m.cols();
  Index r = internal::random<Index>(0, rows-1),
        c = internal::random<Index>(0, cols-1);

  ArrayType m1 = ArrayType::Random(rows, cols),
            m2(rows, cols),
            m3(rows, cols);

  ColVectorType colvec = ColVectorType::Random(rows);
  RowVectorType rowvec = RowVectorType::Random(cols);

  // test addition

  m2 = m1;
  m2.colwise() += colvec;
  VERIFY_IS_APPROX(m2, m1.colwise() + colvec);
  VERIFY_IS_APPROX(m2.col(c), m1.col(c) + colvec);

  VERIFY_RAISES_ASSERT(m2.colwise() += colvec.transpose());
  VERIFY_RAISES_ASSERT(m1.colwise() + colvec.transpose());

  m2 = m1;
  m2.rowwise() += rowvec;
  VERIFY_IS_APPROX(m2, m1.rowwise() + rowvec);
  VERIFY_IS_APPROX(m2.row(r), m1.row(r) + rowvec);

  VERIFY_RAISES_ASSERT(m2.rowwise() += rowvec.transpose());
  VERIFY_RAISES_ASSERT(m1.rowwise() + rowvec.transpose());

  // test substraction

  m2 = m1;
  m2.colwise() -= colvec;
  VERIFY_IS_APPROX(m2, m1.colwise() - colvec);
  VERIFY_IS_APPROX(m2.col(c), m1.col(c) - colvec);

  VERIFY_RAISES_ASSERT(m2.colwise() -= colvec.transpose());
  VERIFY_RAISES_ASSERT(m1.colwise() - colvec.transpose());

  m2 = m1;
  m2.rowwise() -= rowvec;
  VERIFY_IS_APPROX(m2, m1.rowwise() - rowvec);
  VERIFY_IS_APPROX(m2.row(r), m1.row(r) - rowvec);

  VERIFY_RAISES_ASSERT(m2.rowwise() -= rowvec.transpose());
  VERIFY_RAISES_ASSERT(m1.rowwise() - rowvec.transpose());

  // test multiplication

  m2 = m1;
  m2.colwise() *= colvec;
  VERIFY_IS_APPROX(m2, m1.colwise() * colvec);
  VERIFY_IS_APPROX(m2.col(c), m1.col(c) * colvec);

  VERIFY_RAISES_ASSERT(m2.colwise() *= colvec.transpose());
  VERIFY_RAISES_ASSERT(m1.colwise() * colvec.transpose());

  m2 = m1;
  m2.rowwise() *= rowvec;
  VERIFY_IS_APPROX(m2, m1.rowwise() * rowvec);
  VERIFY_IS_APPROX(m2.row(r), m1.row(r) * rowvec);

  VERIFY_RAISES_ASSERT(m2.rowwise() *= rowvec.transpose());
  VERIFY_RAISES_ASSERT(m1.rowwise() * rowvec.transpose());

  // test quotient

  m2 = m1;
  m2.colwise() /= colvec;
  VERIFY_IS_APPROX(m2, m1.colwise() / colvec);
  VERIFY_IS_APPROX(m2.col(c), m1.col(c) / colvec);

  VERIFY_RAISES_ASSERT(m2.colwise() /= colvec.transpose());
  VERIFY_RAISES_ASSERT(m1.colwise() / colvec.transpose());

  m2 = m1;
  m2.rowwise() /= rowvec;
  VERIFY_IS_APPROX(m2, m1.rowwise() / rowvec);
  VERIFY_IS_APPROX(m2.row(r), m1.row(r) / rowvec);

  VERIFY_RAISES_ASSERT(m2.rowwise() /= rowvec.transpose());
  VERIFY_RAISES_ASSERT(m1.rowwise() / rowvec.transpose());
}

template<typename MatrixType> void vectorwiseop_matrix(const MatrixType& m)
{
  typedef typename MatrixType::Index Index;
  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, 1> ColVectorType;
  typedef Matrix<Scalar, 1, MatrixType::ColsAtCompileTime> RowVectorType;

  Index rows = m.rows();
  Index cols = m.cols();
  Index r = internal::random<Index>(0, rows-1),
        c = internal::random<Index>(0, cols-1);

  MatrixType m1 = MatrixType::Random(rows, cols),
            m2(rows, cols),
            m3(rows, cols);

  ColVectorType colvec = ColVectorType::Random(rows);
  RowVectorType rowvec = RowVectorType::Random(cols);

  // test addition

  m2 = m1;
  m2.colwise() += colvec;
  VERIFY_IS_APPROX(m2, m1.colwise() + colvec);
  VERIFY_IS_APPROX(m2.col(c), m1.col(c) + colvec);

  VERIFY_RAISES_ASSERT(m2.colwise() += colvec.transpose());
  VERIFY_RAISES_ASSERT(m1.colwise() + colvec.transpose());

  m2 = m1;
  m2.rowwise() += rowvec;
  VERIFY_IS_APPROX(m2, m1.rowwise() + rowvec);
  VERIFY_IS_APPROX(m2.row(r), m1.row(r) + rowvec);

  VERIFY_RAISES_ASSERT(m2.rowwise() += rowvec.transpose());
  VERIFY_RAISES_ASSERT(m1.rowwise() + rowvec.transpose());

  // test substraction

  m2 = m1;
  m2.colwise() -= colvec;
  VERIFY_IS_APPROX(m2, m1.colwise() - colvec);
  VERIFY_IS_APPROX(m2.col(c), m1.col(c) - colvec);

  VERIFY_RAISES_ASSERT(m2.colwise() -= colvec.transpose());
  VERIFY_RAISES_ASSERT(m1.colwise() - colvec.transpose());

  m2 = m1;
  m2.rowwise() -= rowvec;
  VERIFY_IS_APPROX(m2, m1.rowwise() - rowvec);
  VERIFY_IS_APPROX(m2.row(r), m1.row(r) - rowvec);

  VERIFY_RAISES_ASSERT(m2.rowwise() -= rowvec.transpose());
  VERIFY_RAISES_ASSERT(m1.rowwise() - rowvec.transpose());
}

void test_vectorwiseop()
{
  CALL_SUBTEST_1(vectorwiseop_array(Array22cd()));
  CALL_SUBTEST_2(vectorwiseop_array(Array<double, 3, 2>()));
  CALL_SUBTEST_3(vectorwiseop_array(ArrayXXf(3, 4)));
  CALL_SUBTEST_4(vectorwiseop_matrix(Matrix4cf()));
  CALL_SUBTEST_5(vectorwiseop_matrix(Matrix<float,4,5>()));
  CALL_SUBTEST_6(vectorwiseop_matrix(MatrixXd(7,2)));
}
