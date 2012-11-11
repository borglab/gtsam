// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2008 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2010 Jitse Niesen <jitse@maths.leeds.ac.uk>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "main.h"
#include <limits>
#include <Eigen/Eigenvalues>

template<typename MatrixType> void selfadjointeigensolver(const MatrixType& m)
{
  typedef typename MatrixType::Index Index;
  /* this test covers the following files:
     EigenSolver.h, SelfAdjointEigenSolver.h (and indirectly: Tridiagonalization.h)
  */
  Index rows = m.rows();
  Index cols = m.cols();

  typedef typename MatrixType::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real RealScalar;
  typedef Matrix<Scalar, MatrixType::RowsAtCompileTime, 1> VectorType;
  typedef Matrix<RealScalar, MatrixType::RowsAtCompileTime, 1> RealVectorType;
  typedef typename std::complex<typename NumTraits<typename MatrixType::Scalar>::Real> Complex;

  RealScalar largerEps = 10*test_precision<RealScalar>();

  MatrixType a = MatrixType::Random(rows,cols);
  MatrixType a1 = MatrixType::Random(rows,cols);
  MatrixType symmA =  a.adjoint() * a + a1.adjoint() * a1;
  symmA.template triangularView<StrictlyUpper>().setZero();

  MatrixType b = MatrixType::Random(rows,cols);
  MatrixType b1 = MatrixType::Random(rows,cols);
  MatrixType symmB = b.adjoint() * b + b1.adjoint() * b1;
  symmB.template triangularView<StrictlyUpper>().setZero();

  SelfAdjointEigenSolver<MatrixType> eiSymm(symmA);
  SelfAdjointEigenSolver<MatrixType> eiDirect;
  eiDirect.computeDirect(symmA);
  // generalized eigen pb
  GeneralizedSelfAdjointEigenSolver<MatrixType> eiSymmGen(symmA, symmB);

  VERIFY_IS_EQUAL(eiSymm.info(), Success);
  VERIFY((symmA.template selfadjointView<Lower>() * eiSymm.eigenvectors()).isApprox(
          eiSymm.eigenvectors() * eiSymm.eigenvalues().asDiagonal(), largerEps));
  VERIFY_IS_APPROX(symmA.template selfadjointView<Lower>().eigenvalues(), eiSymm.eigenvalues());
  
  VERIFY_IS_EQUAL(eiDirect.info(), Success);
  VERIFY((symmA.template selfadjointView<Lower>() * eiDirect.eigenvectors()).isApprox(
          eiDirect.eigenvectors() * eiDirect.eigenvalues().asDiagonal(), largerEps));
  VERIFY_IS_APPROX(symmA.template selfadjointView<Lower>().eigenvalues(), eiDirect.eigenvalues());

  SelfAdjointEigenSolver<MatrixType> eiSymmNoEivecs(symmA, false);
  VERIFY_IS_EQUAL(eiSymmNoEivecs.info(), Success);
  VERIFY_IS_APPROX(eiSymm.eigenvalues(), eiSymmNoEivecs.eigenvalues());
  
  // generalized eigen problem Ax = lBx
  eiSymmGen.compute(symmA, symmB,Ax_lBx);
  VERIFY_IS_EQUAL(eiSymmGen.info(), Success);
  VERIFY((symmA.template selfadjointView<Lower>() * eiSymmGen.eigenvectors()).isApprox(
          symmB.template selfadjointView<Lower>() * (eiSymmGen.eigenvectors() * eiSymmGen.eigenvalues().asDiagonal()), largerEps));

  // generalized eigen problem BAx = lx
  eiSymmGen.compute(symmA, symmB,BAx_lx);
  VERIFY_IS_EQUAL(eiSymmGen.info(), Success);
  VERIFY((symmB.template selfadjointView<Lower>() * (symmA.template selfadjointView<Lower>() * eiSymmGen.eigenvectors())).isApprox(
         (eiSymmGen.eigenvectors() * eiSymmGen.eigenvalues().asDiagonal()), largerEps));

  // generalized eigen problem ABx = lx
  eiSymmGen.compute(symmA, symmB,ABx_lx);
  VERIFY_IS_EQUAL(eiSymmGen.info(), Success);
  VERIFY((symmA.template selfadjointView<Lower>() * (symmB.template selfadjointView<Lower>() * eiSymmGen.eigenvectors())).isApprox(
         (eiSymmGen.eigenvectors() * eiSymmGen.eigenvalues().asDiagonal()), largerEps));


  MatrixType sqrtSymmA = eiSymm.operatorSqrt();
  VERIFY_IS_APPROX(MatrixType(symmA.template selfadjointView<Lower>()), sqrtSymmA*sqrtSymmA);
  VERIFY_IS_APPROX(sqrtSymmA, symmA.template selfadjointView<Lower>()*eiSymm.operatorInverseSqrt());

  MatrixType id = MatrixType::Identity(rows, cols);
  VERIFY_IS_APPROX(id.template selfadjointView<Lower>().operatorNorm(), RealScalar(1));

  SelfAdjointEigenSolver<MatrixType> eiSymmUninitialized;
  VERIFY_RAISES_ASSERT(eiSymmUninitialized.info());
  VERIFY_RAISES_ASSERT(eiSymmUninitialized.eigenvalues());
  VERIFY_RAISES_ASSERT(eiSymmUninitialized.eigenvectors());
  VERIFY_RAISES_ASSERT(eiSymmUninitialized.operatorSqrt());
  VERIFY_RAISES_ASSERT(eiSymmUninitialized.operatorInverseSqrt());

  eiSymmUninitialized.compute(symmA, false);
  VERIFY_RAISES_ASSERT(eiSymmUninitialized.eigenvectors());
  VERIFY_RAISES_ASSERT(eiSymmUninitialized.operatorSqrt());
  VERIFY_RAISES_ASSERT(eiSymmUninitialized.operatorInverseSqrt());

  // test Tridiagonalization's methods
  Tridiagonalization<MatrixType> tridiag(symmA);
  // FIXME tridiag.matrixQ().adjoint() does not work
  VERIFY_IS_APPROX(MatrixType(symmA.template selfadjointView<Lower>()), tridiag.matrixQ() * tridiag.matrixT().eval() * MatrixType(tridiag.matrixQ()).adjoint());
  
  if (rows > 1)
  {
    // Test matrix with NaN
    symmA(0,0) = std::numeric_limits<typename MatrixType::RealScalar>::quiet_NaN();
    SelfAdjointEigenSolver<MatrixType> eiSymmNaN(symmA);
    VERIFY_IS_EQUAL(eiSymmNaN.info(), NoConvergence);
  }
}

void test_eigensolver_selfadjoint()
{
  int s;
  for(int i = 0; i < g_repeat; i++) {
    // very important to test 3x3 and 2x2 matrices since we provide special paths for them
    CALL_SUBTEST_1( selfadjointeigensolver(Matrix2d()) );
    CALL_SUBTEST_1( selfadjointeigensolver(Matrix3f()) );
    CALL_SUBTEST_2( selfadjointeigensolver(Matrix4d()) );
    s = internal::random<int>(1,EIGEN_TEST_MAX_SIZE/4);
    CALL_SUBTEST_3( selfadjointeigensolver(MatrixXf(s,s)) );
    s = internal::random<int>(1,EIGEN_TEST_MAX_SIZE/4);
    CALL_SUBTEST_4( selfadjointeigensolver(MatrixXd(s,s)) );
    s = internal::random<int>(1,EIGEN_TEST_MAX_SIZE/4);
    CALL_SUBTEST_5( selfadjointeigensolver(MatrixXcd(s,s)) );
    
    s = internal::random<int>(1,EIGEN_TEST_MAX_SIZE/4);
    CALL_SUBTEST_9( selfadjointeigensolver(Matrix<std::complex<double>,Dynamic,Dynamic,RowMajor>(s,s)) );

    // some trivial but implementation-wise tricky cases
    CALL_SUBTEST_4( selfadjointeigensolver(MatrixXd(1,1)) );
    CALL_SUBTEST_4( selfadjointeigensolver(MatrixXd(2,2)) );
    CALL_SUBTEST_6( selfadjointeigensolver(Matrix<double,1,1>()) );
    CALL_SUBTEST_7( selfadjointeigensolver(Matrix<double,2,2>()) );
  }

  // Test problem size constructors
  s = internal::random<int>(1,EIGEN_TEST_MAX_SIZE/4);
  CALL_SUBTEST_8(SelfAdjointEigenSolver<MatrixXf>(s));
  CALL_SUBTEST_8(Tridiagonalization<MatrixXf>(s));
  
  EIGEN_UNUSED_VARIABLE(s)
}

