/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * testPowerMethod.cpp
 *
 * @file   testPowerMethod.cpp
 * @date   Sept 2020
 * @author Jing Wu
 * @brief  Check eigenvalue and eigenvector computed by power method
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/sfm/PowerMethod.h>

#include <CppUnitLite/TestHarness.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <iostream>
#include <random>

using namespace std;
using namespace gtsam;
using symbol_shorthand::X;

/* ************************************************************************* */
TEST(PowerMethod, powerIteration) {
  // test power iteration, beta is set to 0
  Sparse A(6, 6);
  A.coeffRef(0, 0) = 6;
  Matrix S = Matrix66::Zero();
  PowerMethod<Sparse> apf(A, S.row(0));
  apf.compute(20, 1e-4);
  EXPECT_LONGS_EQUAL(1, apf.eigenvectors().cols());
  EXPECT_LONGS_EQUAL(6, apf.eigenvectors().rows());

  const Vector6 x1 = (Vector(6) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
  Vector6 actual0 = apf.eigenvectors().col(0);
  actual0(0) = abs(actual0(0));
  EXPECT(assert_equal(x1, actual0));

  const double ev1 = 6.0;
  EXPECT_DOUBLES_EQUAL(ev1, apf.eigenvalues(), 1e-5);

  // test power accelerated iteration
  AcceleratedPowerMethod<Sparse> pf(A, S.row(0));
  pf.compute(20, 1e-4);
  // for power method, only 5 ritz vectors converge with 20 iterations
  EXPECT_LONGS_EQUAL(1, pf.eigenvectors().cols());
  EXPECT_LONGS_EQUAL(6, pf.eigenvectors().rows());

  Vector6 actual1 = apf.eigenvectors().col(0);
  actual1(0) = abs(actual1(0));
  EXPECT(assert_equal(x1, actual1));

  EXPECT_DOUBLES_EQUAL(ev1, pf.eigenvalues(), 1e-5);
}

/* ************************************************************************* */
TEST(PowerMethod, useFactorGraph) {
  // Let's make a scalar synchronization graph with 4 nodes
  GaussianFactorGraph fg;
  auto model = noiseModel::Unit::Create(1);
  for (size_t j = 0; j < 3; j++) {
    fg.add(X(j), -I_1x1, X(j + 1), I_1x1, Vector1::Zero(), model);
  }
  fg.add(X(3), -I_1x1, X(0), I_1x1, Vector1::Zero(), model); // extra row

  // Get eigenvalues and eigenvectors with Eigen
  auto L = fg.hessian();
  cout << L.first << endl;
  Eigen::EigenSolver<Matrix> solver(L.first);
  cout << solver.eigenvalues() << endl;
  cout << solver.eigenvectors() << endl;

  // Check that we get zero eigenvalue and "constant" eigenvector
  EXPECT_DOUBLES_EQUAL(0.0, solver.eigenvalues()[0].real(), 1e-9);
  auto v0 = solver.eigenvectors().col(0);
  for (size_t j = 0; j < 3; j++)
    EXPECT_DOUBLES_EQUAL(-0.5, v0[j].real(), 1e-9);

  // test power iteration, beta is set to 0
  Matrix S = Matrix44::Zero();
  // PowerMethod<Matrix> pf(L.first, S.row(0));
  AcceleratedPowerMethod<Matrix> pf(L.first, S.row(0));
  pf.compute(20, 1e-4);
  cout << pf.eigenvalues() << endl;
  cout << pf.eigenvectors() << endl;
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
