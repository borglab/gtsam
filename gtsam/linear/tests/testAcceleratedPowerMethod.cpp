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
 * @file   testAcceleratedPowerMethod.cpp
 * @date   Sept 2020
 * @author Jing Wu
 * @brief  Check eigenvalue and eigenvector computed by accelerated power method
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/AcceleratedPowerMethod.h>

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
TEST(AcceleratedPowerMethod, acceleratedPowerIteration) {
  // test power iteration, beta is set to 0
  Sparse A(6, 6);
  A.coeffRef(0, 0) = 6;
  A.coeffRef(0, 0) = 5;
  A.coeffRef(0, 0) = 4;
  A.coeffRef(0, 0) = 3;
  A.coeffRef(0, 0) = 2;
  A.coeffRef(0, 0) = 1;
  Vector initial = Vector6::Zero();
  const Vector6 x1 = (Vector(6) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
  const double ev1 = 1.0;

  // test accelerated power iteration
  AcceleratedPowerMethod<Sparse> apf(A, initial);
  apf.compute(20, 1e-4);
  EXPECT_LONGS_EQUAL(1, apf.eigenvectors().cols());
  EXPECT_LONGS_EQUAL(6, apf.eigenvectors().rows());

  Vector6 actual1 = apf.eigenvectors();
  // actual1(0) = abs (actual1(0));
  EXPECT(assert_equal(x1, actual1));

  EXPECT_DOUBLES_EQUAL(ev1, apf.eigenvalues(), 1e-5);
}

/* ************************************************************************* */
TEST(AcceleratedPowerMethod, useFactorGraph) {
  // Let's make a scalar synchronization graph with 4 nodes
  GaussianFactorGraph fg;
  auto model = noiseModel::Unit::Create(1);
  for (size_t j = 0; j < 3; j++) {
    fg.add(X(j), -I_1x1, X(j + 1), I_1x1, Vector1::Zero(), model);
  }
  fg.add(X(3), -I_1x1, X(0), I_1x1, Vector1::Zero(), model); // extra row

  // Get eigenvalues and eigenvectors with Eigen
  auto L = fg.hessian();
  Eigen::EigenSolver<Matrix> solver(L.first);

  // Check that we get zero eigenvalue and "constant" eigenvector
  EXPECT_DOUBLES_EQUAL(0.0, solver.eigenvalues()[0].real(), 1e-9);
  auto v0 = solver.eigenvectors().col(0);
  for (size_t j = 0; j < 3; j++)
    EXPECT_DOUBLES_EQUAL(-0.5, v0[j].real(), 1e-9);

  size_t maxIdx = 0;
  for (auto i =0; i<solver.eigenvalues().rows(); ++i) {
    if (solver.eigenvalues()(i).real() >= solver.eigenvalues()(maxIdx).real()) maxIdx = i;
  }
  // Store the max eigenvalue and its according eigenvector
  const auto ev1 = solver.eigenvalues()(maxIdx).real();
  auto ev2 = solver.eigenvectors().col(maxIdx).real();

  Vector initial = Vector4::Zero();
  AcceleratedPowerMethod<Matrix> apf(L.first, initial);
  apf.compute(20, 1e-4);
  EXPECT_DOUBLES_EQUAL(ev1, apf.eigenvalues(), 1e-8);
  EXPECT(assert_equal(ev2, apf.eigenvectors(), 3e-5));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
