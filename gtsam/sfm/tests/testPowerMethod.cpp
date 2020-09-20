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

#include <gtsam/sfm/PowerMethod.h>
#include <gtsam/sfm/ShonanAveraging.h>

#include <CppUnitLite/TestHarness.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <random>

using namespace std;
using namespace gtsam;

ShonanAveraging3 fromExampleName(
    const std::string &name,
    ShonanAveraging3::Parameters parameters = ShonanAveraging3::Parameters()) {
  string g2oFile = findExampleDataFile(name);
  return ShonanAveraging3(g2oFile, parameters);
}

static const ShonanAveraging3 kShonan = fromExampleName("toyExample.g2o");

/* ************************************************************************* */
TEST(PowerMethod, powerIteration) {
  // test power accelerated iteration
  gtsam::Sparse A(6, 6);
  A.coeffRef(0, 0) = 6;
  Matrix S = Matrix66::Zero();
  PowerFunctor apf(A, S, 1, A.rows(), true);
  apf.init();
  apf.compute(20, 1e-4);
  EXPECT_LONGS_EQUAL(6, apf.eigenvectors().cols());
  EXPECT_LONGS_EQUAL(6, apf.eigenvectors().rows());

  const Vector6 x1 = (Vector(6) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
  Vector6 actual0 = apf.eigenvectors().col(0);
  actual0(0) = abs(actual0(0));
  EXPECT(assert_equal(x1, actual0));

  const double ev1 = 6.0;
  EXPECT_DOUBLES_EQUAL(ev1, apf.eigenvalues()(0), 1e-5);

  // test power iteration, beta is set to 0
  PowerFunctor pf(A, S, 1, A.rows());
  pf.init();
  pf.compute(20, 1e-4);
  // for power method, only 5 ritz vectors converge with 20 iteration
  EXPECT_LONGS_EQUAL(5, pf.eigenvectors().cols());
  EXPECT_LONGS_EQUAL(6, pf.eigenvectors().rows());

  Vector6 actual1 = apf.eigenvectors().col(0);
  actual1(0) = abs(actual1(0));
  EXPECT(assert_equal(x1, actual1));

  EXPECT_DOUBLES_EQUAL(ev1, pf.eigenvalues()(0), 1e-5);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
