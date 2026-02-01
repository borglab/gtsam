/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * testFrobeniusFactor.cpp
 *
 * @file   testFrobeniusFactor.cpp
 * @date   March 2019
 * @author Frank Dellaert
 * @brief  Check evaluateError for various Frobenius norm
 */

#include <gtsam/base/lieProxies.h>
#include <gtsam/base/testLie.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/SO4.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/sfm/ShonanFactor.h>
#include <gtsam/slam/FrobeniusFactor.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

//******************************************************************************
namespace so3 {
SO3 id;
Vector3 v1 = (Vector(3) << 0.1, 0, 0).finished();
SO3 R1 = SO3::Expmap(v1);
Vector3 v2 = (Vector(3) << 0.01, 0.02, 0.03).finished();
SO3 R2 = SO3::Expmap(v2);
SO3 R12 = R1.between(R2);
} // namespace so3

//******************************************************************************
namespace submanifold {
SO4 id;
Vector6 v1 = (Vector(6) << 0, 0, 0, 0.1, 0, 0).finished();
SO3 R1 = SO3::Expmap(v1.tail<3>());
SO4 Q1 = SO4::Expmap(v1);
Vector6 v2 = (Vector(6) << 0, 0, 0, 0.01, 0.02, 0.03).finished();
SO3 R2 = SO3::Expmap(v2.tail<3>());
SO4 Q2 = SO4::Expmap(v2);
SO3 R12 = R1.between(R2);
} // namespace submanifold

/* ************************************************************************* */
TEST(ShonanFactor3, evaluateError) {
  auto model = noiseModel::Isotropic::Sigma(3, 1.2);
  for (const size_t p : {5, 4, 3}) {
    Matrix M = Matrix::Identity(p, p);
    M.topLeftCorner(3, 3) = submanifold::R1.matrix();
    SOn Q1(M);
    M.topLeftCorner(3, 3) = submanifold::R2.matrix();
    SOn Q2(M);
    auto factor = ShonanFactor3(1, 2, Rot3(::so3::R12.matrix()), p, model);
    Matrix H1, H2;
    factor.evaluateError(Q1, Q2, H1, H2);

    // Test derivatives
    Values values;
    values.insert(1, Q1);
    values.insert(2, Q2);
    EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
  }
}

/* ************************************************************************* */
TEST(ShonanFactor3, equivalenceToSO3) {
  using namespace ::submanifold;
  auto R12 = ::so3::R12.retract(Vector3(0.1, 0.2, -0.1));
  auto model = noiseModel::Isotropic::Sigma(3, 1.2);
  auto factor3 = FrobeniusBetweenFactor<SO3>(1, 2, R12, model);
  auto factor4 = ShonanFactor3(1, 2, Rot3(R12.matrix()), 4, model);
  const Matrix3 E3(factor3.evaluateError(R1, R2).data());
  const Matrix43 E4(
      factor4.evaluateError(SOn(Q1.matrix()), SOn(Q2.matrix())).data());
  EXPECT(assert_equal((Matrix)E4.topLeftCorner<3, 3>(), E3, 1e-9));
  EXPECT(assert_equal((Matrix)E4.row(3), Matrix13::Zero(), 1e-9));
}

/* ************************************************************************* */
TEST(ShonanFactor2, evaluateError) {
  auto model = noiseModel::Isotropic::Sigma(1, 1.2);
  const Rot2 R1(0.3), R2(0.5), R12(0.2);
  for (const size_t p : {5, 4, 3, 2}) {
    Matrix M = Matrix::Identity(p, p);
    M.topLeftCorner(2, 2) = R1.matrix();
    SOn Q1(M);
    M.topLeftCorner(2, 2) = R2.matrix();
    SOn Q2(M);
    auto factor = ShonanFactor2(1, 2, R12, p, model);
    Matrix H1, H2;
    factor.evaluateError(Q1, Q2, H1, H2);

    // Test derivatives
    Values values;
    values.insert(1, Q1);
    values.insert(2, Q2);
    EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
