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
}  // namespace so3

/* ************************************************************************* */
TEST(FrobeniusPriorSO3, evaluateError) {
  using namespace ::so3;
  auto factor = FrobeniusPrior<SO3>(1, R2.matrix());
  Vector actual = factor.evaluateError(R1);
  Vector expected = R1.vec() - R2.vec();
  EXPECT(assert_equal(expected, actual, 1e-9));

  Values values;
  values.insert(1, R1);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
TEST(FrobeniusPriorSO3, ClosestTo) {
  // Example top-left of SO(4) matrix not quite on SO(3) manifold
  Matrix3 M;
  M << 0.79067393, 0.6051136, -0.0930814,   //
      0.4155925, -0.64214347, -0.64324489,  //
      -0.44948549, 0.47046326, -0.75917576;

  SO3 expected = SO3::ClosestTo(M);

  // manifold optimization gets same result as SVD solution in ClosestTo
  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusPrior<SO3> >(1, M);

  Values initial;
  initial.insert(1, SO3(I_3x3));
  auto result = GaussNewtonOptimizer(graph, initial).optimize();
  EXPECT_DOUBLES_EQUAL(0.0, graph.error(result), 1e-6);
  EXPECT(assert_equal(expected, result.at<SO3>(1), 1e-6));
}

/* ************************************************************************* */
TEST(FrobeniusPriorSO3, ChordalL2mean) {
  // See Hartley13ijcv:
  // Cost function C(R) = \sum FrobeniusPrior(R,R_i)
  // Closed form solution = ClosestTo(C_e), where
  // C_e = \sum R_i !!!!

  // We will test by computing mean of R1=exp(v1) R1^T=exp(-v1):
  using namespace ::so3;
  SO3 expected;  // identity
  Matrix3 M = R1.matrix() + R1.matrix().transpose();
  EXPECT(assert_equal(expected, SO3::ClosestTo(M), 1e-6));
  EXPECT(assert_equal(expected, SO3::ChordalMean({R1, R1.inverse()}), 1e-6));

  // manifold optimization gets same result as ChordalMean
  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusPrior<SO3> >(1, R1.matrix());
  graph.emplace_shared<FrobeniusPrior<SO3> >(1, R1.matrix().transpose());

  Values initial;
  initial.insert<SO3>(1, R1.inverse());
  auto result = GaussNewtonOptimizer(graph, initial).optimize();
  EXPECT_DOUBLES_EQUAL(0.0, graph.error(result), 0.1);  // Why so loose?
  EXPECT(assert_equal(expected, result.at<SO3>(1), 1e-5));
}

/* ************************************************************************* */
TEST(FrobeniusFactorSO3, evaluateError) {
  using namespace ::so3;
  auto factor = FrobeniusFactor<SO3>(1, 2);
  Vector actual = factor.evaluateError(R1, R2);
  Vector expected = R2.vec() - R1.vec();
  EXPECT(assert_equal(expected, actual, 1e-9));

  Values values;
  values.insert(1, R1);
  values.insert(2, R2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
// Commented out as SO(n) not yet supported (and might never be)
// TEST(FrobeniusBetweenFactorSOn, evaluateError) {
//   using namespace ::so3;
//   auto factor =
//       FrobeniusBetweenFactor<SOn>(1, 2, SOn::FromMatrix(R12.matrix()));
//   Vector actual = factor.evaluateError(SOn::FromMatrix(R1.matrix()),
//                                        SOn::FromMatrix(R2.matrix()));
//   Vector expected = Vector9::Zero();
//   EXPECT(assert_equal(expected, actual, 1e-9));

//   Values values;
//   values.insert(1, R1);
//   values.insert(2, R2);
//   EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
// }

/* ************************************************************************* */
TEST(FrobeniusBetweenFactorSO3, evaluateError) {
  using namespace ::so3;
  auto factor = FrobeniusBetweenFactor<SO3>(1, 2, R12);
  Vector actual = factor.evaluateError(R1, R2);
  Vector expected = Vector9::Zero();
  EXPECT(assert_equal(expected, actual, 1e-9));

  Values values;
  values.insert(1, R1);
  values.insert(2, R2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

//******************************************************************************
namespace so4 {
SO4 id;
Vector6 v1 = (Vector(6) << 0.1, 0, 0, 0, 0, 0).finished();
SO4 Q1 = SO4::Expmap(v1);
Vector6 v2 = (Vector(6) << 0.01, 0.02, 0.03, 0.04, 0.05, 0.06).finished();
SO4 Q2 = SO4::Expmap(v2);
}  // namespace so4

/* ************************************************************************* */
TEST(FrobeniusFactorSO4, evaluateError) {
  using namespace ::so4;
  auto factor = FrobeniusFactor<SO4>(1, 2, noiseModel::Unit::Create(6));
  Vector actual = factor.evaluateError(Q1, Q2);
  Vector expected = Q2.vec() - Q1.vec();
  EXPECT(assert_equal(expected, actual, 1e-9));

  Values values;
  values.insert(1, Q1);
  values.insert(2, Q2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
TEST(FrobeniusBetweenFactorSO4, evaluateError) {
  using namespace ::so4;
  Matrix4 M{I_4x4};
  M.topLeftCorner<3, 3>() = ::so3::R12.matrix();
  auto factor = FrobeniusBetweenFactor<SO4>(1, 2, Q1.between(Q2));
  Matrix H1, H2;
  Vector actual = factor.evaluateError(Q1, Q2, H1, H2);
  Vector expected = SO4::VectorN2::Zero();
  EXPECT(assert_equal(expected, actual, 1e-9));

  Values values;
  values.insert(1, Q1);
  values.insert(2, Q2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

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
}  // namespace submanifold

/* ************************************************************************* */
TEST(FrobeniusWormholeFactor, evaluateError) {
  auto model = noiseModel::Isotropic::Sigma(6, 1.2);  // dimension = 6 not 16
  for (const size_t p : {5, 4, 3}) {
    Matrix M = Matrix::Identity(p, p);
    M.topLeftCorner(3, 3) = submanifold::R1.matrix();
    SOn Q1(M);
    M.topLeftCorner(3, 3) = submanifold::R2.matrix();
    SOn Q2(M);
    auto factor =
        FrobeniusWormholeFactor(1, 2, Rot3(::so3::R12.matrix()), p, model);
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
TEST(FrobeniusWormholeFactor, equivalenceToSO3) {
  using namespace ::submanifold;
  auto R12 = ::so3::R12.retract(Vector3(0.1, 0.2, -0.1));
  auto model = noiseModel::Isotropic::Sigma(6, 1.2);  // wrong dimension
  auto factor3 = FrobeniusBetweenFactor<SO3>(1, 2, R12, model);
  auto factor4 = FrobeniusWormholeFactor(1, 2, Rot3(R12.matrix()), 4, model);
  const Matrix3 E3(factor3.evaluateError(R1, R2).data());
  const Matrix43 E4(
      factor4.evaluateError(SOn(Q1.matrix()), SOn(Q2.matrix())).data());
  EXPECT(assert_equal((Matrix)E4.topLeftCorner<3, 3>(), E3, 1e-9));
  EXPECT(assert_equal((Matrix)E4.row(3), Matrix13::Zero(), 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
