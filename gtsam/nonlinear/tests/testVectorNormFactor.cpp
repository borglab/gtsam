/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testVectorNormFactor.cpp
 * @brief Unit tests for VectorNormFactor
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/VectorNormFactor.h>

#if GTSAM_ENABLE_BOOST_SERIALIZATION
#include <gtsam/base/serializationTestHelpers.h>
#endif

using namespace std;
using namespace gtsam;
using symbol_shorthand::G;

static const SharedNoiseModel kModel = noiseModel::Isotropic::Sigma(1, 0.1);

/* ************************************************************************* */
TEST(VectorNormFactor, Error) {
  VectorNormFactor<3> factor(G(0), 5.0, kModel);
  const Vector3 v(3.0, 0.0, 4.0);  // norm 5
  EXPECT(assert_equal(Vector1(0.0), factor.evaluateError(v)));
  EXPECT(assert_equal(Vector1(1.0), factor.evaluateError(Vector3(0, 0, 6))));
}

/* ************************************************************************* */
TEST(VectorNormFactor, Jacobian) {
  VectorNormFactor<3> factor(G(0), 9.81, kModel);
  const Vector3 v(0.3, -0.2, -9.7);
  Matrix actualH;
  factor.evaluateError(v, actualH);
  const Matrix expectedH = numericalDerivative11<Vector, Vector3>(
      [&factor](const Vector3& x) { return factor.evaluateError(x); }, v);
  EXPECT(assert_equal(expectedH, actualH, 1e-5));
}

/* ************************************************************************* */
TEST(VectorNormFactor, JacobianDimension2) {
  VectorNormFactor<2> factor(G(0), 2.0, kModel);
  const Vector2 v(1.0, -1.5);
  Matrix actualH;
  factor.evaluateError(v, actualH);
  const Matrix expectedH = numericalDerivative11<Vector, Vector2>(
      [&factor](const Vector2& x) { return factor.evaluateError(x); }, v);
  EXPECT(assert_equal(expectedH, actualH, 1e-5));
}

/* ************************************************************************* */
TEST(VectorNormFactor, ZeroVector) {
  // The norm is not differentiable at the origin: the error must still be
  // finite and the Jacobian zero (no NaNs).
  VectorNormFactor<3> factor(G(0), 9.81, kModel);
  Matrix actualH;
  const Vector error = factor.evaluateError(Vector3::Zero(), actualH);
  EXPECT(assert_equal(Vector1(-9.81), error));
  EXPECT(assert_equal(Matrix(Matrix::Zero(1, 3)), actualH));
}

/* ************************************************************************* */
TEST(VectorNormFactor, Optimization) {
  // A loose vector prior fixes the direction; the norm factor should pull the
  // magnitude to the target along that direction.
  NonlinearFactorGraph graph;
  const Vector3 direction = Vector3(1.0, 1.0, 1.0).normalized();
  graph.emplace_shared<PriorFactor<Vector3>>(
      G(0), Vector3(8.0 * direction), noiseModel::Isotropic::Sigma(3, 10.0));
  graph.emplace_shared<VectorNormFactor<3>>(
      G(0), 9.81, noiseModel::Isotropic::Sigma(1, 1e-3));

  Values initial;
  initial.insert(G(0), Vector3(8.0 * direction));
  const Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();

  const Vector3 optimized = result.at<Vector3>(G(0));
  DOUBLES_EQUAL(9.81, optimized.norm(), 1e-5);
  EXPECT(assert_equal(direction, Vector3(optimized.normalized()), 1e-3));
}

/* ************************************************************************* */
TEST(VectorNormFactor, Equals) {
  VectorNormFactor<3> factor1(G(0), 9.81, kModel);
  VectorNormFactor<3> factor2(G(0), 9.81, kModel);
  VectorNormFactor<3> factor3(G(0), 1.0, kModel);
  EXPECT(factor1.equals(factor2));
  EXPECT(!factor1.equals(factor3));
  // Comparing against a different factor type must not crash:
  PriorFactor<Vector3> prior(G(0), Vector3::Zero(),
                             noiseModel::Isotropic::Sigma(3, 1.0));
  EXPECT(!factor1.equals(prior));
}

/* ************************************************************************* */
#if GTSAM_ENABLE_BOOST_SERIALIZATION
BOOST_CLASS_EXPORT_GUID(noiseModel::Isotropic, "gtsam_noiseModel_Isotropic")

TEST(VectorNormFactor, Serialization) {
  using namespace gtsam::serializationTestHelpers;
  VectorNormFactor<3> factor(G(0), 9.81, kModel);
  EXPECT(equalsObj<VectorNormFactor<3>>(factor));
  EXPECT(equalsXML<VectorNormFactor<3>>(factor));
  EXPECT(equalsBinary<VectorNormFactor<3>>(factor));
}
#endif

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
