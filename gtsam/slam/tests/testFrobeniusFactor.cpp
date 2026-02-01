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
  * @brief  Check evaluateError for various Frobenius norms
  */

#include <gtsam/base/lieProxies.h>
#include <gtsam/base/testLie.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Similarity2.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/SO4.h>
#include <gtsam/geometry/Gal3.h>
#include <gtsam/geometry/SL4.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/FrobeniusFactor.h>

#include <CppUnitLite/TestHarness.h>

using namespace gtsam;

/* ************************************************************************* */
namespace so3 {
  SO3 id;
  Vector3 v1 = (Vector(3) << 0.1, 0, 0).finished();
  SO3 R1 = SO3::Expmap(v1);
  Vector3 v2 = (Vector(3) << 0.01, 0.02, 0.03).finished();
  SO3 R2 = SO3::Expmap(v2);
  SO3 R12 = R1.between(R2);
}  // namespace so3

/* ************************************************************************* */
TEST(FrobeniusPriorSO3, EvaluateError) {
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
  EXPECT(assert_equal(expected, SO3::ChordalMean({ R1, R1.inverse() }), 1e-6));

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
TEST(FrobeniusFactorSO3, EvaluateError) {
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
// TEST(FrobeniusBetweenFactorSOn, EvaluateError) {
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
TEST(FrobeniusBetweenFactorSO3, EvaluateError) {
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

/* ************************************************************************* */
namespace so4 {
  SO4 id;
  Vector6 v1 = (Vector(6) << 0.1, 0, 0, 0, 0, 0).finished();
  SO4 Q1 = SO4::Expmap(v1);
  Vector6 v2 = (Vector(6) << 0.01, 0.02, 0.03, 0.04, 0.05, 0.06).finished();
  SO4 Q2 = SO4::Expmap(v2);
}  // namespace so4

/* ************************************************************************* */
TEST(FrobeniusFactorSO4, EvaluateError) {
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
TEST(FrobeniusBetweenFactorSO4, EvaluateError) {
  using namespace ::so4;
  Matrix4 M{ I_4x4 };
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

/* ************************************************************************* */
namespace pose2 {
  Pose2 id;
  Pose2 P1 = Pose2(0.1, 0.2, 0.3);
  Pose2 P2 = Pose2(0.4, 0.5, 0.6);
}  // namespace pose2

/* ************************************************************************* */
TEST(FrobeniusFactorPose2, EvaluateError) {
  using namespace ::pose2;
  auto factor = FrobeniusFactor<Pose2>(1, 2, noiseModel::Unit::Create(3));
  Vector actual = factor.evaluateError(P1, P2);
  Vector expected = P2.vec() - P1.vec();
  EXPECT(assert_equal(expected, actual, 1e-9));

  Values values;
  values.insert(1, P1);
  values.insert(2, P2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
TEST(FrobeniusBetweenFactorPose2, EvaluateError) {
  using namespace ::pose2;
  auto factor = FrobeniusBetweenFactor<Pose2>(1, 2, P1.between(P2));
  Matrix H1, H2;
  Vector actual = factor.evaluateError(P1, P2, H1, H2);
  Vector expected = Vector9::Zero();
  EXPECT(assert_equal(expected, actual, 1e-9));

  Values values;
  values.insert(1, P1);
  values.insert(2, P2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
namespace pose3 {
  Pose3 id;
  Pose3 P1 = Pose3(Rot3::Expmap(Vector3(0.1, 0.2, 0.3)), Vector3(0.4, 0.5, 0.6));
  Pose3 P2 = Pose3(Rot3::Expmap(Vector3(0.2, 0.3, 0.4)), Vector3(0.7, 0.8, 0.9));
}  // namespace pose3

/* ************************************************************************* */
TEST(FrobeniusFactorPose3, EvaluateError) {
  using namespace ::pose3;
  auto factor = FrobeniusFactor<Pose3>(1, 2, noiseModel::Unit::Create(6));
  Vector actual = factor.evaluateError(P1, P2);
  Vector expected = P2.vec() - P1.vec();
  EXPECT(assert_equal(expected, actual, 1e-9));

  Values values;
  values.insert(1, P1);
  values.insert(2, P2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
TEST(FrobeniusBetweenFactorPose3, EvaluateError) {
  using namespace ::pose3;
  auto factor = FrobeniusBetweenFactor<Pose3>(1, 2, P1.between(P2));
  Matrix H1, H2;
  Vector actual = factor.evaluateError(P1, P2, H1, H2);
  Vector expected(16);
  expected.setZero();
  EXPECT(assert_equal(expected, actual, 1e-9));

  Values values;
  values.insert(1, P1);
  values.insert(2, P2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
namespace sim2 {
  Similarity2 id;
  Similarity2 P1 = Similarity2::Expmap(Vector4(0.1, 0.2, 0.3, 0.4));
  Similarity2 P2 = Similarity2::Expmap(Vector4(0.2, 0.3, 0.4, 0.5));
}  // namespace sim2

/* ************************************************************************* */
TEST(FrobeniusFactorSimilarity2, EvaluateError) {
  using namespace ::sim2;
  auto factor = FrobeniusFactor<Similarity2>(1, 2, noiseModel::Unit::Create(9));
  Vector actual = factor.evaluateError(P1, P2);
  Vector expected = P2.vec() - P1.vec();
  EXPECT(assert_equal(expected, actual, 1e-9));

  Values values;
  values.insert(1, P1);
  values.insert(2, P2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
TEST(FrobeniusBetweenFactorNLSimilarity2, EvaluateError) {
  using namespace ::sim2;
  auto factor = FrobeniusBetweenFactorNL<Similarity2>(1, 2, P1.between(P2));
  Matrix H1, H2;
  Vector actual = factor.evaluateError(P1, P2, H1, H2);
  Vector expected(9);
  expected.setZero();
  EXPECT(assert_equal(expected, actual, 1e-9));

  Values values;
  values.insert(1, P1);
  values.insert(2, P2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
TEST(FrobeniusBetweenFactorNLSimilarity2, Optimization) {
  using namespace ::sim2;
  auto factor = FrobeniusBetweenFactorNL<Similarity2>(1, 2, P1.between(P2));

  NonlinearFactorGraph graph;
  graph.add(factor);
  graph.add(PriorFactor<Similarity2>(1, P1, noiseModel::Constrained::All(4)));

  Values initial;
  initial.insert(1, P1);
  initial.insert(2, P2.retract(Vector4::Constant(0.1)));

  double initial_error = graph.error(initial);

  GaussNewtonParams params;
  // params.setVerbosity("ERROR");
  GaussNewtonOptimizer optimizer(graph, initial, params);
  Values result = optimizer.optimize();

  double final_error = graph.error(result);

  EXPECT(final_error < initial_error);
}

/* ************************************************************************* */
namespace sim3 {
  Similarity3 id;
  Similarity3 P1 = Similarity3::Expmap(Vector7(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7));
  Similarity3 P2 = Similarity3::Expmap(Vector7(0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8));
}  // namespace sim3

/* ************************************************************************* */
TEST(FrobeniusFactorSimilarity3, EvaluateError) {
  using namespace ::sim3;
  auto factor = FrobeniusFactor<Similarity3>(1, 2, noiseModel::Unit::Create(16));
  Vector actual = factor.evaluateError(P1, P2);
  Vector expected = P2.vec() - P1.vec();
  EXPECT(assert_equal(expected, actual, 1e-9));

  Values values;
  values.insert(1, P1);
  values.insert(2, P2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
TEST(FrobeniusBetweenFactorNLSimilarity3, EvaluateError) {
  using namespace ::sim3;
  auto factor = FrobeniusBetweenFactorNL<Similarity3>(1, 2, P1.between(P2));
  Matrix H1, H2;
  Vector actual = factor.evaluateError(P1, P2, H1, H2);
  Vector expected(16);
  expected.setZero();
  EXPECT(assert_equal(expected, actual, 1e-9));

  Values values;
  values.insert(1, P1);
  values.insert(2, P2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
namespace gal3 {
  Gal3 id;
  Gal3 G1(Rot3::Rz(0.1), Point3(0.2, 0.3, 0.4), Velocity3(0.5, 0.6, 0.7), 0.8);
  Gal3 G2(Rot3::Rz(0.2), Point3(0.3, 0.4, 0.5), Velocity3(0.6, 0.7, 0.8), 0.9);
}  // namespace gal3

/* ************************************************************************* */
TEST(FrobeniusFactorGal3, EvaluateError) {
  using namespace ::gal3;
  auto factor = FrobeniusFactor<Gal3>(1, 2, noiseModel::Unit::Create(25));
  Vector actual = factor.evaluateError(G1, G2);
  Vector expected = G2.vec() - G1.vec();
  EXPECT(assert_equal(expected, actual, 1e-9));

  Values values;
  values.insert(1, G1);
  values.insert(2, G2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
TEST(FrobeniusBetweenFactorGal3, EvaluateError) {
  using namespace ::gal3;
  auto factor = FrobeniusBetweenFactor<Gal3>(1, 2, G1.between(G2));
  Matrix H1, H2;
  Vector actual = factor.evaluateError(G1, G2, H1, H2);
  Vector expected(25);
  expected.setZero();
  EXPECT(assert_equal(expected, actual, 1e-9));

  Values values;
  values.insert(1, G1);
  values.insert(2, G2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
namespace sl4 {
SL4 id;
const Matrix4 T_matrix =
    (Matrix4() << 1, 0, 0, 1, 0, 1, 0, 2, 0, 0, 1, 3, 0, 0, 0, 1).finished();
const SL4 T1(T_matrix);
const Matrix4 T_matrix2 =
    (Matrix4() << 1, 0, 0, 4, 0, 1, 0, 5, 0, 0, 1, 6, 0, 0, 0, 1).finished();
const SL4 T2(T_matrix2);
}  // namespace sl4

/* ************************************************************************* */
TEST(FrobeniusFactorSL4, EvaluateError) {
  using namespace ::sl4;
  auto factor = FrobeniusFactor<SL4>(1, 2, noiseModel::Unit::Create(16));
  Vector actual = factor.evaluateError(T1, T2);
  Vector expected = T2.vec() - T1.vec();
  EXPECT(assert_equal(expected, actual, 1e-9));

  Values values;
  values.insert(1, T1);
  values.insert(2, T2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
TEST(FrobeniusBetweenFactorSL4, EvaluateError) {
  using namespace ::sl4;
  auto factor = FrobeniusBetweenFactor<SL4>(1, 2, T1.between(T2));
  Matrix H1, H2;
  Vector actual = factor.evaluateError(T1, T2, H1, H2);
  Vector expected(16);
  expected.setZero();
  EXPECT(assert_equal(expected, actual, 1e-9));

  Values values;
  values.insert(1, T1);
  values.insert(2, T2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
TEST(FrobeniusBetweenFactorNLSL4, Optimization) {
  using namespace ::sl4;
  auto factor = FrobeniusBetweenFactorNL<SL4>(1, 2, T1.between(T2));

  NonlinearFactorGraph graph;
  graph.add(factor);
  graph.add(PriorFactor<SL4>(1, T1, noiseModel::Constrained::All(15)));

  Values initial;
  initial.insert(1, T1);
  initial.insert(2, T2.retract(Vector15::Constant(0.01)));

  double initial_error = graph.error(initial);

  GaussNewtonParams params;
  GaussNewtonOptimizer optimizer(graph, initial, params);
  Values result = optimizer.optimize();

  double final_error = graph.error(result);

  EXPECT(final_error < initial_error);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
