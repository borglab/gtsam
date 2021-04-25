/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#include <gtsam_unstable/slam/PartialPosePriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

using namespace std;
using namespace gtsam;

namespace NM = gtsam::noiseModel;

// Pose3 parameter representation is [ Rx Ry Rz Tx Ty Tz ].
static const int kIndexRx = 0;
static const int kIndexRy = 1;
static const int kIndexRz = 2;
static const int kIndexTx = 3;
static const int kIndexTy = 4;
static const int kIndexTz = 5;

typedef PartialPosePriorFactor<Pose2> TestPrior2;
typedef PartialPosePriorFactor<Pose3> TestPrior3;
typedef std::vector<size_t> Indices;

/// traits
namespace gtsam {
template<>
struct traits<TestPrior2> : public Testable<TestPrior2> {};

template<>
struct traits<TestPrior3> : public Testable<TestPrior3> {};
}

/* ************************************************************************* */
TEST(PartialPriorFactor, Constructors2) {
  Key poseKey(1);
  Pose2 measurement(-13.1, 3.14, -0.73);

  // Prior on x component of translation.
  TestPrior2 factor1(poseKey, 0, measurement.x(), NM::Isotropic::Sigma(1, 0.25));
  CHECK(assert_equal(1, factor1.prior().rows()));
  CHECK(assert_equal(measurement.x(), factor1.prior()(0)));
  CHECK(assert_container_equality<Indices>({ 0 }, factor1.indices()));

  // Prior on full translation vector.
  const Indices t_indices = { 0, 1 };
  TestPrior2 factor2(poseKey, t_indices, measurement.translation(), NM::Isotropic::Sigma(2, 0.25));
  CHECK(assert_equal(2, factor2.prior().rows()));
  CHECK(assert_equal(measurement.translation(), factor2.prior()));
  CHECK(assert_container_equality<Indices>(t_indices, factor2.indices()));

  // Prior on theta.
  TestPrior2 factor3(poseKey, 2, measurement.theta(), NM::Isotropic::Sigma(1, 0.1));
  CHECK(assert_equal(1, factor3.prior().rows()));
  CHECK(assert_equal(measurement.theta(), factor3.prior()(0)));
  CHECK(assert_container_equality<Indices>({ 2 }, factor3.indices()));
}

/* ************************************************************************* */
TEST(PartialPriorFactor, JacobianPartialTranslation2) {
  Key poseKey(1);
  Pose2 measurement(-13.1, 3.14, -0.73);

  // Prior on x component of translation.
  TestPrior2 factor(poseKey, 0, measurement.x(), NM::Isotropic::Sigma(1, 0.25));

  Pose2 pose = measurement; // Zero-error linearization point.

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Pose2>(
      boost::bind(&TestPrior2::evaluateError, &factor, _1, boost::none), pose);

  // Use the factor to calculate the derivative.
  Matrix actualH1;
  Vector e = factor.evaluateError(pose, actualH1);

  // Make sure we get the correct error and Jacobian.
  CHECK(assert_equal(Vector1::Zero(), e, 1e-5));
  CHECK(assert_equal(expectedH1, actualH1, 1e-5));
}

/* ************************************************************************* */
TEST(PartialPriorFactor, JacobianFullTranslation2) {
  Key poseKey(1);
  Pose2 measurement(-6.0, 3.5, 0.123);

  // Prior on x component of translation.
  TestPrior2 factor(poseKey, { 0, 1 }, measurement.translation(), NM::Isotropic::Sigma(2, 0.25));

  Pose2 pose = measurement; // Zero-error linearization point.

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Pose2>(
      boost::bind(&TestPrior2::evaluateError, &factor, _1, boost::none), pose);

  // Use the factor to calculate the derivative.
  Matrix actualH1;
  Vector e = factor.evaluateError(pose, actualH1);
  CHECK(assert_equal(Vector2::Zero(), e, 1e-5));
  CHECK(assert_equal(expectedH1, actualH1, 1e-5));
}

/* ************************************************************************* */
TEST(PartialPriorFactor, JacobianTheta) {
  Key poseKey(1);
  Pose2 measurement(-1.0, 0.4, -2.5);

  // Prior on x component of translation.
  TestPrior2 factor(poseKey, 2, measurement.theta(), NM::Isotropic::Sigma(1, 0.25));

  Pose2 pose = measurement; // Zero-error linearization point.

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Pose2>(
      boost::bind(&TestPrior2::evaluateError, &factor, _1, boost::none), pose);

  // Use the factor to calculate the derivative.
  Matrix actualH1;
  Vector e = factor.evaluateError(pose, actualH1);
  CHECK(assert_equal(Vector1::Zero(), e, 1e-5));
  CHECK(assert_equal(expectedH1, actualH1, 1e-5));
}

/* ************************************************************************* */
TEST(PartialPriorFactor, Constructors3) {
  Key poseKey(1);
  Pose3 measurement(Rot3::RzRyRx(-0.17, 0.567, M_PI), Point3(10.0, -2.3, 3.14));

  // Single component of translation.
  TestPrior3 factor1(poseKey, kIndexTy, measurement.y(),
      NM::Isotropic::Sigma(1, 0.25));
  CHECK(assert_equal(1, factor1.prior().rows()));
  CHECK(assert_equal(measurement.y(), factor1.prior()(0)));
  CHECK(assert_container_equality<Indices>({ kIndexTy }, factor1.indices()));

  // Full translation vector.
  const Indices t_indices = { kIndexTx, kIndexTy, kIndexTz };
  TestPrior3 factor2(poseKey, t_indices, measurement.translation(),
      NM::Isotropic::Sigma(3, 0.25));
  CHECK(assert_equal(3, factor2.prior().rows()));
  CHECK(assert_equal(measurement.translation(), factor2.prior()));
  CHECK(assert_container_equality<Indices>(t_indices, factor2.indices()));

  // Full tangent vector of rotation.
  const Indices r_indices = { kIndexRx, kIndexRy, kIndexRz };
  TestPrior3 factor3(poseKey, r_indices, Rot3::Logmap(measurement.rotation()),
      NM::Isotropic::Sigma(3, 0.1));
  CHECK(assert_equal(3, factor3.prior().rows()));
  CHECK(assert_equal(Rot3::Logmap(measurement.rotation()), factor3.prior()));
  CHECK(assert_container_equality<Indices>(r_indices, factor3.indices()));
}

/* ************************************************************************* */
TEST(PartialPriorFactor, JacobianAtIdentity3) {
  Key poseKey(1);
  Pose3 measurement = Pose3::identity();
  SharedNoiseModel model = NM::Isotropic::Sigma(1, 0.25);

  TestPrior3 factor(poseKey, kIndexTy, measurement.translation().y(), model);

  Pose3 pose = measurement; // Zero-error linearization point.

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Pose3>(
      boost::bind(&TestPrior3::evaluateError, &factor, _1, boost::none), pose);

  // Use the factor to calculate the derivative.
  Matrix actualH1;
  Vector e = factor.evaluateError(pose, actualH1);
  CHECK(assert_equal(Vector1::Zero(), e, 1e-5));
  CHECK(assert_equal(expectedH1, actualH1, 1e-5));
}

/* ************************************************************************* */
TEST(PartialPriorFactor, JacobianPartialTranslation3) {
  Key poseKey(1);
  Pose3 measurement(Rot3::RzRyRx(0.15, -0.30, 0.45), Point3(-5.0, 8.0, -11.0));
  SharedNoiseModel model = NM::Isotropic::Sigma(1, 0.25);

  TestPrior3 factor(poseKey, kIndexTy, measurement.translation().y(), model);

  Pose3 pose = measurement; // Zero-error linearization point.

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Pose3>(
      boost::bind(&TestPrior3::evaluateError, &factor, _1, boost::none), pose);

  // Use the factor to calculate the derivative.
  Matrix actualH1;
  Vector e = factor.evaluateError(pose, actualH1);
  CHECK(assert_equal(Vector1::Zero(), e, 1e-5));
  CHECK(assert_equal(expectedH1, actualH1, 1e-5));
}

/* ************************************************************************* */
TEST(PartialPriorFactor, JacobianFullTranslation3) {
  Key poseKey(1);
  Pose3 measurement(Rot3::RzRyRx(-0.15, 0.30, -0.45), Point3(5.0, -8.0, 11.0));
  SharedNoiseModel model = NM::Isotropic::Sigma(3, 0.25);

  std::vector<size_t> translationIndices = { kIndexTx, kIndexTy, kIndexTz };
  TestPrior3 factor(poseKey, translationIndices, measurement.translation(), model);

  // Create a linearization point at the zero-error point
  Pose3 pose = measurement; // Zero-error linearization point.

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Pose3>(
      boost::bind(&TestPrior3::evaluateError, &factor, _1, boost::none), pose);

  // Use the factor to calculate the derivative.
  Matrix actualH1;
  Vector e = factor.evaluateError(pose, actualH1);
  CHECK(assert_equal(Vector3::Zero(), e, 1e-5));
  CHECK(assert_equal(expectedH1, actualH1, 1e-5));
}

/* ************************************************************************* */
TEST(PartialPriorFactor, JacobianTxTz3) {
  Key poseKey(1);
  Pose3 measurement(Rot3::RzRyRx(-0.17, 0.567, M_PI), Point3(10.0, -2.3, 3.14));
  SharedNoiseModel model = NM::Isotropic::Sigma(2, 0.25);

  std::vector<size_t> translationIndices = { kIndexTx, kIndexTz };
  TestPrior3 factor(poseKey, translationIndices,
      (Vector(2) << measurement.x(), measurement.z()).finished(), model);

  Pose3 pose = measurement; // Zero-error linearization point.

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Pose3>(
      boost::bind(&TestPrior3::evaluateError, &factor, _1, boost::none), pose);

  // Use the factor to calculate the derivative.
  Matrix actualH1;
  Vector e = factor.evaluateError(pose, actualH1);
  CHECK(assert_equal(Vector2::Zero(), e, 1e-5));
  CHECK(assert_equal(expectedH1, actualH1, 1e-5));
}

/* ************************************************************************* */
TEST(PartialPriorFactor, JacobianPartialRotation3) {
  Key poseKey(1);
  Pose3 measurement(Rot3::RzRyRx(1.15, -5.30, 0.45), Point3(-1.0, 2.0, -17.0));
  SharedNoiseModel model = NM::Isotropic::Sigma(1, 0.25);

  // Constrain one axis of rotation.
  TestPrior3 factor(poseKey, kIndexRx, Rot3::Logmap(measurement.rotation()).x(), model);

  Pose3 pose = measurement; // Zero-error linearization point.

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Pose3>(
      boost::bind(&TestPrior3::evaluateError, &factor, _1, boost::none), pose);

  // Use the factor to calculate the derivative.
  Matrix actualH1;
  Vector e = factor.evaluateError(pose, actualH1);
  CHECK(assert_equal(Vector1::Zero(), e, 1e-5));
  CHECK(assert_equal(expectedH1, actualH1, 1e-5));
}

/* ************************************************************************* */
TEST(PartialPriorFactor, JacobianFullRotation3) {
  Key poseKey(1);
  Pose3 measurement(Rot3::RzRyRx(0.15, -3.30, 0.01), Point3(-2.0, 4.0, -0.3));
  SharedNoiseModel model = NM::Isotropic::Sigma(3, 0.25);

  std::vector<size_t> rotationIndices = { kIndexRx, kIndexRy, kIndexRz };
  TestPrior3 factor(poseKey, rotationIndices, Rot3::Logmap(measurement.rotation()), model);

  Pose3 pose = measurement; // Zero-error linearization point.

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Pose3>(
      boost::bind(&TestPrior3::evaluateError, &factor, _1, boost::none), pose);

  // Use the factor to calculate the derivative.
  Matrix actualH1;
  Vector e = factor.evaluateError(pose, actualH1);
  CHECK(assert_equal(Vector3::Zero(), e, 1e-5));
  CHECK(assert_equal(expectedH1, actualH1, 1e-5));
}

/* ************************************************************************* */
TEST(PartialPriorFactor, FactorGraph1) {
  Key poseKey(1);

  Pose3 pose(Rot3::RzRyRx(-0.17, 0.567, M_PI), Point3(10.0, -2.3, 3.14));
  SharedNoiseModel model = NM::Isotropic::Sigma(6, 0.25);

  Vector6 prior = (Vector(6) << Rot3::Logmap(pose.rotation()), pose.translation()).finished();

  // By specifying all of the parameter indices, this effectively becomes a PosePriorFactor.
  std::vector<size_t> indices = { 0, 1, 2, 3, 4, 5 };
  TestPrior3 factor(poseKey, indices, prior, model);

  NonlinearFactorGraph graph;
  Values initial;
  graph.add(factor);

  // Get an initial pose with a small error from groundtruth. Make sure that the
  // prior factor is able to correct the final result.
  Pose3 pose_error(Rot3::RzRyRx(0.3, -0.03, 0.17), Point3(0.2, -0.14, 0.05));
  initial.insert(poseKey, pose_error * pose);
  initial.print("Initial values:\n");

  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  result.print("Final Result:\n");
  Pose3 pose_optimized = result.at<Pose3>(poseKey);

  CHECK(assert_equal(pose, pose_optimized, 1e-5));
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
