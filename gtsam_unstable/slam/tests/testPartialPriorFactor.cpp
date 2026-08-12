/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#include <gtsam/config.h>
#include <CppUnitLite/TestHarness.h>

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43

#include <gtsam_unstable/slam/PartialPriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/nonlinear/FunctorizedFactor.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

namespace NM = gtsam::noiseModel;

#if GTSAM_ENABLE_BOOST_SERIALIZATION
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic,
                        "gtsam_noiseModel_Isotropic")
#endif

// Pose3 parameter representation is [ Rx Ry Rz Tx Ty Tz ].
static const int kIndexRx = 0;
static const int kIndexRy = 1;
static const int kIndexRz = 2;
static const int kIndexTx = 3;
static const int kIndexTy = 4;
static const int kIndexTz = 5;

typedef PartialPriorFactor<Pose2> TestPartialPriorFactor2;
typedef PartialPriorFactor<Pose3> TestPartialPriorFactor3;
typedef PartialPriorFactor<Point3> TestPartialPriorFactorPoint3;
typedef std::vector<size_t> Indices;

/// traits
namespace gtsam {
template<>
struct traits<TestPartialPriorFactor2> : public Testable<TestPartialPriorFactor2> {};

template<>
struct traits<TestPartialPriorFactor3> : public Testable<TestPartialPriorFactor3> {};

template<>
struct traits<TestPartialPriorFactorPoint3> : public Testable<TestPartialPriorFactorPoint3> {};
}

/* ************************************************************************* */
TEST(PartialPriorFactor, Constructors2) {
  Key poseKey(1);
  Pose2 measurement(-13.1, 3.14, -0.73);

  // Prior on x component of translation.
  TestPartialPriorFactor2 factor1(poseKey, 0, measurement.x(), NM::Isotropic::Sigma(1, 0.25));
  CHECK(assert_equal(1, factor1.prior().rows()));
  CHECK(assert_equal(measurement.x(), factor1.prior()(0)));
  CHECK(assert_container_equality<Indices>({ 0 }, factor1.indices()));

  // Prior on full translation vector.
  const Indices t_indices = { 0, 1 };
  TestPartialPriorFactor2 factor2(poseKey, t_indices, measurement.translation(), NM::Isotropic::Sigma(2, 0.25));
  CHECK(assert_equal(2, factor2.prior().rows()));
  CHECK(assert_equal(measurement.translation(), factor2.prior()));
  CHECK(assert_container_equality<Indices>(t_indices, factor2.indices()));

  // Prior on theta.
  TestPartialPriorFactor2 factor3(poseKey, 2, measurement.theta(), NM::Isotropic::Sigma(1, 0.1));
  CHECK(assert_equal(1, factor3.prior().rows()));
  CHECK(assert_equal(measurement.theta(), factor3.prior()(0)));
  CHECK(assert_container_equality<Indices>({ 2 }, factor3.indices()));
}

/* ************************************************************************* */
TEST(PartialPriorFactor, JacobianPartialTranslation2) {
  Key poseKey(1);
  Pose2 measurement(-13.1, 3.14, -0.73);

#ifdef GTSAM_ROT3_EXPMAP
  double prior = Pose2::LocalCoordinates(measurement)(0);
#else
  double prior = Pose2::Logmap(measurement)(0);
#endif

  // Prior on x component of translation.
  TestPartialPriorFactor2 factor(poseKey, 0, prior,
                                 NM::Isotropic::Sigma(1, 0.25));

  Pose2 pose = measurement; // Zero-error linearization point.

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Pose2>(
		  [&factor](const Pose2& p) { return factor.evaluateError(p); }, pose);

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
#ifdef GTSAM_ROT3_EXPMAP
      Vector2 prior = Pose2::LocalCoordinates(measurement).head<2>();
#else
      Vector2 prior = Pose2::Logmap(measurement).head<2>();
#endif
  TestPartialPriorFactor2 factor(poseKey, {0, 1}, prior,
                                 NM::Isotropic::Sigma(2, 0.25));

  Pose2 pose = measurement; // Zero-error linearization point.

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Pose2>(
		  [&factor](const Pose2& p) { return factor.evaluateError(p); }, pose);

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
  TestPartialPriorFactor2 factor(poseKey, 2, measurement.theta(), NM::Isotropic::Sigma(1, 0.25));

  Pose2 pose = measurement; // Zero-error linearization point.

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Pose2>(
		  [&factor](const Pose2& p) { return factor.evaluateError(p); }, pose);

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
  TestPartialPriorFactor3 factor1(poseKey, kIndexTy, measurement.y(),
      NM::Isotropic::Sigma(1, 0.25));
  CHECK(assert_equal(1, factor1.prior().rows()));
  CHECK(assert_equal(measurement.y(), factor1.prior()(0)));
  CHECK(assert_container_equality<Indices>({ kIndexTy }, factor1.indices()));

  // Full translation vector.
  const Indices t_indices = { kIndexTx, kIndexTy, kIndexTz };
  TestPartialPriorFactor3 factor2(poseKey, t_indices, measurement.translation(),
      NM::Isotropic::Sigma(3, 0.25));
  CHECK(assert_equal(3, factor2.prior().rows()));
  CHECK(assert_equal(measurement.translation(), factor2.prior()));
  CHECK(assert_container_equality<Indices>(t_indices, factor2.indices()));

  // Full tangent vector of rotation.
  const Indices r_indices = { kIndexRx, kIndexRy, kIndexRz };
  TestPartialPriorFactor3 factor3(poseKey, r_indices, Rot3::Logmap(measurement.rotation()),
      NM::Isotropic::Sigma(3, 0.1));
  CHECK(assert_equal(3, factor3.prior().rows()));
  CHECK(assert_equal(Rot3::Logmap(measurement.rotation()), factor3.prior()));
  CHECK(assert_container_equality<Indices>(r_indices, factor3.indices()));
}

/* ************************************************************************* */
TEST(PartialPriorFactor, JacobianAtIdentity3) {
  Key poseKey(1);
  Pose3 measurement = Pose3::Identity();
  SharedNoiseModel model = NM::Isotropic::Sigma(1, 0.25);

  TestPartialPriorFactor3 factor(poseKey, kIndexTy, measurement.translation().y(), model);

  Pose3 pose = measurement; // Zero-error linearization point.

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Pose3>(
		  [&factor](const Pose3& p) { return factor.evaluateError(p); }, pose);

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

  TestPartialPriorFactor3 factor(poseKey, kIndexTy,
                                 Pose3::Logmap(measurement)(4), model);

  Pose3 pose = measurement; // Zero-error linearization point.

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Pose3>(
		  [&factor](const Pose3& p) { return factor.evaluateError(p); }, pose);

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
  TestPartialPriorFactor3 factor(poseKey, translationIndices,
                                 Pose3::Logmap(measurement).tail<3>(), model);

  // Create a linearization point at the zero-error point
  Pose3 pose = measurement; // Zero-error linearization point.

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Pose3>(
		  [&factor](const Pose3& p) { return factor.evaluateError(p); }, pose);

  // Use the factor to calculate the derivative.
  Matrix actualH1;
  Vector e = factor.evaluateError(pose, actualH1);
  CHECK(assert_equal(Vector3::Zero(), e, 1e-5));
  CHECK(assert_equal(expectedH1, actualH1, 1e-5));
}

/* ************************************************************************* */
TEST(PartialPriorFactor, JacobianTxTz3) {
  Key poseKey(1);
  Pose3 p(Rot3::RzRyRx(-0.17, 0.567, M_PI), Point3(10.0, -2.3, 3.14));
  SharedNoiseModel model = NM::Isotropic::Sigma(2, 0.25);

  std::vector<size_t> translationIndices = { kIndexTx, kIndexTz };
  Vector2 measurement;
  measurement << Pose3::Logmap(p)(3), Pose3::Logmap(p)(5);
  TestPartialPriorFactor3 factor(poseKey, translationIndices, measurement,
                                 model);

  Pose3 pose = p; // Zero-error linearization point.

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Pose3>(
		  [&factor](const Pose3& p) { return factor.evaluateError(p); }, pose);

  // Use the factor to calculate the derivative.
  Matrix actualH1;
  Vector e = factor.evaluateError(pose, actualH1);
  // CHECK(assert_equal(Vector2::Zero(), e, 1e-5));
  // CHECK(assert_equal(expectedH1, actualH1, 1e-5));
}

/* ************************************************************************* */
TEST(PartialPriorFactor, JacobianPartialRotation3) {
  Key poseKey(1);
  Pose3 measurement(Rot3::RzRyRx(1.15, -5.30, 0.45), Point3(-1.0, 2.0, -17.0));
  SharedNoiseModel model = NM::Isotropic::Sigma(1, 0.25);

  // Constrain one axis of rotation.
  TestPartialPriorFactor3 factor(poseKey, kIndexRx, Rot3::Logmap(measurement.rotation()).x(), model);

  Pose3 pose = measurement; // Zero-error linearization point.

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Pose3>(
      [&factor](const Pose3& p) { return factor.evaluateError(p); }, pose);

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
  TestPartialPriorFactor3 factor(poseKey, rotationIndices, Rot3::Logmap(measurement.rotation()), model);

  Pose3 pose = measurement; // Zero-error linearization point.

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Pose3>(
		  [&factor](const Pose3& p) { return factor.evaluateError(p); }, pose);

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

  Vector6 prior = Pose3::Logmap(pose);

  // By specifying all of the parameter indices, this effectively becomes a PosePriorFactor.
  std::vector<size_t> indices = { 0, 1, 2, 3, 4, 5 };
  TestPartialPriorFactor3 factor(poseKey, indices, prior, model);

  NonlinearFactorGraph graph;
  Values initial;
  graph.add(factor);

  // Get an initial pose with a small error from groundtruth. Make sure that the
  // prior factor is able to correct the final result.
  Pose3 pose_error(Rot3::RzRyRx(0.3, -0.03, 0.17), Point3(0.2, -0.14, 0.05));
  initial.insert(poseKey, pose_error * pose);
  // initial.print("Initial values:\n");

  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  // result.print("Final Result:\n");
  Pose3 pose_optimized = result.at<Pose3>(poseKey);

  CHECK(assert_equal(pose, pose_optimized, 1e-5));
}

/* ************************************************************************* */
// Vector-space types like Point3 (an Eigen typedef) must also work (see #2413).
TEST(PartialPriorFactor, JacobianPartialPoint3) {
  Key pointKey(1);
  Point3 measurement(1.0, -2.0, 3.0);

  // Prior on z component of the point.
  TestPartialPriorFactorPoint3 factor(pointKey, 2, measurement.z(), NM::Isotropic::Sigma(1, 0.25));

  Point3 point = measurement; // Zero-error linearization point.

  EXPECT(assert_equal(Vector1::Zero(), factor.evaluateError(point), 1e-9));

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Point3>(
      [&factor](const Point3& p) { return factor.evaluateError(p); }, point);

  // Use the factor to calculate the derivative.
  Matrix actualH1;
  factor.evaluateError(point, actualH1);

  // Verify we get the expected error.
  CHECK(assert_equal(expectedH1, actualH1, 1e-5));
}

/* ************************************************************************* */
namespace functorized_wrapper {

// Verifies the deprecated index API matches its explicit functorized form away
// from zero residual.
TEST(PartialPriorFactor, EquivalentFunctorizedFactor) {
  const Key poseKey(1);
  const Indices indices = {kIndexRy, kIndexTy};
  const Vector2 prior{0.25, -1.5};
  const auto model = NM::Isotropic::Sigma(2, 0.25);

  auto projection =
      [indices](const Pose3& pose,
                OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H = {}) {
        Matrix66 Hlocal;
#ifdef GTSAM_ROT3_EXPMAP
        const Vector6 tangent = traits<Pose3>::Local(
            Pose3::Identity(), pose, {}, H ? &Hlocal : nullptr);
#else
        const Vector6 tangent =
            traits<Pose3>::Logmap(pose, H ? &Hlocal : nullptr);
#endif
        if (H) {
          H->setZero(indices.size(), 6);
          for (size_t i = 0; i < indices.size(); ++i) {
            H->row(i) = Hlocal.row(indices.at(i));
          }
        }
        Vector result(indices.size());
        for (size_t i = 0; i < indices.size(); ++i) {
          result(i) = tangent(indices.at(i));
        }
        return result;
      };

  const TestPartialPriorFactor3 wrapper(poseKey, indices, prior, model);
  const auto explicitFactor = MakeFunctorizedFactor<Pose3>(
      poseKey, Vector(prior), model, projection);
  const Pose3 value(Rot3::RzRyRx(0.4, -0.2, 0.3),
                    Point3(2.0, -3.0, 4.0));

  Matrix wrapperH, explicitH;
  const Vector wrapperError = wrapper.evaluateError(value, wrapperH);
  const Vector explicitError = explicitFactor.evaluateError(value, explicitH);
  EXPECT(assert_equal(explicitError, wrapperError, 1e-9));
  EXPECT(assert_equal(explicitH, wrapperH, 1e-9));
}

#if GTSAM_ENABLE_BOOST_SERIALIZATION
// Verifies deserialization reconstructs the internal projection lambda.
TEST(PartialPriorFactor, SerializationReconstructsFunctor) {
  const Key poseKey(1);
  const Indices indices = {kIndexRx, kIndexTz};
  const Vector2 prior{0.1, 2.0};
  const auto model = NM::Isotropic::Sigma(2, 0.25);
  const TestPartialPriorFactor3 original(poseKey, indices, prior, model);
  TestPartialPriorFactor3 restored;
  serializationTestHelpers::roundtrip(original, restored);

  const Pose3 value(Rot3::RzRyRx(-0.3, 0.2, 0.1),
                    Point3(-2.0, 1.0, 5.0));
  Matrix originalH, restoredH;
  const Vector originalError = original.evaluateError(value, originalH);
  const Vector restoredError = restored.evaluateError(value, restoredH);
  EXPECT(assert_equal(original, restored));
  EXPECT(assert_equal(originalError, restoredError, 1e-9));
  EXPECT(assert_equal(originalH, restoredH, 1e-9));
}
#endif

}  // namespace functorized_wrapper
/* ************************************************************************* */
#endif  // GTSAM_ALLOW_DEPRECATED_SINCE_V43

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
