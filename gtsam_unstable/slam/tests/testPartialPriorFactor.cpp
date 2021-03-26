/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#include <gtsam_unstable/slam/PartialPriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;


// Pose representation is [ Rx Ry Rz Tx Ty Tz ].
static const int kIndexRx = 0;
static const int kIndexRy = 1;
static const int kIndexRz = 2;
static const int kIndexTx = 3;
static const int kIndexTy = 4;
static const int kIndexTz = 5;


typedef PartialPriorFactor<Pose3> TestPartialPriorFactor;

/// traits
namespace gtsam {
template<>
struct traits<TestPartialPriorFactor> : public Testable<TestPartialPriorFactor> {
};
}

/* ************************************************************************* */
TEST(PartialPriorFactor, JacobianAtIdentity)
{
  Key poseKey(1);
  Pose3 measurement = Pose3::identity();
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(1, 0.25);

  TestPartialPriorFactor factor(poseKey, kIndexTy, measurement.translation().y(), model);

  // Create a linearization point at the zero-error point
  Pose3 pose = Pose3::identity();

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Pose3>(
      boost::bind(&TestPartialPriorFactor::evaluateError, &factor, _1, boost::none), pose);

  // Use the factor to calculate the derivative.
  Matrix actualH1;
  factor.evaluateError(pose, actualH1);

  // Verify we get the expected error.
  CHECK(assert_equal(expectedH1, actualH1, 1e-5));
}

/* ************************************************************************* */
TEST(PartialPriorFactor, JacobianPartialTranslation) {
  Key poseKey(1);
  Pose3 measurement(Rot3::RzRyRx(0.15, -0.30, 0.45), Point3(-5.0, 8.0, -11.0));
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(1, 0.25);

  TestPartialPriorFactor factor(poseKey, kIndexTy, measurement.translation().y(), model);

  // Create a linearization point at the zero-error point
  Pose3 pose(Rot3::RzRyRx(0.15, -0.30, 0.45), Point3(-5.0, 8.0, -11.0));

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Pose3>(
      boost::bind(&TestPartialPriorFactor::evaluateError, &factor, _1, boost::none), pose);

  // Use the factor to calculate the derivative.
  Matrix actualH1;
  factor.evaluateError(pose, actualH1);

  // Verify we get the expected error.
  CHECK(assert_equal(expectedH1, actualH1, 1e-5));
}

/* ************************************************************************* */
TEST(PartialPriorFactor, JacobianFullTranslation) {
  Key poseKey(1);
  Pose3 measurement(Rot3::RzRyRx(0.15, -0.30, 0.45), Point3(-5.0, 8.0, -11.0));
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.25);

  std::vector<size_t> translationIndices = { kIndexTx, kIndexTy, kIndexTz };
  TestPartialPriorFactor factor(poseKey, translationIndices, measurement.translation(), model);

  // Create a linearization point at the zero-error point
  Pose3 pose(Rot3::RzRyRx(0.15, -0.30, 0.45), Point3(-5.0, 8.0, -11.0));

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Pose3>(
      boost::bind(&TestPartialPriorFactor::evaluateError, &factor, _1, boost::none), pose);

  // Use the factor to calculate the derivative.
  Matrix actualH1;
  factor.evaluateError(pose, actualH1);

  // Verify we get the expected error.
  CHECK(assert_equal(expectedH1, actualH1, 1e-5));
}

/* ************************************************************************* */
TEST(PartialPriorFactor, JacobianFullRotation) {
  Key poseKey(1);
  Pose3 measurement(Rot3::RzRyRx(0.15, -0.30, 0.45), Point3(-5.0, 8.0, -11.0));
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.25);

  std::vector<size_t> rotationIndices = { kIndexRx, kIndexRy, kIndexRz };
  TestPartialPriorFactor factor(poseKey, rotationIndices, Rot3::Logmap(measurement.rotation()), model);

  // Create a linearization point at the zero-error point
  Pose3 pose(Rot3::RzRyRx(0.15, -0.30, 0.45), Point3(-5.0, 8.0, -11.0));

  // Calculate numerical derivatives.
  Matrix expectedH1 = numericalDerivative11<Vector, Pose3>(
      boost::bind(&TestPartialPriorFactor::evaluateError, &factor, _1, boost::none), pose);

  // Use the factor to calculate the derivative.
  Matrix actualH1;
  factor.evaluateError(pose, actualH1);

  // Verify we get the expected error.
  CHECK(assert_equal(expectedH1, actualH1, 1e-5));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
