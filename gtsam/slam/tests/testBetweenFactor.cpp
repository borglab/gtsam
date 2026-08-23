/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testBetweenFactor.cpp
 * @brief
 * @author Duy-Nguyen Ta, Varun Agrawal
 * @date Aug 2, 2013
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/FixedJacobianFactor.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace std::placeholders;
using namespace gtsam;
using namespace gtsam::symbol_shorthand;
using namespace gtsam::noiseModel;

// Verifies the corrected Jacobians at a nonzero residual when enabled.
TEST(BetweenFactor, Rot3) {
  const Rot3 R1 = Rot3::Rodrigues(0.1, 0.2, 0.3);
  const Rot3 R2 = Rot3::Rodrigues(0.4, 0.5, 0.6);
#ifdef GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR
  const Rot3 noise = Rot3::Rodrigues(0.01, 0.01, 0.01);
#else
  const Rot3 noise;
#endif
  const Rot3 measured = R1.between(R2) * noise;

  BetweenFactor<Rot3> factor(R(1), R(2), measured, Isotropic::Sigma(3, 0.05));
  Matrix actualH1, actualH2;
  Vector actual = factor.evaluateError(R1, R2, actualH1, actualH2);

  const Vector3 expected = measured.localCoordinates(R1.between(R2));
  EXPECT(assert_equal(expected, actual));

  Matrix numericalH1 = numericalDerivative21<Vector3, Rot3, Rot3>(
        [&factor](const Rot3& r1, const Rot3& r2) {return factor.evaluateError(r1, r2);},
      R1, R2, 1e-5);
  EXPECT(assert_equal(numericalH1,actualH1, 1E-5));

  Matrix numericalH2 = numericalDerivative22<Vector3,Rot3,Rot3>(
        [&factor](const Rot3& r1, const Rot3& r2) {return factor.evaluateError(r1, r2);},
        R1, R2, 1e-5);
  EXPECT(assert_equal(numericalH2,actualH2, 1E-5));
}

/* ************************************************************************* */
// Constructor scalar
TEST(BetweenFactor, ConstructorScalar) {
  SharedNoiseModel model;
  double measured = 0.0;
  BetweenFactor<double> factor(1, 2, measured, model);
}

/* ************************************************************************* */
// Constructor vector3
TEST(BetweenFactor, ConstructorVector3) {
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 1.0);
  Vector3 measured(1, 2, 3);
  BetweenFactor<Vector3> factor(1, 2, measured, model);
}

/* ************************************************************************* */
// Constructor dynamic sized vector
TEST(BetweenFactor, ConstructorDynamicSizeVector) {
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(5, 1.0);
  Vector measured{{1, 2, 3, 4, 5}};
  BetweenFactor<Vector> factor(1, 2, measured, model);
}

/* ************************************************************************* */
TEST(BetweenFactor, Point3Jacobians) {
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 1.0);
  Point3 measured(1, 2, 3);
  BetweenFactor<Point3> factor(1, 2, measured, model);
  
  Values values;
  values.insert(1, Point3(0, 0, 0));
  values.insert(2, Point3(1, 2, 3));
  Vector3 error = factor.evaluateError(Point3(0, 0, 0), Point3(1, 2, 3));
  EXPECT(assert_equal<Vector3>(Vector3::Zero(), error, 1e-9));
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
TEST(BetweenFactor, Rot3Jacobians) {
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 1.0);
  Rot3 measured = Rot3::Ry(M_PI_2);
  BetweenFactor<Rot3> factor(1, 2, measured, model);
  
  Values values;
  values.insert(1, Rot3());
  values.insert(2, Rot3::Ry(M_PI_2));
  Vector3 error = factor.evaluateError(Rot3(), Rot3::Ry(M_PI_2));
  EXPECT(assert_equal<Vector3>(Vector3::Zero(), error, 1e-9));
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
TEST(BetweenFactor, Pose3Jacobians) {
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(6, 1.0);
  Pose3 measured(Rot3(), Point3(1, 2, 3));
  BetweenFactor<Pose3> factor(1, 2, measured, model);

  Pose3 pose1, pose2(Rot3(), Point3(1, 2, 3));
  Values values;
  values.insert(1, pose1);
  values.insert(2, pose2);
  Vector6 error = factor.evaluateError(pose1, pose2);
  EXPECT(assert_equal<Vector6>(Vector6::Zero(), error, 1e-9));
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
namespace binary_linearization {

class TestBinaryFactor : public NoiseModelFactorT<Vector2, Pose2, Point2> {
 public:
  TestBinaryFactor(const SharedNoiseModel& model, bool isActive = true)
      : NoiseModelFactorT<Vector2, Pose2, Point2>(model, 1, 2),
        isActive_(isActive) {}

  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<TestBinaryFactor>(*this);
  }

  bool active(const Values&) const override { return isActive_; }

  Vector2 evaluateError(const Pose2& pose, const Point2& point,
                        OptionalMatrixType H1,
                        OptionalMatrixType H2) const override {
    if (H1) {
      *H1 = (Matrix23() << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0).finished();
    }
    if (H2) *H2 = -Matrix2::Identity();
    return Vector2(pose.x() - point.x(), pose.y() - point.y());
  }

 private:
  bool isActive_;
};

template <class Value, int Dimension>
bool checkBetweenType(const Value& first, const Value& second) {
  const Value measured = traits<Value>::Between(first, second);
  BetweenFactor<Value> factor(1, 2, measured,
                              noiseModel::Unit::Create(Dimension));
  Values values{{1, genericValue(first)}, {2, genericValue(second)}};
  const auto expected = factor.NoiseModelFactor::linearize(values);
  const auto actual = factor.linearize(values);
  const bool isBinary = static_cast<bool>(std::dynamic_pointer_cast<
      FixedJacobianFactor<Dimension, Dimension, Dimension>>(actual));
  return isBinary && assert_equal(*expected, *actual, 1e-9);
}

// Verifies all supported noise models match generic binary linearization.
TEST(BetweenFactor, BinaryLinearizationNoiseModels) {
  const Matrix2 covariance{{2.0, 0.3}, {0.3, 1.0}};
  const std::vector<SharedNoiseModel> models{
      SharedNoiseModel(), noiseModel::Unit::Create(2),
      noiseModel::Isotropic::Sigma(2, 0.5),
      noiseModel::Diagonal::Sigmas(Vector2{0.5, 0.8}),
      noiseModel::Gaussian::Covariance(covariance),
      noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.345),
                                 noiseModel::Diagonal::Sigmas(
                                     Vector2{0.5, 0.8})),
      noiseModel::Constrained::MixedSigmas(Vector2{0.0, 0.8})};
  const Values values{{1, genericValue(Pose2(1.0, 2.0, 0.3))},
                      {2, genericValue(Point2(4.0, -1.0))}};

  for (const SharedNoiseModel& model : models) {
    const TestBinaryFactor factor(model);
    const auto expected = factor.NoiseModelFactor::linearize(values);
    const auto actual = factor.linearize(values);
    const bool isBinary = static_cast<bool>(
        std::dynamic_pointer_cast<FixedJacobianFactor<2, 3, 2>>(actual));
    CHECK(isBinary);
    EXPECT(assert_equal(*expected, *actual, 1e-9));
  }
}

// Verifies representative fixed-size BetweenFactor instantiations are binary.
TEST(BetweenFactor, BinaryLinearizationFixedTypes) {
  EXPECT((checkBetweenType<Pose2, 3>(Pose2(), Pose2(1.0, 2.0, 0.3))));
  EXPECT((checkBetweenType<Pose3, 6>(
      Pose3(), Pose3(Rot3::Rz(0.2), Point3(1, 2, 3)))));
  EXPECT((checkBetweenType<Rot2, 1>(Rot2(), Rot2::fromAngle(0.3))));
  EXPECT((checkBetweenType<Rot3, 3>(Rot3(), Rot3::Rz(0.2))));
}

// Verifies dynamic dimensions retain the generic JacobianFactor path.
TEST(BetweenFactor, BinaryLinearizationDynamicFallback) {
  const Vector first = Vector3(1.0, 2.0, 3.0);
  const Vector second = Vector3(4.0, 6.0, 8.0);
  BetweenFactor<Vector> factor(1, 2, second - first,
                               noiseModel::Unit::Create(3));
  const Values values{{1, genericValue(first)}, {2, genericValue(second)}};
  const auto expected = factor.NoiseModelFactor::linearize(values);
  const auto actual = factor.linearize(values);
  const GaussianFactor& actualFactor = *actual;
  CHECK(typeid(actualFactor) == typeid(JacobianFactor));
  EXPECT(assert_equal(*expected, *actual, 1e-9));
}

// Verifies inactive and invalid-input behavior matches generic linearization.
TEST(BetweenFactor, BinaryLinearizationEdgeCases) {
  const TestBinaryFactor inactive(noiseModel::Unit::Create(2), false);
  CHECK(!inactive.linearize(Values()));

  const TestBinaryFactor wrongNoise(noiseModel::Unit::Create(1));
  const Values values{{1, genericValue(Pose2())},
                      {2, genericValue(Point2(1.0, 2.0))}};
  CHECK_EXCEPTION(wrongNoise.linearize(values), std::invalid_argument);

  const TestBinaryFactor factor(noiseModel::Unit::Create(2));
  CHECK_EXCEPTION(factor.linearize(Values()), ValuesKeyDoesNotExist);
}

}  // namespace binary_linearization
/* ************************************************************************* */

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
