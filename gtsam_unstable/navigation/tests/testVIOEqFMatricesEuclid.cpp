/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testVIOEqFMatricesEuclid.cpp
 * @brief  Unit tests for Euclidean VIO EqF matrix suite
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam_unstable/navigation/VIOEqFMatrices.h>
#include <gtsam_unstable/navigation/EqVIOSymmetry.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

using namespace gtsam;

namespace {

class SimplePinholeCamera final : public VIOCameraModel {
 public:
  Point2 projectPoint(const Point3& p) const override {
    if (std::abs(p.z()) < 1e-12) {
      throw std::invalid_argument("SimplePinholeCamera: z is near zero");
    }
    return Point2(p.x() / p.z(), p.y() / p.z());
  }

  Vector3 undistortPoint(const Point2& y) const override {
    return Vector3(y.x(), y.y(), 1.0).normalized();
  }

  Matrix23 projectionJacobian(const Vector3& y) const override {
    if (std::abs(y.z()) < 1e-12) {
      throw std::invalid_argument("SimplePinholeCamera: z is near zero");
    }
    Matrix23 J;
    const double z2 = y.z() * y.z();
    J << 1.0 / y.z(), 0.0, -y.x() / z2, 0.0, 1.0 / y.z(), -y.y() / z2;
    return J;
  }
};

VIOSE23 MakeA(const Rot3& R, const Point3& t, const Vector3& w) {
  VIOSE23::Matrix3K x;
  x.col(0) = t;
  x.col(1) = w;
  return VIOSE23(R, x);
}

VIOSensorState SensorFixture() {
  VIOSensorState s;
  s.inputBias = (Vector6() << 0.01, -0.02, 0.03, 0.04, -0.01, 0.02).finished();
  s.pose = Pose3(Rot3::RzRyRx(0.1, -0.05, 0.2), Point3(0.3, -0.4, 1.2));
  s.velocity = Vector3(0.2, -0.1, 0.05);
  s.cameraOffset =
      Pose3(Rot3::RzRyRx(-0.02, 0.03, -0.01), Point3(0.1, 0.02, 0.08));
  return s;
}

VIOState State0() { return VIOState(SensorFixture(), {}); }
VIOState State1() { return VIOState(SensorFixture(), {{Point3(0.8, -0.2, 4.5), 11}}); }
VIOState State3() {
  return VIOState(SensorFixture(),
                  {{Point3(0.8, -0.2, 4.5), 11},
                   {Point3(-0.6, 0.3, 3.8), 22},
                   {Point3(0.1, 0.7, 5.2), 33}});
}

VIOGroup Group0() { return makeVIOGroupIdentity(0); }
VIOGroup Group1() {
  const SOT3 q1(SO3::Expmap((Vector3() << 0.02, -0.01, 0.03).finished()),
                std::log(1.1));
  return makeVIOGroup(
      MakeA(Rot3::RzRyRx(0.03, -0.02, 0.01), Point3(0.05, -0.01, 0.02),
            Vector3(0.01, -0.02, 0.03)),
      (Vector6() << 0.01, 0.0, -0.01, 0.02, -0.01, 0.03).finished(),
      Pose3(Rot3::RzRyRx(-0.01, 0.02, -0.03), Point3(0.02, 0.01, -0.01)),
      VIOLandmarkGroup({q1}));
}
VIOGroup Group3() {
  const SOT3 q1(SO3::Expmap((Vector3() << 0.02, -0.01, 0.03).finished()),
                std::log(1.1));
  const SOT3 q2(SO3::Expmap((Vector3() << -0.01, 0.03, -0.02).finished()),
                std::log(0.95));
  const SOT3 q3(SO3::Expmap((Vector3() << 0.01, 0.02, 0.01).finished()),
                std::log(1.05));
  return makeVIOGroup(
      MakeA(Rot3::RzRyRx(0.03, -0.02, 0.01), Point3(0.05, -0.01, 0.02),
            Vector3(0.01, -0.02, 0.03)),
      (Vector6() << 0.01, 0.0, -0.01, 0.02, -0.01, 0.03).finished(),
      Pose3(Rot3::RzRyRx(-0.01, 0.02, -0.03), Point3(0.02, 0.01, -0.01)),
      VIOLandmarkGroup({q1, q2, q3}));
}

IMUVelocity ImuFixture() {
  IMUVelocity u;
  u.gyr = Vector3(0.02, -0.01, 0.03);
  u.acc = Vector3(0.1, -0.05, 9.7);
  u.gyrBiasVel = Vector3(0.01, 0.0, -0.01);
  u.accBiasVel = Vector3(-0.02, 0.01, 0.0);
  return u;
}

bool IsFinite(const Matrix& M) { return M.array().isFinite().all(); }

}  // namespace

//******************************************************************************
TEST(VIOEqFMatrices, ShapesAndFinite) {
  const auto camera = std::make_shared<SimplePinholeCamera>();
  const EqFCoordinateSuite* suite = getCoordinates(CoordinateChoice::Euclidean);
  EXPECT(suite != nullptr);
  if (!suite) return;

  for (const auto& pair :
       std::vector<std::pair<VIOState, VIOGroup>>{{State0(), Group0()},
                                                   {State1(), Group1()},
                                                   {State3(), Group3()}}) {
    const VIOState& xi0 = pair.first;
    const VIOGroup& X = pair.second;
    const IMUVelocity imu = ImuFixture();
    const VisionMeasurement y =
        measureSystemState(stateGroupAction(X, xi0), camera);

    const Matrix A = suite->stateMatrixA(X, xi0, imu);
    const Matrix B = suite->inputMatrixB(X, xi0);
    const Matrix C = suite->outputMatrixC(xi0, X, y, true);

    EXPECT_LONGS_EQUAL(xi0.dim(), A.rows());
    EXPECT_LONGS_EQUAL(xi0.dim(), A.cols());
    EXPECT_LONGS_EQUAL(xi0.dim(), B.rows());
    EXPECT_LONGS_EQUAL(IMUVelocity::CompDim, B.cols());
    EXPECT_LONGS_EQUAL(2 * static_cast<long>(y.n()), C.rows());
    EXPECT_LONGS_EQUAL(xi0.dim(), C.cols());

    EXPECT(IsFinite(A));
    EXPECT(IsFinite(B));
    EXPECT(IsFinite(C));
  }
}

//******************************************************************************
TEST(VIOEqFMatrices, SmallStepDiscreteConsistency) {
  const EqFCoordinateSuite* suite = getCoordinates(CoordinateChoice::Euclidean);
  EXPECT(suite != nullptr);
  if (!suite) return;

  const VIOState xi0 = State3();
  const VIOGroup X = Group3();
  const IMUVelocity imu = ImuFixture();

  const double dt = 1e-6;
  const Matrix A = suite->stateMatrixA(X, xi0, imu);
  const Matrix Phi = suite->stateMatrixADiscrete(X, xi0, imu, dt);
  const Matrix PhiApprox = Matrix::Identity(xi0.dim(), xi0.dim()) + dt * A;

  EXPECT(assert_equal(PhiApprox, Phi, 1e-4));
}

//******************************************************************************
TEST(VIOEqFMatrices, InnovationLiftConsistency) {
  const EqFCoordinateSuite* suite = getCoordinates(CoordinateChoice::Euclidean);
  EXPECT(suite != nullptr);
  if (!suite) return;

  for (const VIOState& xi0 : std::vector<VIOState>{State1(), State3()}) {
    const Vector gamma = Vector::LinSpaced(xi0.dim(), -0.2, 0.2);
    const double eps = 1e-3;
    const Vector lift = suite->liftInnovation(eps * gamma, xi0);
    const VIOGroup dCont = VIOGroup::Expmap(lift);
    const VIOGroup dDisc = suite->liftInnovationDiscrete(eps * gamma, xi0);

    const Vector err = dCont.localCoordinates(dDisc);
    EXPECT(err.norm() < 5e-3);
  }
}

//******************************************************************************
TEST(VIOEqFMatrices, SimpleRegressionN0) {
  const EqFCoordinateSuite* suite = getCoordinates(CoordinateChoice::Euclidean);
  EXPECT(suite != nullptr);
  if (!suite) return;

  VIOSensorState sensor;
  sensor.inputBias.setZero();
  sensor.pose = Pose3::Identity();
  sensor.velocity.setZero();
  sensor.cameraOffset = Pose3::Identity();
  const VIOState xi0(sensor, {});
  const VIOGroup X = makeVIOGroupIdentity(0);
  const IMUVelocity imu = IMUVelocity::Zero();

  const Matrix B = suite->inputMatrixB(X, xi0);
  Matrix BExpected = Matrix::Zero(21, 12);
  BExpected.block<6, 6>(0, 6).setIdentity();
  BExpected.block<3, 3>(6, 0).setIdentity();
  BExpected.block<3, 3>(12, 3).setIdentity();
  EXPECT(assert_equal(BExpected, B, 1e-12));

  const Matrix A = suite->stateMatrixA(X, xi0, imu);
  Matrix AExpected = Matrix::Zero(21, 21);
  AExpected.block(0, 0, 21, 6) = -BExpected.block(0, 0, 21, 6);
  AExpected.block<3, 3>(9, 12).setIdentity();
  AExpected.block<3, 3>(12, 6) =
      -GRAVITY_CONSTANT * Rot3::Hat(Vector3::UnitZ());
  EXPECT(assert_equal(AExpected, A, 1e-12));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
