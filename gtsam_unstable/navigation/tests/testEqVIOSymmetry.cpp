/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testEqVIOSymmetry.cpp
 * @brief  Unit tests for VIO symmetry actions and lift helpers
 * @author Rohan Bansal
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/GroupAction.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam_unstable/navigation/EqVIOCommon.h>
#include <gtsam_unstable/navigation/EqVIOState.h>
#include <gtsam_unstable/navigation/EqVIOSymmetry.h>

#include <cmath>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <functional>
#include <limits>

using namespace gtsam;
using namespace gtsam::eqvio;

namespace eqvio_test_util {
  inline std::shared_ptr<const VIOCameraModel> CreateDefaultCamera() {
    return std::make_shared<VIOCameraModel>(
        Cal3_S2(450.0, 450.0, 0.0, 400.0, 240.0));
  }
  
  inline VIOSE23 MakeA(const Rot3& R, const Point3& t, const Vector3& w) {
    VIOSE23::Matrix3K x;
    x.col(0) = t;
    x.col(1) = w;
    return VIOSE23(R, x);
  }
  
  inline VIOState RandomStateElement(const std::vector<int>& ids) {
    VIOSensorState sensor;
    sensor.inputBias = VIOBias(Vector3::Random(), Vector3::Random());
    sensor.pose = Pose3::Expmap(Vector6::Random());
    sensor.velocity = Vector3::Random();
    sensor.cameraOffset = Pose3::Expmap(Vector6::Random());
  
    std::vector<Landmark> lms(ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {
      Point3 p = 10.0 * Vector3::Random();
      if (std::abs(p.z()) < 1e-3) p.z() += (p.z() >= 0.0 ? 1.0 : -1.0);
      lms[i] = Landmark{p, ids[i]};
    }
    return VIOState(sensor, lms);
  }
  
  inline VIOGroup RandomGroupElement(const std::vector<int>& ids) {
    const Pose3 Apose = Pose3::Expmap(Vector6::Random());
    const Vector3 w = Vector3::Random();
    const Pose3 B = Pose3::Expmap(Vector6::Random());
    const VIOBias beta(Vector3::Random(), Vector3::Random());
  
    std::vector<SOT3> Q(ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {
      const double scale = 2.0 * static_cast<double>(rand()) / RAND_MAX + 1.0;
      Q[i] =
          SOT3(SO3::Expmap(Vector3::Random()), std::log(scale));
    }
  
    return makeVIOGroup(MakeA(Apose.rotation(), Apose.translation(), w), beta, B,
                        VIOLandmarkGroup(Q));
  }
  
  inline IMUInput RandomVelocityElement() {
    IMUInput vel;
    vel.gyr = Vector3::Random();
    vel.acc = Vector3::Random();
    vel.gyrBiasVel = Vector3::Random();
    vel.accBiasVel = Vector3::Random();
    vel.stamp = 0.0;
    return vel;
  }
  
  inline VisionMeasurement RandomVisionMeasurement(
      const std::vector<int>& ids,
      const std::shared_ptr<const VIOCameraModel>& camera) {
    VisionMeasurement measurement;
  
    for (int id : ids) {
      Vector3 p;
      do {
        p = Vector3::Random();
      } while (p.norm() < 1e-9);
      p.normalize();
      while (p.z() < 1e-1) {
        p = Vector3::Random().normalized();
      }
      measurement[id] = camera->projectPoint(p);
    }
    return measurement;
  }
  
  inline double LogNorm(const VIOGroup& X) { return VIOGroup::Logmap(X).norm(); }
  
  inline double StateDistance(const VIOState& xi1, const VIOState& xi2) {
    if (xi1.n() != xi2.n()) {
      throw std::invalid_argument("StateDistance: landmark count mismatch");
    }
  
    double dist = 0.0;
    dist += xi1.sensor.inputBias.localCoordinates(xi2.sensor.inputBias).norm();
    dist += xi1.sensor.pose.localCoordinates(xi2.sensor.pose).norm();
    dist += xi1.sensor.cameraOffset.localCoordinates(xi2.sensor.cameraOffset).norm();
    dist += (xi1.sensor.velocity - xi2.sensor.velocity).norm();
  
    for (size_t i = 0; i < xi1.n(); ++i) {
      if (xi1.cameraLandmarks[i].id != xi2.cameraLandmarks[i].id) {
        throw std::invalid_argument("StateDistance: landmark ids mismatch");
      }
      dist += (xi1.cameraLandmarks[i].p - xi2.cameraLandmarks[i].p).norm();
    }
    return dist;
  }
  
  inline double MeasurementDistance(const VisionMeasurement& y1,
                                    const VisionMeasurement& y2) {
    Vector y1vec = measurementVector(y1);
    Vector y2vec = measurementVector(y2);
    const double scale = std::max(1.0, std::max(y1vec.norm(), y2vec.norm()));
    const Vector diff = measurementDifference(y1, y2);
    return diff.norm() / scale;
  }
  
  inline Matrix NumericalDifferential(const std::function<Vector(const Vector&)>& f,
                                      const Vector& x0, double h) {
    const int n = static_cast<int>(x0.size());
    const Vector y0 = f(x0);
    const int m = static_cast<int>(y0.size());
    Matrix J = Matrix::Zero(m, n);
    for (int j = 0; j < n; ++j) {
      Vector dx = Vector::Zero(n);
      dx(j) = h;
      J.col(j) = (f(x0 + dx) - f(x0 - dx)) / (2.0 * h);
    }
    return J;
  }
  
  inline bool MatrixClose(const Matrix& A, const Matrix& B, double h = -1.0) {
    if (A.rows() != B.rows() || A.cols() != B.cols()) return false;
    if (!A.array().isFinite().all() || !B.array().isFinite().all()) return false;
  
    if (h < 0.0) h = std::cbrt(std::numeric_limits<double>::epsilon());
  
    for (int i = 0; i < A.rows(); ++i) {
      for (int j = 0; j < A.cols(); ++j) {
        const double tol = std::max(h, h * 1e1 * std::abs(A(i, j)));
        if (std::abs(A(i, j) - B(i, j)) > tol) return false;
      }
    }
    return true;
  }
  
}  // namespace eqvio_test_util

using namespace eqvio_test_util;

namespace {

constexpr int kEqvioActionReps = 25;
constexpr double kEqvioNearZero = 1e-12;


std::vector<Landmark> Lms0() { return {}; }
std::vector<Landmark> Lms3() {
  return {{Point3(1.0, -0.3, 4.2), 11},
          {Point3(-0.7, 0.4, 3.1), 22},
          {Point3(0.2, 0.8, 5.5), 33}};
}

VIOSensorState SensorA() {
  VIOSensorState s;
  s.inputBias = VIOBias(Vector3(0.01, -0.02, 0.04), Vector3(0.05, -0.04, 0.03));
  s.pose = Pose3(Rot3::RzRyRx(0.1, -0.2, 0.25), Point3(0.4, -0.1, 1.0));
  s.velocity = Vector3(0.3, -0.2, 0.1);
  s.cameraOffset =
      Pose3(Rot3::RzRyRx(-0.03, 0.05, -0.02), Point3(0.1, 0.02, 0.07));
  return s;
}

VIOState State0() { return VIOState(SensorA(), Lms0()); }
VIOState State3() { return VIOState(SensorA(), Lms3()); }

VIOGroup Group0() {
  const Rot3 R = Rot3::RzRyRx(0.02, -0.03, 0.04);
  const Point3 t(0.05, -0.02, 0.03);
  const Vector3 w(0.01, -0.03, 0.02);
  return makeVIOGroup(
      MakeA(R, t, w),
      VIOBias(Vector3(-0.02, 0.01, 0.0), Vector3(0.01, -0.01, 0.02)),
      Pose3(Rot3::RzRyRx(-0.02, 0.01, 0.03), Point3(0.02, 0.0, -0.01)),
      VIOLandmarkGroup(0));
}

VIOGroup Group3() {
  const Rot3 R = Rot3::RzRyRx(0.02, -0.03, 0.04);
  const Point3 t(0.05, -0.02, 0.03);
  const Vector3 w(0.01, -0.03, 0.02);
  const SOT3 q1(SO3::Expmap((Vector3() << 0.03, -0.02, 0.01).finished()),
                std::log(1.1));
  const SOT3 q2(SO3::Expmap((Vector3() << -0.01, 0.04, -0.02).finished()),
                std::log(0.95));
  const SOT3 q3(SO3::Expmap((Vector3() << 0.02, 0.01, 0.03).finished()),
                std::log(1.05));
  return makeVIOGroup(
      MakeA(R, t, w),
      VIOBias(Vector3(-0.02, 0.01, 0.0), Vector3(0.01, -0.01, 0.02)),
      Pose3(Rot3::RzRyRx(-0.02, 0.01, 0.03), Point3(0.02, 0.0, -0.01)),
      VIOLandmarkGroup({q1, q2, q3}));
}

VIOGroup Group3b() {
  const Rot3 R = Rot3::RzRyRx(-0.04, 0.01, -0.02);
  const Point3 t(-0.03, 0.04, -0.01);
  const Vector3 w(-0.02, 0.01, 0.03);
  const SOT3 q1(SO3::Expmap((Vector3() << -0.02, 0.01, 0.04).finished()),
                std::log(1.02));
  const SOT3 q2(SO3::Expmap((Vector3() << 0.02, -0.03, 0.01).finished()),
                std::log(0.98));
  const SOT3 q3(SO3::Expmap((Vector3() << 0.01, 0.02, -0.02).finished()),
                std::log(1.08));
  return makeVIOGroup(
      MakeA(R, t, w),
      VIOBias(Vector3(0.02, 0.01, -0.01), Vector3(-0.02, 0.03, -0.01)),
      Pose3(Rot3::RzRyRx(0.01, -0.02, 0.04), Point3(-0.01, 0.03, 0.02)),
      VIOLandmarkGroup({q1, q2, q3}));
}

Matrix NumericalDerivativeWrtGroup(const VIOSymmetry& phi, const VIOGroup& X,
                                   const VIOState& xi, const VIOState& y0,
                                   double h = 1e-6) {
  Matrix H = Matrix::Zero(y0.dim(), static_cast<int>(Dim_groupTangent(X)));
  for (int j = 0; j < static_cast<int>(Dim_groupTangent(X)); ++j) {
    Vector dx = Vector::Zero(static_cast<int>(Dim_groupTangent(X)));
    dx(j) = h;
    const VIOState yPlus = phi(xi, X.retract(dx));
    dx(j) = -h;
    const VIOState yMinus = phi(xi, X.retract(dx));
    H.col(j) =
        (y0.localCoordinates(yPlus) - y0.localCoordinates(yMinus)) / (2.0 * h);
  }
  return H;
}

Matrix NumericalDerivativeWrtState(const VIOSymmetry& phi, const VIOGroup& X,
                                   const VIOState& xi, const VIOState& y0,
                                   double h = 1e-6) {
  Matrix H = Matrix::Zero(y0.dim(), xi.dim());
  for (int j = 0; j < xi.dim(); ++j) {
    Vector dxi = Vector::Zero(xi.dim());
    dxi(j) = h;
    const VIOState yPlus = phi(xi.retract(dxi), X);
    dxi(j) = -h;
    const VIOState yMinus = phi(xi.retract(dxi), X);
    H.col(j) =
        (y0.localCoordinates(yPlus) - y0.localCoordinates(yMinus)) / (2.0 * h);
  }
  return H;
}

}  // namespace

//******************************************************************************
// Verifies the right-action law for VIOSymmetry.
TEST(VIOSymmetry, RightActionLaw) {
  const VIOSymmetry phi;
  const VIOState xi = State3();
  const VIOGroup X1 = Group3();
  const VIOGroup X2 = Group3b();
  EXPECT_RIGHT_ACTION(phi, xi, X1, X2);
}

//******************************************************************************
// Verifies action Jacobians against numerical derivatives for n=0.
TEST(VIOSymmetry, JacobiansN0) {
  const VIOSymmetry phi;
  const VIOGroup X = Group0();
  const VIOState xi = State0();

  Matrix HX, HXi;
  const VIOState y = phi(xi, X, HXi, HX);
  const Matrix HXNum = NumericalDerivativeWrtGroup(phi, X, xi, y);
  const Matrix HXiNum = NumericalDerivativeWrtState(phi, X, xi, y);

  EXPECT(assert_equal(HXNum, HX, 1e-5));
  EXPECT(assert_equal(HXiNum, HXi, 1e-5));
}

//******************************************************************************
// Verifies action Jacobians against numerical derivatives for n=3.
TEST(VIOSymmetry, JacobiansN3) {
  const VIOSymmetry phi;
  const VIOGroup X = Group3();
  const VIOState xi = State3();

  Matrix HX, HXi;
  const VIOState y = phi(xi, X, HXi, HX);
  const Matrix HXNum = NumericalDerivativeWrtGroup(phi, X, xi, y);
  const Matrix HXiNum = NumericalDerivativeWrtState(phi, X, xi, y);

  EXPECT(assert_equal(HXNum, HX, 1e-5));
  EXPECT(assert_equal(HXiNum, HXi, 1e-5));
}

//******************************************************************************
// Verifies eqvio-ported state action identity and composition checks.
TEST(VIOSymmetry, StateActionEqvioPort) {
  srand(0);
  const std::vector<int> ids = {0, 1, 2, 3, 4};
  const VIOGroup groupId = makeVIOGroupIdentity(ids.size());

  for (int rep = 0; rep < kEqvioActionReps; ++rep) {
    const VIOGroup X1 = RandomGroupElement(ids);
    const VIOGroup X2 = RandomGroupElement(ids);
    const VIOState xi0 = RandomStateElement(ids);

    const VIOState xi0Id = stateGroupAction(groupId, xi0);
    const double dist0Id = StateDistance(xi0Id, xi0);
    EXPECT(dist0Id <= kEqvioNearZero);

    const VIOState xi1 = stateGroupAction(X2, stateGroupAction(X1, xi0));
    const VIOState xi2 = stateGroupAction(X1 * X2, xi0);
    const double dist12 = StateDistance(xi1, xi2);
    EXPECT(dist12 <= kEqvioNearZero);
  }
}

//******************************************************************************
// Verifies eqvio-ported output action identity and composition checks.
TEST(VIOSymmetry, OutputActionEqvioPort) {
  srand(0);
  const std::vector<int> ids = {0, 1, 2, 3, 4};
  const VIOGroup groupId = makeVIOGroupIdentity(ids.size());
  const auto camera = CreateDefaultCamera();

  for (int rep = 0; rep < kEqvioActionReps; ++rep) {
    const VIOGroup X1 = RandomGroupElement(ids);
    const VIOGroup X2 = RandomGroupElement(ids);
    const VisionMeasurement y0 = RandomVisionMeasurement(ids, camera);

    const VisionMeasurement y0Id = outputGroupAction(groupId, y0, camera);
    const double dist0Id = MeasurementDistance(y0Id, y0);
    EXPECT(dist0Id <= 1e-5);

    const VisionMeasurement y1 =
        outputGroupAction(X2, outputGroupAction(X1, y0, camera), camera);
    const VisionMeasurement y2 = outputGroupAction(X1 * X2, y0, camera);
    const double dist12 = MeasurementDistance(y1, y2);
    EXPECT(dist12 <= 1e-5);
  }
}

//******************************************************************************
// Verifies measurement equivariance under group action.
TEST(VIOSymmetry, OutputEquivarianceEqvioPort) {
  srand(0);
  const std::vector<int> ids = {0, 1, 2, 3, 4, 5};
  const auto camera = CreateDefaultCamera();

  for (int rep = 0; rep < kEqvioActionReps; ++rep) {
    const VIOGroup X = RandomGroupElement(ids);
    const VIOState xi0 = RandomStateElement(ids);

    const VisionMeasurement y1 =
        measureSystemState(stateGroupAction(X, xi0), camera);
    const VisionMeasurement y2 =
        outputGroupAction(X, measureSystemState(xi0, camera), camera);
    const double dist12 = MeasurementDistance(y1, y2);
    EXPECT(dist12 <= 1e-5);
  }
}

//******************************************************************************
// Verifies discrete lift update matches direct integration.
TEST(VIOSymmetry, LiftAndIntegrationSanity) {
  const VIOState xi = State3();
  IMUInput imu;
  imu.gyr = Vector3(0.03, -0.02, 0.01);
  imu.acc = Vector3(0.2, -0.1, 9.75);
  imu.gyrBiasVel = Vector3(0.01, 0.0, -0.02);
  imu.accBiasVel = Vector3(-0.01, 0.02, 0.0);

  const Vector l = liftVelocity(xi, imu);
  EXPECT_LONGS_EQUAL(33, l.size());

  const double dt = 1e-3;
  const VIOGroup delta = liftVelocityDiscrete(xi, imu, dt);
  const VIOState xiLifted = stateGroupAction(delta, xi);
  const VIOState xiIntegrated = integrateSystemFunction(xi, imu, dt);
  EXPECT(assert_equal(xiIntegrated, xiLifted, 1e-7));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
