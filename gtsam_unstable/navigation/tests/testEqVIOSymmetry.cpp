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
#include <gtsam/base/Matrix.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam_unstable/navigation/EqVIOCommon.h>
#include <gtsam_unstable/navigation/EqVIOState.h>
#include <gtsam_unstable/navigation/EqVIOSymmetry.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

using namespace gtsam;
using namespace gtsam::eqvio;

namespace eqvio_test_util {
inline std::shared_ptr<const CameraModel> CreateDefaultCamera() {
  return std::make_shared<CameraModel>(
      Pose3::Identity(), Cal3_S2(450.0, 450.0, 0.0, 400.0, 240.0));
}

inline State RandomStateElement(const KeyVector& ids) {
  const Pose3 pose = Pose3::Expmap(Vector6::Random());
  std::vector<Point3> lms(ids.size());
  for (size_t i = 0; i < ids.size(); ++i) {
    Point3 p = 10.0 * Vector3::Random();
    p.z() = std::abs(p.z()) + 1.0;
    lms[i] = p;
  }
  return State(Se23(pose.rotation(), Vector3::Random(), pose.translation()),
               Bias(Vector3::Random(), Vector3::Random()),
               Pose3::Expmap(Vector6::Random()), lms);
}

inline VioGroup RandomGroupElement(const KeyVector& ids) {
  const Pose3 Apose = Pose3::Expmap(Vector6::Random());
  const Vector3 w = Vector3::Random();
  const Pose3 B = Pose3::Expmap(Vector6::Random());
  const Bias beta(Vector3::Random(), Vector3::Random());

  std::vector<SOT3> Q(ids.size());
  for (size_t i = 0; i < ids.size(); ++i) {
    const double scale = 2.0 * static_cast<double>(rand()) / RAND_MAX + 1.0;
    const double yaw =
        0.3 * (2.0 * static_cast<double>(rand()) / RAND_MAX - 1.0);
    Q[i] = SOT3(SO3::Expmap(Vector3(0.0, 0.0, yaw)), std::log(scale));
  }

  return makeVioGroup(Se23(Apose.rotation(), w, Apose.translation()), beta, B,
                      LandmarkGroup(Q));
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
    const KeyVector& ids, const std::shared_ptr<const CameraModel>& camera) {
  VisionMeasurement measurement;

  for (Key id : ids) {
    Vector3 p;
    do {
      p = Vector3::Random();
    } while (p.norm() < 1e-9);
    p.normalize();
    while (p.z() < 1e-1) {
      p = Vector3::Random().normalized();
    }
    measurement[id] = camera->project2(p);
  }
  return measurement;
}

inline Vector MeasurementVector(const VisionMeasurement& measurement) {
  Vector y = Vector::Zero(2 * static_cast<int>(measurement.size()));
  int i = 0;
  for (const auto& [id, p] : measurement) {
    (void)id;
    y.segment<2>(2 * i) << p.x(), p.y();
    ++i;
  }
  return y;
}

inline Vector MeasurementDifference(const VisionMeasurement& y1,
                                    const VisionMeasurement& y2) {
  if (y1.size() != y2.size()) {
    throw std::invalid_argument("MeasurementDifference: size mismatch");
  }
  Vector diff = Vector::Zero(2 * static_cast<int>(y1.size()));
  int i = 0;
  for (const auto& [id, p1] : y1) {
    const auto it = y2.find(id);
    if (it == y2.end()) {
      throw std::invalid_argument("MeasurementDifference: id mismatch");
    }
    diff.segment<2>(2 * i) << p1.x() - it->second.x(), p1.y() - it->second.y();
    ++i;
  }
  return diff;
}

inline double LogNorm(const VioGroup& X) { return VioGroup::Logmap(X).norm(); }

inline double StateDistance(const State& xi1, const State& xi2) {
  if (xi1.n() != xi2.n()) {
    throw std::invalid_argument("StateDistance: landmark count mismatch");
  }

  double dist = 0.0;
  dist += xi1.bias.localCoordinates(xi2.bias).norm();
  dist += xi1.pose().localCoordinates(xi2.pose()).norm();
  dist += xi1.cameraOffset.localCoordinates(xi2.cameraOffset).norm();
  dist += (xi1.velocity() - xi2.velocity()).norm();

  for (size_t i = 0; i < xi1.n(); ++i) {
    dist += (xi1.cameraLandmarks[i] - xi2.cameraLandmarks[i]).norm();
  }
  return dist;
}

inline double MeasurementDistance(const VisionMeasurement& y1,
                                  const VisionMeasurement& y2) {
  Vector y1vec = MeasurementVector(y1);
  Vector y2vec = MeasurementVector(y2);
  const double scale = std::max(1.0, std::max(y1vec.norm(), y2vec.norm()));
  const Vector diff = MeasurementDifference(y1, y2);
  return diff.norm() / scale;
}

inline Matrix NumericalDifferential(
    const std::function<Vector(const Vector&)>& f, const Vector& x0, double h) {
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

inline Vector liftVelocity(const State& state, const IMUInput& velocity) {
  const size_t N = state.n();
  Vector lift = Vector::Zero(21 + 4 * static_cast<int>(N));

  const IMUInput v_est = velocity - state.bias;

  Vector6 U_A;
  U_A << v_est.gyr, state.velocity();
  const Vector6 U_B = state.cameraOffset.inverse().AdjointMap() * U_A;
  const Vector3 u_w = -v_est.acc + state.gravityDir() * GRAVITY_CONSTANT;

  lift.segment<6>(0) = U_A;
  lift.segment<3>(6) = u_w;
  lift.segment<6>(9) << velocity.accBiasVel, velocity.gyrBiasVel;
  lift.segment<6>(15) = U_B;

  const Vector6 U_C = state.cameraOffset.inverse().AdjointMap() * U_A;
  const Vector3 omegaC = U_C.head<3>();
  const Vector3 vC = U_C.tail<3>();

  // Lift the landmark transform velocities
  for (size_t i = 0; i < N; ++i) {
    const Vector3 p = state.cameraLandmarks[i];
    Vector4 W;
    W.head<3>() = omegaC + Rot3::Hat(p) * vC / p.squaredNorm();
    W(3) = p.dot(vC) / p.squaredNorm();
    lift.segment<4>(21 + 4 * static_cast<int>(i)) = W;
  }

  return lift;
}

}  // namespace eqvio_test_util

using namespace eqvio_test_util;

namespace {

constexpr int kEqvioActionReps = 25;
constexpr double kEqvioNearZero = 1e-12;

KeyVector QIdsForMeasurement(const VioGroup& X,
                             const VisionMeasurement& measurement) {
  if (N_landmarkCount(X) == 0) return {};
  if (measurement.size() != N_landmarkCount(X)) {
    throw std::invalid_argument(
        "outputGroupAction: measurement count must match group landmark count");
  }
  return measurementIds(measurement);
}

VisionMeasurement outputGroupAction(
    const VioGroup& X, const VisionMeasurement& measurement,
    const std::shared_ptr<const CameraModel>& camera) {
  const LandmarkGroup& Q = std::get<3>(decompose(X));
  VisionMeasurement out;
  if (!camera) {
    throw std::invalid_argument("outputGroupAction: camera model is null");
  }

  const KeyVector qIds = QIdsForMeasurement(X, measurement);
  if (qIds.size() != Q.size()) {
    throw std::invalid_argument(
        "outputGroupAction: invalid Q-to-id mapping cardinality");
  }

  for (size_t i = 0; i < Q.size(); ++i) {
    const Key id = qIds[i];
    const auto it = measurement.find(id);
    if (it == measurement.end()) continue;

    const Vector3 bearing = undistortPoint(*camera, it->second);
    const Vector3 rotated = SOT3Rotation(Q[i]).matrix().transpose() * bearing;
    out[id] = camera->project2(rotated);
  }

  return out;
}

State integrateSystemFunction(const State& state, const IMUInput& velocity,
                              double dt) {
  State out;
  const IMUInput v_est = velocity - state.bias;
  const Pose3 pose = state.pose();
  const Vector3 bodyVelocity = state.velocity();

  out.bias = Bias(state.bias.accelerometer() + dt * velocity.accBiasVel,
                  state.bias.gyroscope() + dt * velocity.gyrBiasVel);

  const Rot3 dR = Rot3::Expmap(dt * v_est.gyr);
  const Vector3 dXWorld =
      dt * (pose.rotation() * bodyVelocity) +
      0.5 * dt * dt *
          (pose.rotation() * v_est.acc + Vector3(0, 0, -GRAVITY_CONSTANT));
  const Point3 dXBody = pose.rotation().unrotate(dXWorld);
  const Pose3 b0Tb1(dR, dXBody);
  const Pose3 nextPose = pose.compose(b0Tb1);

  const Vector3 inertialVelocityDiff =
      pose.rotation() * v_est.acc + Vector3(0, 0, -GRAVITY_CONSTANT);
  const Vector3 nextVelocity = nextPose.rotation().unrotate(
      pose.rotation() * bodyVelocity + dt * inertialVelocityDiff);

  const Pose3 c1Tc0 = state.cameraOffset.inverse()
                          .compose(b0Tb1.inverse())
                          .compose(state.cameraOffset);
  out.cameraLandmarks.resize(state.n());

  for (size_t i = 0; i < state.n(); ++i) {
    out.cameraLandmarks[i] = c1Tc0.transformFrom(state.cameraLandmarks[i]);
  }

  out.kinematics =
      Se23(nextPose.rotation(), nextVelocity, nextPose.translation());
  out.cameraOffset = state.cameraOffset;
  return out;
}

std::vector<Point3> Lms0() { return {}; }
std::vector<Point3> Lms3() {
  return {{Point3(1.0, -0.3, 4.2)},
          {Point3(-0.7, 0.4, 3.1)},
          {Point3(0.2, 0.8, 5.5)}};
}

State State0() {
  return State(Se23(Rot3::RzRyRx(0.1, -0.2, 0.25), Vector3(0.3, -0.2, 0.1),
                    Point3(0.4, -0.1, 1.0)),
               Bias(Vector3(0.01, -0.02, 0.04), Vector3(0.05, -0.04, 0.03)),
               Pose3(Rot3::RzRyRx(-0.03, 0.05, -0.02), Point3(0.1, 0.02, 0.07)),
               Lms0());
}
State State3() {
  return State(Se23(Rot3::RzRyRx(0.1, -0.2, 0.25), Vector3(0.3, -0.2, 0.1),
                    Point3(0.4, -0.1, 1.0)),
               Bias(Vector3(0.01, -0.02, 0.04), Vector3(0.05, -0.04, 0.03)),
               Pose3(Rot3::RzRyRx(-0.03, 0.05, -0.02), Point3(0.1, 0.02, 0.07)),
               Lms3());
}

VioGroup Group0() {
  const Rot3 R = Rot3::RzRyRx(0.02, -0.03, 0.04);
  const Point3 t(0.05, -0.02, 0.03);
  const Vector3 w(0.01, -0.03, 0.02);
  return makeVioGroup(
      Se23(R, w, t),
      Bias(Vector3(-0.02, 0.01, 0.0), Vector3(0.01, -0.01, 0.02)),
      Pose3(Rot3::RzRyRx(-0.02, 0.01, 0.03), Point3(0.02, 0.0, -0.01)),
      LandmarkGroup(0));
}

VioGroup Group3() {
  const Rot3 R = Rot3::RzRyRx(0.02, -0.03, 0.04);
  const Point3 t(0.05, -0.02, 0.03);
  const Vector3 w(0.01, -0.03, 0.02);
  const SOT3 q1(SO3::Expmap((Vector3() << 0.03, -0.02, 0.01).finished()),
                std::log(1.1));
  const SOT3 q2(SO3::Expmap((Vector3() << -0.01, 0.04, -0.02).finished()),
                std::log(0.95));
  const SOT3 q3(SO3::Expmap((Vector3() << 0.02, 0.01, 0.03).finished()),
                std::log(1.05));
  return makeVioGroup(
      Se23(R, w, t),
      Bias(Vector3(-0.02, 0.01, 0.0), Vector3(0.01, -0.01, 0.02)),
      Pose3(Rot3::RzRyRx(-0.02, 0.01, 0.03), Point3(0.02, 0.0, -0.01)),
      LandmarkGroup({q1, q2, q3}));
}

VioGroup Group3b() {
  const Rot3 R = Rot3::RzRyRx(-0.04, 0.01, -0.02);
  const Point3 t(-0.03, 0.04, -0.01);
  const Vector3 w(-0.02, 0.01, 0.03);
  const SOT3 q1(SO3::Expmap((Vector3() << -0.02, 0.01, 0.04).finished()),
                std::log(1.02));
  const SOT3 q2(SO3::Expmap((Vector3() << 0.02, -0.03, 0.01).finished()),
                std::log(0.98));
  const SOT3 q3(SO3::Expmap((Vector3() << 0.01, 0.02, -0.02).finished()),
                std::log(1.08));
  return makeVioGroup(
      Se23(R, w, t),
      Bias(Vector3(0.02, 0.01, -0.01), Vector3(-0.02, 0.03, -0.01)),
      Pose3(Rot3::RzRyRx(0.01, -0.02, 0.04), Point3(-0.01, 0.03, 0.02)),
      LandmarkGroup({q1, q2, q3}));
}

Matrix NumericalDerivativeWrtGroup(const Symmetry& phi, const VioGroup& X,
                                   const State& xi, const State& y0,
                                   double h = 1e-6) {
  Matrix H = Matrix::Zero(y0.dim(), static_cast<int>(Dim_groupTangent(X)));
  for (int j = 0; j < static_cast<int>(Dim_groupTangent(X)); ++j) {
    Vector dx = Vector::Zero(static_cast<int>(Dim_groupTangent(X)));
    dx(j) = h;
    const State yPlus = phi(xi, X.retract(dx));
    dx(j) = -h;
    const State yMinus = phi(xi, X.retract(dx));
    H.col(j) =
        (y0.localCoordinates(yPlus) - y0.localCoordinates(yMinus)) / (2.0 * h);
  }
  return H;
}

Matrix NumericalDerivativeWrtState(const Symmetry& phi, const VioGroup& X,
                                   const State& xi, const State& y0,
                                   double h = 1e-6) {
  Matrix H = Matrix::Zero(y0.dim(), xi.dim());
  for (int j = 0; j < xi.dim(); ++j) {
    Vector dxi = Vector::Zero(xi.dim());
    dxi(j) = h;
    const State yPlus = phi(xi.retract(dxi), X);
    dxi(j) = -h;
    const State yMinus = phi(xi.retract(dxi), X);
    H.col(j) =
        (y0.localCoordinates(yPlus) - y0.localCoordinates(yMinus)) / (2.0 * h);
  }
  return H;
}

}  // namespace

//******************************************************************************
// Verifies the right-action law for Symmetry.
TEST(Symmetry, RightActionLaw) {
  const Symmetry phi;
  const State xi = State3();
  const VioGroup X1 = Group3();
  const VioGroup X2 = Group3b();
  EXPECT_RIGHT_ACTION(phi, xi, X1, X2);
}

//******************************************************************************
// Verifies action Jacobians against numerical derivatives for n=0.
TEST(Symmetry, JacobiansN0) {
  const Symmetry phi;
  const VioGroup X = Group0();
  const State xi = State0();

  Matrix HX, HXi;
  const State y = phi(xi, X, HXi, HX);
  const Matrix HXNum = NumericalDerivativeWrtGroup(phi, X, xi, y);
  const Matrix HXiNum = NumericalDerivativeWrtState(phi, X, xi, y);

  EXPECT(assert_equal(HXNum, HX, 1e-5));
  EXPECT(assert_equal(HXiNum, HXi, 1e-5));
}

//******************************************************************************
// Verifies action Jacobians against numerical derivatives for n=3.
TEST(Symmetry, JacobiansN3) {
  const Symmetry phi;
  const VioGroup X = Group3();
  const State xi = State3();

  Matrix HX, HXi;
  const State y = phi(xi, X, HXi, HX);
  const Matrix HXNum = NumericalDerivativeWrtGroup(phi, X, xi, y);
  const Matrix HXiNum = NumericalDerivativeWrtState(phi, X, xi, y);

  EXPECT(assert_equal(HXNum, HX, 1e-5));
  EXPECT(assert_equal(HXiNum, HXi, 1e-5));
}

//******************************************************************************
// Verifies eqvio-ported state action identity and composition checks.
TEST(Symmetry, StateActionEqvioPort) {
  srand(0);
  const KeyVector ids = {0, 1, 2, 3, 4};
  const VioGroup groupId = makeVioGroupIdentity(ids.size());

  for (int rep = 0; rep < kEqvioActionReps; ++rep) {
    const VioGroup X1 = RandomGroupElement(ids);
    const VioGroup X2 = RandomGroupElement(ids);
    const State xi0 = RandomStateElement(ids);

    const State xi0Id = stateGroupAction(groupId, xi0);
    const double dist0Id = StateDistance(xi0Id, xi0);
    EXPECT(dist0Id <= kEqvioNearZero);

    const State xi1 = stateGroupAction(X2, stateGroupAction(X1, xi0));
    const State xi2 = stateGroupAction(X1 * X2, xi0);
    const double dist12 = StateDistance(xi1, xi2);
    EXPECT(dist12 <= kEqvioNearZero);
  }
}

//******************************************************************************
// Verifies eqvio-ported output action identity and composition checks.
TEST(Symmetry, OutputActionEqvioPort) {
  srand(0);
  const KeyVector ids = {0, 1, 2, 3, 4};
  const VioGroup groupId = makeVioGroupIdentity(ids.size());
  const auto camera = CreateDefaultCamera();

  for (int rep = 0; rep < kEqvioActionReps; ++rep) {
    const VioGroup X1 = RandomGroupElement(ids);
    const VioGroup X2 = RandomGroupElement(ids);
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
TEST(Symmetry, OutputEquivarianceEqvioPort) {
  srand(0);
  const KeyVector ids = {0, 1, 2, 3, 4, 5};
  const auto camera = CreateDefaultCamera();

  for (int rep = 0; rep < kEqvioActionReps; ++rep) {
    const VioGroup X = RandomGroupElement(ids);
    const State xi0 = RandomStateElement(ids);

    const VisionMeasurement y1 =
        measureSystemState(stateGroupAction(X, xi0), ids, camera);
    const VisionMeasurement y2 =
        outputGroupAction(X, measureSystemState(xi0, ids, camera), camera);
    const double dist12 = MeasurementDistance(y1, y2);
    EXPECT(dist12 <= 1e-5);
  }
}

//******************************************************************************
// Verifies discrete lift update matches direct integration.
TEST(Symmetry, LiftAndIntegrationSanity) {
  const State xi = State3();
  IMUInput imu;
  imu.gyr = Vector3(0.03, -0.02, 0.01);
  imu.acc = Vector3(0.2, -0.1, 9.75);
  imu.gyrBiasVel = Vector3(0.01, 0.0, -0.02);
  imu.accBiasVel = Vector3(-0.01, 0.02, 0.0);

  const Vector l = liftVelocity(xi, imu);
  EXPECT_LONGS_EQUAL(33, l.size());

  const double dt = 1e-3;
  const VioGroup delta = liftVelocityDiscrete(xi, imu, dt);
  const State xiLifted = stateGroupAction(delta, xi);
  const State xiIntegrated = integrateSystemFunction(xi, imu, dt);
  EXPECT(assert_equal(xiIntegrated, xiLifted, 1e-7));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
