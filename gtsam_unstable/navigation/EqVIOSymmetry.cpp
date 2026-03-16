/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/// @file EqVIOSymmetry.cpp
/// @brief EqVIO symmetry actions and lift helpers.
/// @author Rohan Bansal

#include <gtsam_unstable/navigation/EqVIOSymmetry.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>

namespace gtsam {
namespace eqvio {

namespace {

double SOT3Scale(const SOT3& Q) { return std::exp(Q.second); }

const SO3& SOT3Rotation(const SOT3& Q) { return Q.first; }

SOT3 MakeSOT3(const SO3& R, double scale) {
  if (scale <= 0.0) {
    throw std::invalid_argument("MakeSOT3: scale must be strictly positive");
  }
  return SOT3(R, std::log(scale));
}

Vector3 SOT3ApplyInverse(const SOT3& Q, const Vector3& p) {
  return (1.0 / SOT3Scale(Q)) * (SOT3Rotation(Q).matrix().transpose() * p);
}

Pose3 APose(const VIOGroup& X) {
  return Pose3(A_sensorKinematics(X).rotation(), A_sensorKinematics(X).x(0));
}

Vector3 AW(const VIOGroup& X) { return A_sensorKinematics(X).x(1); }

VIOSE23 MakeA(const Rot3& R, const Point3& x0, const Vector3& w) {
  VIOSE23::Matrix3K x;
  x.col(0) = x0;
  x.col(1) = w;
  return VIOSE23(R, x);
}

Rot3 RotationFromTwoVectors(const Vector3& from, const Vector3& to) {
  gtsam::Quaternion q;
  q.setFromTwoVectors(from, to);
  return Rot3(q);
}

std::vector<int> QIdsForMeasurement(const VIOGroup& X,
                                    const VisionMeasurement& measurement) {
  if (N_landmarkCount(X) == 0) return {};
  if (measurement.size() != N_landmarkCount(X)) {
    throw std::invalid_argument(
        "outputGroupAction: measurement count must match group landmark count");
  }
  return measurementIds(measurement);
}

Matrix NumericalDerivativeActionWrtGroup(
    const std::function<VIOState(const VIOGroup&)>& f, const VIOGroup& X,
    const VIOState& y0, double h = 1e-6) {
  const int n = static_cast<int>(Dim_groupTangent(X));
  const int m = y0.dim();
  Matrix H = Matrix::Zero(m, n);

  for (int j = 0; j < n; ++j) {
    Vector dx = Vector::Zero(n);
    dx(j) = h;
    const VIOState yPlus = f(X.retract(dx));
    dx(j) = -h;
    const VIOState yMinus = f(X.retract(dx));

    H.col(j) =
        (y0.localCoordinates(yPlus) - y0.localCoordinates(yMinus)) / (2.0 * h);
  }

  return H;
}

}  // namespace

VIOSensorState sensorStateGroupAction(const VIOGroup& X,
                                      const VIOSensorState& sensor) {
  const Pose3 A = APose(X);
  const Vector3 w = AW(X);

  VIOSensorState out;
  out.inputBias = sensor.inputBias + Beta_biasOffset(X);
  out.pose = sensor.pose.compose(A);
  out.velocity = A.rotation().unrotate(sensor.velocity - w);
  out.cameraOffset =
      A.inverse().compose(sensor.cameraOffset).compose(B_cameraExtrinsics(X));
  return out;
}

VIOState stateGroupAction(const VIOGroup& X, const VIOState& state) {
  if (N_landmarkCount(X) != state.n()) {
    throw std::invalid_argument(
        "stateGroupAction: group and state landmark counts do not match");
  }

  VIOState out;
  out.sensor = sensorStateGroupAction(X, state.sensor);
  out.cameraLandmarks.resize(N_landmarkCount(X));

  for (size_t i = 0; i < N_landmarkCount(X); ++i) {
    out.cameraLandmarks[i].p =
        SOT3ApplyInverse(Q_landmarkTransforms(X)[i], state.cameraLandmarks[i].p);
    out.cameraLandmarks[i].id = state.cameraLandmarks[i].id;
  }

  return out;
}

VisionMeasurement outputGroupAction(const VIOGroup& X,
                                    const VisionMeasurement& measurement,
                                    const std::shared_ptr<const VIOCameraModel>& camera) {
  VisionMeasurement out;
  if (!camera) {
    throw std::invalid_argument("outputGroupAction: camera model is null");
  }

  const std::vector<int> qIds = QIdsForMeasurement(X, measurement);
  if (qIds.size() != N_landmarkCount(X)) {
    throw std::invalid_argument(
        "outputGroupAction: invalid Q-to-id mapping cardinality");
  }

  for (size_t i = 0; i < N_landmarkCount(X); ++i) {
    const int id = qIds[i];
    const auto it = measurement.find(id);
    if (it == measurement.end()) continue;

    const Vector3 bearing = camera->undistortPoint(it->second);
    const Vector3 rotated =
        SOT3Rotation(Q_landmarkTransforms(X)[i]).matrix().transpose() * bearing;
    out[id] = camera->projectPoint(rotated);
  }

  return out;
}

Vector liftVelocity(const VIOState& state, const IMUInput& velocity) {
  const size_t N = state.n();
  Vector lift = Vector::Zero(21 + 4 * static_cast<int>(N));

  const VIOSensorState& sensor = state.sensor;
  const IMUInput v_est = velocity - sensor.inputBias;

  Vector6 U_A;
  U_A << v_est.gyr, sensor.velocity;
  const Vector6 U_B = sensor.cameraOffset.inverse().AdjointMap() * U_A;
  const Vector3 u_w = -v_est.acc + sensor.gravityDir() * GRAVITY_CONSTANT;

  lift.segment<6>(0) = U_A;
  lift.segment<3>(6) = u_w;
  lift.segment<6>(9) << velocity.accBiasVel, velocity.gyrBiasVel;
  lift.segment<6>(15) = U_B;

  const Vector6 U_C = sensor.cameraOffset.inverse().AdjointMap() * U_A;
  const Vector3 omegaC = U_C.head<3>();
  const Vector3 vC = U_C.tail<3>();

  for (size_t i = 0; i < N; ++i) {
    const Vector3 p = state.cameraLandmarks[i].p;
    Vector4 W;
    W.head<3>() = omegaC + Rot3::Hat(p) * vC / p.squaredNorm();
    W(3) = p.dot(vC) / p.squaredNorm();
    lift.segment<4>(21 + 4 * static_cast<int>(i)) = W;
  }

  return lift;
}

VIOGroup liftVelocityDiscrete(const VIOState& state, const IMUInput& velocity,
                              double dt) {
  const VIOSensorState& sensor = state.sensor;
  const IMUInput v_est = velocity - sensor.inputBias;

  const Rot3 dR = Rot3::Expmap(dt * v_est.gyr);
  const Vector3 dXWorld =
      dt * (sensor.pose.rotation() * sensor.velocity) +
      0.5 * dt * dt *
          (sensor.pose.rotation() * v_est.acc + Vector3(0, 0, -GRAVITY_CONSTANT));
  const Point3 dXBody = sensor.pose.rotation().unrotate(dXWorld);
  const Pose3 A_pose(dR, dXBody);

  const Vector3 bodyVelocityDiff =
      v_est.acc - sensor.gravityDir() * GRAVITY_CONSTANT;
  const Vector3 w = sensor.velocity - (sensor.velocity + dt * bodyVelocityDiff);

  const Pose3 B = sensor.cameraOffset.inverse().compose(A_pose).compose(
      sensor.cameraOffset);
  const Pose3 cameraPoseChangeInv = sensor.cameraOffset.inverse()
                                        .compose(A_pose.inverse())
                                        .compose(sensor.cameraOffset);

  std::vector<SOT3> q;
  q.resize(state.n());
  for (size_t i = 0; i < state.n(); ++i) {
    const Landmark& lm0 = state.cameraLandmarks[i];
    Landmark lm1;
    lm1.id = lm0.id;
    lm1.p = cameraPoseChangeInv.transformFrom(lm0.p);

    const Rot3 R = RotationFromTwoVectors(lm1.p, lm0.p);
    const double a = lm0.p.norm() / lm1.p.norm();
    q[i] = MakeSOT3(SO3(R.matrix()), a);
  }

  VIOLandmarkGroup Q(q);
  const VIOBias beta(dt * velocity.accBiasVel, dt * velocity.gyrBiasVel);
  const VIOSE23 A = MakeA(A_pose.rotation(), A_pose.translation(), w);
  return makeVIOGroup(A, beta, B, Q);
}

VIOState integrateSystemFunction(const VIOState& state, const IMUInput& velocity,
                                 double dt) {
  VIOState out;
  const VIOSensorState& sensor = state.sensor;
  const IMUInput v_est = velocity - sensor.inputBias;

  out.sensor.inputBias = VIOBias(
      sensor.inputBias.accelerometer() + dt * velocity.accBiasVel,
      sensor.inputBias.gyroscope() + dt * velocity.gyrBiasVel);

  const Rot3 dR = Rot3::Expmap(dt * v_est.gyr);
  const Vector3 dXWorld =
      dt * (sensor.pose.rotation() * sensor.velocity) +
      0.5 * dt * dt *
          (sensor.pose.rotation() * v_est.acc + Vector3(0, 0, -GRAVITY_CONSTANT));
  const Point3 dXBody = sensor.pose.rotation().unrotate(dXWorld);
  const Pose3 poseChange(dR, dXBody);

  out.sensor.pose = sensor.pose.compose(poseChange);

  const Vector3 inertialVelocityDiff =
      sensor.pose.rotation() * v_est.acc + Vector3(0, 0, -GRAVITY_CONSTANT);
  out.sensor.velocity = out.sensor.pose.rotation().unrotate(
      sensor.pose.rotation() * sensor.velocity + dt * inertialVelocityDiff);

  const Pose3 cameraPoseChangeInv = sensor.cameraOffset.inverse()
                                        .compose(poseChange.inverse())
                                        .compose(sensor.cameraOffset);
  out.cameraLandmarks.resize(state.n());
  for (size_t i = 0; i < state.n(); ++i) {
    out.cameraLandmarks[i].p =
        cameraPoseChangeInv.transformFrom(state.cameraLandmarks[i].p);
    out.cameraLandmarks[i].id = state.cameraLandmarks[i].id;
  }

  out.sensor.cameraOffset = sensor.cameraOffset;
  return out;
}

VisionMeasurement measureSystemState(
    const VIOState& state, const std::shared_ptr<const VIOCameraModel>& camera) {
  if (!camera) {
    throw std::invalid_argument("measureSystemState: camera model is null");
  }

  VisionMeasurement out;
  for (const Landmark& lm : state.cameraLandmarks) {
    out[lm.id] = camera->projectPoint(lm.p);
  }
  return out;
}

VIOState VIOSymmetry::operator()(
    const VIOState& xi, const VIOGroup& X,
    OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_xi,
    OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_X) const {
  const VIOState y = stateGroupAction(X, xi);

  if (H_xi) {
    const int d = xi.dim();
    H_xi->setZero(d, d);

    const Pose3 A = APose(X);

    Matrix66 Hpose;
    xi.sensor.pose.compose(A, Hpose, {});
    H_xi->block<6, 6>(6, 6) = Hpose;

    Matrix66 HtmpCamera, Hcamera;
    const Pose3 tmp = A.inverse().compose(xi.sensor.cameraOffset, {},
                                          HtmpCamera);
    (void)tmp.compose(B_cameraExtrinsics(X), Hcamera, {});
    H_xi->block<6, 6>(15, 15) = Hcamera * HtmpCamera;

    H_xi->block<6, 6>(0, 0).setIdentity();
    H_xi->block<3, 3>(12, 12) = A.rotation().matrix().transpose();

    for (size_t i = 0; i < xi.n(); ++i) {
      const int row = VIOSensorState::CompDim + 3 * static_cast<int>(i);
      H_xi->block<3, 3>(row, row) =
          (1.0 / SOT3Scale(Q_landmarkTransforms(X)[i])) *
          SOT3Rotation(Q_landmarkTransforms(X)[i]).matrix().transpose();
    }
  }

  if (H_X) {
    auto f = [&xi, this](const VIOGroup& g) { return this->operator()(xi, g); };
    *H_X = NumericalDerivativeActionWrtGroup(f, X, y);
  }

  return y;
}

}  // namespace eqvio
}  // namespace gtsam
