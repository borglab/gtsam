/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file EqVIOSymmetry.cpp
 * @brief EqVIO symmetry actions and lift helpers.
 * @author Rohan Bansal
 */


#include <gtsam_unstable/navigation/EqVIOSymmetry.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>

namespace gtsam {
namespace eqvio {

namespace {

Pose3 APose(const VioGroup& X) {
  const auto& A = gtsam::get<0>(X);
  return Pose3(A.rotation(), A.x(0));
}

Vector3 AW(const VioGroup& X) {
  const auto& A = gtsam::get<0>(X);
  return A.x(1);
}

Se23 MakeA(const Rot3& R, const Point3& x0, const Vector3& w) {
  Se23::Matrix3K x;
  x.col(0) = x0;
  x.col(1) = w;
  return Se23(R, x);
}

/// Construct the shortest-arc rotation that maps first vector to second vector.
Rot3 RotationFromTwoVectors(const Vector3& from, const Vector3& to) {
  gtsam::Quaternion q;
  q.setFromTwoVectors(from, to);
  return Rot3(q);
}

std::vector<int> QIdsForMeasurement(const VioGroup& X,
                                    const VisionMeasurement& measurement) {
  if (N_landmarkCount(X) == 0) return {};
  if (measurement.size() != N_landmarkCount(X)) {
    throw std::invalid_argument(
        "outputGroupAction: measurement count must match group landmark count");
  }
  return measurementIds(measurement);
}

/// Numerical derivative of the state action with respect to group coordinates.
Matrix NumericalDerivativeActionWrtGroup(
    const std::function<State(const VioGroup&)>& f, const VioGroup& X,
    const State& y0, double h = 1e-6) {
  const int n = static_cast<int>(Dim_groupTangent(X));
  const int m = y0.dim();
  Matrix H = Matrix::Zero(m, n);

  for (int j = 0; j < n; ++j) {
    Vector dx = Vector::Zero(n);
    dx(j) = h;
    const State yPlus = f(X.retract(dx));
    dx(j) = -h;
    const State yMinus = f(X.retract(dx));

    H.col(j) =
        (y0.localCoordinates(yPlus) - y0.localCoordinates(yMinus)) / (2.0 * h);
  }

  return H;
}

Matrix NumericalDifferential(const std::function<Vector(const Vector&)>& f,
                             const Vector& x, double h = 1e-6) {
  const int n = static_cast<int>(x.size());
  const Vector y = f(x);
  const int m = static_cast<int>(y.size());

  Matrix J = Matrix::Zero(m, n);
  for (int j = 0; j < n; ++j) {
    Vector dx = Vector::Zero(n);
    dx(j) = h;
    J.col(j) = (f(x + dx) - f(x - dx)) / (2.0 * h);
  }
  return J;
}

void CheckStateChartAlignment(const State& Xi, const State& Xi0,
                              const char* context) {
  if (Xi.n() != Xi0.n()) {
    throw std::invalid_argument(std::string(context) +
                                ": landmark counts do not match");
  }
  for (size_t i = 0; i < Xi.n(); ++i) {
    if (Xi.cameraLandmarks[i].id != Xi0.cameraLandmarks[i].id) {
      throw std::invalid_argument(std::string(context) +
                                  ": landmark ids are not aligned");
    }
  }
}

Vector2 E3ProjectSphere(const Vector3& eta) {
  static const Matrix23 I23 = Matrix23::Identity();
  static const Vector3 e3 = Vector3::UnitZ();
  return I23 * (eta - e3) / (1.0 - e3.dot(eta));
}

Vector3 E3ProjectSphereInv(const Vector2& y) {
  static const Vector3 e3 = Vector3::UnitZ();
  const Vector3 yBar = (Vector3() << y, 0.0).finished();
  return e3 + 2.0 / (yBar.squaredNorm() + 1.0) * (yBar - e3);
}

Matrix23 E3ProjectSphereDiff(const Vector3& eta) {
  static const Matrix23 I23 = Matrix23::Identity();
  static const Vector3 e3 = Vector3::UnitZ();
  Matrix23 diff =
      I23 * (Matrix3::Identity() * (1.0 - eta.z()) + (eta - e3) * e3.transpose());
  diff /= std::pow(1.0 - e3.dot(eta), 2.0);
  return diff;
}

Matrix32 E3ProjectSphereInvDiff(const Vector2& y) {
  Matrix32 diff;
  diff.block<2, 2>(0, 0) =
      Matrix2::Identity() * (y.squaredNorm() + 1.0) - 2.0 * y * y.transpose();
  diff.block<1, 2>(2, 0) = 2.0 * y.transpose();
  diff *= 2.0 / std::pow(y.squaredNorm() + 1.0, 2.0);
  return diff;
}

Vector2 SphereChartStereo(const Vector3& eta, const Vector3& pole) {
  const Rot3 sphereRot = RotationFromTwoVectors(-pole, Vector3::UnitZ());
  const Vector3 etaRotated = sphereRot.matrix() * eta;
  return E3ProjectSphere(etaRotated);
}

Vector3 SphereChartStereoInv(const Vector2& y, const Vector3& pole) {
  const Vector3 etaRotated = E3ProjectSphereInv(y);
  const Rot3 sphereRot = RotationFromTwoVectors(-pole, Vector3::UnitZ());
  return sphereRot.matrix().transpose() * etaRotated;
}

Matrix23 SphereChartStereoDiff0(const Vector3& pole) {
  const Rot3 sphereRot = RotationFromTwoVectors(-pole, Vector3::UnitZ());
  const Vector3 etaRotated = sphereRot.matrix() * pole;
  return E3ProjectSphereDiff(etaRotated) * sphereRot.matrix();
}

Matrix32 SphereChartStereoInvDiff0(const Vector3& pole) {
  const Rot3 sphereRot = RotationFromTwoVectors(-pole, Vector3::UnitZ());
  return sphereRot.matrix().transpose() * E3ProjectSphereInvDiff(Vector2::Zero());
}

Vector3 PointChartInvDepth(const Point3& q, const Point3& q0) {
  const double rho = 1.0 / q.norm();
  const double rho0 = 1.0 / q0.norm();
  const Vector3 y = q * rho;
  const Vector3 y0 = q0 * rho0;

  Vector3 eps;
  eps.head<2>() = SphereChartStereo(y, y0);
  eps(2) = rho - rho0;
  return eps;
}

Point3 PointChartInvDepthInv(const Vector3& eps, const Point3& q0) {
  const double rho0 = 1.0 / q0.norm();
  const Vector3 y0 = q0 * rho0;
  const Vector3 y = SphereChartStereoInv(eps.head<2>(), y0);

  double rho = eps(2) + rho0;
  if (rho <= 0.0) rho = 1e-6;
  return y / rho;
}

Matrix3 ConvEucToInvDepth(const Point3& q0) {
  const double rho0 = 1.0 / q0.norm();
  const Vector3 y0 = q0 * rho0;

  Matrix3 M;
  M.block<2, 3>(0, 0) = rho0 * SphereChartStereoDiff0(y0) *
                        (Matrix3::Identity() - y0 * y0.transpose());
  M.block<1, 3>(2, 0) = -rho0 * rho0 * y0.transpose();
  return M;
}

Matrix3 ConvInvDepthToEuc(const Point3& q0) {
  const double rho0 = 1.0 / q0.norm();
  const Vector3 y0 = q0 * rho0;

  Matrix3 M;
  M.block<3, 2>(0, 0) = SphereChartStereoInvDiff0(y0) / rho0;
  M.block<3, 1>(0, 2) = -y0 / (rho0 * rho0);
  return M;
}

Vector StateChartInvDepth(const State& Xi, const State& Xi0) {
  CheckStateChartAlignment(Xi, Xi0, "stateChart_invdepth");

  Vector eps = Vector::Zero(Xi0.dim());
  eps.segment<6>(0) = (Xi.sensor.inputBias - Xi0.sensor.inputBias).vector();
  eps.segment<6>(6) = Xi0.sensor.pose.localCoordinates(Xi.sensor.pose);
  eps.segment<3>(12) = Xi.sensor.velocity - Xi0.sensor.velocity;
  eps.segment<6>(15) =
      Xi0.sensor.cameraOffset.localCoordinates(Xi.sensor.cameraOffset);

  for (size_t i = 0; i < Xi0.n(); ++i) {
    eps.segment<3>(SensorState::CompDim + 3 * static_cast<int>(i)) =
        PointChartInvDepth(Xi.cameraLandmarks[i].p, Xi0.cameraLandmarks[i].p);
  }
  return eps;
}

State StateChartInvDepthInv(const Vector& eps, const State& Xi0) {
  if (eps.size() != Xi0.dim()) {
    throw std::invalid_argument(
        "stateChartInv_invdepth: chart vector dimension mismatch");
  }

  State Xi;
  Xi.sensor.inputBias = Xi0.sensor.inputBias + eps.segment<6>(0);
  Xi.sensor.pose = Xi0.sensor.pose.retract(eps.segment<6>(6));
  Xi.sensor.velocity = Xi0.sensor.velocity + eps.segment<3>(12);
  Xi.sensor.cameraOffset = Xi0.sensor.cameraOffset.retract(eps.segment<6>(15));

  Xi.cameraLandmarks.resize(Xi0.n());
  for (size_t i = 0; i < Xi0.n(); ++i) {
    const int k = SensorState::CompDim + 3 * static_cast<int>(i);
    Xi.cameraLandmarks[i].id = Xi0.cameraLandmarks[i].id;
    Xi.cameraLandmarks[i].p =
        PointChartInvDepthInv(eps.segment<3>(k), Xi0.cameraLandmarks[i].p);
  }
  return Xi;
}

Matrix EqFInputMatrixB_invdepth(const VioGroup& X, const State& xi0);

Matrix EqFStateMatrixA_invdepth(const VioGroup& X, const State& xi0,
                                const IMUInput& imuVel) {
  const int N = static_cast<int>(xi0.n());
  Matrix A0t = Matrix::Zero(xi0.dim(), xi0.dim());

  const Matrix B = EqFInputMatrixB_invdepth(X, xi0);
  A0t.block(0, 0, xi0.dim(), 3) = -B.block(0, 3, xi0.dim(), 3);
  A0t.block(0, 3, xi0.dim(), 3) = -B.block(0, 0, xi0.dim(), 3);
  A0t.block<3, 3>(9, 12).setIdentity();
  A0t.block<3, 3>(12, 6) = -GRAVITY_CONSTANT * Rot3::Hat(xi0.sensor.gravityDir());

  const State xiHat = stateGroupAction(X, xi0);
  const IMUInput vEst = imuVel - xiHat.sensor.inputBias;
  Vector6 U_I;
  U_I << vEst.gyr, xiHat.sensor.velocity;

  const Pose3 A = APose(X);
  const Vector6 commonTwist =
      xi0.sensor.cameraOffset.inverse().AdjointMap() * A.AdjointMap() * U_I;
  A0t.block<6, 6>(15, 15) = Pose3::adjointMap(commonTwist);

  const Matrix3 R_IC = xiHat.sensor.cameraOffset.rotation().matrix();
  const Matrix3 R_A = A.rotation().matrix();
  for (int i = 0; i < N; ++i) {
    const Point3 q0 = xi0.cameraLandmarks[static_cast<size_t>(i)].p;
    const Matrix3 Qhat_i =
        SOT3ScaledRotation(Q_landmarkTransforms(X)[static_cast<size_t>(i)]);
    A0t.block<3, 3>(SensorState::CompDim + 3 * i, 12) =
        -ConvEucToInvDepth(q0) * Qhat_i * R_IC.transpose() * R_A.transpose();
  }

  const Matrix66 commonTerm =
      B_cameraExtrinsics(X).inverse().AdjointMap() * Pose3::adjointMap(commonTwist);
  for (int i = 0; i < N; ++i) {
    const Point3 q0 = xi0.cameraLandmarks[static_cast<size_t>(i)].p;
    Matrix36 temp;
    temp << Rot3::Hat(q0) *
                SOT3Rotation(Q_landmarkTransforms(X)[static_cast<size_t>(i)])
                    .matrix(),
        -SOT3Scale(Q_landmarkTransforms(X)[static_cast<size_t>(i)]) *
            SOT3Rotation(Q_landmarkTransforms(X)[static_cast<size_t>(i)])
                .matrix();
    A0t.block<3, 6>(SensorState::CompDim + 3 * i, 15) =
        ConvEucToInvDepth(q0) * temp * commonTerm;
  }

  const Vector6 U_C = xiHat.sensor.cameraOffset.inverse().AdjointMap() * U_I;
  const Vector3 v_C = U_C.tail<3>();
  for (int i = 0; i < N; ++i) {
    const Point3 q0 = xi0.cameraLandmarks[static_cast<size_t>(i)].p;
    const Matrix3 Qhat_i =
        SOT3ScaledRotation(Q_landmarkTransforms(X)[static_cast<size_t>(i)]);
    const Point3 qhat_i = xiHat.cameraLandmarks[static_cast<size_t>(i)].p;
    const Matrix3 A_qi =
        -Qhat_i *
        (Rot3::Hat(qhat_i) * Rot3::Hat(v_C) - 2.0 * v_C * qhat_i.transpose() +
         qhat_i * v_C.transpose()) *
        Qhat_i.inverse() * (1.0 / qhat_i.squaredNorm());
    A0t.block<3, 3>(SensorState::CompDim + 3 * i,
                    SensorState::CompDim + 3 * i) =
        ConvEucToInvDepth(q0) * A_qi * ConvInvDepthToEuc(q0);
  }

  return A0t;
}

Matrix EqFInputMatrixB_invdepth(const VioGroup& X, const State& xi0) {
  const int N = static_cast<int>(xi0.n());
  Matrix Bt = Matrix::Zero(xi0.dim(), IMUInput::CompDim);

  const State xiHat = stateGroupAction(X, xi0);
  const Pose3 A = APose(X);

  Bt.block<3, 3>(0, 9).setIdentity();
  Bt.block<3, 3>(3, 6).setIdentity();

  const Matrix3 R_A = A.rotation().matrix();
  Bt.block<3, 3>(6, 0) = R_A;
  Bt.block<3, 3>(9, 0) = Rot3::Hat(A.translation()) * R_A;
  Bt.block<3, 3>(12, 0) = R_A * Rot3::Hat(xiHat.sensor.velocity);
  Bt.block<3, 3>(12, 3) = R_A;

  const Matrix3 RT_IC = xiHat.sensor.cameraOffset.rotation().matrix().transpose();
  const Point3 x_IC = xiHat.sensor.cameraOffset.translation();
  for (int i = 0; i < N; ++i) {
    const Point3 q0 = xi0.cameraLandmarks[static_cast<size_t>(i)].p;
    const Matrix3 Qhat_i =
        SOT3ScaledRotation(Q_landmarkTransforms(X)[static_cast<size_t>(i)]);
    const Point3 qhat_i = xiHat.cameraLandmarks[static_cast<size_t>(i)].p;
    Bt.block<3, 3>(SensorState::CompDim + 3 * i, 0) =
        ConvEucToInvDepth(q0) *
        Qhat_i * (Rot3::Hat(qhat_i) * RT_IC + RT_IC * Rot3::Hat(x_IC));
  }

  return Bt;
}

Matrix23 EqFoutputMatrixCiStar_base(
    const Point3& q0, const SOT3& QHat,
    const std::shared_ptr<const CameraModel>& camera, const Point2& y) {
  if (!camera) {
    throw std::invalid_argument("EqFoutputMatrixCiStar_base: null camera");
  }

  using Matrix43 = Eigen::Matrix<double, 4, 3>;
  using Matrix34 = Eigen::Matrix<double, 3, 4>;
  using Matrix24 = Eigen::Matrix<double, 2, 4>;

  const Vector3 qHat = SOT3ApplyInverse(QHat, q0);
  const Vector3 yHat = qHat.normalized();

  Matrix43 m2g = Matrix43::Zero();
  m2g.block<3, 3>(0, 0) = -Rot3::Hat(q0);
  m2g.row(3) = -q0.transpose();
  m2g /= q0.squaredNorm();

  const auto DRho = [&camera](const Vector3& yVec) -> Matrix24 {
    Matrix34 DRhoVec = Matrix34::Zero();
    DRhoVec.block<3, 3>(0, 0) = Rot3::Hat(yVec);
    return projectionJacobian(*camera, yVec) * DRhoVec;
  };

  const Vector3 yTru = undistortPoint(*camera, y);
  const Matrix24 drhoSym = 0.5 * (DRho(yTru) + DRho(yHat));
  const Matrix44 adjQInv = QHat.inverse().AdjointMap();
  return drhoSym * adjQInv * m2g;
}

Matrix23 EqFoutputMatrixCiStar_invdepth(
    const Point3& q0, const SOT3& QHat,
    const std::shared_ptr<const CameraModel>& camera, const Point2& y) {
  const double r0 = q0.norm();
  const Vector3 y0 = q0 / r0;
  Matrix3 ind2euc;
  ind2euc.block<3, 2>(0, 0) = r0 * SphereChartStereoInvDiff0(y0);
  ind2euc.block<3, 1>(0, 2) = -r0 * q0;
  return EqFoutputMatrixCiStar_base(q0, QHat, camera, y) * ind2euc;
}

Vector liftInnovation_invdepth(const Vector& totalInnovation,
                               const State& xi0) {
  if (totalInnovation.size() != xi0.dim()) {
    throw std::invalid_argument(
        "liftInnovation_invdepth: innovation dimension mismatch");
  }

  const int N = static_cast<int>(xi0.n());
  Vector lift = Vector::Zero(21 + 4 * N);

  lift.segment<6>(9) = totalInnovation.segment<6>(0);
  lift.segment<6>(0) = totalInnovation.segment<6>(6);

  const Vector3 gammaV = totalInnovation.segment<3>(12);
  lift.segment<3>(6) = -gammaV - Rot3::Hat(lift.segment<3>(0)) * xi0.sensor.velocity;

  lift.segment<6>(15) =
      totalInnovation.segment<6>(15) +
      xi0.sensor.cameraOffset.inverse().AdjointMap() * lift.segment<6>(0);

  for (int i = 0; i < N; ++i) {
    const Point3 qi0 = xi0.cameraLandmarks[static_cast<size_t>(i)].p;
    const Vector3 gammaQi0 =
        ConvInvDepthToEuc(qi0) *
        totalInnovation.segment<3>(SensorState::CompDim + 3 * i);

    lift.segment<3>(21 + 4 * i) = -qi0.cross(gammaQi0) / qi0.squaredNorm();
    lift(21 + 4 * i + 3) = -qi0.dot(gammaQi0) / qi0.squaredNorm();
  }

  return lift;
}

VioGroup liftInnovationDiscrete_invdepth(const Vector& totalInnovation,
                                         const State& xi0) {
  if (totalInnovation.size() != xi0.dim()) {
    throw std::invalid_argument(
        "liftInnovationDiscrete_invdepth: innovation dimension mismatch");
  }

  const Bias beta(totalInnovation.segment<6>(0));
  const Pose3 A_pose = Pose3::Expmap(totalInnovation.segment<6>(6));
  const Vector3 w = xi0.sensor.velocity -
                    A_pose.rotation().matrix() *
                        (xi0.sensor.velocity + totalInnovation.segment<3>(12));

  const Pose3 B = xi0.sensor.cameraOffset.inverse()
                      .compose(A_pose)
                      .compose(xi0.sensor.cameraOffset)
                      .compose(Pose3::Expmap(totalInnovation.segment<6>(15)));

  std::vector<SOT3> Q;
  Q.reserve(xi0.n());

  for (size_t i = 0; i < xi0.n(); ++i) {
    const Point3 qi0 = xi0.cameraLandmarks[i].p;
    const Point3 qi1 =
        PointChartInvDepthInv(totalInnovation.segment<3>(
                                  SensorState::CompDim + 3 * static_cast<int>(i)),
                              qi0);
    const Rot3 R = RotationFromTwoVectors(qi1.normalized(), qi0.normalized());
    const double a = qi0.norm() / qi1.norm();
    Q.emplace_back(MakeSOT3(SO3(R.matrix()), a));
  }

  return makeVioGroup(MakeA(A_pose.rotation(), A_pose.translation(), w), beta, B,
                      LandmarkGroup(Q));
}

}  // namespace

SensorState sensorStateGroupAction(const VioGroup& X,
                                      const SensorState& sensor) {
  const auto& Beta = gtsam::get<1>(X);
  const auto& B = gtsam::get<2>(X);
  const Pose3 A = APose(X);
  const Vector3 w = AW(X);

  SensorState out;
  out.inputBias = sensor.inputBias + Beta;
  out.pose = sensor.pose.compose(A);
  out.velocity = A.rotation().unrotate(sensor.velocity - w);
  out.cameraOffset = A.inverse().compose(sensor.cameraOffset).compose(B);
  return out;
}

State stateGroupAction(const VioGroup& X, const State& state) {
  const auto& Q = gtsam::get<3>(X);
  if (Q.size() != state.n()) {
    throw std::invalid_argument(
        "stateGroupAction: group and state landmark counts do not match");
  }

  State out;
  out.sensor = sensorStateGroupAction(X, state.sensor);
  out.cameraLandmarks.resize(Q.size());

  for (size_t i = 0; i < Q.size(); ++i) {
    out.cameraLandmarks[i].p =
        SOT3ApplyInverse(Q[i], state.cameraLandmarks[i].p);
    out.cameraLandmarks[i].id = state.cameraLandmarks[i].id;
  }

  return out;
}

VisionMeasurement outputGroupAction(const VioGroup& X,
                                    const VisionMeasurement& measurement,
                                    const std::shared_ptr<const CameraModel>& camera) {
  const auto& Q = gtsam::get<3>(X);
  VisionMeasurement out;
  if (!camera) {
    throw std::invalid_argument("outputGroupAction: camera model is null");
  }

  const std::vector<int> qIds = QIdsForMeasurement(X, measurement);
  if (qIds.size() != Q.size()) {
    throw std::invalid_argument(
        "outputGroupAction: invalid Q-to-id mapping cardinality");
  }

  for (size_t i = 0; i < Q.size(); ++i) {
    const int id = qIds[i];
    const auto it = measurement.find(id);
    if (it == measurement.end()) continue;

    const Vector3 bearing = undistortPoint(*camera, it->second);
    const Vector3 rotated = SOT3Rotation(Q[i]).matrix().transpose() * bearing;
    out[id] = camera->project2(rotated);
  }

  return out;
}

Vector liftVelocity(const State& state, const IMUInput& velocity) {
  const size_t N = state.n();
  Vector lift = Vector::Zero(21 + 4 * static_cast<int>(N));

  const SensorState& sensor = state.sensor;
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

  // Lift the landmark transform velocities
  for (size_t i = 0; i < N; ++i) {
    const Vector3 p = state.cameraLandmarks[i].p;
    Vector4 W;
    W.head<3>() = omegaC + Rot3::Hat(p) * vC / p.squaredNorm();
    W(3) = p.dot(vC) / p.squaredNorm();
    lift.segment<4>(21 + 4 * static_cast<int>(i)) = W;
  }

  return lift;
}

VioGroup liftVelocityDiscrete(const State& state, const IMUInput& velocity,
                              double dt) {
  const SensorState& sensor = state.sensor;
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

  // Construct the landmark transform velocities
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

  LandmarkGroup Q(q);
  const Bias beta(dt * velocity.accBiasVel, dt * velocity.gyrBiasVel);
  const Se23 A = MakeA(A_pose.rotation(), A_pose.translation(), w);
  return makeVioGroup(A, beta, B, Q);
}

State integrateSystemFunction(const State& state, const IMUInput& velocity,
                                 double dt) {
  State out;
  const SensorState& sensor = state.sensor;
  const IMUInput v_est = velocity - sensor.inputBias;

  out.sensor.inputBias = Bias(
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

  // Transform the body-fixed landmarks
  for (size_t i = 0; i < state.n(); ++i) {
    out.cameraLandmarks[i].p =
        cameraPoseChangeInv.transformFrom(state.cameraLandmarks[i].p);
    out.cameraLandmarks[i].id = state.cameraLandmarks[i].id;
  }

  out.sensor.cameraOffset = sensor.cameraOffset;
  return out;
}

/// Predict ideal normalized image measurements from current landmark state.
VisionMeasurement measureSystemState(
    const State& state, const std::shared_ptr<const CameraModel>& camera) {
  if (!camera) {
    throw std::invalid_argument("measureSystemState: camera model is null");
  }

  // Project each landmark through the camera model
  VisionMeasurement out;
  for (const Landmark& lm : state.cameraLandmarks) {
    out[lm.id] = camera->project2(lm.p);
  }
  return out;
}

const EqFCoordinateSuite EqFCoordinateSuite_invdepth{
    StateChartInvDepth,
    StateChartInvDepthInv,
    EqFStateMatrixA_invdepth,
    EqFInputMatrixB_invdepth,
    EqFoutputMatrixCiStar_invdepth,
    liftInnovation_invdepth,
    liftInnovationDiscrete_invdepth};

Matrix EqFCoordinateSuite::outputMatrixC(
    const State& xi0, const VioGroup& X, const VisionMeasurement& y,
    const std::shared_ptr<const CameraModel>& camera,
    bool useEquivariance) const {
  if (!camera) {
    throw std::invalid_argument("EqFCoordinateSuite::outputMatrixC: null camera");
  }
  const int M = static_cast<int>(xi0.n());
  const std::vector<int> yIds = measurementIds(y);
  const int N = static_cast<int>(yIds.size());
  const auto& Q = gtsam::get<3>(X);

  Matrix C = Matrix::Zero(2 * N, SensorState::CompDim + Landmark::CompDim * M);

  for (int i = 0; i < M; ++i) {
    const int idNum = xi0.cameraLandmarks[static_cast<size_t>(i)].id;
    const auto itY = std::find(yIds.begin(), yIds.end(), idNum);
    if (itY == yIds.end()) continue;

    const size_t k = static_cast<size_t>(i);
    const int j = static_cast<int>(std::distance(yIds.begin(), itY));
    const Point3& qi0 = xi0.cameraLandmarks[static_cast<size_t>(i)].p;
    const SOT3& Qk = Q[k];

    const Matrix23 Ci = useEquivariance
                            ? outputMatrixCiStar(qi0, Qk, camera, y.at(idNum))
                            : outputMatrixCi(qi0, Qk, camera);
    if (!Ci.array().isFinite().all()) {
      const Point3 qHat = SOT3ApplyInverse(Qk, qi0);
      const Point2 yObs = y.at(idNum);
      const Vector3 yUnd = undistortPoint(*camera, yObs);
      throw std::runtime_error(
          "EqFCoordinateSuite::outputMatrixC: non-finite Ci for id " +
          std::to_string(idNum) + ", qi0_norm=" + std::to_string(qi0.norm()) +
          ", qHat_norm=" + std::to_string(qHat.norm()) +
          ", Q_scale=" + std::to_string(SOT3Scale(Qk)) +
          ", yUnd_norm=" + std::to_string(yUnd.norm()));
    }
    C.block<2, 3>(2 * j, SensorState::CompDim + 3 * i) = Ci;
  }

  if (!C.array().isFinite().all()) {
    throw std::runtime_error("EqFCoordinateSuite::outputMatrixC produced NaN/Inf");
  }
  return C;
}

Matrix EqFCoordinateSuite::stateMatrixADiscrete(const VioGroup& X,
                                                const State& xi0,
                                                const IMUInput& imuVel,
                                                double dt) const {
  auto a0Discrete = [&](const Vector& epsilon) {
    const State xiE = stateChartInv(epsilon, xi0);
    const State xiHat = stateGroupAction(X, xi0);
    const State xi = stateGroupAction(X, xiE);
    const VioGroup lambdaTilde =
        liftVelocityDiscrete(xi, imuVel, dt) *
        liftVelocityDiscrete(xiHat, imuVel, dt).inverse();
    const State xiE1 = stateGroupAction(X * lambdaTilde * X.inverse(), xiE);
    return stateChart(xiE1, xi0);
  };

  return NumericalDifferential(a0Discrete, Vector::Zero(xi0.dim()));
}

Matrix23 EqFCoordinateSuite::outputMatrixCi(
    const Point3& q0, const SOT3& QHat,
    const std::shared_ptr<const CameraModel>& camera) const {
  if (!camera) {
    throw std::invalid_argument("EqFCoordinateSuite::outputMatrixCi: null camera");
  }
  const Vector3 qHat = SOT3ApplyInverse(QHat, q0);
  const Point2 yHat = camera->project2(qHat);
  return outputMatrixCiStar(q0, QHat, camera, yHat);
}

State Symmetry::operator()(
    const State& xi, const VioGroup& X,
    OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_xi,
    OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H_X) const {
  const State y = stateGroupAction(X, xi);

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
    const auto& B = gtsam::get<2>(X);
    const auto& Q = gtsam::get<3>(X);
    tmp.compose(B, Hcamera, {});
    H_xi->block<6, 6>(15, 15) = Hcamera * HtmpCamera;

    H_xi->block<6, 6>(0, 0).setIdentity();
    H_xi->block<3, 3>(12, 12) = A.rotation().matrix().transpose();

    // Compute the Jacobian of the landmark transform velocities
    for (size_t i = 0; i < xi.n(); ++i) {
      const int row = SensorState::CompDim + 3 * static_cast<int>(i);
      H_xi->block<3, 3>(row, row) =
          (1.0 / SOT3Scale(Q[i])) * SOT3Rotation(Q[i]).matrix().transpose();
    }
  }

  if (H_X) {
    auto f = [&xi, this](const VioGroup& g) { return this->operator()(xi, g); };
    *H_X = NumericalDerivativeActionWrtGroup(f, X, y);
  }

  return y;
}

}  // namespace eqvio
}  // namespace gtsam
