/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VIOEqFMatrices.cpp
 * @brief   EqF coordinate suites for VIO foundations
 */

#include <gtsam_unstable/navigation/VIOEqFMatrices.h>

#include <gtsam_unstable/navigation/EqVIOSymmetry.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace gtsam {
namespace eqvio {

namespace {

double SOT3Scale(const SOT3& Q) { return std::exp(Q.second); }

const SO3& SOT3Rotation(const SOT3& Q) { return Q.first; }

Vector3 SOT3ApplyInverse(const SOT3& Q, const Vector3& p) {
  return (1.0 / SOT3Scale(Q)) * (SOT3Rotation(Q).matrix().transpose() * p);
}

Matrix3 SOT3ScaledRotation(const SOT3& Q) {
  return SOT3Rotation(Q).matrix() * SOT3Scale(Q);
}

SOT3 MakeSOT3(const SO3& R, double scale) {
  if (scale <= 0.0) {
    throw std::invalid_argument("MakeSOT3: scale must be strictly positive");
  }
  return SOT3(R, std::log(scale));
}

Pose3 APose(const VIOGroup& X) {
  return Pose3(A_sensorKinematics(X).rotation(), A_sensorKinematics(X).x(0));
}

VIOSE23 MakeA(const Rot3& R, const Point3& t, const Vector3& w) {
  VIOSE23::Matrix3K x;
  x.col(0) = t;
  x.col(1) = w;
  return VIOSE23(R, x);
}

Rot3 RotationFromTwoVectors(const Vector3& from, const Vector3& to) {
  gtsam::Quaternion q;
  q.setFromTwoVectors(from, to);
  return Rot3(q);
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

void CheckStateChartAlignment(const VIOState& Xi, const VIOState& Xi0,
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

Vector StateChartInvDepth(const VIOState& Xi, const VIOState& Xi0) {
  CheckStateChartAlignment(Xi, Xi0, "stateChart_invdepth");

  Vector eps = Vector::Zero(Xi0.dim());
  eps.segment<6>(0) =
      (Xi.sensor.inputBias - Xi0.sensor.inputBias).vector();
  eps.segment<6>(6) = Xi0.sensor.pose.localCoordinates(Xi.sensor.pose);
  eps.segment<3>(12) = Xi.sensor.velocity - Xi0.sensor.velocity;
  eps.segment<6>(15) = Xi0.sensor.cameraOffset.localCoordinates(Xi.sensor.cameraOffset);

  for (size_t i = 0; i < Xi0.n(); ++i) {
    eps.segment<3>(VIOSensorState::CompDim + 3 * static_cast<int>(i)) =
        PointChartInvDepth(Xi.cameraLandmarks[i].p, Xi0.cameraLandmarks[i].p);
  }
  return eps;
}

VIOState StateChartInvDepthInv(const Vector& eps, const VIOState& Xi0) {
  if (eps.size() != Xi0.dim()) {
    throw std::invalid_argument(
        "stateChartInv_invdepth: chart vector dimension mismatch");
  }

  VIOState Xi;
  Xi.sensor.inputBias = Xi0.sensor.inputBias + eps.segment<6>(0);
  Xi.sensor.pose = Xi0.sensor.pose.retract(eps.segment<6>(6));
  Xi.sensor.velocity = Xi0.sensor.velocity + eps.segment<3>(12);
  Xi.sensor.cameraOffset = Xi0.sensor.cameraOffset.retract(eps.segment<6>(15));

  Xi.cameraLandmarks.resize(Xi0.n());
  for (size_t i = 0; i < Xi0.n(); ++i) {
    const int k = VIOSensorState::CompDim + 3 * static_cast<int>(i);
    Xi.cameraLandmarks[i].id = Xi0.cameraLandmarks[i].id;
    Xi.cameraLandmarks[i].p =
        PointChartInvDepthInv(eps.segment<3>(k), Xi0.cameraLandmarks[i].p);
  }
  return Xi;
}

Matrix EqFInputMatrixB_euclid(const VIOGroup& X, const VIOState& xi0);
Matrix EqFInputMatrixB_invdepth(const VIOGroup& X, const VIOState& xi0);

Matrix EqFStateMatrixA_euclid(const VIOGroup& X, const VIOState& xi0,
                              const IMUInput& imuVel) {
  const int N = static_cast<int>(xi0.n());
  Matrix A0t = Matrix::Zero(xi0.dim(), xi0.dim());

  const Matrix B = EqFInputMatrixB_euclid(X, xi0);
  A0t.block(0, 0, xi0.dim(), 3) = -B.block(0, 3, xi0.dim(), 3);
  A0t.block(0, 3, xi0.dim(), 3) = -B.block(0, 0, xi0.dim(), 3);
  A0t.block<3, 3>(9, 12).setIdentity();
  A0t.block<3, 3>(12, 6) = -GRAVITY_CONSTANT * Rot3::Hat(xi0.sensor.gravityDir());

  const VIOState xiHat = stateGroupAction(X, xi0);
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
    const Matrix3 Qhat_i = SOT3ScaledRotation(
        Q_landmarkTransforms(X)[static_cast<size_t>(i)]);
    A0t.block<3, 3>(VIOSensorState::CompDim + 3 * i, 12) =
        -Qhat_i * R_IC.transpose() * R_A.transpose();
  }

  const Matrix66 commonTerm =
      B_cameraExtrinsics(X).inverse().AdjointMap() * Pose3::adjointMap(commonTwist);
  for (int i = 0; i < N; ++i) {
    Matrix36 temp;
    temp << Rot3::Hat(xi0.cameraLandmarks[static_cast<size_t>(i)].p) *
                SOT3Rotation(Q_landmarkTransforms(X)[static_cast<size_t>(i)]).matrix(),
        -SOT3Scale(Q_landmarkTransforms(X)[static_cast<size_t>(i)]) *
            SOT3Rotation(Q_landmarkTransforms(X)[static_cast<size_t>(i)]).matrix();
    A0t.block<3, 6>(VIOSensorState::CompDim + 3 * i, 15) = temp * commonTerm;
  }

  const Vector6 U_C = xiHat.sensor.cameraOffset.inverse().AdjointMap() * U_I;
  const Vector3 v_C = U_C.tail<3>();
  for (int i = 0; i < N; ++i) {
    const Matrix3 Qhat_i = SOT3ScaledRotation(
        Q_landmarkTransforms(X)[static_cast<size_t>(i)]);
    const Vector3 qhat_i = xiHat.cameraLandmarks[static_cast<size_t>(i)].p;
    const Matrix3 A_qi =
        -Qhat_i *
        (Rot3::Hat(qhat_i) * Rot3::Hat(v_C) - 2.0 * v_C * qhat_i.transpose() +
         qhat_i * v_C.transpose()) *
        Qhat_i.inverse() * (1.0 / qhat_i.squaredNorm());
    A0t.block<3, 3>(VIOSensorState::CompDim + 3 * i,
                    VIOSensorState::CompDim + 3 * i) = A_qi;
  }

  return A0t;
}

Matrix EqFStateMatrixA_invdepth(const VIOGroup& X, const VIOState& xi0,
                                const IMUInput& imuVel) {
  const int N = static_cast<int>(xi0.n());
  Matrix A0t = Matrix::Zero(xi0.dim(), xi0.dim());

  const Matrix B = EqFInputMatrixB_invdepth(X, xi0);
  A0t.block(0, 0, xi0.dim(), 3) = -B.block(0, 3, xi0.dim(), 3);
  A0t.block(0, 3, xi0.dim(), 3) = -B.block(0, 0, xi0.dim(), 3);
  A0t.block<3, 3>(9, 12).setIdentity();
  A0t.block<3, 3>(12, 6) = -GRAVITY_CONSTANT * Rot3::Hat(xi0.sensor.gravityDir());

  const VIOState xiHat = stateGroupAction(X, xi0);
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
    const Matrix3 Qhat_i = SOT3ScaledRotation(
        Q_landmarkTransforms(X)[static_cast<size_t>(i)]);
    A0t.block<3, 3>(VIOSensorState::CompDim + 3 * i, 12) =
        -ConvEucToInvDepth(q0) * Qhat_i * R_IC.transpose() * R_A.transpose();
  }

  const Matrix66 commonTerm =
      B_cameraExtrinsics(X).inverse().AdjointMap() * Pose3::adjointMap(commonTwist);
  for (int i = 0; i < N; ++i) {
    const Point3 q0 = xi0.cameraLandmarks[static_cast<size_t>(i)].p;
    Matrix36 temp;
    temp << Rot3::Hat(q0) * SOT3Rotation(Q_landmarkTransforms(X)[static_cast<size_t>(i)]).matrix(),
        -SOT3Scale(Q_landmarkTransforms(X)[static_cast<size_t>(i)]) *
            SOT3Rotation(Q_landmarkTransforms(X)[static_cast<size_t>(i)]).matrix();
    A0t.block<3, 6>(VIOSensorState::CompDim + 3 * i, 15) =
        ConvEucToInvDepth(q0) * temp * commonTerm;
  }

  const Vector6 U_C = xiHat.sensor.cameraOffset.inverse().AdjointMap() * U_I;
  const Vector3 v_C = U_C.tail<3>();
  for (int i = 0; i < N; ++i) {
    const Point3 q0 = xi0.cameraLandmarks[static_cast<size_t>(i)].p;
    const Matrix3 Qhat_i = SOT3ScaledRotation(
        Q_landmarkTransforms(X)[static_cast<size_t>(i)]);
    const Point3 qhat_i = xiHat.cameraLandmarks[static_cast<size_t>(i)].p;
    const Matrix3 A_qi =
        -Qhat_i *
        (Rot3::Hat(qhat_i) * Rot3::Hat(v_C) - 2.0 * v_C * qhat_i.transpose() +
         qhat_i * v_C.transpose()) *
        Qhat_i.inverse() * (1.0 / qhat_i.squaredNorm());
    A0t.block<3, 3>(VIOSensorState::CompDim + 3 * i,
                    VIOSensorState::CompDim + 3 * i) =
        ConvEucToInvDepth(q0) * A_qi * ConvInvDepthToEuc(q0);
  }

  return A0t;
}

Matrix EqFInputMatrixB_euclid(const VIOGroup& X, const VIOState& xi0) {
  const int N = static_cast<int>(xi0.n());
  Matrix Bt = Matrix::Zero(xi0.dim(), IMUInput::CompDim);

  const VIOState xiHat = stateGroupAction(X, xi0);
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
    const Matrix3 Qhat_i = SOT3ScaledRotation(
        Q_landmarkTransforms(X)[static_cast<size_t>(i)]);
    const Point3 qhat_i = xiHat.cameraLandmarks[static_cast<size_t>(i)].p;
    Bt.block<3, 3>(VIOSensorState::CompDim + 3 * i, 0) =
        Qhat_i * (Rot3::Hat(qhat_i) * RT_IC + RT_IC * Rot3::Hat(x_IC));
  }

  return Bt;
}

Matrix EqFInputMatrixB_invdepth(const VIOGroup& X, const VIOState& xi0) {
  const int N = static_cast<int>(xi0.n());
  Matrix Bt = Matrix::Zero(xi0.dim(), IMUInput::CompDim);

  const VIOState xiHat = stateGroupAction(X, xi0);
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
    const Matrix3 Qhat_i = SOT3ScaledRotation(
        Q_landmarkTransforms(X)[static_cast<size_t>(i)]);
    const Point3 qhat_i = xiHat.cameraLandmarks[static_cast<size_t>(i)].p;
    Bt.block<3, 3>(VIOSensorState::CompDim + 3 * i, 0) =
        ConvEucToInvDepth(q0) *
        Qhat_i * (Rot3::Hat(qhat_i) * RT_IC + RT_IC * Rot3::Hat(x_IC));
  }

  return Bt;
}

Matrix23 EqFoutputMatrixCiStar_euclid(
    const Point3& q0, const SOT3& QHat,
    const std::shared_ptr<const VIOCameraModel>& camera, const Point2& y) {
  if (!camera) {
    throw std::invalid_argument("EqFoutputMatrixCiStar_euclid: null camera");
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
    return camera->projectionJacobian(yVec) * DRhoVec;
  };

  const Vector3 yTru = camera->undistortPoint(y);
  const Matrix24 drhoSym = 0.5 * (DRho(yTru) + DRho(yHat));
  const Matrix44 adjQInv = QHat.inverse().AdjointMap();
  return drhoSym * adjQInv * m2g;
}

Matrix23 EqFoutputMatrixCiStar_invdepth(
    const Point3& q0, const SOT3& QHat,
    const std::shared_ptr<const VIOCameraModel>& camera, const Point2& y) {
  const double r0 = q0.norm();
  const Vector3 y0 = q0 / r0;
  Matrix3 ind2euc;
  ind2euc.block<3, 2>(0, 0) = r0 * SphereChartStereoInvDiff0(y0);
  ind2euc.block<3, 1>(0, 2) = -r0 * q0;
  return EqFoutputMatrixCiStar_euclid(q0, QHat, camera, y) * ind2euc;
}

Vector liftInnovation_euclid(const Vector& totalInnovation, const VIOState& xi0) {
  if (totalInnovation.size() != xi0.dim()) {
    throw std::invalid_argument(
        "liftInnovation_euclid: innovation dimension mismatch");
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
    const Vector3 gammaQi0 =
        totalInnovation.segment<3>(VIOSensorState::CompDim + 3 * i);
    const Point3 qi0 = xi0.cameraLandmarks[static_cast<size_t>(i)].p;

    lift.segment<3>(21 + 4 * i) = -qi0.cross(gammaQi0) / qi0.squaredNorm();
    lift(21 + 4 * i + 3) = -qi0.dot(gammaQi0) / qi0.squaredNorm();
  }

  return lift;
}

Vector liftInnovation_invdepth(const Vector& totalInnovation,
                               const VIOState& xi0) {
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
        totalInnovation.segment<3>(VIOSensorState::CompDim + 3 * i);

    lift.segment<3>(21 + 4 * i) = -qi0.cross(gammaQi0) / qi0.squaredNorm();
    lift(21 + 4 * i + 3) = -qi0.dot(gammaQi0) / qi0.squaredNorm();
  }

  return lift;
}

VIOGroup liftInnovationDiscrete_euclid(const Vector& totalInnovation,
                                       const VIOState& xi0) {
  if (totalInnovation.size() != xi0.dim()) {
    throw std::invalid_argument(
        "liftInnovationDiscrete_euclid: innovation dimension mismatch");
  }

  const VIOBias beta(totalInnovation.segment<6>(0));
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
    const Point3 qi = xi0.cameraLandmarks[i].p;
    const Point3 qi1 = qi + totalInnovation.segment<3>(
                                VIOSensorState::CompDim + 3 * static_cast<int>(i));
    const Rot3 R = RotationFromTwoVectors(qi1, qi);
    const double a = qi.norm() / qi1.norm();
    Q.emplace_back(MakeSOT3(SO3(R.matrix()), a));
  }

  return makeVIOGroup(MakeA(A_pose.rotation(), A_pose.translation(), w), beta, B,
                      VIOLandmarkGroup(Q));
}

VIOGroup liftInnovationDiscrete_invdepth(const Vector& totalInnovation,
                                         const VIOState& xi0) {
  if (totalInnovation.size() != xi0.dim()) {
    throw std::invalid_argument(
        "liftInnovationDiscrete_invdepth: innovation dimension mismatch");
  }

  const VIOBias beta(totalInnovation.segment<6>(0));
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
                                  VIOSensorState::CompDim + 3 * static_cast<int>(i)),
                              qi0);
    const Rot3 R = RotationFromTwoVectors(qi1.normalized(), qi0.normalized());
    const double a = qi0.norm() / qi1.norm();
    Q.emplace_back(MakeSOT3(SO3(R.matrix()), a));
  }

  return makeVIOGroup(MakeA(A_pose.rotation(), A_pose.translation(), w), beta, B,
                      VIOLandmarkGroup(Q));
}

}  // namespace

const EqFCoordinateSuite EqFCoordinateSuite_euclid{
    [](const VIOState& Xi, const VIOState& Xi0) { return Xi0.localCoordinates(Xi); },
    [](const Vector& eps, const VIOState& Xi0) { return Xi0.retract(eps); },
    EqFStateMatrixA_euclid,
    EqFInputMatrixB_euclid,
    EqFoutputMatrixCiStar_euclid,
    liftInnovation_euclid,
    liftInnovationDiscrete_euclid};

const EqFCoordinateSuite EqFCoordinateSuite_invdepth{
    StateChartInvDepth,
    StateChartInvDepthInv,
    EqFStateMatrixA_invdepth,
    EqFInputMatrixB_invdepth,
    EqFoutputMatrixCiStar_invdepth,
    liftInnovation_invdepth,
    liftInnovationDiscrete_invdepth};

Matrix EqFCoordinateSuite::outputMatrixC(const VIOState& xi0, const VIOGroup& X,
                                         const VisionMeasurement& y,
                                         const std::shared_ptr<const VIOCameraModel>& camera,
                                         bool useEquivariance) const {
  if (!camera) {
    throw std::invalid_argument("EqFCoordinateSuite::outputMatrixC: null camera");
  }
  const int M = static_cast<int>(xi0.n());
  const std::vector<int> yIds = measurementIds(y);
  const int N = static_cast<int>(yIds.size());

  Matrix C = Matrix::Zero(2 * N, VIOSensorState::CompDim + Landmark::CompDim * M);

  for (int i = 0; i < M; ++i) {
    const int idNum = xi0.cameraLandmarks[static_cast<size_t>(i)].id;
    const auto itY = std::find(yIds.begin(), yIds.end(), idNum);
    if (itY == yIds.end()) continue;

    const size_t k = static_cast<size_t>(i);
    const int j = static_cast<int>(std::distance(yIds.begin(), itY));
    const Point3& qi0 = xi0.cameraLandmarks[static_cast<size_t>(i)].p;
    const SOT3& Qk = Q_landmarkTransforms(X)[k];

    const Matrix23 Ci = useEquivariance
                            ? outputMatrixCiStar(qi0, Qk, camera, y.at(idNum))
                            : outputMatrixCi(qi0, Qk, camera);
    if (!Ci.array().isFinite().all()) {
      const Point3 qHat = SOT3ApplyInverse(Qk, qi0);
      const Point2 yObs = y.at(idNum);
      const Vector3 yUnd = camera->undistortPoint(yObs);
      throw std::runtime_error(
          "EqFCoordinateSuite::outputMatrixC: non-finite Ci for id " +
          std::to_string(idNum) + ", qi0_norm=" + std::to_string(qi0.norm()) +
          ", qHat_norm=" + std::to_string(qHat.norm()) +
          ", Q_scale=" + std::to_string(SOT3Scale(Qk)) +
          ", yUnd_norm=" + std::to_string(yUnd.norm()));
    }
    C.block<2, 3>(2 * j, VIOSensorState::CompDim + 3 * i) = Ci;
  }

  if (!C.array().isFinite().all()) {
    throw std::runtime_error("EqFCoordinateSuite::outputMatrixC produced NaN/Inf");
  }
  return C;
}

Matrix EqFCoordinateSuite::stateMatrixADiscrete(const VIOGroup& X,
                                                const VIOState& xi0,
                                                const IMUInput& imuVel,
                                                double dt) const {
  auto a0Discrete = [&](const Vector& epsilon) {
    const VIOState xiE = stateChartInv(epsilon, xi0);
    const VIOState xiHat = stateGroupAction(X, xi0);
    const VIOState xi = stateGroupAction(X, xiE);
    const VIOGroup lambdaTilde =
        liftVelocityDiscrete(xi, imuVel, dt) *
        liftVelocityDiscrete(xiHat, imuVel, dt).inverse();
    const VIOState xiE1 = stateGroupAction(X * lambdaTilde * X.inverse(), xiE);
    return stateChart(xiE1, xi0);
  };

  return NumericalDifferential(a0Discrete, Vector::Zero(xi0.dim()));
}

Matrix23 EqFCoordinateSuite::outputMatrixCi(
    const Point3& q0, const SOT3& QHat,
    const std::shared_ptr<const VIOCameraModel>& camera) const {
  if (!camera) {
    throw std::invalid_argument("EqFCoordinateSuite::outputMatrixCi: null camera");
  }
  const Vector3 qHat = SOT3ApplyInverse(QHat, q0);
  const Point2 yHat = camera->projectPoint(qHat);
  return outputMatrixCiStar(q0, QHat, camera, yHat);
}

const EqFCoordinateSuite* getCoordinates(CoordinateChoice coordinateChoice) {
  if (coordinateChoice == CoordinateChoice::Euclidean) {
    return &EqFCoordinateSuite_euclid;
  }
  if (coordinateChoice == CoordinateChoice::InvDepth) {
    return &EqFCoordinateSuite_invdepth;
  }
  return nullptr;
}

}  // namespace eqvio
}  // namespace gtsam
