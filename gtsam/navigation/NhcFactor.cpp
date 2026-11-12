/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   NhcFactor.cpp
 *  @author inuex35
 *  @brief  Implementation of the non-holonomic constraint (NHC) factors
 *  @date   July 4, 2026
 **/

#include <gtsam/navigation/NhcFactor.h>

#include <iostream>

namespace gtsam {

namespace {
/**
 * Lateral and vertical velocity of the vehicle reference point,
 *   v_vehicle = bRv^T * (wRb^T * v_world + omega x leverArm),
 * returning the last two components of v_vehicle, with analytic Jacobians.
 * The pose and mounting Jacobians are for right perturbations of the
 * rotations; the pose translation block is zero.
 */
Vector2 lateralVerticalVelocity(const Pose3& pose, const Vector3& velocity,
                                const Rot3& mountRotation,
                                const Vector3& angularVelocity,
                                const Vector3& leverArm,
                                OptionalMatrixType H_pose,
                                OptionalMatrixType H_velocity,
                                OptionalMatrixType H_mountRotation) {
  const Matrix3 bRw = pose.rotation().matrix().transpose();
  const Vector3 bodyVelocity = bRw * velocity;
  const Vector3 pointVelocity = bodyVelocity + angularVelocity.cross(leverArm);

  Matrix3 H_unrotate;
  const Vector3 vehicleVelocity =
      mountRotation.unrotate(pointVelocity, H_unrotate);
  const Matrix3 vRb = mountRotation.matrix().transpose();

  if (H_pose) {
    Matrix26 H;
    H << (vRb * skewSymmetric(bodyVelocity)).bottomRows<2>(), Matrix23::Zero();
    *H_pose = H;
  }
  if (H_velocity) *H_velocity = Matrix23((vRb * bRw).bottomRows<2>());
  if (H_mountRotation) *H_mountRotation = Matrix23(H_unrotate.bottomRows<2>());
  return vehicleVelocity.tail<2>();
}
}  // namespace

/* ************************************************************************* */
NhcFactor::NhcFactor(Key poseKey, Key velocityKey,
                     const Vector3& angularVelocity, const Vector3& leverArm,
                     const SharedNoiseModel& model, const Rot3& mountRotation)
    : Base(model, poseKey, velocityKey),
      angularVelocity_(angularVelocity),
      leverArm_(leverArm),
      mountRotation_(mountRotation) {}

void NhcFactor::print(const std::string& s,
                      const KeyFormatter& keyFormatter) const {
  std::cout << (s.empty() ? "" : s + " ") << "NhcFactor("
            << keyFormatter(key<1>()) << "," << keyFormatter(key<2>()) << ")\n";
  std::cout << "  angularVelocity: " << angularVelocity_.transpose() << "\n";
  std::cout << "  leverArm:        " << leverArm_.transpose() << "\n";
  mountRotation_.print("  mountRotation: ");
  if (noiseModel_) noiseModel_->print("  noise model: ");
}

bool NhcFactor::equals(const NonlinearFactor& expected, double tol) const {
  const This* other = dynamic_cast<const This*>(&expected);
  return other != nullptr && Base::equals(*other, tol) &&
         traits<Vector3>::Equals(angularVelocity_, other->angularVelocity_,
                                 tol) &&
         traits<Vector3>::Equals(leverArm_, other->leverArm_, tol) &&
         mountRotation_.equals(other->mountRotation_, tol);
}

Vector NhcFactor::evaluateError(const Pose3& pose, const Vector3& velocity,
                                OptionalMatrixType H_pose,
                                OptionalMatrixType H_velocity) const {
  return lateralVerticalVelocity(pose, velocity, mountRotation_,
                                 angularVelocity_, leverArm_, H_pose,
                                 H_velocity, nullptr);
}

/* ************************************************************************* */
NhcFactorCalib::NhcFactorCalib(Key poseKey, Key velocityKey,
                               Key mountRotationKey,
                               const Vector3& angularVelocity,
                               const Vector3& leverArm,
                               const SharedNoiseModel& model)
    : Base(model, poseKey, velocityKey, mountRotationKey),
      angularVelocity_(angularVelocity),
      leverArm_(leverArm) {}

void NhcFactorCalib::print(const std::string& s,
                           const KeyFormatter& keyFormatter) const {
  std::cout << (s.empty() ? "" : s + " ") << "NhcFactorCalib("
            << keyFormatter(key<1>()) << "," << keyFormatter(key<2>()) << ","
            << keyFormatter(key<3>()) << ")\n";
  std::cout << "  angularVelocity: " << angularVelocity_.transpose() << "\n";
  std::cout << "  leverArm:        " << leverArm_.transpose() << "\n";
  if (noiseModel_) noiseModel_->print("  noise model: ");
}

bool NhcFactorCalib::equals(const NonlinearFactor& expected, double tol) const {
  const This* other = dynamic_cast<const This*>(&expected);
  return other != nullptr && Base::equals(*other, tol) &&
         traits<Vector3>::Equals(angularVelocity_, other->angularVelocity_,
                                 tol) &&
         traits<Vector3>::Equals(leverArm_, other->leverArm_, tol);
}

Vector NhcFactorCalib::evaluateError(const Pose3& pose, const Vector3& velocity,
                                     const Rot3& mountRotation,
                                     OptionalMatrixType H_pose,
                                     OptionalMatrixType H_velocity,
                                     OptionalMatrixType H_mountRotation) const {
  return lateralVerticalVelocity(pose, velocity, mountRotation,
                                 angularVelocity_, leverArm_, H_pose,
                                 H_velocity, H_mountRotation);
}

}  // namespace gtsam
