/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LeggedEstimatorFactors.h
 * @date February 2026
 * @author Frank Dellaert
 */
  
#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/navigation/LeggedEstimator.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

namespace gtsam {

/// Return the tangent-space start index of a foot block.
inline int leggedFootBlockStart(size_t foot) {
  return 9 + 3 * static_cast<int>(foot);
}

/**
 * Predict the IMU-frame contact vector for an ExtendedPose3 state.
 *
 * `Pose3::transformTo` returns the Jacobian with respect to the embedded
 * `(R, p)` pose as `[skew(q)  -I]`, where `q = R^T (f - p)`. The foothold
 * block in `ExtendedPose3` uses body-frame tangent coordinates, and `x(i)` has
 * component Jacobian `R`, so the foothold chain rule is
 *
 *   d q / d delta_f = (d q / d f_world) * (d f_world / d delta_f)
 *                   = R^T * R
 *                   = I.
 *
 * We therefore reuse `transformTo` only for the pose block and write the foot
 * block directly as the identity.
 */
inline Vector3 extendedPoseContactPrediction(const ExtendedPose3d& state,
                                             size_t footColumn,
                                             OptionalMatrixType H = {}) {
  Matrix36 prediction_H_pose;
  const Pose3 pose(state.rotation(), state.x(0));
  const Point3 foothold = state.x(footColumn);
  const Vector3 prediction = pose.transformTo(foothold, prediction_H_pose);

  if (H) {
    H->setZero(3, static_cast<Eigen::Index>(state.dim()));
    H->block(0, 0, 3, 6) = prediction_H_pose;
    const int start = leggedFootBlockStart(footColumn - 2);
    H->block(0, start, 3, 3) = I_3x3;
  }

  return prediction;
}

/// Contact factor for the `ExtendedPose3(2+k)` graph-update variant.
class ExtendedPoseContactFactor : public NoiseModelFactorN<ExtendedPose3d> {
  using Base = NoiseModelFactorN<ExtendedPose3d>;

 public:
  using Base::evaluateError;

  /// Construct from a state key, foot column, and IMU-frame measurement.
  ExtendedPoseContactFactor(Key key, size_t footColumn,
                            const Point3& measurement,
                            const SharedNoiseModel& model)
      : Base(model, key), footColumn_(footColumn), measurement_(measurement) {}

  /// Return a deep copy.
  NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<NonlinearFactor>(
        NonlinearFactor::shared_ptr(new ExtendedPoseContactFactor(*this)));
  }

  /// Evaluate the contact residual and optional Jacobian.
  Vector evaluateError(const ExtendedPose3d& state,
                       OptionalMatrixType H) const override {
    const Vector3 prediction =
        extendedPoseContactPrediction(state, footColumn_, H);

    return prediction - measurement_;
  }

 private:
  size_t footColumn_;
  Point3 measurement_;
};

/// Height factor for the `ExtendedPose3(2+k)` graph-update variant.
class ExtendedPoseHeightFactor : public NoiseModelFactorN<ExtendedPose3d> {
  using Base = NoiseModelFactorN<ExtendedPose3d>;

 public:
  using Base::evaluateError;

  /// Construct from a state key, foot column, and terrain height.
  ExtendedPoseHeightFactor(Key key, size_t footColumn, double terrainHeight,
                           const SharedNoiseModel& model)
      : Base(model, key),
        footColumn_(footColumn),
        terrainHeight_(terrainHeight) {}

  /// Return a deep copy.
  NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<NonlinearFactor>(
        NonlinearFactor::shared_ptr(new ExtendedPoseHeightFactor(*this)));
  }

  /// Evaluate the height residual and optional Jacobian.
  Vector evaluateError(const ExtendedPose3d& state,
                       OptionalMatrixType H) const override {
    if (H) {
      H->setZero(1, static_cast<Eigen::Index>(state.dim()));
      const Matrix3 R = state.rotation().matrix();
      const int start = leggedFootBlockStart(footColumn_ - 2);
      H->block(0, start, 1, 3) = R.row(2);
    }

    return Vector1(state.x(footColumn_).z() - terrainHeight_);
  }

 private:
  size_t footColumn_;
  double terrainHeight_;
};

/// Contact factor between a NavState and a foothold point variable.
class NavStatePointContactFactor : public NoiseModelFactorN<NavState, Point3> {
  using Base = NoiseModelFactorN<NavState, Point3>;

 public:
  using Base::evaluateError;

  /// Construct from a NavState key, foothold key, and IMU-frame measurement.
  NavStatePointContactFactor(Key navKey, Key pointKey,
                             const Point3& measurement,
                             const SharedNoiseModel& model)
      : Base(model, navKey, pointKey), measurement_(measurement) {}

  /// Return a deep copy.
  NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<NonlinearFactor>(
        NonlinearFactor::shared_ptr(new NavStatePointContactFactor(*this)));
  }

  /// Evaluate the contact residual and optional Jacobians.
  Vector evaluateError(const NavState& navState, const Point3& foothold,
                       OptionalMatrixType H1,
                       OptionalMatrixType H2) const override {
    Matrix36 prediction_H_pose;
    Matrix3 prediction_H_foothold;
    const Vector3 prediction = navState.pose().transformTo(
        foothold, prediction_H_pose, prediction_H_foothold);

    if (H1) {
      H1->setZero(3, 9);
      H1->block(0, 0, 3, 6) = prediction_H_pose;
    }
    if (H2) {
      *H2 = prediction_H_foothold;
    }

    return prediction - measurement_;
  }

 private:
  Point3 measurement_;
};

/// Contact factor between a Pose3 and a foothold point variable.
class Pose3PointContactFactor : public NoiseModelFactorN<Pose3, Point3> {
  using Base = NoiseModelFactorN<Pose3, Point3>;

 public:
  using Base::evaluateError;

  /// Construct from a Pose3 key, foothold key, and IMU-frame measurement.
  Pose3PointContactFactor(Key poseKey, Key pointKey, const Point3& measurement,
                          const SharedNoiseModel& model)
      : Base(model, poseKey, pointKey), measurement_(measurement) {}

  /// Return a deep copy.
  NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<NonlinearFactor>(
        NonlinearFactor::shared_ptr(new Pose3PointContactFactor(*this)));
  }

  /// Evaluate the contact residual and optional Jacobians.
  Vector evaluateError(const Pose3& pose, const Point3& foothold,
                       OptionalMatrixType H1,
                       OptionalMatrixType H2) const override {
    return pose.transformTo(foothold, H1, H2) - measurement_;
  }

 private:
  Point3 measurement_;
};

/// Height factor on a standalone foothold point variable.
class PointHeightFactor : public NoiseModelFactorN<Point3> {
  using Base = NoiseModelFactorN<Point3>;

 public:
  using Base::evaluateError;

  /// Construct from a foothold key and terrain height.
  PointHeightFactor(Key key, double terrainHeight,
                    const SharedNoiseModel& model)
      : Base(model, key), terrainHeight_(terrainHeight) {}

  /// Return a deep copy.
  NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<NonlinearFactor>(
        NonlinearFactor::shared_ptr(new PointHeightFactor(*this)));
  }

  /// Evaluate the height residual and optional Jacobian.
  Vector evaluateError(const Point3& foothold,
                       OptionalMatrixType H) const override {
    if (H) {
      H->resize(1, 3);
      *H << 0.0, 0.0, 1.0;
    }
    return Vector1(foothold.z() - terrainHeight_);
  }

 private:
  double terrainHeight_;
};

}  // namespace gtsam
