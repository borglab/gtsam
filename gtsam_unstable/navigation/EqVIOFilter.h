/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file EqVIOFilter.h
 * @brief Standalone equivariant VIO filter.
 * @author Rohan Bansal
 */

#pragma once

#include <gtsam/navigation/EquivariantFilter.h>
#include <gtsam_unstable/dllexport.h>
#include <gtsam_unstable/navigation/EqVIOSymmetry.h>

#include <memory>
#include <vector>

namespace gtsam {
namespace eqvio {

/**
 * @brief Runtime parameters for the standalone EqVIO filter.
 *
 * These parameters mirror the process/measurement tuning knobs used by the
 * EqVIO runtime implementation. All variances are per-axis scalar multipliers
 * applied to identity blocks in the corresponding covariance matrices.
 */
struct EqVIOFilterParams {
  /// Initial landmark depth used when a new feature is first triangulated from
  /// bearing.
  double initialPointDepth = 10.0;
  /// Initial 3x3 covariance scalar assigned to newly inserted landmarks.
  double initialPointVariance = 1.0;
  /// Absolute reprojection residual threshold for outlier rejection.
  double outlierThresholdAbs = 1e8;
  /// Fraction of features to keep after ranking potential outliers (in [0, 1]).
  double featureRetention = 0.3;
  double biasOmegaProcessVariance = 0.001;
  double biasAccelProcessVariance = 0.001;
  double attitudeProcessVariance = 0.001;
  double positionProcessVariance = 0.001;
  double velocityProcessVariance = 0.001;
  double cameraAttitudeProcessVariance = 0.001;
  double cameraPositionProcessVariance = 0.001;
  double pointProcessVariance = 0.001;
  /// IMU driving noise covariance in stacked order `[gyr, acc, gyr_bias_walk,
  /// acc_bias_walk]`.
  Eigen::Matrix<double, 12, 12> inputNoise =
      Eigen::Matrix<double, 12, 12>::Identity() * 1e-3;
};

/**
 * @brief Standalone EqVIO filter built on top of `EquivariantFilter`.
 *
 * Prediction uses the base equivariant `predictWithJacobian(...)` path per IMU
 * hold, while dynamic landmark add/remove bookkeeping stays in this runtime
 * filter.
 */
class GTSAM_UNSTABLE_EXPORT EqVIOFilter
    : public EquivariantFilter<State, Symmetry> {
 public:
  using Base = EquivariantFilter<State, Symmetry>;

 private:
  static constexpr size_t kMaxMissedFrames = 1;

  EqVIOFilterParams params_;
  bool initialized_ = false;
  // Runtime key ordering aligned with `cameraLandmarks` and covariance blocks.
  KeyVector landmarkKeys_;
  std::vector<size_t> missedFrameCounts_;
  FastMap<Key, size_t> landmarkIndexByKey_;

 public:
  /// Construct with explicit parameter bundle and default identity initial
  /// state.
  explicit EqVIOFilter(const EqVIOFilterParams& params);
  /// Construct with explicit initial reference state, covariance, landmark
  /// keys, and parameters.
  EqVIOFilter(const State& xi_ref, const Matrix& Sigma,
              const KeyVector& landmarkKeys, const EqVIOFilterParams& params);

  /**
   * @brief Initialize filter attitude from gravity direction in an IMU sample.
   *
   * The measured acceleration vector is treated as gravity (small-motion
   * assumption) and aligned with world +Z.
   */
  void initializeFromIMU(const IMUInput& imu);

  /**
   * @brief Propagate filter state across one IMU hold interval.
   *
   * The hold `(imu, dt)` is propagated via the explicit base-class
   * `predictWithJacobian(...)` path, so mean and covariance advance together.
   *
   * @param imu IMU sample defining the zero-order hold.
   * @param dt Hold duration in seconds.
   */
  void predict(const IMUInput& imu, double dt);

  /**
   * @brief Apply one visual correction step with explicit measurement
   * covariance.
   *
   * @param measurement Observed image points keyed by landmark id.
   * @param camera Camera model used for projection/linearization.
   * @param R Measurement covariance matching the input measurement dimension.
   * @throws std::invalid_argument if `R` is not exactly `2M x 2M`, where
   *         `M = measurement.size()`.
   */
  void update(const VisionMeasurement& measurement,
              const std::shared_ptr<const CameraModel>& camera,
              const Matrix& R);

  /// True after IMU-based initialization.
  bool isInitialized() const { return initialized_; }

  /// Number of currently tracked landmarks.
  size_t landmarkCount() const { return state().n(); }

  /// Current estimated body position in world frame.
  Point3 position() const { return state().pose().translation(); }

  /// Current estimated body velocity in world frame.
  Vector3 velocity() const { return state().velocity(); }

 private:
  /// Allocate identity covariance with dimension `21 + 3*nLandmarks`.
  static Matrix defaultCovariance(size_t nLandmarks);

  /// Build process noise matrix for current state dimension.
  Matrix stateProcessNoise(size_t nLandmarks) const;

  /// Run innovation update on currently matched measurements.
  void innovationUpdate(const VisionMeasurement& measurement,
                        const std::shared_ptr<const CameraModel>& camera,
                        const Matrix& measurementCovariance);

  /// Validate/store externally supplied landmark keys for seeded states.
  void setLandmarkKeys(const KeyVector& landmarkKeys);

  /// Refresh the key-to-index lookup cache after any structure change.
  void rebuildLandmarkIndex();

  /// Batch landmark add/remove bookkeeping around one vision update.
  void reconcileLandmarks(VisionMeasurement& measurement,
                          const std::shared_ptr<const CameraModel>& camera);

  /// Compute absolute-residual outliers and erase them from `measurement`.
  KeyVector detectOutliers(
      VisionMeasurement& measurement,
      const std::shared_ptr<const CameraModel>& camera) const;

  /// Return keys whose associated SOT3 scale is numerically invalid.
  KeyVector invalidLandmarkKeys() const;

  /// Rebuild state, group, covariance, and lookup caches in one pass.
  void applyLandmarkStructureChange(
      const std::vector<size_t>& retainedIndices,
      const std::vector<std::pair<Key, Point3>>& newLandmarks);

  /// Rebuild covariance after applying a batch landmark structure change.
  Matrix rebuildCovariance(const std::vector<size_t>& retainedIndices,
                           size_t newLandmarkCount) const;
};

}  // namespace eqvio
}  // namespace gtsam
