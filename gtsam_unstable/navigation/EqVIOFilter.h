/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/// @file EqVIOFilter.h
/// @brief Standalone equivariant VIO filter for gtsam_unstable.

#pragma once

#include <gtsam/navigation/EquivariantFilter.h>
#include <gtsam_unstable/navigation/EqVIOSymmetry.h>
#include <gtsam_unstable/dllexport.h>

#include <memory>
#include <vector>

namespace gtsam {
namespace eqvio {

/// Runtime parameters for the standalone EqVIO filter.
struct GTSAM_UNSTABLE_EXPORT EqVIOFilterParams {
  double initialPointDepth = 10.0;
  double initialPointVariance = 1.0;
  double measurementNoiseVariance = 1e-4;
  double outlierThresholdAbs = 1e8;
  double outlierThresholdProb = 1e8;
  double featureRetention = 0.3;
  double biasOmegaProcessVariance = 0.001;
  double biasAccelProcessVariance = 0.001;
  double attitudeProcessVariance = 0.001;
  double positionProcessVariance = 0.001;
  double velocityProcessVariance = 0.001;
  double cameraAttitudeProcessVariance = 0.001;
  double cameraPositionProcessVariance = 0.001;
  double pointProcessVariance = 0.001;
  Eigen::Matrix<double, IMUInput::CompDim, IMUInput::CompDim> inputNoise =
      Eigen::Matrix<double, IMUInput::CompDim, IMUInput::CompDim>::Identity() *
      1e-3;
};

/// Standalone EqVIO filter.
class GTSAM_UNSTABLE_EXPORT EqVIOFilter
    : public EquivariantFilter<State, Symmetry> {
 public:
  using Base = EquivariantFilter<State, Symmetry>;

  /// Internal filter state view.
  struct View {
    State xi0;
    VioGroup X = makeVioGroupIdentity();
    Matrix Sigma =
        Matrix::Identity(SensorState::CompDim, SensorState::CompDim);
  };

 private:
 EqVIOFilterParams params_;
  View view_;
  bool initialized_ = false;

 public:
  EqVIOFilter();
  explicit EqVIOFilter(const EqVIOFilterParams& params);
  EqVIOFilter(const State& xi0, const Matrix& Sigma0,
              const EqVIOFilterParams& params);

  /// Initialize orientation from gravity in the first IMU sample.
  void initializeFromIMU(const IMUInput& imu);
  /// Set manifold reference/origin and covariance.
  void setReferenceState(const State& xi0, const Matrix& Sigma0);

  /// Propagate observer by one IMU hold over `dt`.
  void propagate(const IMUInput& imu, double dt);
  /// Propagate covariance only over `dt` with one IMU hold.
  void propagateCovariance(const IMUInput& imu, double dt);
  /// Propagate observer state only over `dt` with one IMU hold.
  void propagateState(const IMUInput& imu, double dt);
  /// Apply one visual correction at current observer time.
  void correct(const VisionMeasurement& measurement,
               const std::shared_ptr<const CameraModel>& camera,
               const Matrix& R = Matrix());

  /// Current full state estimate.
  State stateEstimate() const;
  /// True after IMU-based initialization.
  bool isInitialized() const { return initialized_; }
  /// Access internal reference/group/covariance state.
  const View& view() const { return view_; }

 private:
  static Matrix defaultCovariance(size_t nLandmarks);

  void syncBase(bool resetReference);
  void syncFromBase();

  Matrix stateProcessNoise(size_t nLandmarks) const;
  double getMedianSceneDepth() const;

  void addNewLandmarks(const VisionMeasurement& measurement,
                       const std::shared_ptr<const CameraModel>& camera);
  void addLandmarksInternal(std::vector<Landmark>& newLandmarks,
                            const Matrix& newLandmarkCov);
  void removeLandmarkByIndex(int idx);
  void removeLandmarkById(int id);
  void removeOldLandmarks(const std::vector<int>& measurementIds);
  void removeOutliers(VisionMeasurement& measurement,
                      const std::shared_ptr<const CameraModel>& camera);
  void removeInvalidLandmarksNow();
  Matrix3 getLandmarkCovById(int id) const;
  Matrix2 outputCovarianceById(
      int id, const Point2& y,
      const std::shared_ptr<const CameraModel>& camera) const;

  void update(const VisionMeasurement& measurement,
              const std::shared_ptr<const CameraModel>& camera,
              const Matrix& outputGainMatrix);
};

}  // namespace eqvio
}  // namespace gtsam
