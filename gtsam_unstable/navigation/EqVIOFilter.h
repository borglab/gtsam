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
    : public EquivariantFilter<VIOState, VIOSymmetry> {
 public:
  using Base = EquivariantFilter<VIOState, VIOSymmetry>;

  /// Internal filter state view.
  struct View {
    VIOState xi0;
    VIOGroup X = makeVIOGroupIdentity();
    Matrix Sigma =
        Matrix::Identity(VIOSensorState::CompDim, VIOSensorState::CompDim);
    double currentTime = -1.0;
  };

 private:
  EqVIOFilterParams params_;
  View view_;
  bool initialized_ = false;
  std::vector<IMUInput> imuBuffer_;

 public:
  EqVIOFilter();
  explicit EqVIOFilter(const EqVIOFilterParams& params);
  EqVIOFilter(const VIOState& xi0, const Matrix& Sigma0,
              const EqVIOFilterParams& params, double time = 0.0);

  /// Initialize orientation from gravity in the first IMU sample.
  void initializeFromIMU(const IMUInput& imu);
  /// Set manifold reference/origin and covariance.
  void setReferenceState(const VIOState& xi0, const Matrix& Sigma0);

  /// Queue one IMU input sample.
  void processIMUData(const IMUInput& imu);
  /// Integrate to `stamp` and apply one vision correction.
  void processVisionData(double stamp, const VisionMeasurement& measurement,
                         const std::shared_ptr<const VIOCameraModel>& camera,
                         const Matrix& R = Matrix());

  /// Current full state estimate.
  VIOState stateEstimate() const;
  /// Current filter time.
  double currentTime() const { return view_.currentTime; }
  /// True after IMU-based initialization.
  bool isInitialized() const { return initialized_; }
  /// Access internal reference/group/covariance state.
  const View& view() const { return view_; }

 private:
  static Matrix defaultCovariance(size_t nLandmarks);
  static Rot3 rotationFromTwoVectors(const Vector3& from, const Vector3& to);
  static void removeRows(Matrix& mat, int startRow, int numRows);
  static void removeCols(Matrix& mat, int startCol, int numCols);

  void syncBase(bool resetReference);
  void syncFromBase();

  bool integrateUpToTime(double newTime);

  void integrateRiccatiStateFast(
      const IMUInput& imu, double dt,
      const Eigen::Matrix<double, IMUInput::CompDim, IMUInput::CompDim>&
          inputGainMatrix,
      const Matrix& stateGainMatrix);
  Matrix stateProcessNoise(size_t nLandmarks) const;
  double getMedianSceneDepth() const;

  void addNewLandmarks(const VisionMeasurement& measurement,
                       const std::shared_ptr<const VIOCameraModel>& camera);
  void addLandmarksInternal(std::vector<Landmark>& newLandmarks,
                            const Matrix& newLandmarkCov);
  void removeLandmarkByIndex(int idx);
  void removeLandmarkById(int id);
  void removeOldLandmarks(const std::vector<int>& measurementIds);
  void removeOutliers(VisionMeasurement& measurement,
                      const std::shared_ptr<const VIOCameraModel>& camera);
  void removeInvalidLandmarksNow();
  Matrix3 getLandmarkCovById(int id) const;
  Matrix2 outputCovarianceById(
      int id, const Point2& y,
      const std::shared_ptr<const VIOCameraModel>& camera) const;

  void update(const VisionMeasurement& measurement,
              const std::shared_ptr<const VIOCameraModel>& camera,
              const Matrix& outputGainMatrix);
};

}  // namespace eqvio
}  // namespace gtsam
