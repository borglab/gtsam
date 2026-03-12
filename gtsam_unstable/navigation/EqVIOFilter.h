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
#include <gtsam_unstable/navigation/VIOEqFMatrices.h>
#include <gtsam_unstable/dllexport.h>

#include <vector>

namespace gtsam {

/// Runtime parameters for the standalone EqVIO filter.
struct GTSAM_UNSTABLE_EXPORT EqVIOFilterParams {
  CoordinateChoice coordinateChoice = CoordinateChoice::InvDepth;
  bool useDiscreteStateMatrix = false;
  bool useDiscreteVelocityLift = false;
  bool useDiscreteInnovationLift = false;
  bool useEquivariantOutput = true;
  bool removeLostLandmarks = true;
  double initialPointDepth = 10.0;
  double initialPointVariance = 1.0;
  double measurementNoiseVariance = 1e-4;
  Eigen::Matrix<double, IMUVelocity::CompDim, IMUVelocity::CompDim> inputNoise =
      Eigen::Matrix<double, IMUVelocity::CompDim, IMUVelocity::CompDim>::Identity() *
      1e-3;
};

/// Lightweight EqVIO filter independent from the eqvio application code.
class GTSAM_UNSTABLE_EXPORT EqVIOFilter
    : public EquivariantFilter<VIOState, VIOSymmetry> {
 public:
  using Base = EquivariantFilter<VIOState, VIOSymmetry>;

  /// Minimal debug view of internal EqF state.
  struct View {
    const EqFCoordinateSuite* coordinateSuite = &EqFCoordinateSuite_invdepth;
    VIOState xi0;
    VIOGroup X = makeVIOGroupIdentity();
    Matrix Sigma = Matrix::Identity(VIOSensorState::CompDim, VIOSensorState::CompDim);
    double currentTime = -1.0;
  };

 private:
  EqVIOFilterParams params_;
  View view_;
  bool initialized_ = false;
  std::vector<IMUVelocity> imuBuffer_;

 public:
  EqVIOFilter();
  explicit EqVIOFilter(const EqVIOFilterParams& params);
  EqVIOFilter(const VIOState& xi0, const Matrix& Sigma0,
              const EqVIOFilterParams& params, double time = 0.0);

  /// Initialize orientation from gravity in the first IMU sample.
  void initializeFromIMU(const IMUVelocity& imu);
  /// Set new manifold origin and covariance (group estimate reset to identity).
  void setReferenceState(const VIOState& xi0, const Matrix& Sigma0);

  /// Queue an IMU sample for integration.
  void processIMUData(const IMUVelocity& imu);
  /// Integrate to `measurement.stamp` and perform one correction step.
  void processVisionData(const VisionMeasurement& measurement,
                         const Matrix& R = Matrix());

  /// Current full state estimate `phi(X, xi0)`.
  VIOState stateEstimate() const;
  /// Current filter time.
  double currentTime() const { return view_.currentTime; }
  /// Whether the filter has been initialized from IMU.
  bool isInitialized() const { return initialized_; }
  /// Current reference/group/covariance tuple.
  const View& view() const { return view_; }

 private:
  static Matrix defaultCovariance(size_t nLandmarks);
  static Rot3 rotationFromTwoVectors(const Vector3& from, const Vector3& to);
  static Vector measurementResidual(const VisionMeasurement& z,
                                    const VisionMeasurement& zhat,
                                    const char* context);
  static void removeRows(Matrix& mat, int startRow, int numRows);
  static void removeCols(Matrix& mat, int startCol, int numCols);

  void syncBase(bool resetReference);
  void syncFromBase();

  bool integrateUpToTime(double newTime);
  void integrateObserverState(const IMUVelocity& imu, double dt);

  void synchronizeLandmarksToMeasurement(const VisionMeasurement& measurement);
  void addLandmarksInternal(const std::vector<Landmark>& newLandmarks);
  void removeLandmarkByIndex(int idx);
  void removeOldLandmarks(const std::vector<int>& measurementIds);

  void updateVision(const VisionMeasurement& measurement, const Matrix& R);
};

}  // namespace gtsam

