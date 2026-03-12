/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/// @file EqVIOFilter.cpp
/// @brief Standalone equivariant VIO filter for gtsam_unstable.

#include <gtsam_unstable/navigation/EqVIOFilter.h>

#include <algorithm>
#include <cassert>
#include <numeric>
#include <stdexcept>
#include <string>

namespace gtsam {

EqVIOFilter::EqVIOFilter() : EqVIOFilter(EqVIOFilterParams()) {}

EqVIOFilter::EqVIOFilter(const EqVIOFilterParams& params)
    : Base(VIOState(), defaultCovariance(0), makeVIOGroupIdentity()),
      params_(params) {
  view_.coordinateSuite = getCoordinates(params_.coordinateChoice);
  if (!view_.coordinateSuite) {
    throw std::invalid_argument("EqVIOFilter: invalid coordinate choice");
  }
  view_.xi0 = VIOState();
  view_.X = makeVIOGroupIdentity();
  view_.Sigma = defaultCovariance(0);
  view_.currentTime = -1.0;
  syncBase(true);
}

EqVIOFilter::EqVIOFilter(const VIOState& xi0, const Matrix& Sigma0,
                         const EqVIOFilterParams& params, double time)
    : Base(VIOState(), defaultCovariance(0), makeVIOGroupIdentity()),
      params_(params) {
  view_.coordinateSuite = getCoordinates(params_.coordinateChoice);
  if (!view_.coordinateSuite) {
    throw std::invalid_argument("EqVIOFilter: invalid coordinate choice");
  }
  setReferenceState(xi0, Sigma0);
  view_.currentTime = time;
  initialized_ = true;
}

void EqVIOFilter::initializeFromIMU(const IMUVelocity& imu) {
  view_.xi0.sensor.inputBias.setZero();
  view_.xi0.sensor.velocity.setZero();

  Vector3 gDir = imu.acc;
  if (gDir.norm() < 1e-9) gDir = Vector3::UnitZ();
  const Rot3 R0 = rotationFromTwoVectors(gDir.normalized(), Vector3::UnitZ());
  view_.xi0.sensor.pose = Pose3(R0, Point3::Zero());
  view_.currentTime = imu.stamp;
  initialized_ = true;
  syncBase(true);
}

void EqVIOFilter::setReferenceState(const VIOState& xi0, const Matrix& Sigma0) {
  if (Sigma0.rows() != xi0.dim() || Sigma0.cols() != xi0.dim()) {
    throw std::invalid_argument(
        "EqVIOFilter::setReferenceState: covariance dimension mismatch");
  }
  view_.xi0 = xi0;
  view_.X = makeVIOGroupIdentity(xi0.n());
  view_.Sigma = Sigma0;
  syncBase(true);
}

void EqVIOFilter::processIMUData(const IMUVelocity& imu) {
  if (!initialized_) initializeFromIMU(imu);
  imuBuffer_.push_back(imu);
}

void EqVIOFilter::processVisionData(const VisionMeasurement& measurement,
                                    const Matrix& R) {
  if (!initialized_) return;
  if (!integrateUpToTime(measurement.stamp)) return;

  synchronizeLandmarksToMeasurement(measurement);
  if (measurement.camCoordinates.empty()) return;
  updateVision(measurement, R);
}

VIOState EqVIOFilter::stateEstimate() const {
  if (groupN(view_.X) != view_.xi0.n()) {
    throw std::invalid_argument(
        "EqVIOFilter::stateEstimate: group/state landmark count mismatch");
  }
  return stateGroupAction(view_.X, view_.xi0);
}

Matrix EqVIOFilter::defaultCovariance(size_t nLandmarks) {
  const int d = VIOSensorState::CompDim + 3 * static_cast<int>(nLandmarks);
  return Matrix::Identity(d, d);
}

Rot3 EqVIOFilter::rotationFromTwoVectors(const Vector3& from,
                                         const Vector3& to) {
  Quaternion q;
  q.setFromTwoVectors(from, to);
  return Rot3(q);
}

Vector EqVIOFilter::measurementResidual(const VisionMeasurement& z,
                                        const VisionMeasurement& zhat,
                                        const char* context) {
  Vector residual = Vector::Zero(z.dim());
  int i = 0;
  for (const auto& [id, y] : z.camCoordinates) {
    const auto it = zhat.camCoordinates.find(id);
    if (it == zhat.camCoordinates.end()) {
      throw std::invalid_argument(std::string(context) +
                                  ": prediction missing measurement id " +
                                  std::to_string(id));
    }
    residual.segment<2>(2 * i) = y - it->second;
    ++i;
  }
  return residual;
}

void EqVIOFilter::removeRows(Matrix& mat, int startRow, int numRows) {
  const int rows = mat.rows();
  const int cols = mat.cols();
  assert(startRow + numRows <= rows);
  mat.block(startRow, 0, rows - numRows - startRow, cols) =
      mat.block(startRow + numRows, 0, rows - numRows - startRow, cols);
  mat.conservativeResize(rows - numRows, Eigen::NoChange);
}

void EqVIOFilter::removeCols(Matrix& mat, int startCol, int numCols) {
  const int rows = mat.rows();
  const int cols = mat.cols();
  assert(startCol + numCols <= cols);
  mat.block(0, startCol, rows, cols - numCols - startCol) =
      mat.block(0, startCol + numCols, rows, cols - numCols - startCol);
  mat.conservativeResize(Eigen::NoChange, cols - numCols);
}

void EqVIOFilter::syncBase(bool resetReference) {
  if (resetReference) {
    resetReferenceAndGroup(view_.xi0, view_.Sigma, view_.X);
  } else {
    setGroupEstimateAndSyncState(view_.X);
    setErrorCovariance(view_.Sigma);
  }
  syncFromBase();
}

void EqVIOFilter::syncFromBase() {
  view_.X = groupEstimate();
  view_.Sigma = errorCovariance();
}

bool EqVIOFilter::integrateUpToTime(double newTime) {
  if (newTime <= view_.currentTime || view_.currentTime < 0.0 ||
      imuBuffer_.empty()) {
    return false;
  }

  for (size_t i = 0; i < imuBuffer_.size(); ++i) {
    const double t0 = std::max(imuBuffer_[i].stamp, view_.currentTime);
    const double t1 =
        i + 1 < imuBuffer_.size()
            ? std::min(imuBuffer_[i + 1].stamp, newTime)
            : newTime;
    const double dt = std::max(t1 - t0, 0.0);
    if (dt <= 0.0) continue;
    integrateObserverState(imuBuffer_[i], dt);
  }

  view_.currentTime = newTime;

  auto it = std::find_if(imuBuffer_.begin(), imuBuffer_.end(),
                         [this](const IMUVelocity& imu) {
                           return imu.stamp >= this->view_.currentTime;
                         });
  if (it != imuBuffer_.begin()) {
    --it;
    imuBuffer_.erase(imuBuffer_.begin(), it);
  }

  return true;
}

void EqVIOFilter::integrateObserverState(const IMUVelocity& imu, double dt) {
  if (dt <= 0.0) return;

  const Matrix A =
      params_.useDiscreteStateMatrix
          ? (view_.coordinateSuite->stateMatrixADiscrete(view_.X, view_.xi0, imu,
                                                         dt) -
             Matrix::Identity(view_.xi0.dim(), view_.xi0.dim())) /
                dt
          : view_.coordinateSuite->stateMatrixA(view_.X, view_.xi0, imu);
  const Matrix B = view_.coordinateSuite->inputMatrixB(view_.X, view_.xi0);
  const Matrix Qc = B * params_.inputNoise * B.transpose();

  auto liftFunctor = [imu, dt, this](const VIOState& xi) -> Vector {
    if (params_.useDiscreteVelocityLift) {
      return (VIOGroup::Logmap(liftVelocityDiscrete(xi, imu, dt)) / dt).eval();
    }
    return liftVelocity(xi, imu);
  };

  Base::template predictWithJacobian<1>(liftFunctor, A, Qc, dt);
  syncFromBase();
}

void EqVIOFilter::synchronizeLandmarksToMeasurement(
    const VisionMeasurement& measurement) {
  if (params_.removeLostLandmarks) {
    removeOldLandmarks(measurement.getIds());
  }

  std::vector<Landmark> newLandmarks;
  const std::vector<int> existingIds = view_.xi0.ids();
  for (const auto& [id, y] : measurement.camCoordinates) {
    if (std::find(existingIds.begin(), existingIds.end(), id) !=
        existingIds.end()) {
      continue;
    }
    if (!measurement.camera) {
      throw std::invalid_argument(
          "EqVIOFilter::synchronizeLandmarksToMeasurement: null camera");
    }
    Vector3 bearing = measurement.camera->undistortPoint(y);
    if (bearing.norm() < 1e-9) continue;
    bearing.normalize();
    newLandmarks.push_back(
        Landmark{bearing * params_.initialPointDepth, static_cast<int>(id)});
  }
  addLandmarksInternal(newLandmarks);
}

void EqVIOFilter::addLandmarksInternal(
    const std::vector<Landmark>& newLandmarks) {
  if (newLandmarks.empty()) return;

  view_.xi0.cameraLandmarks.insert(view_.xi0.cameraLandmarks.end(),
                                   newLandmarks.begin(), newLandmarks.end());

  std::vector<SOT3> q;
  q.reserve(groupN(view_.X) + newLandmarks.size());
  for (size_t i = 0; i < groupN(view_.X); ++i) q.push_back(groupQ(view_.X)[i]);
  for (size_t i = 0; i < newLandmarks.size(); ++i) q.push_back(SOT3::Identity());

  view_.X = makeVIOGroup(groupA(view_.X), groupBeta(view_.X), groupB(view_.X),
                         VIOLandmarkGroup(q));

  const int oldSize = view_.Sigma.rows();
  const int newSize = view_.xi0.dim();
  const int added = newSize - oldSize;
  view_.Sigma.conservativeResize(newSize, newSize);
  view_.Sigma.block(oldSize, 0, added, oldSize).setZero();
  view_.Sigma.block(0, oldSize, oldSize, added).setZero();
  view_.Sigma.block(oldSize, oldSize, added, added) =
      Matrix::Identity(added, added) * params_.initialPointVariance;

  syncBase(true);
}

void EqVIOFilter::removeLandmarkByIndex(int idx) {
  view_.xi0.cameraLandmarks.erase(view_.xi0.cameraLandmarks.begin() + idx);

  std::vector<SOT3> q;
  q.reserve(groupN(view_.X) - 1);
  for (size_t i = 0; i < groupN(view_.X); ++i) {
    if (static_cast<int>(i) == idx) continue;
    q.push_back(groupQ(view_.X)[i]);
  }
  view_.X = makeVIOGroup(groupA(view_.X), groupBeta(view_.X), groupB(view_.X),
                         VIOLandmarkGroup(q));

  removeRows(view_.Sigma, VIOSensorState::CompDim + 3 * idx, 3);
  removeCols(view_.Sigma, VIOSensorState::CompDim + 3 * idx, 3);

  syncBase(true);
}

void EqVIOFilter::removeOldLandmarks(const std::vector<int>& measurementIds) {
  const std::vector<int> currentIds = view_.xi0.ids();
  std::vector<int> lostIndices(currentIds.size());
  std::iota(lostIndices.begin(), lostIndices.end(), 0);
  const auto lostEnd = std::remove_if(
      lostIndices.begin(), lostIndices.end(), [&](int lidx) {
        const int id = currentIds[static_cast<size_t>(lidx)];
        return std::find(measurementIds.begin(), measurementIds.end(), id) !=
               measurementIds.end();
      });
  lostIndices.erase(lostEnd, lostIndices.end());
  std::reverse(lostIndices.begin(), lostIndices.end());
  for (int idx : lostIndices) removeLandmarkByIndex(idx);
}

void EqVIOFilter::updateVision(const VisionMeasurement& measurement,
                               const Matrix& R) {
  if (!measurement.camera) {
    throw std::invalid_argument("EqVIOFilter::updateVision: null camera");
  }

  const VisionMeasurement prediction = measureSystemState(stateEstimate(), measurement.camera);
  const Vector yTilde =
      measurementResidual(measurement, prediction, "EqVIOFilter::updateVision");

  const Matrix Ct = view_.coordinateSuite->outputMatrixC(
      view_.xi0, view_.X, measurement, params_.useEquivariantOutput);
  const Matrix Rused = (R.rows() == Ct.rows() && R.cols() == Ct.rows())
                           ? R
                           : Matrix::Identity(Ct.rows(), Ct.rows()) *
                                 params_.measurementNoiseVariance;
  const Matrix SInv = (Ct * view_.Sigma * Ct.transpose() + Rused).inverse();
  const Matrix K = view_.Sigma * Ct.transpose() * SInv;
  const Vector Gamma = K * yTilde;

  VIOGroup delta = params_.useDiscreteInnovationLift
                       ? view_.coordinateSuite->liftInnovationDiscrete(
                             Gamma, view_.xi0)
                       : VIOGroup::Expmap(
                             view_.coordinateSuite->liftInnovation(Gamma, view_.xi0));

  setGroupEstimateAndSyncState(delta * view_.X);
  setErrorCovariance(view_.Sigma - K * Ct * view_.Sigma);
  syncFromBase();
}

}  // namespace gtsam
