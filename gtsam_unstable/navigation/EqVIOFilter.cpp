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
#include <cmath>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string>

namespace gtsam {
namespace eqvio {


EqVIOFilter::EqVIOFilter() : EqVIOFilter(EqVIOFilterParams()) {}

EqVIOFilter::EqVIOFilter(const EqVIOFilterParams& params)
    : Base(VIOState(), defaultCovariance(0), makeVIOGroupIdentity()),
      params_(params) {
  view_.xi0 = VIOState();
  view_.xi0.sensor.inputBias = VIOBias::Identity();
  view_.xi0.sensor.pose = Pose3::Identity();
  view_.xi0.sensor.velocity.setZero();
  view_.xi0.sensor.cameraOffset = Pose3::Identity();
  view_.X = makeVIOGroupIdentity();
  view_.Sigma = defaultCovariance(0);
  view_.currentTime = -1.0;
  syncBase(true);
}

EqVIOFilter::EqVIOFilter(const VIOState& xi0, const Matrix& Sigma0,
                         const EqVIOFilterParams& params, double time)
    : Base(VIOState(), defaultCovariance(0), makeVIOGroupIdentity()),
      params_(params) {
  view_.xi0 = xi0;
  view_.X = makeVIOGroupIdentity(view_.xi0.n());
  view_.Sigma = Sigma0;
  view_.currentTime = time;
  initialized_ = true;
  syncBase(true);
}

void EqVIOFilter::initializeFromIMU(const IMUInput& imu) {
  view_.xi0.sensor.inputBias = VIOBias::Identity();
  view_.xi0.sensor.pose = Pose3::Identity();
  view_.xi0.sensor.velocity.setZero();

  Vector3 approxGravity = imu.acc;
  if (approxGravity.norm() < 1e-9) approxGravity = Vector3::UnitZ();
  const Rot3 R0 =
      rotationFromTwoVectors(approxGravity.normalized(), Vector3::UnitZ());
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

void EqVIOFilter::processIMUData(const IMUInput& imu) {
  if (!initialized_) {
    initializeFromIMU(imu);
  }
  imuBuffer_.push_back(imu);
}

void EqVIOFilter::processVisionData(
    double stamp, const VisionMeasurement& measurement,
    const std::shared_ptr<const VIOCameraModel>& camera, const Matrix& R) {
  const bool integrationFlag = integrateUpToTime(stamp);
  if (!integrationFlag || !initialized_) return;

  if (params_.removeLostLandmarks) {
    removeOldLandmarks(measurementIds(measurement));
  }

  VisionMeasurement matchedMeasurement = measurement;
  removeOutliers(matchedMeasurement, camera);
  addNewLandmarks(matchedMeasurement, camera);

  if (matchedMeasurement.empty()) {
    syncBase(false);
    return;
  }

  update(matchedMeasurement, camera, R);
  if (params_.removeInvalidLandmarks) {
    removeInvalidLandmarksNow();
  }
  syncBase(false);

  assert(!view_.Sigma.hasNaN());
}

VIOState EqVIOFilter::stateEstimate() const {
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
    return;
  }
  if (referenceState().n() != N_landmarkCount(view_.X)) {
    throw std::invalid_argument(
        "EqVIOFilter::syncBase(false): referenceState.n()=" +
        std::to_string(referenceState().n()) +
        ", X.n()=" + std::to_string(N_landmarkCount(view_.X)) +
        ", xi0.n()=" + std::to_string(view_.xi0.n()));
  }
  setGroupEstimateAndSyncState(view_.X);
  setErrorCovariance(view_.Sigma);
}

void EqVIOFilter::syncFromBase() {
  view_.X = groupEstimate();
  view_.Sigma = errorCovariance();
}

void EqVIOFilter::integrateRiccatiStateFast(
    const IMUInput& imu, double dt,
    const Eigen::Matrix<double, IMUInput::CompDim, IMUInput::CompDim>&
        inputGainMatrix,
    const Matrix& stateGainMatrix) {
  const Matrix A0t = EqFCoordinateSuite_invdepth.stateMatrixA(view_.X, view_.xi0, imu);
  const Matrix Bt = EqFCoordinateSuite_invdepth.inputMatrixB(view_.X, view_.xi0);
  const Matrix A0tExp = Matrix::Identity(view_.xi0.dim(), view_.xi0.dim()) + dt * A0t;
  view_.Sigma = A0tExp * view_.Sigma * A0tExp.transpose() +
                dt * (Bt * inputGainMatrix * Bt.transpose() + stateGainMatrix);
}

Matrix EqVIOFilter::stateProcessNoise(size_t nLandmarks) const {
  Matrix Q = Matrix::Identity(
      VIOSensorState::CompDim + 3 * static_cast<int>(nLandmarks),
      VIOSensorState::CompDim + 3 * static_cast<int>(nLandmarks));
  Q.block<3, 3>(0, 0) *= params_.biasOmegaProcessVariance;
  Q.block<3, 3>(3, 3) *= params_.biasAccelProcessVariance;
  Q.block<3, 3>(6, 6) *= params_.attitudeProcessVariance;
  Q.block<3, 3>(9, 9) *= params_.positionProcessVariance;
  Q.block<3, 3>(12, 12) *= params_.velocityProcessVariance;
  Q.block<3, 3>(15, 15) *= params_.cameraAttitudeProcessVariance;
  Q.block<3, 3>(18, 18) *= params_.cameraPositionProcessVariance;
  if (nLandmarks > 0) {
    Q.block(VIOSensorState::CompDim, VIOSensorState::CompDim,
            3 * static_cast<int>(nLandmarks), 3 * static_cast<int>(nLandmarks)) *=
        params_.pointProcessVariance;
  }
  return Q;
}

bool EqVIOFilter::integrateUpToTime(double newTime) {
  if (newTime <= view_.currentTime || view_.currentTime < 0.0 ||
      imuBuffer_.empty()) {
    return false;
  }

  double accumulatedTime = 0.0;
  IMUInput accumulatedVelocity = IMUInput::Zero();
  for (size_t i = 0; i < imuBuffer_.size(); ++i) {
    const double t0 = std::max(imuBuffer_.at(i).stamp, view_.currentTime);
    const double t1 =
        i + 1 < imuBuffer_.size() ? std::min(imuBuffer_.at(i + 1).stamp, newTime)
                                  : newTime;
    const double dt = std::max(t1 - t0, 0.0);
    accumulatedTime += dt;
    accumulatedVelocity = accumulatedVelocity + imuBuffer_.at(i) * dt;
  }

  if (accumulatedTime > 0.0) {
    accumulatedVelocity = accumulatedVelocity * (1.0 / accumulatedTime);
    integrateRiccatiStateFast(accumulatedVelocity, accumulatedTime,
                              params_.inputNoise, stateProcessNoise(view_.xi0.n()));
    setErrorCovariance(view_.Sigma);
  }

  for (size_t i = 0; i < imuBuffer_.size(); ++i) {
    const double t0 = std::max(imuBuffer_.at(i).stamp, view_.currentTime);
    const double t1 =
        i + 1 < imuBuffer_.size() ? std::min(imuBuffer_.at(i + 1).stamp, newTime)
                                  : newTime;
    const double dt = std::max(t1 - t0, 0.0);
    if (dt <= 0.0) continue;

    const IMUInput imu = imuBuffer_.at(i);
    auto liftFunctor = [imu, dt](const VIOState& xi) -> Vector {
      return (VIOGroup::Logmap(liftVelocityDiscrete(xi, imu, dt)) / dt).eval();
    };
    const Matrix A = Matrix::Zero(view_.xi0.dim(), view_.xi0.dim());
    const Matrix Qc = Matrix::Zero(view_.xi0.dim(), view_.xi0.dim());
    Base::template predictWithJacobian<1>(liftFunctor, A, Qc, dt);
    syncFromBase();
  }

  view_.currentTime = newTime;

  auto it = std::find_if(imuBuffer_.begin(), imuBuffer_.end(),
                         [this](const IMUInput& imu) {
                           return imu.stamp >= this->view_.currentTime;
                         });
  if (it != imuBuffer_.begin()) {
    --it;
    imuBuffer_.erase(imuBuffer_.begin(), it);
  }

  syncFromBase();
  return true;
}

void EqVIOFilter::addNewLandmarks(
    const VisionMeasurement& measurement,
    const std::shared_ptr<const VIOCameraModel>& camera) {
  if (measurement.empty()) return;
  if (!camera) {
    throw std::invalid_argument("EqVIOFilter::addNewLandmarks: null camera");
  }

  std::vector<Landmark> newLandmarks;
  const std::vector<int> existingIds = view_.xi0.ids();
  for (const auto& [id, coord] : measurement) {
    if (std::find(existingIds.begin(), existingIds.end(), id) != existingIds.end()) {
      continue;
    }
    const Vector3 bearing = camera->undistortPoint(coord);
    newLandmarks.push_back(Landmark{bearing, id});
  }

  if (newLandmarks.empty()) return;

  const double initialDepth =
      params_.useMedianDepth ? getMedianSceneDepth() : params_.initialPointDepth;
  for (Landmark& lm : newLandmarks) lm.p *= initialDepth;

  const Matrix newLandmarksCov =
      Matrix::Identity(3 * static_cast<int>(newLandmarks.size()),
                       3 * static_cast<int>(newLandmarks.size())) *
      params_.initialPointVariance;
  addLandmarksInternal(newLandmarks, newLandmarksCov);
}

void EqVIOFilter::addLandmarksInternal(std::vector<Landmark>& newLandmarks,
                                       const Matrix& newLandmarkCov) {
  if (newLandmarks.empty()) return;

  view_.xi0.cameraLandmarks.insert(view_.xi0.cameraLandmarks.end(),
                                   newLandmarks.begin(), newLandmarks.end());

  std::vector<SOT3> q;
  q.reserve(N_landmarkCount(view_.X) + newLandmarks.size());
  for (size_t i = 0; i < N_landmarkCount(view_.X); ++i) {
    q.push_back(Q_landmarkTransforms(view_.X)[i]);
  }
  for (size_t i = 0; i < newLandmarks.size(); ++i) {
    q.push_back(SOT3::Identity());
  }

  view_.X = makeVIOGroup(A_sensorKinematics(view_.X), Beta_biasOffset(view_.X),
                         B_cameraExtrinsics(view_.X), VIOLandmarkGroup(q));

  const int oldSize = view_.Sigma.rows();
  const int newN = static_cast<int>(newLandmarks.size());
  view_.Sigma.conservativeResize(oldSize + 3 * newN, oldSize + 3 * newN);
  view_.Sigma.block(oldSize, 0, 3 * newN, oldSize).setZero();
  view_.Sigma.block(0, oldSize, oldSize, 3 * newN).setZero();
  view_.Sigma.block(oldSize, oldSize, 3 * newN, 3 * newN) = newLandmarkCov;

  syncBase(true);
}

void EqVIOFilter::removeOldLandmarks(const std::vector<int>& measurementIds) {
  const std::vector<int> existingIds = view_.xi0.ids();
  std::vector<int> lostIndices(existingIds.size());
  std::iota(lostIndices.begin(), lostIndices.end(), 0);
  if (lostIndices.empty()) return;

  const auto lostIndicesEnd = std::remove_if(
      lostIndices.begin(), lostIndices.end(), [&](const int& lidx) {
        const int oldId = existingIds[static_cast<size_t>(lidx)];
        return std::any_of(measurementIds.begin(), measurementIds.end(),
                           [&oldId](const int& measId) { return measId == oldId; });
      });
  lostIndices.erase(lostIndicesEnd, lostIndices.end());

  if (lostIndices.empty()) return;

  std::reverse(lostIndices.begin(), lostIndices.end());
  for (const int idx : lostIndices) {
    removeLandmarkByIndex(idx);
  }
}

void EqVIOFilter::removeLandmarkByIndex(int idx) {
  view_.xi0.cameraLandmarks.erase(view_.xi0.cameraLandmarks.begin() + idx);

  std::vector<SOT3> q;
  q.reserve(N_landmarkCount(view_.X) - 1);
  for (size_t i = 0; i < N_landmarkCount(view_.X); ++i) {
    if (static_cast<int>(i) == idx) continue;
    q.push_back(Q_landmarkTransforms(view_.X)[i]);
  }

  view_.X = makeVIOGroup(A_sensorKinematics(view_.X), Beta_biasOffset(view_.X),
                         B_cameraExtrinsics(view_.X), VIOLandmarkGroup(q));

  removeRows(view_.Sigma, VIOSensorState::CompDim + 3 * idx, 3);
  removeCols(view_.Sigma, VIOSensorState::CompDim + 3 * idx, 3);

  syncBase(true);
}

void EqVIOFilter::removeLandmarkById(int id) {
  const auto it = std::find_if(
      view_.xi0.cameraLandmarks.begin(), view_.xi0.cameraLandmarks.end(),
      [&id](const Landmark& lm) { return lm.id == id; });
  assert(it != view_.xi0.cameraLandmarks.end());
  removeLandmarkByIndex(
      static_cast<int>(std::distance(view_.xi0.cameraLandmarks.begin(), it)));
}

Matrix3 EqVIOFilter::getLandmarkCovById(int id) const {
  const auto it = std::find_if(
      view_.xi0.cameraLandmarks.begin(), view_.xi0.cameraLandmarks.end(),
      [&id](const Landmark& lm) { return lm.id == id; });
  assert(it != view_.xi0.cameraLandmarks.end());
  const int i =
      static_cast<int>(std::distance(view_.xi0.cameraLandmarks.begin(), it));
  return view_.Sigma.block<3, 3>(VIOSensorState::CompDim + 3 * i,
                                 VIOSensorState::CompDim + 3 * i);
}

Matrix2 EqVIOFilter::outputCovarianceById(
    int id, const Point2& y,
    const std::shared_ptr<const VIOCameraModel>& camera) const {
  (void)y;
  const Matrix3 lmCov = getLandmarkCovById(id);
  const auto it = std::find_if(
      view_.xi0.cameraLandmarks.begin(), view_.xi0.cameraLandmarks.end(),
      [&id](const Landmark& lm) { return lm.id == id; });
  assert(it != view_.xi0.cameraLandmarks.end());

  const size_t i =
      static_cast<size_t>(std::distance(view_.xi0.cameraLandmarks.begin(), it));
  const SOT3& Q_i = Q_landmarkTransforms(view_.X)[i];

  const Matrix23 C0i = EqFCoordinateSuite_invdepth.outputMatrixCi(it->p, Q_i, camera);
  return C0i * lmCov * C0i.transpose();
}

void EqVIOFilter::removeInvalidLandmarksNow() {
  std::set<int> invalidIds;
  for (size_t i = 0; i < N_landmarkCount(view_.X); ++i) {
    const double a = SOT3Scale(Q_landmarkTransforms(view_.X)[i]);
    if (!std::isfinite(a) || a <= 1e-8 || a > 1e8) {
      invalidIds.insert(view_.xi0.cameraLandmarks[i].id);
    }
  }
  for (const int id : invalidIds) {
    removeLandmarkById(id);
  }
}

void EqVIOFilter::removeOutliers(
    VisionMeasurement& measurement,
    const std::shared_ptr<const VIOCameraModel>& camera) {
  const size_t maxOutliers = static_cast<size_t>(
      (1.0 - params_.featureRetention) * measurement.size());
  if (!camera) return;

  const VisionMeasurement yHat = measureSystemState(stateEstimate(), camera);

  std::vector<int> proposedOutliers;
  std::map<int, double> absoluteOutliers;
  for (const auto& [lmId, yHatI] : yHat) {
    if (measurement.count(lmId) == 0) continue;
    const double errAbs = (measurement.at(lmId) - yHatI).norm();
    if (errAbs > params_.outlierThresholdAbs) {
      absoluteOutliers[lmId] = errAbs;
      proposedOutliers.push_back(lmId);
    }
  }

  std::map<int, double> probabilisticOutliers;
  for (const auto& [lmId, yI] : measurement) {
    if (absoluteOutliers.count(lmId)) continue;
    const auto itHat = yHat.find(lmId);
    if (itHat == yHat.end()) continue;
    const Point2 yTildeI = yI - itHat->second;

    const Matrix2 outputCov = outputCovarianceById(lmId, measurement.at(lmId), camera);
    const double errProb = yTildeI.transpose() * outputCov.inverse() * yTildeI;
    if (errProb > params_.outlierThresholdProb) {
      probabilisticOutliers[lmId] = errProb;
      proposedOutliers.push_back(lmId);
    }
  }

  std::sort(proposedOutliers.begin(), proposedOutliers.end(),
            [&absoluteOutliers, &probabilisticOutliers](int lmId1, int lmId2) {
              if (absoluteOutliers.count(lmId1)) {
                if (absoluteOutliers.count(lmId2)) {
                  return absoluteOutliers.at(lmId1) < absoluteOutliers.at(lmId2);
                }
                return false;
              }
              if (absoluteOutliers.count(lmId2)) return true;
              return probabilisticOutliers.at(lmId1) < probabilisticOutliers.at(lmId2);
            });
  std::reverse(proposedOutliers.begin(), proposedOutliers.end());
  if (proposedOutliers.size() > maxOutliers) {
    proposedOutliers.erase(proposedOutliers.begin() + maxOutliers,
                           proposedOutliers.end());
  }

  for (const int lmId : proposedOutliers) {
    removeLandmarkById(lmId);
    measurement.erase(lmId);
  }
}

double EqVIOFilter::getMedianSceneDepth() const {
  const std::vector<Landmark> landmarks = stateEstimate().cameraLandmarks;
  if (landmarks.empty()) return params_.initialPointDepth;

  std::vector<double> depthsSquared(landmarks.size());
  std::transform(landmarks.begin(), landmarks.end(), depthsSquared.begin(),
                 [](const Landmark& lm) { return lm.p.squaredNorm(); });
  const auto midway = depthsSquared.begin() + depthsSquared.size() / 2;
  std::nth_element(depthsSquared.begin(), midway, depthsSquared.end());

  double medianDepth = params_.initialPointDepth;
  if (midway != depthsSquared.end()) {
    medianDepth = std::pow(*midway, 0.5);
  }
  return medianDepth;
}

void EqVIOFilter::update(const VisionMeasurement& measurement,
                         const std::shared_ptr<const VIOCameraModel>& camera,
                         const Matrix& outputGainMatrix) {
  if (measurement.empty()) return;
  if (!camera) {
    throw std::invalid_argument("EqVIOFilter::update: null camera");
  }
  const VisionMeasurement estimatedMeasurement =
      measureSystemState(stateEstimate(), camera);
  Vector yTilde;
  try {
    yTilde = measurementDifference(measurement, estimatedMeasurement);
  } catch (const std::exception& e) {
    throw std::invalid_argument(std::string("EqVIOFilter::update: ") + e.what());
  }
  const Matrix Ct = EqFCoordinateSuite_invdepth.outputMatrixC(
      view_.xi0, view_.X, measurement, camera, true);

  const Matrix Rused =
      (outputGainMatrix.rows() == Ct.rows() && outputGainMatrix.cols() == Ct.rows())
          ? outputGainMatrix
          : Matrix::Identity(Ct.rows(), Ct.rows()) * params_.measurementNoiseVariance;

  const Vector zhat = measurementVector(estimatedMeasurement);
  const Vector z = measurementVector(measurement);
  Base::updateWithVector(zhat, Ct, z, Rused, [this](const Vector& delta_xi) -> Vector {
    return EqFCoordinateSuite_invdepth.liftInnovation(delta_xi, view_.xi0);
  });
  syncFromBase();
}

}  // namespace eqvio
}  // namespace gtsam
