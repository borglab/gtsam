/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file EqVIOFilter.cpp
 * @brief Standalone equivariant VIO filter.
 * @author Rohan Bansal
 */

#include <gtsam_unstable/navigation/EqVIOFilter.h>

#include <algorithm>
#include <cassert>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace gtsam {
namespace eqvio {

namespace {

inline int dense(size_t n) { return static_cast<int>(n); }

inline int landmarkOffset(size_t landmarkIndex) {
  return 21 + 3 * dense(landmarkIndex);
}

}  // namespace

/**
 * @brief Construct filter with given parameter bundle and identity initial
 * state.
 *
 * The base `EquivariantFilter` storage is initialized directly.
 */
EqVIOFilter::EqVIOFilter(const EqVIOFilterParams& params)
    : Base(State(), defaultCovariance(0), makeVioGroupIdentity()),
      params_(params) {
  State xi0;
  resetReferenceAndGroup(xi0, defaultCovariance(0), makeVioGroupIdentity());
  setLandmarkKeys({});
}

/**
 * @brief Construct filter from explicit reference state, covariance, and keys.
 *
 * Landmark keys are part of the runtime bookkeeping, so seeded landmark states
 * must provide a matching external key ordering.
 */
EqVIOFilter::EqVIOFilter(const State& xi_ref, const Matrix& Sigma,
                         const KeyVector& landmarkKeys,
                         const EqVIOFilterParams& params)
    : Base(State(), defaultCovariance(0), makeVioGroupIdentity()),
      params_(params) {
  resetReferenceAndGroup(xi_ref, Sigma, makeVioGroupIdentity(xi_ref.n()));
  setLandmarkKeys(landmarkKeys);
  initialized_ = true;
}

/**
 * @brief Initialize reference attitude from measured gravity direction.
 *
 * This method sets bias and velocity to nominal zeros and computes the shortest
 * rotation mapping measured acceleration to world +Z.
 */
void EqVIOFilter::initializeFromIMU(const IMUInput& imu) {
  State xi_ref = referenceState();
  xi_ref.bias = Bias::Identity();
  xi_ref.kinematics = Se23::Identity();

  Vector3 approxGravity = imu.acc;
  if (approxGravity.norm() < 1e-9) approxGravity = Vector3::UnitZ();
  // Build initial attitude that aligns measured gravity with +Z.
  Quaternion q;
  q.setFromTwoVectors(approxGravity.normalized(), Vector3::UnitZ());
  const Rot3 R0(q);
  xi_ref.kinematics = Se23(R0, Vector3::Zero(), Point3::Zero());
  initialized_ = true;
  resetReferenceAndGroup(xi_ref, errorCovariance(), groupEstimate());
}

/**
 * @brief Propagate mean/covariance jointly over one IMU hold interval.
 */
void EqVIOFilter::predict(const IMUInput& imu, double dt) {
  if (!initialized_ || dt <= 0.0) {
    return;
  }

  const Matrix A = EqFStateMatrixA(groupEstimate(), referenceState(), imu);
  const Lift lift_u(imu, dt);

  const Matrix B = EqFInputMatrixB(groupEstimate(), referenceState());
  const Matrix Qc = B * params_.inputNoise * B.transpose() +
                    stateProcessNoise(referenceState().n());

  Base::template predictWithJacobian<1>(lift_u, A, Qc, dt);
}

/**
 * @brief Visual update entry point including feature management.
 *
 * The update sequence is:
 * 1. Drop stale landmarks,
 * 2. Reject outliers and remove them from filter state,
 * 3. Add newly observed landmarks,
 * 4. Perform EKF-like correction,
 * 5. Remove numerically invalid landmarks.
 */
void EqVIOFilter::update(const VisionMeasurement& measurement,
                         const std::shared_ptr<const CameraModel>& camera,
                         const Matrix& R) {
  const int fullDim = 2 * dense(measurement.size());
  if (fullDim == 0 || R.rows() != fullDim || R.cols() != fullDim) {
    throw std::invalid_argument("EqVIOFilter::update: measurement covariance "
                                "must be 2M x 2M");
  }

  if (!initialized_) {
    return;
  }

  VisionMeasurement matchedMeasurement = measurement;
  reconcileLandmarks(matchedMeasurement, camera);

  if (matchedMeasurement.empty()) {
    return;
  }

  const int matchedDim = 2 * dense(matchedMeasurement.size());
  const Matrix matchedR =
      matchedDim == fullDim
          ? R
          : Matrix::Identity(matchedDim, matchedDim) * R(0, 0);
  innovationUpdate(matchedMeasurement, camera, matchedR);

  const KeyVector invalidKeys = invalidLandmarkKeys();
  if (!invalidKeys.empty()) {
    const KeySet invalidKeySet(invalidKeys.begin(), invalidKeys.end());
    std::vector<size_t> retainedIndices;
    retainedIndices.reserve(landmarkKeys_.size());
    for (size_t i = 0; i < landmarkKeys_.size(); ++i) {
      if (invalidKeySet.count(landmarkKeys_[i]) == 0) {
        retainedIndices.push_back(i);
      }
    }
    applyLandmarkStructureChange(retainedIndices, {});
  }

  assert(!errorCovariance().hasNaN());
}

/// Identity covariance helper sized for current sensor + landmark dimensions.
Matrix EqVIOFilter::defaultCovariance(size_t nLandmarks) {
  const int d = static_cast<int>(stateDim(nLandmarks));
  return Matrix::Identity(d, d);
}

/// Build block-diagonal process covariance from scalar per-component variances.
Matrix EqVIOFilter::stateProcessNoise(size_t nLandmarks) const {
  const int d = static_cast<int>(stateDim(nLandmarks));
  Matrix Q = Matrix::Identity(d, d);
  Q.block<3, 3>(0, 0) *= params_.biasOmegaProcessVariance;
  Q.block<3, 3>(3, 3) *= params_.biasAccelProcessVariance;
  Q.block<3, 3>(6, 6) *= params_.attitudeProcessVariance;
  Q.block<3, 3>(9, 9) *= params_.positionProcessVariance;
  Q.block<3, 3>(12, 12) *= params_.velocityProcessVariance;
  Q.block<3, 3>(15, 15) *= params_.cameraAttitudeProcessVariance;
  Q.block<3, 3>(18, 18) *= params_.cameraPositionProcessVariance;
  if (nLandmarks > 0) {
    const int landmarkDim = 3 * dense(nLandmarks);
    Q.block(21, 21, landmarkDim, landmarkDim) *= params_.pointProcessVariance;
  }
  return Q;
}

/**
 * @brief Apply innovation update for matched measurements.
 *
 * The supplied covariance must exactly match the current measurement
 * dimension.
 */
void EqVIOFilter::innovationUpdate(
    const VisionMeasurement& measurement,
    const std::shared_ptr<const CameraModel>& camera,
    const Matrix& measurementCovariance) {
  if (measurement.empty()) return;
  if (!camera) {
    throw std::invalid_argument(
        "EqVIOFilter::innovationUpdate: camera is null");
  }

  VisionMeasurement filteredMeasurement;
  VisionMeasurement estimatedMeasurement;
  const State& estimate = state();
  for (const auto& [key, y] : measurement) {
    const auto it = landmarkIndexByKey_.find(key);
    if (it == landmarkIndexByKey_.end()) {
      throw std::invalid_argument(
          "EqVIOFilter::innovationUpdate: measurement key not in filter state");
    }
    try {
      estimatedMeasurement[key] = camera->project2(estimate.cameraLandmarks[it->second]);
      filteredMeasurement[key] = y;
    } catch (const CheiralityException&) {
      // Skip this observation; it will be removed by landmark bookkeeping.
    }
  }

  if (filteredMeasurement.empty()) {
    return;
  }

  const Matrix Ct =
      EqFoutputMatrixC(referenceState(), landmarkKeys_, groupEstimate(),
                       filteredMeasurement, camera);

  const Matrix Rused = filteredMeasurement.size() == measurement.size()
                           ? measurementCovariance
                           : Matrix::Identity(Ct.rows(), Ct.rows()) *
                                 measurementCovariance(0, 0);

  const Vector zhat = measurementVector(estimatedMeasurement);
  const Vector z = measurementVector(filteredMeasurement);
  Base::updateWithVector(zhat, Ct, z, Rused,
                         [this](const Vector& delta_xi) -> Vector {
                           return liftInnovation(delta_xi, referenceState());
                         });
}

/**
 * @brief Validate/store landmark keys for the current state dimension.
 */
void EqVIOFilter::setLandmarkKeys(const KeyVector& landmarkKeys) {
  if (landmarkKeys.size() != referenceState().n()) {
    throw std::invalid_argument(
        "EqVIOFilter::setLandmarkKeys: key count must match landmark count");
  }

  KeySet uniqueKeys;
  for (Key key : landmarkKeys) {
    if (!uniqueKeys.insert(key).second) {
      throw std::invalid_argument(
          "EqVIOFilter::setLandmarkKeys: duplicate landmark key");
    }
  }

  landmarkKeys_ = landmarkKeys;
  missedFrameCounts_.assign(landmarkKeys_.size(), 0);
  rebuildLandmarkIndex();
}

/// Refresh the O(1) lookup table aligned with `landmarkKeys_`.
void EqVIOFilter::rebuildLandmarkIndex() {
  landmarkIndexByKey_.clear();
  for (size_t i = 0; i < landmarkKeys_.size(); ++i) {
    landmarkIndexByKey_[landmarkKeys_[i]] = i;
  }
}

/**
 * @brief Batch landmark bookkeeping around one visual update.
 *
 * Existing tracks survive one missed frame, absolute-residual outliers are
 * pruned from the current measurement/update, and new landmarks are inserted in
 * one structure rebuild.
 */
void EqVIOFilter::reconcileLandmarks(
    VisionMeasurement& measurement,
    const std::shared_ptr<const CameraModel>& camera) {
  if (!camera && !measurement.empty()) {
    throw std::invalid_argument(
        "EqVIOFilter::reconcileLandmarks: camera is null");
  }

  KeySet observedKeys;
  for (const auto& [key, _] : measurement) {
    observedKeys.insert(key);
  }

  for (size_t i = 0; i < landmarkKeys_.size(); ++i) {
    if (observedKeys.count(landmarkKeys_[i]) != 0) {
      missedFrameCounts_[i] = 0;
    } else {
      ++missedFrameCounts_[i];
    }
  }

  KeyVector removalKeys = detectOutliers(measurement, camera);

  const KeyVector invalidKeys = invalidLandmarkKeys();
  removalKeys.insert(removalKeys.end(), invalidKeys.begin(), invalidKeys.end());
  for (size_t i = 0; i < landmarkKeys_.size(); ++i) {
    if (missedFrameCounts_[i] > kMaxMissedFrames) {
      removalKeys.push_back(landmarkKeys_[i]);
    }
  }

  const KeySet removalKeySet(removalKeys.begin(), removalKeys.end());
  std::vector<size_t> retainedIndices;
  retainedIndices.reserve(landmarkKeys_.size());
  for (size_t i = 0; i < landmarkKeys_.size(); ++i) {
    if (removalKeySet.count(landmarkKeys_[i]) == 0) {
      retainedIndices.push_back(i);
    }
  }

  std::vector<std::pair<Key, Point3>> newLandmarks;
  newLandmarks.reserve(measurement.size());
  for (const auto& [key, coordinate] : measurement) {
    if (landmarkIndexByKey_.count(key) != 0) continue;

    const Point3 landmark =
        undistortPoint(*camera, coordinate) * params_.initialPointDepth;
    newLandmarks.emplace_back(key, landmark);
  }

  if (retainedIndices.size() == landmarkKeys_.size() && newLandmarks.empty()) {
    return;
  }

  applyLandmarkStructureChange(retainedIndices, newLandmarks);
}

/**
 * @brief Reject outliers using only absolute reprojection residual.
 *
 * This keeps the example/runtime path cheap and avoids using a partial
 * innovation covariance proxy for gating.
 */
KeyVector EqVIOFilter::detectOutliers(
    VisionMeasurement& measurement,
    const std::shared_ptr<const CameraModel>& camera) const {
  const size_t maxOutliers = static_cast<size_t>(
      (1.0 - params_.featureRetention) * measurement.size());
  if (!camera || measurement.empty() || maxOutliers == 0 ||
      landmarkKeys_.empty()) {
    return {};
  }

  std::vector<std::pair<Key, double>> residuals;
  residuals.reserve(measurement.size());
  for (const auto& [lmId, y] : measurement) {
    const auto itIndex = landmarkIndexByKey_.find(lmId);
    if (itIndex == landmarkIndexByKey_.end()) {
      continue;
    }

    double errAbs = std::numeric_limits<double>::infinity();
    try {
      const Point2 yHat = camera->project2(state().cameraLandmarks[itIndex->second]);
      errAbs = (y - yHat).norm();
    } catch (const CheiralityException&) {
      // Keep inf residual to prioritize removal.
    }

    if (errAbs > params_.outlierThresholdAbs) {
      residuals.emplace_back(lmId, errAbs);
    }
  }

  std::sort(residuals.begin(), residuals.end(),
            [](const auto& lhs, const auto& rhs) {
              return lhs.second > rhs.second;
            });
  if (residuals.size() > maxOutliers) {
    residuals.resize(maxOutliers);
  }

  KeyVector outlierKeys;
  outlierKeys.reserve(residuals.size());
  for (const auto& [lmId, _] : residuals) {
    measurement.erase(lmId);
    outlierKeys.push_back(lmId);
  }
  return outlierKeys;
}

/// Return keys whose landmark-group scale has become numerically invalid.
KeyVector EqVIOFilter::invalidLandmarkKeys() const {
  const LandmarkGroup& Q = std::get<3>(decompose(groupEstimate()));
  KeyVector invalidKeys;
  invalidKeys.reserve(Q.size());
  for (size_t i = 0; i < Q.size(); ++i) {
    const double scale = SOT3Scale(Q[i]);
    if (!std::isfinite(scale) || scale <= 1e-8 || scale > 1e8) {
      invalidKeys.push_back(landmarkKeys_[i]);
    }
  }
  return invalidKeys;
}

/**
 * @brief Rebuild the dynamic landmark structure in one pass.
 *
 * The physical estimate is preserved for retained landmarks, while newly
 * observed landmarks enter with identity group transform and diagonal initial
 * covariance.
 */
void EqVIOFilter::applyLandmarkStructureChange(
    const std::vector<size_t>& retainedIndices,
    const std::vector<std::pair<Key, Point3>>& newLandmarks) {
  const State& currentReference = referenceState();
  const auto& [A, Beta, B, Q] = decompose(groupEstimate());

  State nextReference;
  nextReference.kinematics = currentReference.kinematics;
  nextReference.bias = currentReference.bias;
  nextReference.cameraOffset = currentReference.cameraOffset;
  nextReference.cameraLandmarks.reserve(retainedIndices.size() +
                                        newLandmarks.size());

  KeyVector nextKeys;
  std::vector<size_t> nextMissedFrameCounts;
  std::vector<SOT3> nextQ;
  nextKeys.reserve(retainedIndices.size() + newLandmarks.size());
  nextMissedFrameCounts.reserve(retainedIndices.size() + newLandmarks.size());
  nextQ.reserve(retainedIndices.size() + newLandmarks.size());

  for (size_t index : retainedIndices) {
    nextReference.cameraLandmarks.push_back(
        currentReference.cameraLandmarks[index]);
    nextKeys.push_back(landmarkKeys_[index]);
    nextMissedFrameCounts.push_back(missedFrameCounts_[index]);
    nextQ.push_back(Q[index]);
  }

  for (const auto& [key, landmark] : newLandmarks) {
    nextReference.cameraLandmarks.push_back(landmark);
    nextKeys.push_back(key);
    nextMissedFrameCounts.push_back(0);
    nextQ.push_back(SOT3::Identity());
  }

  const Matrix nextCovariance =
      rebuildCovariance(retainedIndices, newLandmarks.size());
  const VioGroup nextGroup = makeVioGroup(A, Beta, B, LandmarkGroup(nextQ));

  resetReferenceAndGroup(nextReference, nextCovariance, nextGroup);
  landmarkKeys_ = std::move(nextKeys);
  missedFrameCounts_ = std::move(nextMissedFrameCounts);
  rebuildLandmarkIndex();
}

/// Rebuild covariance after a batch landmark add/remove operation.
Matrix EqVIOFilter::rebuildCovariance(
    const std::vector<size_t>& retainedIndices, size_t newLandmarkCount) const {
  const Matrix& currentCovariance = errorCovariance();
  const int newLandmarkBlockCount = dense(newLandmarkCount);
  const int newDimension =
      static_cast<int>(stateDim(retainedIndices.size() + newLandmarkCount));

  Matrix rebuilt = Matrix::Zero(newDimension, newDimension);
  rebuilt.block(0, 0, 21, 21) = currentCovariance.block(0, 0, 21, 21);

  for (size_t newI = 0; newI < retainedIndices.size(); ++newI) {
    const int srcI = landmarkOffset(retainedIndices[newI]);
    const int dstI = landmarkOffset(newI);

    rebuilt.block(0, dstI, 21, 3) = currentCovariance.block(0, srcI, 21, 3);
    rebuilt.block(dstI, 0, 3, 21) = currentCovariance.block(srcI, 0, 3, 21);

    for (size_t newJ = 0; newJ < retainedIndices.size(); ++newJ) {
      const int srcJ = landmarkOffset(retainedIndices[newJ]);
      const int dstJ = landmarkOffset(newJ);
      rebuilt.block(dstI, dstJ, 3, 3) =
          currentCovariance.block(srcI, srcJ, 3, 3);
    }
  }

  if (newLandmarkBlockCount > 0) {
    const int newOffset = landmarkOffset(retainedIndices.size());
    rebuilt.block(newOffset, newOffset, 3 * newLandmarkBlockCount,
                  3 * newLandmarkBlockCount) =
        Matrix::Identity(3 * newLandmarkBlockCount, 3 * newLandmarkBlockCount) *
        params_.initialPointVariance;
  }

  return rebuilt;
}

}  // namespace eqvio
}  // namespace gtsam
