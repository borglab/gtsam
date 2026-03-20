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

namespace {

Vector measurementVector(const VisionMeasurement& measurement) {
  Vector v = Vector::Zero(static_cast<int>(2 * measurement.size()));
  int i = 0;
  for (const auto& item : measurement) {
    v.segment<2>(2 * i) = item.second;
    ++i;
  }
  return v;
}

Vector measurementDifference(const VisionMeasurement& lhs,
                             const VisionMeasurement& rhs) {
  if (lhs.size() != rhs.size()) {
    throw std::invalid_argument("measurementDifference: size mismatch");
  }

  Vector diff = Vector::Zero(static_cast<int>(2 * lhs.size()));
  auto itL = lhs.begin();
  auto itR = rhs.begin();
  int i = 0;
  for (; itL != lhs.end(); ++itL, ++itR) {
    if (itL->first != itR->first) {
      throw std::invalid_argument("measurementDifference: id mismatch");
    }
    diff.segment<2>(2 * i) = itL->second - itR->second;
    ++i;
  }
  return diff;
}

}  // namespace

EqVIOFilter::EqVIOFilter() : EqVIOFilter(EqVIOFilterParams()) {}

EqVIOFilter::EqVIOFilter(const EqVIOFilterParams& params)
    : Base(State(), defaultCovariance(0), makeVioGroupIdentity()),
      params_(params) {
  view_.xi0 = State();
  view_.xi0.sensor.inputBias = Bias::Identity();
  view_.xi0.sensor.pose = Pose3::Identity();
  view_.xi0.sensor.velocity.setZero();
  view_.xi0.sensor.cameraOffset = Pose3::Identity();
  view_.X = makeVioGroupIdentity();
  view_.Sigma = defaultCovariance(0);
  syncBase(true);
}

EqVIOFilter::EqVIOFilter(const State& xi0, const Matrix& Sigma0,
                         const EqVIOFilterParams& params)
    : Base(State(), defaultCovariance(0), makeVioGroupIdentity()),
      params_(params) {
  view_.xi0 = xi0;
  view_.X = makeVioGroupIdentity(view_.xi0.n());
  view_.Sigma = Sigma0;
  initialized_ = true;
  syncBase(true);
}

void EqVIOFilter::initializeFromIMU(const IMUInput& imu) {
  view_.xi0.sensor.inputBias = Bias::Identity();
  view_.xi0.sensor.pose = Pose3::Identity();
  view_.xi0.sensor.velocity.setZero();

  Vector3 approxGravity = imu.acc;
  if (approxGravity.norm() < 1e-9) approxGravity = Vector3::UnitZ();
  // Build initial attitude that aligns measured gravity with +Z.
  Quaternion q;
  q.setFromTwoVectors(approxGravity.normalized(), Vector3::UnitZ());
  const Rot3 R0(q);
  view_.xi0.sensor.pose = Pose3(R0, Point3::Zero());
  initialized_ = true;
  syncBase(true);
}

void EqVIOFilter::setReferenceState(const State& xi0, const Matrix& Sigma0) {
  if (Sigma0.rows() != xi0.dim() || Sigma0.cols() != xi0.dim()) {
    throw std::invalid_argument(
        "EqVIOFilter::setReferenceState: covariance dimension mismatch");
  }
  view_.xi0 = xi0;
  view_.X = makeVioGroupIdentity(xi0.n());
  view_.Sigma = Sigma0;
  syncBase(true);
}

void EqVIOFilter::propagate(const IMUInput& imu, double dt) {
  propagateCovariance(imu, dt);
  propagateState(imu, dt);
}

void EqVIOFilter::propagateCovariance(const IMUInput& imu, double dt) {
  if (!initialized_ || dt <= 0.0) {
    return;
  }
  const Matrix A0t =
      EqFCoordinateSuite_invdepth.stateMatrixA(view_.X, view_.xi0, imu);
  const Matrix Bt = EqFCoordinateSuite_invdepth.inputMatrixB(view_.X, view_.xi0);
  const Matrix A0tExp =
      Matrix::Identity(view_.xi0.dim(), view_.xi0.dim()) + dt * A0t;
  view_.Sigma = A0tExp * view_.Sigma * A0tExp.transpose() +
                dt * (Bt * params_.inputNoise * Bt.transpose() +
                      stateProcessNoise(view_.xi0.n()));
  setErrorCovariance(view_.Sigma);
}

void EqVIOFilter::propagateState(const IMUInput& imu, double dt) {
  if (!initialized_ || dt <= 0.0) {
    return;
  }
  auto liftFunctor = [imu, dt](const State& xi) -> Vector {
    return (VioGroup::Logmap(liftVelocityDiscrete(xi, imu, dt)) / dt).eval();
  };
  const Matrix A = Matrix::Zero(view_.xi0.dim(), view_.xi0.dim());
  const Matrix Qc = Matrix::Zero(view_.xi0.dim(), view_.xi0.dim());
  Base::template predictWithJacobian<1>(liftFunctor, A, Qc, dt);
  syncFromBase();
}

void EqVIOFilter::correct(const VisionMeasurement& measurement,
                          const std::shared_ptr<const CameraModel>& camera,
                          const Matrix& R) {
  if (!initialized_) {
    return;
  }

  removeOldLandmarks(measurementIds(measurement));

  VisionMeasurement matchedMeasurement = measurement;
  removeOutliers(matchedMeasurement, camera);
  addNewLandmarks(matchedMeasurement, camera);

  if (matchedMeasurement.empty()) {
    syncBase(false);
    return;
  }

  update(matchedMeasurement, camera, R);
  removeInvalidLandmarksNow();

  syncBase(false);

  assert(!view_.Sigma.hasNaN());
}

State EqVIOFilter::stateEstimate() const {
  return stateGroupAction(view_.X, view_.xi0);
}

Matrix EqVIOFilter::defaultCovariance(size_t nLandmarks) {
  const int d = SensorState::CompDim + 3 * static_cast<int>(nLandmarks);
  return Matrix::Identity(d, d);
}

void EqVIOFilter::syncBase(bool resetReference) {
  if (resetReference) {
    resetReferenceAndGroup(view_.xi0, view_.Sigma, view_.X);
    return;
  }
  // ensure referenceState().n() == N_landmarkCount(view_.X))
  setGroupEstimateAndSyncState(view_.X);
  setErrorCovariance(view_.Sigma);
}

void EqVIOFilter::syncFromBase() {
  view_.X = groupEstimate();
  view_.Sigma = errorCovariance();
}

Matrix EqVIOFilter::stateProcessNoise(size_t nLandmarks) const {
  Matrix Q = Matrix::Identity(
      SensorState::CompDim + 3 * static_cast<int>(nLandmarks),
      SensorState::CompDim + 3 * static_cast<int>(nLandmarks));
  Q.block<3, 3>(0, 0) *= params_.biasOmegaProcessVariance;
  Q.block<3, 3>(3, 3) *= params_.biasAccelProcessVariance;
  Q.block<3, 3>(6, 6) *= params_.attitudeProcessVariance;
  Q.block<3, 3>(9, 9) *= params_.positionProcessVariance;
  Q.block<3, 3>(12, 12) *= params_.velocityProcessVariance;
  Q.block<3, 3>(15, 15) *= params_.cameraAttitudeProcessVariance;
  Q.block<3, 3>(18, 18) *= params_.cameraPositionProcessVariance;
  if (nLandmarks > 0) {
    Q.block(SensorState::CompDim, SensorState::CompDim,
            3 * static_cast<int>(nLandmarks), 3 * static_cast<int>(nLandmarks)) *=
        params_.pointProcessVariance;
  }
  return Q;
}

void EqVIOFilter::addNewLandmarks(
    const VisionMeasurement& measurement,
    const std::shared_ptr<const CameraModel>& camera) {
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
    const Vector3 bearing = undistortPoint(*camera, coord);
    newLandmarks.push_back(Landmark{bearing, id});
  }

  if (newLandmarks.empty()) return;

  const double initialDepth = params_.initialPointDepth;
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

  const auto& [A, Beta, B, Q] = view_.X;
  std::vector<SOT3> q;
  q.reserve(Q.size() + newLandmarks.size());
  for (size_t i = 0; i < Q.size(); ++i) {
    q.push_back(Q[i]);
  }
  for (size_t i = 0; i < newLandmarks.size(); ++i) {
    q.push_back(SOT3::Identity());
  }

  view_.X = makeVioGroup(A, Beta, B, LandmarkGroup(q));

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

  const auto& [A, Beta, B, Q] = view_.X;
  std::vector<SOT3> q;
  q.reserve(Q.size() - 1);
  for (size_t i = 0; i < Q.size(); ++i) {
    if (static_cast<int>(i) == idx) continue;
    q.push_back(Q[i]);
  }

  view_.X = makeVioGroup(A, Beta, B, LandmarkGroup(q));

  removeRows(view_.Sigma, SensorState::CompDim + 3 * idx, 3);
  removeCols(view_.Sigma, SensorState::CompDim + 3 * idx, 3);

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
  return view_.Sigma.block<3, 3>(SensorState::CompDim + 3 * i,
                                 SensorState::CompDim + 3 * i);
}

Matrix2 EqVIOFilter::outputCovarianceById(
    int id, const Point2&,
    const std::shared_ptr<const CameraModel>& camera) const {
  const Matrix3 lmCov = getLandmarkCovById(id);
  const auto it = std::find_if(
      view_.xi0.cameraLandmarks.begin(), view_.xi0.cameraLandmarks.end(),
      [&id](const Landmark& lm) { return lm.id == id; });
  assert(it != view_.xi0.cameraLandmarks.end());

  const size_t i =
      static_cast<size_t>(std::distance(view_.xi0.cameraLandmarks.begin(), it));
  const auto& Q = gtsam::get<3>(view_.X);
  const SOT3& Q_i = Q[i];

  const Matrix23 C0i = EqFCoordinateSuite_invdepth.outputMatrixCi(it->p, Q_i, camera);
  return C0i * lmCov * C0i.transpose();
}

void EqVIOFilter::removeInvalidLandmarksNow() {
  const auto& Q = gtsam::get<3>(view_.X);
  std::set<int> invalidIds;
  for (size_t i = 0; i < Q.size(); ++i) {
    const double a = SOT3Scale(Q[i]);
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
    const std::shared_ptr<const CameraModel>& camera) {
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
                         const std::shared_ptr<const CameraModel>& camera,
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
