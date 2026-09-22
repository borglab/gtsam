/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file CertifiableLocalizationUtils.h
 * @brief Shared data preparation for certifiable localization examples.
 * @author Avinash Subramanian
 * @author Frederike Dümbgen
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/FrobeniusFactor.h>
#include <gtsam/slam/KnownLandmarkFactor.h>
#include <gtsam/slam/dataset.h>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

namespace gtsam::examples {

using BearingRangeFactor3D = BearingRangeFactor<Pose3, Point3>;

/** A Cartesian point observation and covariance expressed in frame k. */
struct CartesianLandmarkMeasurement {
  Point3 measured_kP;
  Matrix3 kPCovariance;
};

/**
 * Convert a bearing-range factor measurement to Cartesian frame-k form.
 *
 * For kP = kRange * kBearing, the measurement-coordinate Jacobian is
 * DkP_dbearingRange = [kRange * B, u]. The Cartesian covariance is therefore
 * DkP_dbearingRange * bearingRangeCovariance * DkP_dbearingRange.transpose().
 * This conversion is exact at the measurement linearization point; the two
 * nonlinear Gaussian models are not globally identical.
 */
inline CartesianLandmarkMeasurement bearingRangeToCartesian(
    const BearingRangeFactor3D& factor) {
  const auto bearingRangeGaussian =
      std::dynamic_pointer_cast<noiseModel::Gaussian>(factor.noiseModel());
  if (!bearingRangeGaussian) {
    throw std::invalid_argument(
        "bearingRangeToCartesian requires a Gaussian noise model");
  }

  const auto& measured_kBearingRange = factor.measured();
  const Unit3& kBearing = measured_kBearingRange.bearing();
  const double kRange = measured_kBearingRange.range();
  const Point3 measured_kP = kRange * kBearing;

  Matrix3 DkP_dbearingRange;
  DkP_dbearingRange.leftCols<2>() = kRange * kBearing.basis();
  DkP_dbearingRange.rightCols<1>() = kBearing.unitVector();
  const Matrix3 kPCovariance = DkP_dbearingRange *
                               bearingRangeGaussian->covariance() *
                               DkP_dbearingRange.transpose();
  return {measured_kP, kPCovariance};
}

/** Common graph and kTw values used by the certifiable examples. */
struct CertifiableLocalizationProblem {
  NonlinearFactorGraph graph;
  Values groundTruth_kTws;
  Values initial_kTws;
  KeyVector poseKeys;
};

/** Read conventional wTk/wL data and construct the certifiable problem. */
inline CertifiableLocalizationProblem loadCertifiableLocalization(
    const std::string& filename) {
  const auto [g2oGraph, wTkAndwLValues] = readG2o(filename, true);

  CertifiableLocalizationProblem problem;
  for (const auto& sourceFactor : *g2oGraph) {
    const auto bearingRangeFactor =
        std::dynamic_pointer_cast<BearingRangeFactor3D>(sourceFactor);
    if (bearingRangeFactor) {
      const Key k = bearingRangeFactor->key1();
      const Key l = bearingRangeFactor->key2();
      const Point3& wL = wTkAndwLValues->at<Point3>(l);
      const auto [measured_kP, kPCovariance] =
          bearingRangeToCartesian(*bearingRangeFactor);
      problem.graph.emplace_shared<KnownLandmarkFactor2<Pose3>>(
          k, wL, measured_kP, noiseModel::Gaussian::Covariance(kPCovariance));
      continue;
    }

    const auto betweenFactor =
        std::dynamic_pointer_cast<BetweenFactor<Pose3>>(sourceFactor);
    if (betweenFactor) {
      const Key i = betweenFactor->key1();
      const Key j = betweenFactor->key2();
      const Pose3& measured_iTj = betweenFactor->measured();
      problem.graph.emplace_shared<FrobeniusLeftBetweenFactor<Pose3>>(
          i, j, measured_iTj, betweenFactor->noiseModel());
      continue;
    }

    throw std::invalid_argument(
        "loadCertifiableLocalization encountered an unexpected factor");
  }

  const Vector6 perturbation{0.02, -0.015, 0.01, 0.08, -0.05, 0.06};
  const auto wTks = wTkAndwLValues->extract<Pose3>();
  problem.poseKeys.reserve(wTks.size());
  size_t index = 0;
  for (const auto& [k, wTk] : wTks) {
    const Pose3 kTw = wTk.inverse();
    problem.poseKeys.push_back(k);
    problem.groundTruth_kTws.insert(k, kTw);
    problem.initial_kTws.insert(
        k, kTw.retract(static_cast<double>(index + 1) * perturbation));
    ++index;
  }

  return problem;
}

/** Mean and maximum pose errors for a collection of kTw states. */
struct PoseErrorStatistics {
  double meanPoseError;
  double maximumPoseError;
};

/** Compute pose errors between ordered expected and actual kTw poses. */
inline PoseErrorStatistics poseErrorStatistics(
    const std::vector<Pose3>& expected_kTws,
    const std::vector<Pose3>& actual_kTws) {
  if (expected_kTws.empty() || expected_kTws.size() != actual_kTws.size()) {
    throw std::invalid_argument(
        "poseErrorStatistics requires equally sized, nonempty kTw vectors");
  }
  double totalPoseError = 0.0;
  double maximumPoseError = 0.0;
  for (size_t index = 0; index < expected_kTws.size(); ++index) {
    const double poseError =
        expected_kTws[index].localCoordinates(actual_kTws[index]).norm();
    totalPoseError += poseError;
    maximumPoseError = std::max(maximumPoseError, poseError);
  }
  return {totalPoseError / static_cast<double>(expected_kTws.size()),
          maximumPoseError};
}

/** Compute pose errors between expected and actual kTw values. */
inline PoseErrorStatistics poseErrorStatistics(const Values& expected_kTws,
                                               const Values& actual_kTws,
                                               const KeyVector& poseKeys) {
  std::vector<Pose3> orderedExpected_kTws, orderedActual_kTws;
  orderedExpected_kTws.reserve(poseKeys.size());
  orderedActual_kTws.reserve(poseKeys.size());
  for (const Key k : poseKeys) {
    orderedExpected_kTws.push_back(expected_kTws.at<Pose3>(k));
    orderedActual_kTws.push_back(actual_kTws.at<Pose3>(k));
  }
  return poseErrorStatistics(orderedExpected_kTws, orderedActual_kTws);
}

}  // namespace gtsam::examples
