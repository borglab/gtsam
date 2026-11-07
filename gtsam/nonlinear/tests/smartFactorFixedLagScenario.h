/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    smartFactorFixedLagScenario.h
 * @brief   Shared scenario for smart factors in the fixed-lag smoothers
 * @date    Sep 21, 2026
 */

#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/FixedLagSmoother.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <random>
#include <vector>

namespace gtsam {
namespace smart_factor_fixed_lag {

using symbol_shorthand::X;
typedef SmartProjectionPoseFactor<Cal3_S2> SmartFactor;

/**
 * A camera orbiting a cluster of landmarks, as in GitHub issue #1976, with
 * noisy odometry between consecutive poses, noisy pixel measurements, and
 * landmark tracks of limited length that start and end while the window
 * slides. Frame i has timestamp i.
 */
struct Scenario {
  size_t numFrames = 30;
  double lag = 6.0;
  size_t trackLength = 12;        ///< frames during which a landmark is visible
  size_t trackStartStep = 7;      ///< frames between consecutive track starts
  Point3 center{0.0, 0.0, 20.0};  ///< center of the landmark cluster
  double landmarkSpread = 6.0;    ///< half width of the landmark cluster
  double orbitRadius = 30.0;
  double orbitStep = 10.0 * M_PI / 180.0;  ///< angle between frames
  double minDepth = 0.1;      ///< a landmark closer than this is not seen
  double imageSize = 1000.0;  ///< width and height, in pixels
  Cal3_S2::shared_ptr K = std::make_shared<Cal3_S2>(
      500.0, 500.0, 0.0, imageSize / 2, imageSize / 2);
  SharedNoiseModel pixelNoise = noiseModel::Isotropic::Sigma(2, 1.0);
  SharedNoiseModel odometryNoise = noiseModel::Diagonal::Sigmas(
      Vector6{0.005, 0.005, 0.005, 0.03, 0.03, 0.03});
  SharedNoiseModel priorNoise = noiseModel::Isotropic::Sigma(6, 1e-3);
  /// Sigma of an absolute pose measurement of every frame, which anchors the
  /// trajectory; zero for none.
  double absolutePoseSigma = 0.0;

  std::vector<Pose3> groundTruth;
  std::vector<Point3> landmarks;
  /// measurements[i][j] is the pixel of landmark j in frame i, if seen.
  std::vector<std::vector<std::optional<Point2>>> measurements;
  std::vector<Pose3> odometry;       ///< odometry[i] measures x_{i-1}^{-1} x_i
  std::vector<Pose3> deadReckoning;  ///< initial values from odometry alone
  std::vector<Vector6> absolutePoseNoise;  ///< unit noise of absolute poses

  explicit Scenario(size_t numLandmarks = 24, unsigned seed = 42) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> gauss(0.0, 1.0);
    std::uniform_real_distribution<double> uniform(-1.0, 1.0);
    for (size_t i = 0; i < numFrames; ++i) groundTruth.push_back(orbitPose(i));
    for (size_t j = 0; j < numLandmarks; ++j) {
      const double x = uniform(rng), y = uniform(rng), z = uniform(rng);
      landmarks.push_back(center + landmarkSpread * Point3(x, y, z));
    }

    measurements.resize(numFrames);
    for (size_t i = 0; i < numFrames; ++i) {
      const PinholePose<Cal3_S2> camera(groundTruth[i], K);
      for (size_t j = 0; j < numLandmarks; ++j) {
        std::optional<Point2> measurement;
        const size_t start = (trackStartStep * j) % numFrames;
        if (i >= start && i < start + trackLength &&
            groundTruth[i].transformTo(landmarks[j]).z() > minDepth) {
          const double u = gauss(rng), v = gauss(rng);
          const Point2 pixel = camera.project(landmarks[j]) + Point2(u, v);
          if (pixel.x() > 0 && pixel.x() < imageSize && pixel.y() > 0 &&
              pixel.y() < imageSize)
            measurement = pixel;
        }
        measurements[i].push_back(measurement);
      }
    }

    const Vector6 sigmas = odometryNoise->sigmas();
    odometry.resize(numFrames);
    deadReckoning.resize(numFrames);
    deadReckoning[0] = groundTruth[0];
    for (size_t i = 1; i < numFrames; ++i) {
      Vector6 noise;
      for (int k = 0; k < 6; ++k) noise(k) = sigmas(k) * gauss(rng);
      odometry[i] =
          groundTruth[i - 1].between(groundTruth[i]) * Pose3::Expmap(noise);
      deadReckoning[i] = deadReckoning[i - 1] * odometry[i];
    }
    absolutePoseNoise.resize(numFrames);
    for (Vector6& noise : absolutePoseNoise)
      for (int k = 0; k < 6; ++k) noise(k) = gauss(rng);
  }

  /// Pose of frame i on the orbit around the center, looking at it.
  Pose3 orbitPose(size_t i) const {
    const double angle = orbitStep * i;
    const Point3 position(center.x() + orbitRadius * std::cos(angle),
                          center.y() + orbitRadius * std::sin(angle), 0.0);
    return PinholeBase::LookatPose(position, center, Vector3::UnitZ());
  }

  /**
   * Prior or odometry factor, absolute pose measurement if enabled, and the
   * initial value of frame i: previousEstimate composed with the odometry
   * step if given, else dead reckoning.
   */
  void frameFactors(size_t i, NonlinearFactorGraph* graph, Values* values,
                    const Pose3* previousEstimate = nullptr) const {
    if (i == 0)
      graph->addPrior(X(0), groundTruth[0], priorNoise);
    else
      graph->emplace_shared<BetweenFactor<Pose3>>(X(i - 1), X(i), odometry[i],
                                                  odometryNoise);
    if (absolutePoseSigma > 0.0)
      graph->addPrior(
          X(i),
          groundTruth[i].retract(absolutePoseSigma * absolutePoseNoise[i]),
          noiseModel::Isotropic::Sigma(6, absolutePoseSigma));
    values->insert(X(i), previousEstimate ? *previousEstimate * odometry[i]
                                          : deadReckoning[i]);
  }

  /// Full batch solution using all measurements up to and including frame upTo.
  Values fullBatch(size_t upTo, const SmartProjectionParams& params =
                                    SmartProjectionParams()) const {
    NonlinearFactorGraph graph;
    Values initial;
    for (size_t i = 0; i <= upTo; ++i) frameFactors(i, &graph, &initial);
    for (size_t j = 0; j < landmarks.size(); ++j) {
      auto factor = std::make_shared<SmartFactor>(pixelNoise, K, params);
      for (size_t i = 0; i <= upTo; ++i)
        if (measurements[i][j]) factor->add(*measurements[i][j], X(i));
      if (factor->size() >= 2) graph.push_back(factor);
    }
    return LevenbergMarquardtOptimizer(graph, initial).optimize();
  }
};

/// One landmark track as seen by the caller of a fixed-lag smoother.
struct Track {
  SmartFactor::shared_ptr factor;  ///< factor believed to be in the smoother
  long slot = -1;                  ///< its slot, or -1 when not in the smoother
  size_t restarts = 0;  ///< times the track was restarted after being consumed
  size_t longestFactor = 0;  ///< most measurements one factor ever held
};

/// Per-frame bookkeeping returned by the driver.
struct FrameStatistics {
  size_t smartFactors = 0;
  size_t linearContainerFactors = 0;
  size_t largestLinearContainer = 0;  ///< most keys in one marginal factor
  size_t conditionedFactors = 0;
  size_t consumedTracks = 0;
  double translationErrorToFullBatch = 0.0;  ///< newest pose, meters
  double translationErrorToTruth = 0.0;      ///< newest pose, meters
};

/// Options of the replace-protocol driver.
struct ProtocolOptions {
  SmartProjectionParams params;
  bool compareToFullBatch = true;
  /// Add to the factor in the smoother instead of replacing it by a copy;
  /// only the batch smoother supports this.
  bool growInPlace = false;
  /// Do by hand what MarginalizationMode::CONDITION does inside the smoother.
  bool conditionByCaller = false;
  /// With conditionByCaller, fix at these values instead of the estimate.
  const Values* fixedValuesOverride = nullptr;
};

/// Options that skip the comparison with the full batch solution.
inline ProtocolOptions noBatchComparison() {
  ProtocolOptions options;
  options.compareToFullBatch = false;
  return options;
}

/// Keys the smoother will marginalize in the update of the given frame.
template <class SMOOTHER>
KeyVector keysLeavingWindow(const SMOOTHER& smoother, size_t frame,
                            double lag) {
  KeyVector leaving;
  for (const auto& [key, timestamp] : smoother.timestamps())
    if (timestamp < double(frame) - lag) leaving.push_back(key);
  return leaving;
}

/**
 * Fix the leaving keys of a factor at fixedValues, like
 * NonlinearFactor::conditionOn: a factor whose keys would all be fixed is
 * returned unchanged and left to marginalization.
 */
inline SmartFactor::shared_ptr fixLeavingKeys(
    const SmartFactor::shared_ptr& factor, const KeyVector& leavingKeys,
    const Values& fixedValues) {
  KeyVector keysToFix;
  for (Key key : factor->keys())
    if (std::find(leavingKeys.begin(), leavingKeys.end(), key) !=
        leavingKeys.end())
      keysToFix.push_back(key);
  if (keysToFix.empty() || keysToFix.size() == factor->keys().size())
    return factor;
  SmartFactor::shared_ptr fixed = factor;
  for (Key key : keysToFix) fixed = fixed->fixCamera(key, fixedValues);
  return fixed;
}

/// Whether every key of a factor still has a timestamp in the smoother.
template <class SMOOTHER>
bool allKeysInWindow(const NonlinearFactor& factor, const SMOOTHER& smoother) {
  for (Key key : factor.keys())
    if (!smoother.timestamps().count(key)) return false;
  return true;
}

/// Whether the track's factor is the one in the smoother at its slot.
inline bool trackInGraph(const Track& track,
                         const NonlinearFactorGraph& graph) {
  return track.slot >= 0 && size_t(track.slot) < graph.size() &&
         graph[track.slot] == track.factor;
}

/// Restart a track whose factor was consumed by marginalization.
inline void restartTrack(Track* track, size_t* consumed) {
  track->factor.reset();
  track->slot = -1;
  ++track->restarts;
  ++*consumed;
}

/**
 * After an update, follow conditioned replacements, map each pending track to
 * its slot, and refresh the factor pointers. A track whose slot no longer
 * holds a smart factor was consumed by marginalization and restarts.
 */
inline void refreshTracks(const NonlinearFactorGraph& graph,
                          const FixedLagSmoother::Result& result,
                          size_t callerFactors,
                          const std::vector<std::pair<size_t, size_t>>& pending,
                          std::vector<Track>* tracks, size_t* consumed) {
  std::map<long, long> replacements;
  for (size_t k = 0; k < result.conditionedFactorIndices.size(); ++k)
    replacements[long(result.conditionedFactorIndices[k])] =
        long(result.newFactorsIndices.at(callerFactors + k));
  for (Track& track : *tracks) {
    const auto replacement = replacements.find(track.slot);
    if (replacement != replacements.end()) track.slot = replacement->second;
  }
  for (const auto& [j, index] : pending)
    (*tracks)[j].slot = long(result.newFactorsIndices.at(index));
  for (Track& track : *tracks) {
    if (track.slot < 0) continue;
    track.factor = std::dynamic_pointer_cast<SmartFactor>(graph[track.slot]);
    if (!track.factor)
      restartTrack(&track, consumed);
    else
      track.longestFactor =
          std::max(track.longestFactor, track.factor->measured().size());
  }
}

/// Count factor types in the smoother and measure the newest pose.
template <class SMOOTHER>
FrameStatistics frameStatistics(const SMOOTHER& smoother,
                                const Scenario& scenario, size_t frame,
                                const ProtocolOptions& options) {
  FrameStatistics statistics;
  for (const auto& factor : smoother.getFactors()) {
    if (!factor) continue;
    if (dynamic_cast<const SmartFactor*>(factor.get()))
      ++statistics.smartFactors;
    if (dynamic_cast<const LinearContainerFactor*>(factor.get())) {
      ++statistics.linearContainerFactors;
      statistics.largestLinearContainer =
          std::max(statistics.largestLinearContainer, factor->size());
    }
  }
  const Pose3 estimate =
      smoother.calculateEstimate().template at<Pose3>(X(frame));
  statistics.translationErrorToTruth =
      (estimate.translation() - scenario.groundTruth[frame].translation())
          .norm();
  if (options.compareToFullBatch) {
    const Pose3 reference =
        scenario.fullBatch(frame, options.params).at<Pose3>(X(frame));
    statistics.translationErrorToFullBatch =
        (estimate.translation() - reference.translation()).norm();
  }
  return statistics;
}

/**
 * Replace protocol for frame i: every landmark seen now gets a new factor with
 * all its measurements, which replaces its previous factor; with growInPlace
 * the factor in the smoother gets the measurement instead. With
 * conditionByCaller, the factors of keys leaving the window are fixed first.
 * Appends to newFactors and toRemove, and returns (landmark, index in
 * newFactors) for every factor added.
 */
template <class SMOOTHER>
std::vector<std::pair<size_t, size_t>> replaceTrackFactors(
    const SMOOTHER& smoother, const Scenario& scenario, size_t i,
    const ProtocolOptions& options, std::vector<Track>* tracks,
    NonlinearFactorGraph* newFactors, FactorIndices* toRemove,
    size_t* consumed) {
  KeyVector leavingKeys;
  Values fixedValues;
  if (options.conditionByCaller) {
    leavingKeys = keysLeavingWindow(smoother, i, scenario.lag);
    if (!leavingKeys.empty())
      fixedValues = options.fixedValuesOverride
                        ? options.fixedValuesOverride->extract(leavingKeys)
                        : smoother.calculateEstimate(leavingKeys);
  }

  const NonlinearFactorGraph& graph = smoother.getFactors();
  std::vector<std::pair<size_t, size_t>> pending;
  for (size_t j = 0; j < scenario.landmarks.size(); ++j) {
    Track& track = (*tracks)[j];
    const bool inGraph = trackInGraph(track, graph);
    SmartFactor::shared_ptr factor;
    if (scenario.measurements[i][j]) {
      if (track.factor && track.slot >= 0 && !inGraph)
        restartTrack(&track, consumed);
      // A factor not yet in the smoother may hold a measurement of a pose
      // that has since left the window; it cannot be reused.
      if (track.factor && track.slot < 0 &&
          !allKeysInWindow(*track.factor, smoother))
        restartTrack(&track, consumed);
      if (options.growInPlace && track.factor) {
        track.factor->add(*scenario.measurements[i][j], X(i));
        if (inGraph) continue;
        factor = track.factor;
      } else {
        factor = std::make_shared<SmartFactor>(scenario.pixelNoise, scenario.K,
                                               options.params);
        if (track.factor) *factor = *track.factor;  // keeps fixed cameras
        factor->add(*scenario.measurements[i][j], X(i));
      }
      if (options.conditionByCaller)
        factor = fixLeavingKeys(factor, leavingKeys, fixedValues);
    } else if (options.conditionByCaller && inGraph) {
      // Not observed now, but a factor of a leaving key still needs fixing.
      factor = fixLeavingKeys(track.factor, leavingKeys, fixedValues);
      if (factor == track.factor) continue;
    } else {
      continue;
    }
    if (inGraph) toRemove->push_back(size_t(track.slot));
    track.factor = factor;
    track.slot = -1;
    // A single measurement does not make a factor yet.
    if (factor->measured().size() >= 2) {
      pending.emplace_back(j, newFactors->size());
      newFactors->push_back(factor);
    }
  }
  return pending;
}

/**
 * Drive a fixed-lag smoother through the scenario with the replace protocol:
 * a new factor with all measurements replaces a landmark's factor, or grows
 * it in place with growInPlace. A track whose factor was consumed by
 * marginalization restarts.
 */
template <class SMOOTHER>
std::vector<FrameStatistics> runReplaceProtocol(
    SMOOTHER& smoother, const Scenario& scenario,
    std::vector<Track>* tracks = nullptr,
    const ProtocolOptions& options = ProtocolOptions()) {
  std::vector<Track> localTracks;
  if (!tracks) tracks = &localTracks;
  tracks->assign(scenario.landmarks.size(), Track());
  std::vector<FrameStatistics> statistics;

  for (size_t i = 0; i < scenario.numFrames; ++i) {
    NonlinearFactorGraph newFactors;
    Values newValues;
    const Pose3 previousEstimate =
        i > 0 ? smoother.calculateEstimate().template at<Pose3>(X(i - 1))
              : Pose3();
    scenario.frameFactors(i, &newFactors, &newValues,
                          i > 0 ? &previousEstimate : nullptr);

    FactorIndices toRemove;
    size_t consumed = 0;
    const auto pending =
        replaceTrackFactors(smoother, scenario, i, options, tracks, &newFactors,
                            &toRemove, &consumed);
    const size_t callerFactors = newFactors.size();
    const FixedLagSmoother::Result result =
        smoother.update(newFactors, newValues, {{X(i), double(i)}}, toRemove);
    refreshTracks(smoother.getFactors(), result, callerFactors, pending, tracks,
                  &consumed);

    FrameStatistics frame = frameStatistics(smoother, scenario, i, options);
    frame.conditionedFactors = result.conditionedFactorIndices.size();
    frame.consumedTracks = consumed;
    statistics.push_back(frame);
  }
  return statistics;
}

/**
 * Between factor on two Pose2 that supports conditioning on its first pose,
 * which leaves an equivalent prior on the second. Tests the smoothers without
 * smart factors.
 */
class ConditionableBetween : public BetweenFactor<Pose2> {
 public:
  using BetweenFactor<Pose2>::BetweenFactor;

  NonlinearFactor::shared_ptr conditionOn(
      const Values& fixedValues) const override {
    if (!fixedValues.exists(key1()) || fixedValues.exists(key2()))
      return nullptr;
    return std::make_shared<PriorFactor<Pose2>>(
        key2(), fixedValues.at<Pose2>(key1()) * measured(), noiseModel());
  }
};

}  // namespace smart_factor_fixed_lag
}  // namespace gtsam
