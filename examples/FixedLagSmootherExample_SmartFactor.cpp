/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file FixedLagSmootherExample_SmartFactor.cpp
 * @brief Smart projection factors in the fixed-lag smoothers, with landmark
 * tracks that outlive the lag window.
 *
 * A vehicle drives a loop with a forward-looking camera among random
 * landmarks, each measured when in view unless the detector drops it, so
 * tracks start, pause, and end at random. Each landmark is one
 * SmartProjectionPoseFactor. A factor inside the smoother is never modified: a
 * copy with the new measurement replaces it. A track that outlives the window
 * is marginalized with the leaving pose (MARGINALIZE, the default) or continued
 * with that pose held fixed (CONDITION). Both modes run on both smoothers
 * against the full batch solution; see gtsam/slam/doc/SmartFactorsFixedLag.md.
 * The optional argument is the random seed.
 */

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <vector>

using namespace gtsam;
using symbol_shorthand::X;
typedef SmartProjectionPoseFactor<Cal3_S2> SmartFactor;

/* ************************************************************************* */
/// Trajectory, landmarks, and the measurements they produce.
struct Simulation {
  size_t numFrames = 60;
  double frameInterval = 0.5;  ///< seconds
  size_t numLandmarks = 120;
  double dropout = 0.3;  ///< probability that the detector misses a landmark
  Cal3_S2::shared_ptr K =
      std::make_shared<Cal3_S2>(450.0, 450.0, 0.0, 320.0, 240.0);
  Pose3 body_P_camera{Rot3::Ypr(-M_PI / 2, 0.0, -M_PI / 2),
                      Point3(0.2, 0.0, 0.5)};  ///< looks along the body x axis
  SharedNoiseModel pixelNoise = noiseModel::Isotropic::Sigma(2, 1.0);
  SharedNoiseModel odometryNoise = noiseModel::Diagonal::Sigmas(
      Vector6{0.004, 0.004, 0.004, 0.02, 0.02, 0.02});
  SharedNoiseModel priorNoise = noiseModel::Isotropic::Sigma(6, 1e-3);

  std::vector<Pose3> truth, odometry;
  std::vector<Point3> landmarks;
  /// Per frame: landmark -> pixel
  std::vector<std::map<size_t, Point2>> measurements;

  explicit Simulation(unsigned seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> gauss(0.0, 1.0);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    // A loop of radius 25 m on the ground, heading along the tangent
    for (size_t i = 0; i < numFrames; ++i) {
      const double angle = 2 * M_PI * i / numFrames;
      truth.emplace_back(
          Rot3::Yaw(angle + M_PI / 2),
          Point3(25.0 * std::cos(angle), 25.0 * std::sin(angle), 0.0));
    }
    // Landmarks 20 to 40 m from the center, 1 to 4 m above the ground
    for (size_t j = 0; j < numLandmarks; ++j) {
      const double angle = 2 * M_PI * uniform(rng),
                   radius = 20.0 + 20.0 * uniform(rng);
      landmarks.emplace_back(radius * std::cos(angle), radius * std::sin(angle),
                             1.0 + 3.0 * uniform(rng));
    }
    // Seen when in front of the camera within 40 m, detected, and in the image
    measurements.resize(numFrames);
    for (size_t i = 0; i < numFrames; ++i) {
      const PinholePose<Cal3_S2> camera(truth[i] * body_P_camera, K);
      for (size_t j = 0; j < numLandmarks; ++j) {
        const Point3 local = camera.pose().transformTo(landmarks[j]);
        if (local.z() < 2.0 || local.norm() > 40.0 || uniform(rng) < dropout)
          continue;
        const double u = gauss(rng), v = gauss(rng);
        const Point2 pixel = camera.project(landmarks[j]) + Point2(u, v);
        if (pixel.x() >= 0 && pixel.x() <= 640 && pixel.y() >= 0 &&
            pixel.y() <= 480)
          measurements[i][j] = pixel;
      }
    }
    // Odometry with noise
    const Vector6 sigmas = odometryNoise->sigmas();
    odometry.resize(numFrames);
    for (size_t i = 1; i < numFrames; ++i) {
      Vector6 noise;
      for (int k = 0; k < 6; ++k) noise(k) = sigmas(k) * gauss(rng);
      odometry[i] = truth[i - 1].between(truth[i]) * Pose3::Expmap(noise);
    }
  }

  /// Prior or odometry factor of frame i and the initial value of its pose
  void frameFactors(size_t i, const Pose3& previous,
                    NonlinearFactorGraph* graph, Values* values) const {
    if (i == 0) {
      graph->addPrior(X(0), truth[0], priorNoise);
      values->insert(X(0), truth[0]);
    } else {
      graph->emplace_shared<BetweenFactor<Pose3>>(X(i - 1), X(i), odometry[i],
                                                  odometryNoise);
      values->insert(X(i), previous * odometry[i]);
    }
  }

  /// Full batch solution with all measurements up to frame upTo
  Values fullBatch(size_t upTo) const {
    NonlinearFactorGraph graph;
    Values initial;
    std::map<size_t, SmartFactor::shared_ptr> factors;
    for (size_t i = 0; i <= upTo; ++i) {
      frameFactors(i, i > 0 ? initial.at<Pose3>(X(i - 1)) : Pose3(), &graph,
                   &initial);
      for (const auto& [j, pixel] : measurements[i]) {
        auto& factor = factors[j];
        if (!factor)
          factor = std::make_shared<SmartFactor>(pixelNoise, K, body_P_camera);
        factor->add(pixel, X(i));
      }
    }
    for (const auto& [j, factor] : factors)
      if (factor->size() >= 2) graph.push_back(factor);
    return LevenbergMarquardtOptimizer(graph, initial).optimize();
  }
};

/* ************************************************************************* */
/// A landmark track: the factor in the smoother and its slot, -1 if not added.
struct Track {
  SmartFactor::shared_ptr factor;
  long slot = -1;
};

struct Summary {
  double meanToBatch = 0.0, maxToBatch = 0.0, finalToTruth = 0.0;
  size_t restarts = 0, longestTrack = 0;
};

/// Run one smoother through the simulation with the replace protocol and
/// compare the newest pose with the full batch solution every fifth frame.
template <class SMOOTHER>
Summary run(SMOOTHER& smoother, const Simulation& sim) {
  std::map<size_t, Track> tracks;
  Summary summary;
  for (size_t i = 0; i < sim.numFrames; ++i) {
    NonlinearFactorGraph newFactors;
    Values newValues;
    sim.frameFactors(
        i,
        i > 0 ? smoother.template calculateEstimate<Pose3>(X(i - 1)) : Pose3(),
        &newFactors, &newValues);

    // Replace every track's factor by a copy with the new measurement. A track
    // restarts when its factor was consumed by marginalization, or when it was
    // seen once and that pose has since left the window.
    FactorIndices toRemove;
    // landmark, index in newFactors
    std::vector<std::pair<size_t, size_t>> pending;
    for (const auto& [j, pixel] : sim.measurements[i]) {
      Track& track = tracks[j];
      const bool inSmoother =
          track.slot >= 0 && smoother.getFactors()[track.slot] == track.factor;
      if (track.factor && track.slot < 0 &&
          !smoother.timestamps().count(track.factor->keys().front())) {
        track.factor.reset();
        ++summary.restarts;
      }
      auto factor = track.factor
                        ? std::make_shared<SmartFactor>(*track.factor)
                        : std::make_shared<SmartFactor>(sim.pixelNoise, sim.K,
                                                        sim.body_P_camera);
      factor->add(pixel, X(i));
      if (inSmoother) toRemove.push_back(track.slot);
      track = {factor, -1};
      if (factor->size() >= 2) {
        pending.emplace_back(j, newFactors.size());
        newFactors.push_back(factor);
      }
    }
    const size_t ours = newFactors.size();
    const auto result = smoother.update(
        newFactors, newValues, {{X(i), i * sim.frameInterval}}, toRemove);

    // Slots of our factors, then of their conditioned copies; read the factors
    // back, and a slot without a smart factor means consumed.
    for (const auto& [j, index] : pending)
      tracks[j].slot = result.newFactorsIndices[index];
    for (size_t k = 0; k < result.conditionedFactorIndices.size(); ++k)
      for (auto& [j, track] : tracks)
        if (track.slot == long(result.conditionedFactorIndices[k]))
          track.slot = result.newFactorsIndices[ours + k];
    for (auto& [j, track] : tracks) {
      if (track.slot < 0) continue;
      track.factor = std::dynamic_pointer_cast<SmartFactor>(
          smoother.getFactors()[track.slot]);
      if (track.factor) {
        summary.longestTrack =
            std::max(summary.longestTrack, track.factor->measured().size());
      } else {
        track.slot = -1;
        ++summary.restarts;
      }
    }

    if ((i + 1) % 5 == 0) {
      const Point3 estimate =
          smoother.template calculateEstimate<Pose3>(X(i)).translation();
      const double toBatch =
          (estimate - sim.fullBatch(i).at<Pose3>(X(i)).translation()).norm();
      summary.meanToBatch += toBatch / (sim.numFrames / 5);
      summary.maxToBatch = std::max(summary.maxToBatch, toBatch);
      summary.finalToTruth = (estimate - sim.truth[i].translation()).norm();
    }
  }
  return summary;
}

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  const Simulation sim(argc > 1 ? unsigned(std::atoi(argv[1])) : 7u);
  size_t total = 0;
  for (const auto& frame : sim.measurements) total += frame.size();
  const Point3 finalTruth = sim.truth.back().translation();
  const Point3 finalBatch = sim.fullBatch(sim.numFrames - 1)
                                .at<Pose3>(X(sim.numFrames - 1))
                                .translation();
  std::cout << std::fixed << std::setprecision(3)
            << "Simulation: " << sim.numFrames << " frames, "
            << sim.numLandmarks << " landmarks, " << total
            << " measurements\nFull batch final error to truth: "
            << (finalBatch - finalTruth).norm() << " m\n\n";

  const double lag = 8 * sim.frameInterval;  // eight frames
  ISAM2Params isamParams;
  isamParams.findUnusedFactorSlots = true;
  isamParams.relinearizeSkip = 1;
  isamParams.relinearizeThreshold = 0.01;

  std::cout << std::left << std::setw(30) << "smoother / mode" << std::setw(16)
            << "mean|fl-batch|" << std::setw(15) << "max|fl-batch|"
            << std::setw(14) << "final|fl-gt|" << std::setw(10) << "restarts"
            << "longest track\n";
  const auto report = [](const std::string& name, const Summary& summary) {
    std::cout << std::setw(30) << name << std::setw(16) << summary.meanToBatch
              << std::setw(15) << summary.maxToBatch << std::setw(14)
              << summary.finalToTruth << std::setw(10) << summary.restarts
              << summary.longestTrack << "\n";
  };
  for (auto mode :
       {FixedLagSmoother::MARGINALIZE, FixedLagSmoother::CONDITION}) {
    const std::string modeName =
        mode == FixedLagSmoother::MARGINALIZE ? "MARGINALIZE" : "CONDITION";
    IncrementalFixedLagSmoother incremental(lag, isamParams);
    incremental.setMarginalizationMode(mode);
    report("Incremental / " + modeName, run(incremental, sim));
    BatchFixedLagSmoother batch(lag);
    batch.setMarginalizationMode(mode);
    report("Batch / " + modeName, run(batch, sim));
  }
  return 0;
}
/* ************************************************************************* */
