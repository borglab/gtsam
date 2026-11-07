/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VisualFixedLagSmootherExample_SmartFactor.cpp
 * @brief   Smart projection factors in a fixed-lag smoother, on the visual
 *          odometry dataset of StereoVOExample_large.cpp
 */

/**
 * A camera moves forward for 26 frames and tracks landmarks for up to 20
 * frames. Every track is one SmartProjectionPoseFactor on the left image, the
 * visual odometry poses of the dataset provide the odometry, and a fixed-lag
 * smoother with a lag of 5 frames estimates the trajectory. A factor inside the
 * smoother is never modified: a copy with the new measurement replaces it. A
 * track that outlives the window is marginalized with the leaving pose
 * (MARGINALIZE, the default) or continued with that pose held fixed
 * (CONDITION); see gtsam/slam/doc/SmartFactorsFixedLag.md.
 */

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/dataset.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>

using namespace std;
using namespace gtsam;
using symbol_shorthand::X;

typedef SmartProjectionPoseFactor<Cal3_S2> SmartFactor;

/// A landmark track: the factor in the smoother and its slot, -1 if not added.
struct Track {
  SmartFactor::shared_ptr factor;
  long slot = -1;
};

int main(int argc, char* argv[]) {
  // Left camera calibration, visual odometry poses, and left pixel per frame
  double fx, fy, s, u0, v0, b;
  ifstream calibration(findExampleDataFile("VO_calibration.txt"));
  calibration >> fx >> fy >> s >> u0 >> v0 >> b;
  auto K = make_shared<Cal3_S2>(fx, fy, s, u0, v0);

  map<size_t, Pose3> voPoses;
  ifstream poseFile(findExampleDataFile("VO_camera_poses_large.txt"));
  size_t frame;
  MatrixRowMajor m(4, 4);
  while (poseFile >> frame) {
    for (int i = 0; i < 16; i++) poseFile >> m.data()[i];
    voPoses[frame] = Pose3(m);
  }

  // frame -> (landmark, pixel)
  map<size_t, vector<pair<size_t, Point2>>> frames;
  ifstream factorFile(findExampleDataFile("VO_stereo_factors_large.txt"));
  size_t landmark;
  double uL, uR, v, x, y, z;
  while (factorFile >> frame >> landmark >> uL >> uR >> v >> x >> y >> z)
    frames[frame].emplace_back(landmark, Point2(uL, v));
  const size_t first = frames.begin()->first, last = frames.rbegin()->first;

  auto pixelNoise = noiseModel::Isotropic::Sigma(2, 1.0);
  auto odometryNoise =
      noiseModel::Diagonal::Sigmas(Vector6{0.01, 0.01, 0.01, 0.1, 0.1, 0.1});
  auto priorNoise = noiseModel::Diagonal::Sigmas(
      Vector6{0.001, 0.001, 0.001, 0.01, 0.01, 0.01});

  // Prior or odometry factor of frame i, with the initial value of its pose
  auto addFrame = [&](size_t i, const Pose3& previous,
                      NonlinearFactorGraph* graph, Values* values) {
    if (i == first) {
      graph->addPrior(X(i), voPoses[i], priorNoise);
      values->insert(X(i), voPoses[i]);
    } else {
      const Pose3 odometry = voPoses[i - 1].between(voPoses[i]);
      graph->emplace_shared<BetweenFactor<Pose3>>(X(i - 1), X(i), odometry,
                                                  odometryNoise);
      values->insert(X(i), previous * odometry);
    }
  };

  // Reference: the full batch solution over all frames up to upTo
  auto fullBatch = [&](size_t upTo) {
    NonlinearFactorGraph graph;
    Values initial;
    map<size_t, SmartFactor::shared_ptr> factors;
    for (size_t i = first; i <= upTo; ++i) {
      addFrame(i, i > first ? initial.at<Pose3>(X(i - 1)) : Pose3(), &graph,
               &initial);
      for (const auto& [landmark, pixel] : frames[i]) {
        auto& factor = factors[landmark];
        if (!factor) factor = make_shared<SmartFactor>(pixelNoise, K);
        factor->add(pixel, X(i));
      }
    }
    for (const auto& [landmark, factor] : factors)
      if (factor->size() >= 2) graph.push_back(factor);
    return LevenbergMarquardtOptimizer(graph, initial).optimize();
  };
  map<size_t, Point3> batchPositions;
  for (size_t i = first; i <= last; ++i)
    batchPositions[i] = fullBatch(i).at<Pose3>(X(i)).translation();

  ISAM2Params params;
  params.findUnusedFactorSlots = true;  // reuse the slots of removed factors
  for (auto mode :
       {FixedLagSmoother::MARGINALIZE, FixedLagSmoother::CONDITION}) {
    IncrementalFixedLagSmoother smoother(5.0, params);  // lag of 5 frames
    smoother.setMarginalizationMode(mode);
    cout << (mode == FixedLagSmoother::MARGINALIZE ? "MARGINALIZE"
                                                   : "CONDITION")
         << "\nframe  poses  smart factors  longest track  |to batch| (m)\n";
    map<size_t, Track> tracks;

    for (size_t i = first; i <= last; ++i) {
      NonlinearFactorGraph newFactors;
      Values newValues;
      addFrame(
          i, i > first ? smoother.calculateEstimate<Pose3>(X(i - 1)) : Pose3(),
          &newFactors, &newValues);

      // Replace every track's factor by a copy with the new measurement. A
      // track whose factor was consumed by marginalization starts afresh, and
      // so does one seen once whose pose has since left the window.
      FactorIndices toRemove;
      vector<pair<size_t, size_t>> pending;  // landmark, index in newFactors
      for (const auto& [landmark, pixel] : frames[i]) {
        Track& track = tracks[landmark];
        const bool inSmoother =
            track.slot >= 0 &&
            smoother.getFactors()[track.slot] == track.factor;
        if (track.slot >= 0 && !inSmoother) track.factor.reset();
        if (track.factor && track.slot < 0 &&
            !smoother.timestamps().count(track.factor->keys().front()))
          track.factor.reset();
        auto factor = track.factor ? make_shared<SmartFactor>(*track.factor)
                                   : make_shared<SmartFactor>(pixelNoise, K);
        factor->add(pixel, X(i));
        if (inSmoother) toRemove.push_back(track.slot);
        track = {factor, -1};
        if (factor->size() >= 2) {
          pending.emplace_back(landmark, newFactors.size());
          newFactors.push_back(factor);
        }
      }
      const size_t ours = newFactors.size();
      const auto result =
          smoother.update(newFactors, newValues, {{X(i), double(i)}}, toRemove);

      // Slots of our factors, then of their conditioned copies; read the
      // factors back, and a slot without a smart factor means consumed.
      for (const auto& [landmark, index] : pending)
        tracks[landmark].slot = result.newFactorsIndices[index];
      for (size_t k = 0; k < result.conditionedFactorIndices.size(); ++k)
        for (auto& [landmark, track] : tracks)
          if (track.slot == long(result.conditionedFactorIndices[k]))
            track.slot = result.newFactorsIndices[ours + k];
      for (auto& [landmark, track] : tracks) {
        if (track.slot < 0) continue;
        track.factor = dynamic_pointer_cast<SmartFactor>(
            smoother.getFactors()[track.slot]);
        if (!track.factor) track.slot = -1;
      }

      size_t smart = 0, longest = 0;
      for (const auto& factor : smoother.getFactors())
        if (auto smartFactor = dynamic_pointer_cast<SmartFactor>(factor)) {
          ++smart;
          longest = max(longest, smartFactor->measured().size());
        }
      const Point3 position =
          smoother.calculateEstimate<Pose3>(X(i)).translation();
      cout << setw(5) << i << setw(7) << smoother.timestamps().size()
           << setw(15) << smart << setw(15) << longest << fixed
           << setprecision(3) << setw(16)
           << (position - batchPositions[i]).norm() << "\n";
    }
    cout << "\n";
  }
  return 0;
}
