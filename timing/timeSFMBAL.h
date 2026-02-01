/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeSFMBAL.h
 * @brief   Common code for timeSFMBAL scripts
 * @author  Frank Dellaert
 * @date    July 5, 2015
 */

#pragma once

#include <gtsam/base/timing.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/dataset.h>

#include <string>
#include <vector>

using namespace std;
using namespace gtsam;
using symbol_shorthand::C;
using symbol_shorthand::K;
using symbol_shorthand::P;

static bool gUseSchur = true;
static SharedNoiseModel gNoiseModel = noiseModel::Unit::Create(2);

// parse options and read BAL file
SfmData preamble(int argc, char* argv[]) {
  // primitive argument parsing:
  if (argc > 1) {
    if (strcmp(argv[1], "--colamd") == 0) {
      gUseSchur = false;
    } else if (argc > 2) {
      throw runtime_error("Usage: timeSFMBALxxx [--colamd] [BALfile]");
    }
  }

  // Load BAL file
  SfmData db;
  string filename;
  if (argc > 1 && strcmp(argv[argc - 1], "--colamd") != 0)
    filename = argv[argc - 1];
  else
    filename = findExampleDataFile("dubrovnik-16-22106-pre");
  return SfmData::FromBalFile(filename);
}

// Build graph using conventional GeneralSFMFactor
inline NonlinearFactorGraph buildGeneralSfmGraph(
    const SfmData& db, std::optional<double> priorStddev = std::nullopt) {
  using Camera = PinholeCamera<Cal3Bundler>;
  using SfmFactor = GeneralSFMFactor<Camera, Point3>;

  NonlinearFactorGraph graph;
  std::vector<bool> hasCameraMeasurement(db.numberCameras(), false);
  for (size_t j = 0; j < db.numberTracks(); j++) {
    auto& measurements = db.tracks[j].measurements;
    if (measurements.size() < 2) continue;
    for (const SfmMeasurement& measurement : measurements) {
      size_t i = measurement.first;
      Point2 z = measurement.second;
      graph.emplace_shared<SfmFactor>(z, gNoiseModel, C(i), P(j));
      hasCameraMeasurement[i] = true;
    }
  }

  if (priorStddev) {
    /// Add a weak prior on all cameras
    auto priorNoise = noiseModel::Isotropic::Sigma(9, *priorStddev);
    for (size_t i = 0; i < hasCameraMeasurement.size(); ++i) {
      if (!hasCameraMeasurement[i]) continue;
      graph.addPrior<Camera>(C(i), db.cameras[i], priorNoise);
    }

    /// Add a weak prior on all points
    auto pointPriorNoise = noiseModel::Isotropic::Sigma(3, *priorStddev);
    for (size_t j = 0; j < db.numberTracks(); ++j) {
      graph.addPrior<Point3>(P(j), db.tracks[j].p, pointPriorNoise);
    }
  }

  return graph;
}

// Build initial values for conventional GeneralSFMFactor
inline Values buildGeneralSfmInitial(const SfmData& db) {
  Values initial;
  size_t i = 0, j = 0;
  for (const SfmCamera& camera : db.cameras) initial.insert(C(i++), camera);
  for (const SfmTrack& track : db.tracks) initial.insert(P(j++), track.p);
  return initial;
}

inline Ordering createSchurOrdering(const SfmData& db,
                                    bool separateCalibration) {
  Ordering ordering;
  for (size_t j = 0; j < db.numberTracks(); j++) ordering.push_back(P(j));
  for (size_t i = 0; i < db.numberCameras(); i++) {
    ordering.push_back(C(i));
    if (separateCalibration) ordering.push_back(K(i));
  }
  return ordering;
}

inline std::vector<std::pair<string, Ordering>> createOrderings(
    const SfmData& db, const GaussianFactorGraph& linear) {
  return {
      {"Burn", createSchurOrdering(db, false)},
      {"Metis", Ordering::Metis(linear)},
      {"Schur", createSchurOrdering(db, false)},
      {"Colamd", Ordering::Colamd(linear)},
  };
}

// Create ordering and optimize
int optimize(const SfmData& db, const NonlinearFactorGraph& graph,
             const Values& initial, bool separateCalibration = false) {
  using symbol_shorthand::P;

  // Set parameters to be similar to ceres
  LevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  //  params.setLinearSolverType("SEQUENTIAL_CHOLESKY");
  params.setVerbosityLM("SUMMARY");
  params.setRelativeErrorTol(0.01);  // 1% relative error tol

  if (gUseSchur) {
    // Create Schur-complement ordering
    params.setOrdering(createSchurOrdering(db, separateCalibration));
  }

  // Optimize
  {
    gttic_(optimize);
    LevenbergMarquardtOptimizer lm(graph, initial, params);
    Values result = lm.optimize();
  }

  tictoc_finishedIteration_();
  tictoc_print_();

  return 0;
}
