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

#include <gtsam/sfm/SfmData.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/timing.h>

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
  if (argc > 2) {
    if (strcmp(argv[1], "--colamd"))
      gUseSchur = false;
    else
      throw runtime_error("Usage: timeSFMBALxxx [--colamd] [BALfile]");
  }

  // Load BAL file
  SfmData db;
  string filename;
  if (argc > 1)
    filename = argv[argc - 1];
  else
    filename = findExampleDataFile("dubrovnik-16-22106-pre");
  return SfmData::FromBalFile(filename);
}

// Create ordering and optimize
int optimize(const SfmData& db, const NonlinearFactorGraph& graph,
             const Values& initial, bool separateCalibration = false) {
  using symbol_shorthand::P;

  // Set parameters to be similar to ceres
  LevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
//  params.setLinearSolverType("SEQUENTIAL_CHOLESKY");
//  params.setVerbosityLM("SUMMARY");

  if (gUseSchur) {
    // Create Schur-complement ordering
    Ordering ordering;
    for (size_t j = 0; j < db.numberTracks(); j++) ordering.push_back(P(j));
    for (size_t i = 0; i < db.numberCameras(); i++) {
      ordering.push_back(C(i));
      if (separateCalibration) ordering.push_back(K(i));
    }
    params.setOrdering(ordering);
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
