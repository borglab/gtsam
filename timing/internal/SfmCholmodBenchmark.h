/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SfmCholmodBenchmark.h
 * @brief Optional CHOLMOD backend for compact point-batch BAL Schur systems.
 */

#pragma once

#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <cstddef>

namespace gtsam::timing::bal {

struct SparseSchurOptimizationResult {
  double elapsedSeconds = 0.0;
  double initialError = 0.0;
  double finalError = 0.0;
  size_t lmIterations = 0;
  size_t lmInnerIterations = 0;
  size_t linearSolves = 0;
  double assemblySeconds = 0.0;
  double factorAndSolveSeconds = 0.0;
  double backSubstituteSeconds = 0.0;
};

/** Return whether this timing build includes the optional CHOLMOD backend. */
bool cholmodBackendAvailable();

/** Optimize explicit point batches using the optional CHOLMOD backend. */
SparseSchurOptimizationResult runPointBatchSchurCholmodOptimization(
    const NonlinearFactorGraph& graph, const Values& initial,
    const LevenbergMarquardtParams& parameters);

}  // namespace gtsam::timing::bal
