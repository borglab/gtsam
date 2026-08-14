/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SfmPcgBenchmark.h
 * @brief Private end-to-end BAL PCG benchmark interface.
 */

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "SfmBalBenchmark.h"

namespace gtsam::timing::bal {

/** End-to-end statistics from LM using GTSAM's parallel block-Jacobi PCG. */
struct PcgOptimizationResult {
  double elapsedSeconds = 0.0;
  double initialError = 0.0;
  double finalError = 0.0;
  size_t lmIterations = 0;
  size_t lmInnerIterations = 0;
  size_t linearSolves = 0;
  size_t pcgIterations = 0;
  size_t nonConvergedLinearSolves = 0;
  double operatorSetupSeconds = 0.0;
  double preconditionerSetupSeconds = 0.0;
  double solveSeconds = 0.0;
};

/**
 * Optimize a prepared nonlinear graph with parallel block-Jacobi PCG at every
 * LM inner iteration.
 */
PcgOptimizationResult runParallelPcgOptimization(
    const NonlinearFactorGraph& graph, const Values& initial,
    const LevenbergMarquardtParams& parameters);

/**
 * Compare direct, full-system PCG, and reduced-camera PCG BAL optimization.
 */
void runEndToEndPcgComparison(const std::vector<std::string>& filenames,
                              const BalBenchmarkConfig& config);

}  // namespace gtsam::timing::bal
