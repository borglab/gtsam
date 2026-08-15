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

#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <cstddef>
#include <memory>
#include <vector>

namespace gtsam::timing::bal {

struct CompactCameraSystem;

/** Reusable optional CHOLMOD solver for a compact reduced-camera system. */
class CholmodCameraSystemSolver {
  class Impl;
  std::unique_ptr<Impl> impl_;

 public:
  CholmodCameraSystemSolver();
  ~CholmodCameraSystemSolver();

  CholmodCameraSystemSolver(const CholmodCameraSystemSolver&) = delete;
  CholmodCameraSystemSolver& operator=(const CholmodCameraSystemSolver&) =
      delete;

  /** Factor and solve the reduced camera system, reusing symbolic analysis. */
  Vector solve(const CompactCameraSystem& system,
               const std::vector<size_t>& cameraPermutation = {});
};

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
