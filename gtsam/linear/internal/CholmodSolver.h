/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file CholmodSolver.h
 * @brief Optional reusable CHOLMOD solver for Gaussian factor graphs.
 */

#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

#include <memory>

namespace gtsam::internal {

class GTSAM_EXPORT CholmodSolver {
 public:
  CholmodSolver();
  ~CholmodSolver();

  CholmodSolver(const CholmodSolver&) = delete;
  CholmodSolver& operator=(const CholmodSolver&) = delete;

  static bool available();
  VectorValues solve(const GaussianFactorGraph& graph,
                     const Ordering& ordering);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::internal
