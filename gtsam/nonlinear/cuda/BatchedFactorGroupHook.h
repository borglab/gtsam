/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BatchedFactorGroupHook.h
 * @brief   Deferred cross-item batching hook for Python-backed factor
 *          groups shared across a Batched* optimizer's items
 * @author  Shaji
 * @date    Nov 21, 2026
 */

#pragma once

#include <gtsam/nonlinear/Values.h>

#include <functional>

namespace gtsam::cuda {

/// Named aliases (rather than spelling out std::function<...> inline),
/// matching gtsam::CustomErrorFunction's precedent -- the wrap generator's
/// .i parser cannot handle a multi-argument std::function signature typed
/// out directly.
using BatchedFactorGroupStageFunction =
    std::function<void(int, const Values&)>;
using BatchedFactorGroupComputeBatchFunction = std::function<void()>;

/**
 * A group of Python-backed factors (e.g. one DFG-TAMP FactorGroup) that is
 * identically shaped across every item of a batch. `stage` records one
 * item's current Values with no computation; `computeBatch` then runs the
 * real net evaluation once, across every item staged since the last call,
 * and caches each item's result. That item's own per-item CustomFactor
 * callback -- still registered normally inside each item's
 * NonlinearFactorGraph, unchanged -- returns the cached result instead of
 * computing fresh, so no GaussianFactorGraph/ordering/Jacobian-assembly code
 * needs to change for this batching to take effect.
 */
struct BatchedFactorGroupHook {
  BatchedFactorGroupStageFunction stage;
  BatchedFactorGroupComputeBatchFunction computeBatch;

  BatchedFactorGroupHook() = default;
  /// Constructor taking both callbacks, matching CustomFactor's
  /// constructor-parameter pattern so pybind11/wrap can convert Python
  /// callables without needing field-level bindings.
  BatchedFactorGroupHook(const BatchedFactorGroupStageFunction& stageFunction,
                         const BatchedFactorGroupComputeBatchFunction&
                             computeBatchFunction)
      : stage(stageFunction), computeBatch(computeBatchFunction) {}
};

}  // namespace gtsam::cuda
