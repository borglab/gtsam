/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    StagCost.h
 * @brief   Base class for the cost of the future half of the STAG graph
 * @date    November 8, 2026
 * @author  Akash Ratheesh
 */

#pragma once

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <cstddef>

namespace gtsam {

/**
 * Base class for the cost of the future half of the STAG graph.
 *
 * A cost emits factors; it never computes a scalar itself. For each step k of
 * the horizon H, runningCost() gives the factors on x_k and u_k, and
 * terminalCost() gives the factors on x_H. The caller owns the keys and the
 * horizon and passes in each step's keys and time, so a cost never creates
 * keys and works for any horizon.
 *
 * @tparam STATE    State type of x_k.
 * @tparam CONTROL  Control type of u_k.
 */
template <class STATE, class CONTROL>
class StagCost {
 public:
  /// Virtual destructor.
  virtual ~StagCost() = default;

  /**
   * Factors of the running cost of step k, k = 0 to H-1, on x_k (stateKey)
   * and u_k (controlKey). time is the time of step k. The graph may hold
   * zero or more factors.
   */
  virtual NonlinearFactorGraph runningCost(size_t k, Key stateKey,
                                           Key controlKey,
                                           double time) const = 0;

  /**
   * Factors of the terminal cost on x_H (stateKey). time is the time of
   * step H. The default returns an empty graph.
   */
  virtual NonlinearFactorGraph terminalCost(Key /*stateKey*/,
                                            double /*time*/) const {
    return NonlinearFactorGraph();
  }
};

}  // namespace gtsam
