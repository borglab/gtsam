/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    StagMPCController.h
 * @brief   Base class and parameters for MPC controllers that optimize the
 *          future half of the STAG graph
 * @date    November 8, 2026
 * @author  Akash Ratheesh
 */

#pragma once

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <iostream>
#include <memory>
#include <string>

namespace gtsam {

/**
 * Parameters shared by all STAG MPC controllers. As with
 * NonlinearOptimizerParams, each controller derives its own parameters from
 * this struct.
 */
struct StagMPCControllerParams {
  bool verbose = false;  ///< Print progress while optimizing.

  /// Virtual destructor.
  virtual ~StagMPCControllerParams() = default;

  /// Print the parameters.
  virtual void print(const std::string& str = "") const {
    std::cout << str << "\n";
    std::cout << "  verbose: " << (verbose ? "true" : "false") << "\n";
  }

  /// Equal if all parameters are equal.
  bool equals(const StagMPCControllerParams& other,
              double /*tol*/ = 1e-9) const {
    return verbose == other.verbose;
  }
};

/**
 * Base class for MPC controllers on the future half of the STAG graph.
 *
 * As with NonlinearOptimizer, the base class does not store parameters: each
 * derived controller stores its own parameters, derived from
 * StagMPCControllerParams, and returns them from _params().
 */
class StagMPCController {
 public:
  /// Shared pointer to a controller.
  using shared_ptr = std::shared_ptr<StagMPCController>;

  /// Virtual destructor.
  virtual ~StagMPCController() = default;

  /**
   * Optimize the future graph, starting from the initial values, and return
   * the optimized values. controlKeys are the keys of the control variables
   * in the graph.
   */
  virtual Values optimize(const NonlinearFactorGraph& futureGraph,
                          const Values& initial,
                          const KeyVector& controlKeys) = 0;

 protected:
  /// The controller's parameters, as the base parameter type.
  virtual const StagMPCControllerParams& _params() const = 0;
};

}  // namespace gtsam
