/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file NonlinearMultifrontalSolver.h
 * @brief Multifrontal solver for nonlinear factor graphs.
 * @author Frank Dellaert
 * @date   January 2026
 */

#pragma once

#include <gtsam/linear/MultifrontalSolver.h>
#include <gtsam/nonlinear/LMDampingParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <optional>

namespace gtsam {

/**
 * Multifrontal solver for nonlinear factor graphs.
 *
 * This class extends MultifrontalSolver to solve nonlinear problems.
 * The linearization is provided externally via load(), or via
 * eliminateInPlace() which combines loading and elimination.
 */
class GTSAM_EXPORT NonlinearMultifrontalSolver : public MultifrontalSolver {
 public:
  using DampingParams = LMDampingParams;

  /**
   * Construct the solver from a nonlinear factor graph and linearization point.
   * This computes the symbolic structure (including fixed keys) from the
   * nonlinear graph and uses the values to determine variable dimensions.
   * Call load() with a linearized graph before eliminating.
   * @param graph The nonlinear factor graph to build structure from.
   * @param values The linearization point used to determine variable dims.
   * @param ordering The variable ordering to use.
   * @param params Tunable parameters for traversal and reporting.
   */
  NonlinearMultifrontalSolver(const NonlinearFactorGraph& graph,
                              const Values& values, const Ordering& ordering,
                              MultifrontalSolver::Parameters params = {},
                              DampingParams dampingParams = DampingParams());

  /**
   * Precompute symbolic structure from a nonlinear factor graph and values.
   * This avoids linearization; call load() with a linearized graph before
   * eliminating.
   */
  static MultifrontalSolver::PrecomputedData Precompute(
      const NonlinearFactorGraph& graph, const Values& values,
      const Ordering& ordering);

  /**
   * Load new numerical values from the factor graph.
   * This overrides the base load() to optionally cache `diag(J^T J)` for
   * exact diagonal damping when enabled.
   */
  void load(const GaussianFactorGraph& graph);

  /**
   * Eliminate with optional LM-style damping.
   * When lambda is provided, adds damping on frontal blocks before
   * factorization.
   */
  void eliminateInPlace(double lambda = 0.0);

  /**
   * Load and eliminate the graph in a single traversal with optional damping.
   * This calls fillAb() and factorization per clique in post-order.
   * @param graph The linearized factor graph (structure must match).
   * @param lambda Optional damping value; non-positive disables damping.
   */
  void eliminateInPlace(const GaussianFactorGraph& graph, double lambda = 0.0);

 private:
  DampingParams dampingParams_;
  VectorValues exactHessianDiagonal_;
  bool hasExactHessianDiagonal_ = false;
};

}  // namespace gtsam
