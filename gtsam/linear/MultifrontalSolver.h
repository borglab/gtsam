/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file MultifrontalSolver.h
 * @brief Imperative-style multifrontal solver for Gaussian factor graphs.
 * @author Frank Dellaert
 * @date   December 2025
 */

#pragma once

#include <gtsam/inference/Key.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

#include <iosfwd>
#include <map>
#include <memory>
#include <vector>

namespace gtsam {

class GaussianBayesTree;
class MultifrontalClique;

/**
 * Imperative-style multifrontal solver for Gaussian factor graphs.
 *
 * This class pre-allocates all necessary memory for the elimination tree and
 * provides efficient methods for loading new factors, eliminating the graph,
 * and solving for the update vector.
 */
class GTSAM_EXPORT MultifrontalSolver {
 public:
  /// Shared pointer to a MultifrontalClique.
  using CliquePtr = std::shared_ptr<MultifrontalClique>;
  /// Node type for tree traversal utilities.
  using Node = MultifrontalClique;

 private:
  std::vector<CliquePtr> roots_;    ///< Roots of the elimination tree.
  std::vector<CliquePtr> cliques_;  ///< All cliques in the solver.
  std::map<Key, size_t> dims_;      ///< Map from variable key to dimension.
  mutable VectorValues solution_;   ///< Cached solution vector.

 public:
  /**
   * Construct the solver from a factor graph and an ordering.
   * This builds the symbolic junction tree and pre-allocates all matrices.
   * @param graph The factor graph to solve.
   * @param ordering The variable ordering to use for elimination.
   * @param mergeDimCap Merge a child if its frontal dimension plus the
   * parent's total dimension is below this threshold (0 disables merging).
   * @param reportStream Optional stream to report clique structure stats
   * (frontals, separators, total dims, and children).
   */
  MultifrontalSolver(const GaussianFactorGraph& graph, const Ordering& ordering,
                     size_t mergeDimCap = 0,
                     std::ostream* reportStream = nullptr);

  /**
   * Load new numerical values from the factor graph.
   * This overwrites the values in the pre-allocated matrices.
   *
   * @param graph The factor graph with updated values (structure must match).
   */
  void load(const GaussianFactorGraph& graph);

  /**
   * Eliminate the graph using Cholesky factorization.
   * This operates in-place on the pre-allocated matrices.
   */
  void eliminateInPlace();

  /**
   * Compute a Bayes tree from the in-place Cholesky factorization.
   * Requires eliminateInPlace() to have been called beforehand.
   * @return A GaussianBayesTree representing the eliminated factor graph
   * encoded by the current multifrontal factorization.
   */
  GaussianBayesTree computeBayesTree() const;

  /**
   * Solve for the update vector.
   *
   * @return Reference to the internally cached solution vector.
   */
  const VectorValues& updateSolution() const;

  /// Accessor for the roots of the elimination tree.
  const std::vector<CliquePtr>& roots() const { return roots_; }

  /// Get the total number of cliques in the solver.
  size_t cliqueCount() const { return cliques_.size(); }

  /// Print the solver state.
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /// Output stream operator for MultifrontalSolver.
  friend std::ostream& operator<<(std::ostream& os,
                                  const MultifrontalSolver& solver);
};

std::ostream& operator<<(std::ostream& os, const MultifrontalSolver& solver);

}  // namespace gtsam
