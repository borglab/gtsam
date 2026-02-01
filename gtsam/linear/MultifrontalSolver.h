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

#include <gtsam/base/ForestTraversal.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/MultifrontalParameters.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/symbolic/SymbolicJunctionTree.h>

#include <iosfwd>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace gtsam {

class GaussianBayesTree;
class MultifrontalClique;

/**
 * Exception for unsupported use of the new multifrontal solver.
 * Includes guidance for using the legacy solver instead.
 */
class GTSAM_EXPORT MultifrontalSolverNotSupported : public std::runtime_error {
 public:
  explicit MultifrontalSolverNotSupported(const std::string& reason)
      : std::runtime_error(BuildMessage(reason)) {}

 private:
  static std::string BuildMessage(const std::string& reason) {
    std::string message = "MultifrontalSolver not supported: " + reason + ". ";
    message +=
        "Enable GTSAM_ALLOW_DEPRECATED_SINCE_V43 to default to the legacy "
        "solver, or set linearSolverType = MULTIFRONTAL_CHOLESKY.";
    return message;
  }
};

/**
 * Imperative-style multifrontal solver for Gaussian factor graphs.
 *
 * This class pre-allocates all necessary memory for the elimination tree and
 * provides efficient methods for loading new factors, eliminating the graph,
 * and solving for the update vector.
 *
 * @note Only JacobianFactor inputs are supported. Any non-Jacobian factors
 * will throw during construction/precompute or load.
 *
 * @note Clique merging has two optional phases: an initial leaf-merge pass
 * (leafMergeDimCap) that merges multiple leaf children into a common parent
 * while the parent's total dimension plus merged frontal dimensions stays
 * below the cap, followed by a bottom-up pass (mergeDimCap) that merges any
 * remaining small child cliques into their parent. Both phases run before
 * numeric elimination and can reduce tiny cliques and improve cache locality.
 * Defaults are conservative and may need tuning per machine or dataset.
 */
class GTSAM_EXPORT MultifrontalSolver
    : public ForestTraversal<MultifrontalSolver, MultifrontalClique> {
 public:
  /// Tuning parameters for traversal and reporting.
  using Parameters = MultifrontalParameters;

  struct PrecomputedData {
    std::map<Key, size_t> dims;         ///< Map from variable key to dimension.
    std::unordered_set<Key> fixedKeys;  ///< Keys fixed by constrained factors.
    SymbolicJunctionTree junctionTree;  ///< Precomputed symbolic junction tree.
  };

  /// Shared pointer to a MultifrontalClique.
  using CliquePtr = std::shared_ptr<MultifrontalClique>;
  /// Node type for tree traversal utilities.
  using Node = MultifrontalClique;

 protected:
  std::vector<CliquePtr> roots_;       ///< Roots of the elimination tree.
  std::vector<CliquePtr> cliques_;     ///< All cliques in the solver.
  std::map<Key, size_t> dims_;         ///< Map from variable key to dimension.
  mutable VectorValues solution_;      ///< Cached solution vector.
  std::unordered_set<Key> fixedKeys_;  ///< Keys fixed by constrained factors.
  bool loaded_ = false;                ///< Whether load() has been called.
  bool eliminated_ = false;            ///< Whether eliminateInPlace() ran.
  Parameters params_;                  ///< Tunable solver parameters.
  double lastOldError_ = 0.0;          ///< Cached old linearized error.
  double lastNewError_ = 0.0;          ///< Cached new linearized error.
  bool hasDeltaError_ = false;         ///< Whether updateSolution computed it.

 public:
  /**
   * Construct the solver from a factor graph and an ordering.
   * This builds the symbolic junction tree and pre-allocates all matrices.
   * Call load() before eliminating to populate numerical values.
   * @param graph The factor graph to solve.
   *              Must contain only JacobianFactor instances.
   * @param ordering The variable ordering to use for elimination.
   * @param params Tunable parameters for traversal and reporting.
   */
  MultifrontalSolver(const GaussianFactorGraph& graph, const Ordering& ordering,
                     const Parameters& params = Parameters{});

  /**
   * Construct the solver from precomputed symbolic data.
   * Call load() before eliminating to populate numerical values.
   * @param data Precomputed symbolic structure and sizing data.
   * @param ordering The variable ordering to use for seeding solution storage.
   * @param params Tunable parameters for traversal and reporting.
   */
  MultifrontalSolver(PrecomputedData data, const Ordering& ordering,
                     const Parameters& params = Parameters{});

  /// Precompute symbolic structure and sizing data from a factor graph.
  /// Only JacobianFactor inputs are supported.
  static PrecomputedData Precompute(const GaussianFactorGraph& graph,
                                    const Ordering& ordering);

  /**
   * Load new numerical values from the factor graph.
   * This overwrites the values in the pre-allocated matrices.
   *
   * @param graph The factor graph with updated values (structure must match
   *              the graph used to construct/precompute this solver, apart
   *              from updated numerical values).
   */
  void load(const GaussianFactorGraph& graph);

  /**
   * Eliminate the graph using Cholesky factorization.
   * This operates in-place on the pre-allocated matrices.
   */
  void eliminateInPlace();

  /**
   * Load and eliminate the graph in a single traversal.
   * This calls fillAb() and eliminateInPlace() per clique in post-order.
   */
  void eliminateInPlace(const GaussianFactorGraph& graph);

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
  const VectorValues& updateSolution();

  /**
   * Return the linearized delta error from the last updateSolution() call.
   * Optionally returns the old and new linearized errors.
   */
  double deltaError(double* oldError = nullptr,
                    double* newError = nullptr) const;

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
