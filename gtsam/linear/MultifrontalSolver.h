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
#include <gtsam/symbolic/IndexedJunctionTree.h>

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
 * This class precomputes the elimination tree, allocates fixed solve storage,
 * and provides efficient methods for loading new factors, eliminating the
 * graph, and solving for the update vector. Each clique lazily allocates its
 * packed Jacobian fallback rows on the first compatible load and reuses that
 * storage thereafter.
 *
 * @note Only JacobianFactor and BatchJacobianFactor inputs are supported. Other
 * Gaussian factor types will throw during construction/precompute or load.
 *
 * @note Clique merging has two symbolic phases. First, leafMergeDimCap merges
 * compatible multi-factor leaf siblings with identical separators into
 * bounded algebraic cliques; one-factor leaves remain separate so direct batch
 * factors can use the fused leaf path. Then mergeDimCap can merge remaining
 * small children into their parents. Afterward, leaf task aggregation can
 * schedule independent leaves together, with LeafMode::SameSeparator also
 * accumulating compatible Cholesky updates in separator-local matrices.
 */
class GTSAM_EXPORT MultifrontalSolver
    : public ForestTraversal<MultifrontalSolver, MultifrontalClique> {
 public:
  /// Tuning parameters for traversal and reporting.
  using Parameters = MultifrontalParameters;

  /// Precomputed symbolic and sizing data for multifrontal solver construction.
  struct PrecomputedData {
    std::map<Key, size_t> dims;         ///< Map from variable key to dimension.
    std::unordered_set<Key> fixedKeys;  ///< Keys fixed by constrained factors.
    IndexedJunctionTree
        indexedJunctionTree;        ///< Precomputed indexed junction tree.
    std::vector<size_t> rowCounts;  ///< Row counts indexed by factor index.
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
  bool partiallyEliminated_ = false;   ///< Whether partial elimination ran.
  Ordering ordering_;                  ///< Complete variable ordering.
  size_t firstPhaseSize_ = 0;          ///< Prefix eliminated in partial mode.
  Parameters params_;                  ///< Tunable solver parameters.
  double lastOldError_ = 0.0;          ///< Cached old linearized error.
  double lastNewError_ = 0.0;          ///< Cached new linearized error.
  bool hasDeltaError_ = false;         ///< Whether updateSolution computed it.

 public:
  /**
   * Construct the solver from a factor graph and an ordering.
   * This builds the indexed junction tree and allocates fixed solve matrices.
   * Packed Jacobian fallback storage is allocated lazily during the first
   * load.
   * Call load() before eliminating to populate numerical values.
   * @param graph The factor graph to solve.
   *              Must contain only JacobianFactor or BatchJacobianFactor
   *              instances.
   * @param ordering The variable ordering to use for elimination.
   * @param params Tunable parameters for traversal and reporting.
   */
  MultifrontalSolver(const GaussianFactorGraph& graph, const Ordering& ordering,
                     const Parameters& params = Parameters{});

  /**
   * Construct a solver configured for partial multifrontal elimination.
   * The leading `firstPhaseSize` keys in `ordering` are eliminated, while the
   * remaining keys stay assembled in retained clique factors.
   */
  MultifrontalSolver(const GaussianFactorGraph& graph, const Ordering& ordering,
                     size_t firstPhaseSize,
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

  /** Construct a partial solver from precomputed symbolic data. */
  MultifrontalSolver(PrecomputedData data, const Ordering& ordering,
                     size_t firstPhaseSize,
                     const Parameters& params = Parameters{});

  /**
   * Precompute symbolic structure and sizing data from a factor graph.
   * This builds an IndexedJunctionTree that can be reused across multiple
   * solver instances when the graph structure and ordering remain unchanged.
   * Only JacobianFactor and BatchJacobianFactor inputs are supported.
   *
   * @param graph The factor graph (must contain only supported Jacobian-style
   *              factor instances)
   * @param ordering The variable elimination ordering
   * @return PrecomputedData containing the indexed junction tree and sizing
   * info
   */
  static PrecomputedData Precompute(const GaussianFactorGraph& graph,
                                    const Ordering& ordering);

  /**
   * Load new numerical values from the factor graph.
   * The first load builds cached factor load plans and allocates only the
   * Jacobian rows needed by the selected numerical path. Later compatible
   * loads overwrite and reuse that packed storage.
   *
   * @param graph The factor graph with updated values (structure must match
   *              the graph used to construct/precompute this solver, apart
   *              from updated numerical values).
   */
  void load(const GaussianFactorGraph& graph);

  /**
   * Eliminate the graph using Cholesky factorization.
   * This operates in-place on the clique's reusable numerical storage.
   */
  void eliminateInPlace();

  /**
   * Load and eliminate the graph in a single traversal.
   * This calls fillAb() and eliminateInPlace() per clique in post-order.
   */
  void eliminateInPlace(const GaussianFactorGraph& graph);

  /**
   * Eliminate the configured ordering prefix and assemble the retained
   * clique factors without factorizing the retained variables.
   */
  void eliminatePartialInPlace();

  /** Load a graph and partially eliminate it in one bottom-up traversal. */
  void eliminatePartialInPlace(const GaussianFactorGraph& graph);

  /**
   * Materialize one Hessian factor per retained clique after partial
   * elimination. Each returned factor owns a compact copy of its active
   * upper-triangular information block because it may outlive the clique.
   */
  GaussianFactorGraph remainingFactorGraph() const;

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
   * Seed retained variables and back-substitute through eliminated cliques.
   */
  const VectorValues& updateSolution(const VectorValues& retainedSolution);

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
