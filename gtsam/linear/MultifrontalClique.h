/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file MultifrontalClique.h
 * @brief Imperative multifrontal clique data structure.
 * @author Frank Dellaert
 * @date   December 2025
 */

#pragma once

#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/base/VerticalBlockMatrix.h>
#include <gtsam/dllexport.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/symbolic/SymbolicFactor.h>

#include <iosfwd>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace gtsam {

class GaussianConditional;

/// Map from variable key to dimension.
using KeyDimMap = std::map<Key, size_t>;

namespace internal {

/// Helper class to track original factor indices and row counts.
class IndexedSymbolicFactor : public SymbolicFactor {
 public:
  size_t index_;
  size_t rows_;
  IndexedSymbolicFactor(const KeyVector& keys, size_t index, size_t rows)
      : SymbolicFactor(), index_(index), rows_(rows) {
    keys_ = keys;
  }
};

/// Sum variable dimensions for a key range, skipping unknown keys.
template <typename KeyRange>
inline size_t sumDims(const KeyDimMap& dims, const KeyRange& keys) {
  size_t dim = 0;
  for (Key key : keys) {
    auto it = dims.find(key);
    if (it != dims.end()) dim += it->second;
  }
  return dim;
}

}  // namespace internal

/**
 * Imperative multifrontal clique structure used by MultifrontalSolver.
 */
class GTSAM_EXPORT MultifrontalClique {
 public:
  using shared_ptr = std::shared_ptr<MultifrontalClique>;
  using Children = std::vector<shared_ptr>;
  struct ChildInfo {
    shared_ptr clique;
    KeySet separatorKeys;
  };

  std::weak_ptr<MultifrontalClique> parent;  /// < Parent clique.
  Children children;  /// < Child cliques used for traversal.

  /// Construct a clique from factor indices and cache static structure.
  /// @param factorIndices Indices of factors associated with this clique.
  /// @param parent Weak pointer to the parent clique.
  /// @param frontals Frontal keys for this clique.
  /// @param separatorKeys Separator keys for this clique.
  /// @param dims Key->dimension map.
  /// @param vbmRows Number of rows needed for the vertical block matrix.
  /// @param solution Solution storage for cached pointers.
  /// @param fixedKeys Keys fixed to zero by constraints (may be null).
  explicit MultifrontalClique(std::vector<size_t> factorIndices,
                              const std::weak_ptr<MultifrontalClique>& parent,
                              const KeyVector& frontals,
                              const KeySet& separatorKeys,
                              const KeyDimMap& dims, size_t vbmRows,
                              VectorValues* solution,
                              const std::unordered_set<Key>* fixedKeys);

  /// @name Setup (non-const)
  /// @{

  /// Cache the children list and compute parent indices.
  void finalize(std::vector<ChildInfo> children);

  /// Load factor values into the pre-allocated Ab matrix.
  /// @param graph The factor graph with updated values (structure must match
  ///              the graph used to build this clique, apart from updated
  ///              numerical values). Only JacobianFactor inputs are supported.
  void fillAb(const GaussianFactorGraph& graph);

  /// Zero out the info matrix, re-add Hessians, accumulate Jacobians and
  /// children.
  void prepareForElimination();

  /// Perform Cholesky factorization on the frontal block.
  void factorize();

  /**
   * Add identity damping to the frontal block.
   * @param lambda Damping factor
   */
  void addIdentityDamping(double lambda);

  /**
   * Add diagonal damping to the frontal block.
   * @param lambda Damping factor
   * @param minDiagonal Minimum diagonal value
   * @param maxDiagonal Maximum diagonal value
   */
  void addDiagonalDamping(double lambda, double minDiagonal,
                          double maxDiagonal);

  /// @}

  /// @name Read-only methods
  /// @{

  /// Return the total frontal dimension for this clique.
  size_t frontalDim() const { return static_cast<size_t>(rhsScratch_.size()); }

  /// Return the total separator dimension for this clique.
  size_t separatorDim() const {
    return static_cast<size_t>(separatorScratch_.size());
  }

  /// Get the cached problem size for traversal scheduling.
  ///
  /// This is a proxy for the work in the subtree rooted at this clique, used
  /// only to decide whether to spawn parallel traversal tasks.
  int problemSize() const {
    constexpr size_t kMaxInt =
        static_cast<size_t>(std::numeric_limits<int>::max());
    const size_t clamped = subtreeWork_ > kMaxInt ? kMaxInt : subtreeWork_;
    return static_cast<int>(clamped);
  }

  /// Return the number of frontal keys in this clique.
  size_t numFrontals() const { return frontalPtrs_.size(); }

  /// Build a GaussianConditional from the in-place factorization.
  std::shared_ptr<GaussianConditional> conditional() const;

  /// Get the vertical block matrix Ab.
  const VerticalBlockMatrix& Ab() const { return Ab_; }

  /// Get the information matrix (const).
  const SymmetricBlockMatrix& info() const { return info_; }

  /// Check if this clique is using QR elimination.
  bool useQR() const { return solveMode_ == SolveMode::QrLeaf; }

  /**
   * Print this clique.
   * @param s Optional string prefix.
   * @param keyFormatter Ignored; retained for API compatibility.
   */
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /// @}

  /// @name Solve (non-const)
  /// @{

  /**
   * Eliminate this clique and propagate its separator contribution upward.
   *
   * Computes the local information matrix from the stacked Jacobian (Ab),
   * incorporates child separator contributions, and performs partial Cholesky
   * on the frontal blocks. Requires parent indices to be precomputed.
   */
  void eliminateInPlace();

  /**
   * Apply this clique's separator contribution into the parent clique.
   * @param parent Parent clique to update.
   */
  void updateParent(MultifrontalClique& parent) const;

  /**
   * Solve for this clique's frontal variables and write them back to the
   * cached solution vectors.
   *
   * Uses block back-substitution using the upper triangular-part of the
   * Cholesky-stored information matrix, solving the triangular system for the
   * frontal blocks.
   */
  void updateSolution() const;
  /// @}

  friend std::ostream& operator<<(std::ostream& os,
                                  const MultifrontalClique& clique);

 private:
  enum class SolveMode { Cholesky, QrLeaf };

  /// Cache pointers to frontal and separator update vectors.
  void cacheSolutionPointers(VectorValues* delta, const KeyVector& frontals,
                             const KeySet& separatorKeys);

  /// Linear lookup for block index in small cliques.
  DenseIndex blockIndex(Key key) const;

  /// Update a parent information matrix with this clique's separator
  /// contribution.
  void updateParentInfo(SymmetricBlockMatrix& parentInfo) const;

  /// Accumulate children separator updates into this clique's info matrix
  /// (single-threaded).
  void gatherUpdatesSequential();

  /// Accumulate children separator updates into this clique's info matrix
  /// (multi-threaded).
  void gatherUpdatesParallel(size_t numThreads);

  /// Compute block dimensions from variable dimensions (excluding RHS).
  std::vector<size_t> blockDims(const KeyDimMap& dims,
                                const KeyVector& frontals,
                                const KeySet& separatorKeys) const;

  /**
   * Pre-allocate matrices for this clique.
   * @param blockDims Block dimensions (excluding RHS).
   * @param totalNumRows Number of rows for the vertical block matrix.
   */
  void initializeMatrices(const std::vector<size_t>& blockDims,
                          size_t totalNumRows);

  /// Allocate the information matrix if needed.
  void allocateInfo();

  /**
   * Add a Jacobian factor's contributions into the Ab matrix.
   * @return Number of rows added.
   */
  size_t addJacobianFactor(const JacobianFactor& factor, size_t rowOffset);

  void setParentIndices(const std::vector<DenseIndex>& indices) {
    parentIndices_ = indices;
  }
  // Build-time metadata.
  std::vector<size_t> factorIndices_;
  KeyVector orderedKeys_;  ///< Keys ordered by block index (frontals+seps).
  const std::unordered_set<Key>* fixedKeys_ = nullptr;
  std::vector<size_t> blockDims_;
  std::vector<DenseIndex>
      parentIndices_;  ///< Parent block indices for separators + RHS.
  std::vector<Vector*> frontalPtrs_;  ///< Pointers into solution frontals.
  std::vector<const Vector*>
      separatorPtrs_;  ///< Pointers into solution separator.

  // Load-time state.
  VerticalBlockMatrix Ab_;
  SolveMode solveMode_ = SolveMode::Cholesky;

  // Elimination-time state.
  mutable SymmetricBlockMatrix info_;
  mutable VerticalBlockMatrix RSd_;  ///< Cached [R S d] from elimination.
  mutable bool RSdReady_ = false;

  // Solve-time scratch space.
  mutable Vector rhsScratch_;  /// < Cached RHS workspace for back-substitution.
  mutable Vector
      separatorScratch_;  /// < Cached separator stack for back-substitution.

  size_t localWork_ = 0;    /// < Local work proxy (dimension-based).
  size_t subtreeWork_ = 0;  /// < Subtree-max work proxy for task scheduling.

  static constexpr double kQrAspectRatio = 2.0;
};

std::ostream& operator<<(std::ostream& os, const MultifrontalClique& clique);

}  // namespace gtsam
