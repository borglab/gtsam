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
#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace gtsam {

class GaussianConditional;

namespace internal {

/// Helper class to track original factor indices.
class IndexedSymbolicFactor : public SymbolicFactor {
 public:
  size_t index_;
  IndexedSymbolicFactor(const GaussianFactor& factor, size_t index)
      : SymbolicFactor(factor), index_(index) {}
};

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
    KeyVector separatorKeys;
  };

  std::weak_ptr<MultifrontalClique> parent;  ///< Parent clique.
  Children children;        ///< Child cliques used for traversal.
  size_t frontalDim = 0;    ///< Frontal dimension.
  size_t separatorDim = 0;  ///< Separator dimension.

  /// Construct a clique from factor indices.
  /// @param factorIndices Indices of factors associated with this clique.
  /// @param parent Weak pointer to the parent clique.
  explicit MultifrontalClique(std::vector<size_t> factorIndices,
                              const std::weak_ptr<MultifrontalClique>& parent);

  /// @name Setup (non-const)
  /// @{

  /// Cache dimensions, cache value pointers, pre-allocate matrices,
  /// and cache constraint metadata.
  void finalize(const KeyVector& frontals, const KeyVector& separatorKeys,
                const std::map<Key, size_t>& dims,
                const GaussianFactorGraph& graph, VectorValues* solution,
                std::vector<ChildInfo> children);

  /// Load factor values into the pre-allocated Ab matrix and Hessians into
  /// sbm_.
  /// @param graph The factor graph with updated values.
  void fillAb(const GaussianFactorGraph& graph);
  /// @}

  /// @name Read-only methods
  /// @{

  /// Get the cached problem size for traversal scheduling.
  int problemSize() const {
    return static_cast<int>(frontalDim + separatorDim);
  }

  /// Get the number of factors in this clique.
  size_t factorCount() const;

  /// Return the number of frontal keys in this clique.
  size_t numFrontals() const { return numFrontals_; }

  /// Build a GaussianConditional from the in-place factorization.
  std::shared_ptr<GaussianConditional> conditional() const;

  /// Get the vertical block matrix Ab.
  const VerticalBlockMatrix& Ab() const { return Ab_; }

  /// Get the symmetric block matrix (mutable).
  SymmetricBlockMatrix& sbm() { return sbm_; }

  /// Get the symmetric block matrix (const).
  const SymmetricBlockMatrix& sbm() const { return sbm_; }

  /**
   * Count rows needed for the vertical block matrix.
   * @param graph The factor graph.
   * @return Total number of rows.
   */
  size_t countRows(const GaussianFactorGraph& graph) const;

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
   * Computes the local normal equations (SBM) from the stacked Jacobian (Ab),
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
   * Cholesky-stored SBM, solving the triangular system for the frontal blocks.
   */
  void updateSolution() const;
  /// @}

  friend std::ostream& operator<<(std::ostream& os,
                                  const MultifrontalClique& clique);

 private:
  /// Cache pointers to frontal and separator update vectors.
  void cacheSolutionPointers(VectorValues* delta, const KeyVector& frontals,
                             const KeyVector& separatorKeys);

  /// Cache constraint metadata (fixed frontals, constrained factors).
  void cacheConstraintInfo(const GaussianFactorGraph& graph);

  /// Compute block dimensions from variable dimensions (excluding RHS).
  std::vector<size_t> blockDims(const std::map<Key, size_t>& dims,
                                const KeyVector& frontals,
                                const KeyVector& separatorKeys) const;

  /// Pre-allocate matrices for this clique.
  /// @param blockDims Block dimensions (excluding RHS).
  /// @param verticalBlockMatrixRows Number of rows for the vertical block
  /// matrix.
  void initializeMatrices(const std::vector<size_t>& blockDims,
                          size_t verticalBlockMatrixRows);

  /// Add a Jacobian factor's contributions into the Ab matrix.
  void addJacobianFactor(const JacobianFactor& factor, size_t rowOffset);

  /// Add a Hessian factor's contributions into the sbm_ matrix.
  void addHessianFactor(const HessianFactor& factor);

  void setParentIndices(const std::vector<size_t>& indices) {
    parentIndices_ = indices;
  }
  VerticalBlockMatrix Ab_;
  mutable SymmetricBlockMatrix sbm_;
  mutable Vector rhsScratch_;  ///< Cached RHS workspace for back-substitution.
  mutable Vector
      separatorScratch_;  ///< Cached separator stack for back-substitution.

  std::vector<size_t> factorIndices_;
  std::map<Key, size_t> blockIndex_;  ///< Key->block index for fast Ab fills.
  size_t numFrontals_ = 0;
  std::unordered_set<size_t>
      fixedFrontals_;  ///< Frontal block indices fixed by constraints.
  std::vector<size_t> parentIndices_;  ///< Parent block indices for separators and RHS.
  std::vector<Vector*> frontalPtrs_;   ///< Pointers into solution frontals.
  std::vector<const Vector*>
      separatorPtrs_;  ///< Pointers into solution separator.
};

std::ostream& operator<<(std::ostream& os, const MultifrontalClique& clique);

}  // namespace gtsam
