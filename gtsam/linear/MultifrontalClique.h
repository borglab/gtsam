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
#include <gtsam/linear/MultifrontalParameters.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/internal/BatchHessianMapping.h>
#include <gtsam/nonlinear/LMDampingParams.h>
#include <gtsam/symbolic/SymbolicFactor.h>

#include <cstdint>
#include <iosfwd>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace gtsam {

class BatchJacobianFactorBase;
class GaussianConditional;
class HessianFactor;
class JacobianFactor;

/// Map from variable key to dimension.
using KeyDimMap = std::map<Key, size_t>;

namespace internal {

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

  std::weak_ptr<MultifrontalClique> parent;  ///< Parent clique.
  Children children;        ///< Child cliques used for traversal.
  size_t frontalDim = 0;    ///< Frontal dimension.
  size_t separatorDim = 0;  ///< Separator dimension.

  /**
   * Construct a clique from factor indices and cache static structure.
   * @param factorIndices Indices of factors associated with this clique.
   * @param parent Weak pointer to the parent clique.
   * @param frontals Frontal keys for this clique.
   * @param separatorKeys Separator keys for this clique.
   * @param dims Key->dimension map.
   * @param vbmRows Number of rows needed for the vertical block matrix.
   * @param solution Solution storage for cached pointers.
   * @param fixedKeys Keys fixed to zero by constraints (may be null).
   */
  explicit MultifrontalClique(std::vector<size_t> factorIndices,
                              const std::weak_ptr<MultifrontalClique>& parent,
                              const KeyVector& frontals,
                              const KeySet& separatorKeys,
                              const KeyDimMap& dims, size_t vbmRows,
                              VectorValues* solution,
                              const std::unordered_set<Key>* fixedKeys,
                              size_t numEliminatedFrontals);

  /// @name Setup (non-const)
  /// @{

  /**
   * Cache the children list, compute parent indices, and lock in QR usage.
   * @param children Child cliques plus separator metadata.
   * @param params Parameters controlling QR mode and thresholds.
   */
  void finalize(std::vector<ChildInfo> children,
                const MultifrontalParameters& params);

  /**
   * Load factor values into a lazily allocated, reusable Ab matrix.
   * @param graph The factor graph with updated values (structure must match
   *              the graph used to build this clique, apart from updated
   *              numerical values). Only JacobianFactor and BatchJacobianFactor
   *              inputs are supported.
   */
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

  /**
   * Add diagonal damping to the frontal block using an externally provided
   * Hessian diagonal `diag(J^T J)` keyed by variable.
   *
   * This matches the legacy LM diagonal damping definition based on the
   * diagonal of the original linearized system rather than the post-Schur
   * clique information matrix.
   *
   * @param lambda Damping factor
   * @param hessianDiagonal Map from key to diagonal vector (dimension-matched).
   * @param minDiagonal Minimum diagonal value
   * @param maxDiagonal Maximum diagonal value
   */
  void addExactDiagonalDamping(double lambda,
                               const VectorValues& hessianDiagonal,
                               double minDiagonal, double maxDiagonal);

  /// @}

  /// @name Read-only methods
  /// @{

  /// Return the clique dimension used for traversal scheduling.
  int problemSize() const {
    return static_cast<int>(frontalDim + separatorDim);
  }

  /// Return the number of frontal keys in this clique.
  size_t numFrontals() const { return frontalPtrs_.size(); }

  /// Return whether every symbolic frontal in this clique is eliminated.
  bool fullyEliminated() const { return numFrontals() == totalFrontals_; }

  /// Return keys ordered by block index (frontals followed by separators).
  const KeyVector& orderedKeys() const { return orderedKeys_; }

  /// Build a GaussianConditional from the in-place factorization.
  std::shared_ptr<GaussianConditional> conditional() const;

  /**
   * Build the assembled factor on variables retained after partial
   * elimination. The returned factor owns its information because it may
   * outlive this clique. Consequently, exporting requires one copy of the
   * active upper-triangular block. The clique must have been prepared, and any
   * leading eliminated frontals must have been factorized.
   */
  std::shared_ptr<HessianFactor> remainingFactor() const;

  /// Get the vertical block matrix Ab.
  const VerticalBlockMatrix& Ab() const { return Ab_; }

  /// Get the information matrix (const).
  const SymmetricBlockMatrix& info() const { return info_; }

  /// Check if this clique is using QR elimination.
  bool useQR() const { return solveMode_ == SolveMode::QrLeaf; }

  /// Check if this leaf avoids materializing its separator Hessian.
  bool useCompactCholesky() const {
    return solveMode_ == SolveMode::CompactCholeskyLeaf ||
           solveMode_ == SolveMode::FusedStarCandidate ||
           solveMode_ == SolveMode::FusedStarCholeskyLeaf;
  }

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
   * Eliminate in-place, invalidating Ab_, and updating RSd_ and info_.
   *
   * Computes the local information matrix from the stacked Jacobian (Ab),
   * incorporates child separator contributions, and performs partial QR or
   * Cholesky on the frontal blocks.
   */
  void eliminateInPlace();

  /**
   * Version of eliminate that applies damping before eliminate.
   *
   * @param lambda Optional damping value; non-positive disables damping.
   * @param dampingParams Parameters controlling LM-style damping.
   * @param exactHessianDiagonal `diag(J^T J)` values for diagonal damping.
   */
  void eliminateInPlace(double lambda, const LMDampingParams& dampingParams,
                        const VectorValues& exactHessianDiagonal);

  /**
   * Solve for this clique's frontal variables and write them back to the
   * cached solution vectors.
   *
   * Uses block back-substitution using the upper triangular-part of the
   * Cholesky-stored information matrix, solving the triangular system for the
   * frontal blocks.
   */
  void updateSolution();

  /// Access the last old error computed during updateSolution().
  double lastOldError() const { return lastOldError_; }

  /// Access the last new error computed during updateSolution().
  double lastNewError() const { return lastNewError_; }

  /// Return the constant error term for this clique (nonzero for roots).
  double constantTermError() const;
  /// @}

  friend std::ostream& operator<<(std::ostream& os,
                                  const MultifrontalClique& clique);

 private:
  enum class SolveMode {
    Cholesky,
    QrLeaf,
    CompactCholeskyLeaf,
    FusedStarCandidate,
    FusedStarCholeskyLeaf
  };

  /// Cache pointers to frontal and separator update vectors.
  void cacheSolutionPointers(VectorValues* delta, const KeyVector& frontals,
                             const KeyVector& separatorKeys);

  /// Linear lookup for block index in small cliques.
  DenseIndex blockIndex(Key key) const;

  enum class FactorLoadKind : uint8_t { Empty, Jacobian, Batch };

  struct FactorLoadPlan {
    FactorLoadKind kind = FactorLoadKind::Empty;
    size_t factorIndex = 0;
    SharedDiagonal model;
    size_t rows = 0;
    /// Packed row offset in Ab_ for factors that require Jacobian fallback.
    size_t abRowOffset = 0;
    /// Slot indices for factor keys (or -1 for fixed keys). See
    /// `linear/doc/BatchFactor_Performance_Notes.html` for load-plan usage.
    std::vector<DenseIndex> blockIndices;
    /// Row-group destinations in this clique's augmented information matrix.
    internal::BatchHessianMapping localMapping;
    /// Retained row-group destinations in the parent information matrix.
    internal::BatchHessianMapping parentMapping;
    /// Retained row-group destinations in a separator-local accumulator.
    internal::BatchHessianMapping separatorMapping;
    bool canDirectUpdate = false;

    /// Construct a plan for a null graph entry.
    static FactorLoadPlan forNullFactor(size_t factorIndex);

    /// Construct a plan for a conventional Jacobian factor.
    static FactorLoadPlan forJacobian(size_t factorIndex,
                                      const JacobianFactor& factor,
                                      const MultifrontalClique& clique);

    /// Construct a plan for a compact batch Jacobian factor.
    static FactorLoadPlan forBatch(size_t factorIndex,
                                   const BatchJacobianFactorBase& factor,
                                   const MultifrontalClique& clique);

    /// Return whether this plan represents a directly assembled batch factor.
    bool isDirectBatch() const {
      return kind == FactorLoadKind::Batch && canDirectUpdate;
    }

    /// Return whether this factor requires packed Jacobian rows in Ab_.
    bool needsMaterializedRows(const MultifrontalClique& clique) const;

    /// Reserve this plan's packed rows from a clique-wide row cursor.
    void assignMaterializedRows(size_t* nextRow);

    /// Assert the structural invariants established by the plan factories.
    void assertInvariants(const GaussianFactor* factor,
                          const MultifrontalClique& clique) const;

    /// Return whether this factor is supported by a fused star leaf.
    bool supportsFusedStarLeaf() const;

    /// Cache mappings required by the clique's resolved solve mode.
    void buildSolveMappings(const BatchJacobianFactorBase& factor,
                            const MultifrontalClique& clique);

   private:
    /// Map factor keys to clique-local block indices.
    void mapKeys(const KeyVector& keys, const MultifrontalClique& clique);

    /// Build the clique-local mapping shared by direct Hessian paths.
    void buildLocalBatchMapping(const BatchJacobianFactorBase& factor,
                                const MultifrontalClique& clique);

    /// Map retained factor slots into one compact-update destination.
    internal::BatchHessianMapping buildRetainedMapping(
        const BatchJacobianFactorBase& factor, DenseIndex numFrontals,
        const std::vector<DenseIndex>& targetIndices,
        const std::vector<DenseIndex>& targetScalarOffsets) const;
  };

  /// Build and cache loading metadata for factors in this clique.
  void buildLoadPlans(const GaussianFactorGraph& graph);

  /// Allocate numerical solve storage after the clique mode is resolved.
  void allocateSolveStorage();

  /// Resolve deferred QR and fused-star choices after factor plans are known.
  void resolveLeafSolveMode();

  /// Cache scalar-column destinations for fused Schur updates.
  void buildFusedStarMappings();

  /// Update a parent information matrix with this clique's separator
  /// contribution.
  void updateParentInfo(SymmetricBlockMatrix& parentInfo) const;

  /// Accumulate one ordinary separator block into its owned parent column.
  void updateParentMaterializedColumn(SymmetricBlockMatrix& parentInfo,
                                      DenseIndex sourceSeparatorBlock) const;

  /// Accumulate one QR separator block into narrow owned-column scratch.
  void updateParentQrColumnScratch(DenseIndex sourceSeparatorBlock,
                                   Matrix* scratch) const;

  /// Return the augmented-RHS diagonal from a column-owned child update.
  double parentRhsDiagonal() const;

  /// Update a separator-local information matrix without parent scattering.
  void updateSeparatorInfo(SymmetricBlockMatrix& separatorInfo) const;

  /// Assemble only the frontal rows [A B a] of a compact Cholesky leaf.
  void prepareCompactCholesky();

  /// Factor [A B a] in place into [R S d].
  void factorizeCompactCholesky();

  /// Factor fused-star frontal rows without allocating an LLT workspace.
  void factorizeFusedStarCholesky();

  /// Add a fused-star retained update directly into its destination.
  void updateFusedStarInfo(SymmetricBlockMatrix& targetInfo,
                           const std::vector<DenseIndex>& targetIndices,
                           const std::vector<DenseIndex>& targetScalarOffsets,
                           bool useParentMappedSlots) const;

  /// Dispatch one Cholesky representation into a mapped destination.
  void updateCholeskyInfo(SymmetricBlockMatrix& targetInfo,
                          const std::vector<DenseIndex>& targetIndices,
                          const std::vector<DenseIndex>& targetBlockOffsets,
                          bool useParentMappedSlots) const;

  /// Add the original separator normal equations directly into the parent.
  void updateDirectFactors(SymmetricBlockMatrix& targetInfo,
                           const std::vector<DenseIndex>& targetIndices,
                           bool useParentMappedSlots) const;

  /// Accumulate children separator updates into this clique's info matrix
  /// (single-threaded).
  void gatherUpdatesSequential();

  /// Accumulate children separator updates into this clique's info matrix
  /// (multi-threaded).
  void gatherUpdatesParallel(size_t numThreads);

  /// Accumulate identical-separator Cholesky leaves, then scatter once/group.
  void gatherSameSeparatorUpdates();

  /// Compute block dimensions from variable dimensions (excluding RHS).
  std::vector<size_t> blockDims(const KeyDimMap& dims,
                                const KeyVector& frontals,
                                const KeySet& separatorKeys) const;

  /// Apply damping for QR elimination by writing into extra Ab_ rows.
  void applyDampingQR(double lambda, const LMDampingParams& dampingParams,
                      const VectorValues& exactHessianDiagonal);

  /// Apply damping for Cholesky elimination by adding to the info_ matrix.
  void applyDampingCholesky(double lambda, const LMDampingParams& dampingParams,
                            const VectorValues& exactHessianDiagonal);

  /**
   * Add a Jacobian factor's contributions into the Ab matrix.
   * @return Number of rows added.
   */
  size_t addJacobianFactor(const JacobianFactor& factor, size_t rowOffset,
                           const FactorLoadPlan& plan);

  /**
   * Add a compact batch Jacobian factor's contributions into the Ab matrix.
   * @return Number of rows added.
   */
  size_t addBatchJacobianFactor(const BatchJacobianFactorBase& factor,
                                size_t rowOffset, const FactorLoadPlan& plan);

  void setParentIndices(const std::vector<DenseIndex>& indices,
                        const std::vector<DenseIndex>& scalarOffsets) {
    parentIndices_ = indices;
    parentScalarOffsets_ = scalarOffsets;
  }

  // Construction-time metadata (set once in the constructor).
  std::vector<size_t> factorIndices_;
  KeyVector orderedKeys_;     ///< Keys ordered by block index (frontals+seps).
  size_t totalFrontals_ = 0;  ///< Symbolic frontals before a phase split.
  const std::unordered_set<Key>* fixedKeys_ = nullptr;
  std::unordered_map<Key, DenseIndex> blockIndexCache_;
  std::vector<Vector*> frontalPtrs_;          ///< Solution frontals.
  std::vector<const Vector*> separatorPtrs_;  ///< Solution separator.
  std::vector<size_t> blockDims_;  ///< Cached block dimensions (excluding RHS).
  size_t factorRows_ = 0;  ///< Total symbolic factor rows before packing.
  mutable const GaussianFactorGraph* activeLoadGraph_ = nullptr;
  mutable bool allBatchFactors_ = false;
  mutable bool hasDirectBatchFactors_ = false;
  mutable std::vector<FactorLoadPlan> loadPlans_;
  mutable bool loadPlansBuilt_ = false;
  mutable size_t materializedRows_ = 0;  ///< Packed fallback rows in Ab_.
  mutable size_t fillRows_ = 0;

  // Finalize-time metadata (set once after children are known).
  std::vector<DenseIndex>
      parentIndices_;  ///< Parent block indices for separators + RHS.
  std::vector<DenseIndex>
      parentScalarOffsets_;  ///< Cached scalar offsets for parent blocks.
  std::vector<DenseIndex> separatorIndices_;  ///< Identity separator mapping.
  std::vector<DenseIndex>
      separatorScalarOffsets_;  ///< Cached separator-local scalar offsets.
  SolveMode solveMode_ = SolveMode::Cholesky;
  SolveMode starFallbackMode_ = SolveMode::Cholesky;
  /// Whether one automatic-QR star awaits inspection of its sole factor.
  bool deferredSingleFactorAutoQr_ = false;
  std::vector<DenseIndex> parentSchurScalarOffsets_;
  std::vector<DenseIndex> separatorSchurScalarOffsets_;

  std::vector<std::vector<size_t>> sameSeparatorChildGroups_;
  std::vector<std::vector<DenseIndex>> sameSeparatorParentIndices_;
  std::vector<std::vector<DenseIndex>> sameSeparatorParentScalarOffsets_;
  std::vector<SymmetricBlockMatrix> sameSeparatorInfos_;
  std::vector<uint8_t> childInSameSeparatorGroup_;

  struct ParentGatherPlan;
  // Built after deferred modes resolve and reused across numerical reloads.
  std::shared_ptr<ParentGatherPlan> parentGatherPlan_;

  // Lazily allocated after load-plan construction. Direct batch factors need
  // no rows; QR additionally reserves frontal damping rows.
  VerticalBlockMatrix Ab_;

  // Cached factorization storage reused by parent updates and result exports.
  mutable VerticalBlockMatrix RSd_;  ///< Cached [R S d] from elimination.
  mutable SymmetricBlockMatrix info_;

  // Elimination-time state.
  bool RSdReady_ = false;
  bool infoReady_ = false;

  // Solve-time scratch space.
  Vector rhsScratch_;        ///< Cached RHS workspace for back-substitution.
  Vector separatorScratch_;  ///< Cached separator stack for back-substitution.

  // Solve-time cached error contributions.
  double lastOldError_ = 0.0;
  double lastNewError_ = 0.0;
};

std::ostream& operator<<(std::ostream& os, const MultifrontalClique& clique);

}  // namespace gtsam
