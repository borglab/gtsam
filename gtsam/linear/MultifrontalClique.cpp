/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file MultifrontalClique.cpp
 * @brief Implementation of imperative multifrontal clique data structure.
 * @author Frank Dellaert
 * @date   December 2025
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/config.h>
#include <gtsam/linear/BatchJacobianFactor.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/MultifrontalClique.h>
#include <gtsam/linear/MultifrontalSolver.h>

#include <Eigen/Cholesky>

#ifdef GTSAM_USE_TBB
#include <tbb/blocked_range.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#endif

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <thread>

namespace gtsam {

namespace {

// Print keys in [start, end) using the provided formatter.
void printKeyRange(std::ostream& os, const KeyVector& keys, size_t start,
                   size_t end, const KeyFormatter& formatter) {
  os << "[";
  for (size_t i = start; i < end; ++i) {
    os << formatter(keys[i]);
    if (i + 1 < end) os << ", ";
  }
  os << "]";
}

// Build a stacked separator vector x_sep in the provided scratch buffer.
Vector& buildSeparatorVector(const std::vector<const Vector*>& separatorPtrs,
                             Vector* scratch) {
  size_t offset = 0;
  for (const Vector* values : separatorPtrs) {
    scratch->segment(offset, values->size()) = *values;
    offset += values->size();
  }
  return *scratch;
}

#ifndef NDEBUG
bool containsKey(const KeyVector& orderedKeys, Key key) {
  return std::find(orderedKeys.begin(), orderedKeys.end(), key) !=
         orderedKeys.end();
}

bool validateFactorKeys(const GaussianFactorGraph& graph,
                        const std::vector<size_t>& factorIndices,
                        const KeyVector& orderedKeys,
                        const std::unordered_set<Key>* fixedKeys) {
  for (size_t index : factorIndices) {
    assert(index < graph.size());
    const GaussianFactor::shared_ptr& gf = graph[index];
    if (!gf) continue;
    for (Key key : gf->keys()) {
      if (containsKey(orderedKeys, key)) continue;
      if (fixedKeys && fixedKeys->count(key)) continue;
      return false;
    }
  }
  return true;
}
#endif

size_t hardwareThreads() {
  static const size_t kHardwareThreads = [] {
    size_t n = std::thread::hardware_concurrency();
    return n == 0 ? size_t{1} : n;
  }();
  return kHardwareThreads;
}

SymmetricBlockMatrix makeZeroLocalInfo(const std::vector<size_t>& blockDims) {
  SymmetricBlockMatrix local(blockDims, true);
  local.setZero();
  return local;
}

}  // namespace

namespace {
// Global toggle for the direct BatchJacobianFactor hessian path.
// The path is documented in `linear/doc/BatchFactor_Performance_Notes.html`.
constexpr bool kUseDirectBatchHessianUpdate = true;
}  // namespace

MultifrontalClique::MultifrontalClique(
    std::vector<size_t> factorIndices,
    const std::weak_ptr<MultifrontalClique>& parent, const KeyVector& frontals,
    const KeySet& separatorKeys, const KeyDimMap& dims, size_t vbmRows,
    VectorValues* solution, const std::unordered_set<Key>* fixedKeys,
    size_t numEliminatedFrontals) {
  factorIndices_ = std::move(factorIndices);
  this->parent = parent;
  fixedKeys_ = fixedKeys;

  if (frontals.empty()) {
    throw std::runtime_error(
        "MultifrontalSolver: cluster has no frontal keys.");
  }
  if (numEliminatedFrontals > frontals.size()) {
    throw std::invalid_argument(
        "MultifrontalClique: eliminated frontal count exceeds clique size.");
  }
  totalFrontals_ = frontals.size();

  // Cache keys in block order for fast linear lookup in small cliques.
  orderedKeys_.clear();
  orderedKeys_.reserve(frontals.size() + separatorKeys.size());
  orderedKeys_.insert(orderedKeys_.end(), frontals.begin(), frontals.end());
  orderedKeys_.insert(orderedKeys_.end(), separatorKeys.begin(),
                      separatorKeys.end());
  blockIndexCache_.reserve(orderedKeys_.size());
  for (DenseIndex i = 0; i < static_cast<DenseIndex>(orderedKeys_.size());
       ++i) {
    blockIndexCache_.emplace(orderedKeys_[i], i);
  }

  // Cache total frontal/separator dimensions for scheduling and sizing.
  frontalDim = 0;
  separatorDim = internal::sumDims(dims, separatorKeys);
  for (size_t i = 0; i < frontals.size(); ++i) {
    const size_t dim = dims.at(frontals[i]);
    if (i < numEliminatedFrontals)
      frontalDim += dim;
    else
      separatorDim += dim;
  }

  rhsScratch_.resize(frontalDim);
  separatorScratch_.resize(separatorDim);

  // Cache pointers into the solution for fast back-substitution.
  KeyVector eliminatedFrontals(frontals.begin(),
                               frontals.begin() + numEliminatedFrontals);
  KeyVector retainedKeys(frontals.begin() + numEliminatedFrontals,
                         frontals.end());
  retainedKeys.insert(retainedKeys.end(), separatorKeys.begin(),
                      separatorKeys.end());
  cacheSolutionPointers(solution, eliminatedFrontals, retainedKeys);

  // Cache sizing for allocation at finalize time.
  blockDims_ = this->blockDims(dims, frontals, separatorKeys);
  factorRows_ = vbmRows;
}

void MultifrontalClique::finalize(std::vector<ChildInfo> children,
                                  const MultifrontalParameters& params) {
  this->children.clear();
  this->children.reserve(children.size());
  for (const auto& child : children) {
    this->children.push_back(child.clique);
  }

  // Compute parent indices for all children (separator blocks + RHS block).
  for (const auto& child : children) {
    if (!child.clique) continue;
    std::vector<DenseIndex> indices;
    indices.reserve(child.separatorKeys.size() + 1);
    for (Key key : child.separatorKeys) {
      indices.push_back(blockIndex(key));
    }
    // The RHS block is always the last block in Ab/info.
    indices.push_back(static_cast<DenseIndex>(orderedKeys_.size()));
    child.clique->setParentIndices(indices);
  }

  // In leaf cliques, check whether to use QR elimination.
  const bool isLeaf = this->children.empty();
  const bool eliminatesWholeClique = fullyEliminated();
  const bool hasRows =
      frontalDim > 0 && factorRows_ >= static_cast<size_t>(frontalDim);
  bool useQR = false;
  if (params.qrMode == MultifrontalParameters::QRMode::Allow) {
    useQR = eliminatesWholeClique && isLeaf && hasRows &&
            (frontalDim + separatorDim > params.qrAspectRatio * frontalDim);
  } else if (params.qrMode == MultifrontalParameters::QRMode::Force) {
    useQR = eliminatesWholeClique && isLeaf && hasRows;
  }
  // Below this size the ordinary dense clique update is faster; above it,
  // avoiding the quadratic separator allocation and writeback dominates.
  const bool useCompactCholesky =
      !useQR && eliminatesWholeClique && isLeaf && !parent.expired() &&
      separatorDim >= params.compactCholeskySeparatorDimThreshold &&
      separatorDim > frontalDim;
  solveMode_ = useQR ? SolveMode::QrLeaf
                     : (useCompactCholesky ? SolveMode::CompactCholeskyLeaf
                                           : SolveMode::Cholesky);
  RSdReady_ = false;

  // If using QR, also reserve room for optional damping rows.
  const DenseIndex baseRows = static_cast<DenseIndex>(factorRows_);
  const DenseIndex totalRows =
      baseRows + (useQR ? static_cast<DenseIndex>(frontalDim) : 0);
  Ab_ = VerticalBlockMatrix(blockDims_, totalRows, true);
  // Ab's structure is fixed; clear it once and reuse across loads.
  Ab_.matrix().setZero();
  if (useQR) {
    RSd_ = VerticalBlockMatrix(blockDims_, totalRows, true);
  } else {
    RSd_ = VerticalBlockMatrix(blockDims_, static_cast<DenseIndex>(frontalDim),
                               true);
    if (!useCompactCholesky) {
      info_ = SymmetricBlockMatrix(blockDims_, true);
    }
  }
}

DenseIndex MultifrontalClique::blockIndex(Key key) const {
  const auto it = blockIndexCache_.find(key);
  assert(it != blockIndexCache_.end());
  return it->second;
}

void MultifrontalClique::cacheSolutionPointers(VectorValues* solution,
                                               const KeyVector& frontals,
                                               const KeyVector& separatorKeys) {
  frontalPtrs_.clear();
  separatorPtrs_.clear();
  frontalPtrs_.reserve(frontals.size());
  separatorPtrs_.reserve(separatorKeys.size());
  for (Key key : frontals) {
    frontalPtrs_.push_back(&solution->at(key));
  }
  for (Key key : separatorKeys) {
    separatorPtrs_.push_back(&solution->at(key));
  }
}

std::vector<size_t> MultifrontalClique::blockDims(
    const KeyDimMap& dims, const KeyVector& frontals,
    const KeySet& separatorKeys) const {
  std::vector<size_t> blockDims;
  blockDims.reserve(frontals.size() + separatorKeys.size());
  for (Key k : frontals) blockDims.push_back(dims.at(k));
  for (Key k : separatorKeys) blockDims.push_back(dims.at(k));
  return blockDims;
}

size_t MultifrontalClique::addJacobianFactor(const JacobianFactor& factor,
                                             size_t rowOffset,
                                             const FactorLoadPlan& plan) {
  const size_t rhsBlockIdx = Ab_.nBlocks() - 1;
  auto blockIt = plan.blockIndices.begin();
  for (auto factorIt = factor.begin(); factorIt != factor.end();
       ++factorIt, ++blockIt) {
    assert(blockIt != plan.blockIndices.end());
    const auto blockIdx = *blockIt;
    if (blockIdx >= 0) {
      Ab_(blockIdx).middleRows(rowOffset, plan.rows) = factor.getA(factorIt);
    }
  }
  assert(blockIt == plan.blockIndices.end());

  Ab_(rhsBlockIdx).middleRows(rowOffset, plan.rows) = factor.getb();
  if (plan.model && !plan.model->isConstrained()) {
    plan.model->WhitenInPlace(Ab_.matrix().middleRows(rowOffset, plan.rows));
  }
  return plan.rows;
}

size_t MultifrontalClique::addBatchJacobianFactor(
    const BatchJacobianFactorBase& batchFactor, size_t rowOffset,
    const FactorLoadPlan& plan) {
  const size_t rows =
      batchFactor.scatterInto(Ab_, rowOffset, plan.blockIndices);
  if (plan.model && !plan.model->isConstrained()) {
    plan.model->WhitenInPlace(Ab_.matrix().middleRows(rowOffset, rows));
  }
  return rows;
}

void MultifrontalClique::buildLoadPlans(
    const GaussianFactorGraph& graph) const {
  if (loadPlansBuilt_) return;
  loadPlans_.clear();
  loadPlans_.reserve(factorIndices_.size());
  allBatchFactors_ = true;
  hasDirectBatchFactors_ = false;
  size_t rowOffset = 0;
  for (size_t factorIndex : factorIndices_) {
    assert(factorIndex < graph.size());
    const GaussianFactor::shared_ptr& factor = graph[factorIndex];
    if (!factor) {
      FactorLoadPlan emptyPlan;
      emptyPlan.kind = FactorLoadKind::Jacobian;
      emptyPlan.factorIndex = factorIndex;
      loadPlans_.push_back(std::move(emptyPlan));
      allBatchFactors_ = false;
      continue;
    }

    FactorLoadPlan plan;
    plan.factorIndex = factorIndex;
    plan.rowOffset = rowOffset;
    const JacobianFactor* jacobianFactor =
        dynamic_cast<JacobianFactor*>(factor.get());
    if (jacobianFactor) {
      plan.kind = FactorLoadKind::Jacobian;
      allBatchFactors_ = false;
      plan.rows = jacobianFactor->rows();
      rowOffset += plan.rows;
      plan.model = jacobianFactor->get_model();
      for (Key key : jacobianFactor->keys()) {
        if (fixedKeys_ && fixedKeys_->count(key)) {
          plan.blockIndices.push_back(-1);
        } else {
          plan.blockIndices.push_back(blockIndex(key));
        }
      }
      assert(plan.blockIndices.size() == jacobianFactor->keys().size());
      loadPlans_.push_back(std::move(plan));
      continue;
    }

    const BatchJacobianFactorBase* batchFactor =
        dynamic_cast<BatchJacobianFactorBase*>(factor.get());
    if (batchFactor) {
      bool hasFixedKey = false;
      for (Key key : batchFactor->keys()) {
        if (fixedKeys_ && fixedKeys_->count(key)) {
          hasFixedKey = true;
          break;
        }
      }
      if (hasFixedKey) {
        allBatchFactors_ = false;
      }
      plan.kind = FactorLoadKind::Batch;
      plan.rows = batchFactor->rows();
      rowOffset += plan.rows;
      plan.model = batchFactor->get_model();
      plan.canDirectUpdate =
          !hasFixedKey && !(plan.model && plan.model->isConstrained());
      if (!plan.canDirectUpdate) {
        allBatchFactors_ = false;
      }
      for (Key key : batchFactor->keys()) {
        if (fixedKeys_ && fixedKeys_->count(key)) {
          plan.blockIndices.push_back(-1);
        } else {
          plan.blockIndices.push_back(blockIndex(key));
        }
      }
      assert(plan.blockIndices.size() == batchFactor->keys().size());
      plan.blockIndicesWithRhs = plan.blockIndices;
      plan.blockIndicesWithRhs.push_back(
          static_cast<DenseIndex>(orderedKeys_.size()));
      if (plan.canDirectUpdate) {
        batchFactor->buildMappedSlots(plan.blockIndicesWithRhs,
                                      plan.mappedSlots);
        if (useCompactCholesky()) {
          assert(parentIndices_.size() ==
                 orderedKeys_.size() - numFrontals() + 1);
          std::vector<DenseIndex> parentSlots;
          parentSlots.reserve(plan.blockIndices.size() + 1);
          for (const DenseIndex localIndex : plan.blockIndices) {
            if (localIndex < static_cast<DenseIndex>(numFrontals())) {
              parentSlots.push_back(-1);
            } else {
              parentSlots.push_back(parentIndices_.at(
                  localIndex - static_cast<DenseIndex>(numFrontals())));
            }
          }
          parentSlots.push_back(parentIndices_.back());
          batchFactor->buildMappedSlots(parentSlots, plan.parentMappedSlots);
        }
      }
      hasDirectBatchFactors_ |= plan.canDirectUpdate;
      loadPlans_.push_back(std::move(plan));
      continue;
    }

    throw MultifrontalSolverNotSupported(
        "only JacobianFactor or BatchJacobianFactor inputs are supported");
  }
  assert(loadPlans_.size() == factorIndices_.size());
  loadPlansBuilt_ = true;
}

void MultifrontalClique::fillAb(const GaussianFactorGraph& graph) {
  activeLoadGraph_ = &graph;
  assert(validateFactorKeys(graph, factorIndices_, orderedKeys_, fixedKeys_));
  RSdReady_ = false;
  infoReady_ = false;

  if (!loadPlansBuilt_) {
    buildLoadPlans(graph);
  }

  assert(loadPlans_.size() == factorIndices_.size());
  fillRows_ = 0;
  if (allBatchFactors_ && !useQR() && !useCompactCholesky() &&
      kUseDirectBatchHessianUpdate) {
    // All-factor direct mode intentionally skips materializing Ab rows and
    // writes directly into info_. This is the highest-impact branch in the doc.
    fillRows_ = 0;
    RSdReady_ = false;
    assert((useQR() && RSd_.matrix().rows() ==
                           static_cast<DenseIndex>(Ab_.matrix().rows())) ||
           (RSd_.matrix().rows() == static_cast<DenseIndex>(frontalDim)));
    return;
  }

  if (useCompactCholesky()) {
    for (const FactorLoadPlan& plan : loadPlans_) {
      const auto* factor = graph[plan.factorIndex].get();
      if (!factor) continue;
      if (plan.kind == FactorLoadKind::Batch && plan.canDirectUpdate &&
          kUseDirectBatchHessianUpdate) {
        // The compact batch API forms [A B a] directly from row-sparse blocks.
        continue;
      }
      if (plan.kind == FactorLoadKind::Jacobian) {
        const auto* jacobianFactor =
            dynamic_cast<const JacobianFactor*>(factor);
        assert(jacobianFactor);
        addJacobianFactor(*jacobianFactor, plan.rowOffset, plan);
      } else {
        const auto* batchFactor =
            dynamic_cast<const BatchJacobianFactorBase*>(factor);
        assert(batchFactor);
        addBatchJacobianFactor(*batchFactor, plan.rowOffset, plan);
      }
    }
    fillRows_ = factorRows_;
    return;
  }

  size_t rowOffset = 0;
  for (const FactorLoadPlan& plan : loadPlans_) {
    const auto* factor = graph[plan.factorIndex].get();
    if (!factor) continue;
    if (plan.kind == FactorLoadKind::Jacobian) {
      auto* jacobianFactor = dynamic_cast<const JacobianFactor*>(factor);
      assert(jacobianFactor);
      rowOffset += addJacobianFactor(*jacobianFactor, rowOffset, plan);
      continue;
    }
    if (plan.kind == FactorLoadKind::Batch) {
      auto* batchFactor = dynamic_cast<const BatchJacobianFactorBase*>(factor);
      assert(batchFactor);
      if (plan.canDirectUpdate && kUseDirectBatchHessianUpdate && !useQR() &&
          !useCompactCholesky()) {
        // Batch direct path: avoid Jacobian expansion work here and defer to
        // prepareForElimination(). See the performance doc for batching
        // details.
        continue;
      }
      rowOffset += addBatchJacobianFactor(*batchFactor, rowOffset, plan);
      continue;
    }
    throw MultifrontalSolverNotSupported(
        "only JacobianFactor or BatchJacobianFactor inputs are supported");
  }

  assert((useQR() && RSd_.matrix().rows() ==
                         static_cast<DenseIndex>(Ab_.matrix().rows())) ||
         (RSd_.matrix().rows() == static_cast<DenseIndex>(frontalDim)));
  assert(useQR() || useCompactCholesky() || info_.nBlocks() > 0);
  fillRows_ = rowOffset;
}

void MultifrontalClique::eliminateInPlace() {
  prepareForElimination();
  // There is no damping here and although we do not assert it here, we are
  // counting on the facts that no damping was *ever* applied during the
  // existence of this clique, just like we count on the sparsity pattern of
  // zeros to remain the same.
  factorize();
}

void MultifrontalClique::prepareForElimination() {
  // QR leaf cliques skip info matrix assembly entirely.
  if (useQR()) return;
  if (useCompactCholesky()) {
    prepareCompactCholesky();
    return;
  }
  assert(info_.nBlocks() > 0);
  info_.setZero();

  const bool canUseDirectBatchHessianUpdate =
      kUseDirectBatchHessianUpdate && activeLoadGraph_ && !useQR();
  if (canUseDirectBatchHessianUpdate &&
      (allBatchFactors_ || hasDirectBatchFactors_)) {
    // Direct update path for batch factors:
    // precomputed mapped slots let each row group update info_ directly.
    // This avoids repeated key lookup and dense block materialization.
    assert(loadPlans_.size() == factorIndices_.size());
    for (const FactorLoadPlan& plan : loadPlans_) {
      if (!plan.canDirectUpdate || plan.kind != FactorLoadKind::Batch) {
        continue;
      }
      const auto* factor = (*activeLoadGraph_)[plan.factorIndex].get();
      if (!factor) continue;
      const auto* batchFactor =
          dynamic_cast<const BatchJacobianFactorBase*>(factor);
      if (!batchFactor) continue;
      if (plan.mappedSlots.empty()) {
        batchFactor->updateHessian(plan.blockIndicesWithRhs, &info_);
      } else {
        batchFactor->updateHessianWithMappedSlots(plan.mappedSlots, &info_);
      }
    }
    if (fillRows_ > 0 && !allBatchFactors_) {
      info_.selfadjointView().rankUpdate(
          Ab_.matrix().topRows(static_cast<DenseIndex>(fillRows_)).transpose());
    }
  } else if (Ab_.matrix().rows() > 0 && fillRows_ > 0) {
    info_.selfadjointView().rankUpdate(
        Ab_.matrix().topRows(static_cast<DenseIndex>(fillRows_)).transpose());
  }

  // Heuristic: avoid parallel overhead on small cliques.
  const size_t minChildren =
      std::max<size_t>(1024, 4 * static_cast<size_t>(info_.rows()));
  const size_t numChildren = children.size();
  if (numChildren < minChildren) {  // Typical for chains: many small cliques.
    gatherUpdatesSequential();
  } else {
    // Cap by available work.
    // Parallel path (TBB if available, else std::thread).
    const size_t numThreads = std::min(hardwareThreads(), numChildren);
    if (numThreads <= 1)
      gatherUpdatesSequential();
    else
      gatherUpdatesParallel(numThreads);
  }
  infoReady_ = true;
}

void MultifrontalClique::prepareCompactCholesky() {
  assert(fullyEliminated());
  assert(children.empty());
  assert(RSd_.matrix().rows() == static_cast<DenseIndex>(frontalDim));
  assert(loadPlans_.size() == factorIndices_.size());
  RSd_.matrix().setZero();

  const DenseIndex nf = static_cast<DenseIndex>(numFrontals());
  const DenseIndex rhsBlock = RSd_.nBlocks() - 1;
  for (const FactorLoadPlan& plan : loadPlans_) {
    if (plan.rows == 0) continue;
    if (plan.kind == FactorLoadKind::Batch && plan.canDirectUpdate &&
        kUseDirectBatchHessianUpdate) {
      assert(activeLoadGraph_ && !plan.mappedSlots.empty());
      const auto* factor = (*activeLoadGraph_)[plan.factorIndex].get();
      const auto* batchFactor =
          dynamic_cast<const BatchJacobianFactorBase*>(factor);
      assert(batchFactor);
      batchFactor->updateFrontalHessianWithMappedSlots(plan.mappedSlots, nf,
                                                       &RSd_);
      continue;
    }
    for (const DenseIndex I : plan.blockIndices) {
      if (I < 0 || I >= nf) continue;
      const DenseIndex row = RSd_.offset(I);
      const DenseIndex dim = static_cast<DenseIndex>(blockDims_.at(I));
      const auto Ai = Ab_(I).middleRows(static_cast<DenseIndex>(plan.rowOffset),
                                        static_cast<DenseIndex>(plan.rows));
      for (const DenseIndex J : plan.blockIndices) {
        if (J < 0) continue;
        const auto Aj =
            Ab_(J).middleRows(static_cast<DenseIndex>(plan.rowOffset),
                              static_cast<DenseIndex>(plan.rows));
        RSd_(J).middleRows(row, dim).noalias() += Ai.transpose() * Aj;
      }
      const auto b =
          Ab_(rhsBlock).middleRows(static_cast<DenseIndex>(plan.rowOffset),
                                   static_cast<DenseIndex>(plan.rows));
      RSd_(rhsBlock).middleRows(row, dim).noalias() += Ai.transpose() * b;
    }
  }
}

void MultifrontalClique::factorize() {
  if (useQR()) {
    // Copy Ab_ to preserve its invariant; QR writes in place.
    assert(RSd_.matrix().rows() == Ab_.matrix().rows());
    assert(RSd_.matrix().cols() == Ab_.matrix().cols());
    RSd_.matrix() = Ab_.matrix();
    inplace_QR(RSd_.matrix());
    assert(RSd_.rowStart() == 0);
    RSd_.rowEnd() = static_cast<DenseIndex>(frontalDim);
  } else if (useCompactCholesky()) {
    factorizeCompactCholesky();
  } else {
    info_.choleskyPartial(numFrontals());
    info_.split(numFrontals(), &RSd_);
    info_.blockStart() = 0;
  }
  RSdReady_ = true;
}

void MultifrontalClique::factorizeCompactCholesky() {
  const DenseIndex n = static_cast<DenseIndex>(frontalDim);
  auto A = RSd_.matrix().topLeftCorner(n, n);
  if (!A.diagonal().allFinite() || (A.diagonal().array() <= 0.0).any()) {
    throw CholeskyFailed();
  }
  Eigen::LLT<Matrix, Eigen::Upper> llt(A);
  if (llt.info() != Eigen::Success) throw CholeskyFailed();
  const Matrix R = llt.matrixU();
  constexpr double kMinimumNormalizedPivotSquared = 1.0 / (4096.0 * 4096.0);
  for (DenseIndex i = 0; i < n; ++i) {
    const double pivot = R(i, i);
    if (!std::isfinite(pivot) ||
        pivot * pivot < kMinimumNormalizedPivotSquared * A(i, i)) {
      throw CholeskyFailed();
    }
  }
  A = R;
  A.template triangularView<Eigen::StrictlyLower>().setZero();
  if (n < RSd_.matrix().cols()) {
    auto B = RSd_.matrix().rightCols(RSd_.matrix().cols() - n);
    A.template triangularView<Eigen::Upper>().transpose().solveInPlace(B);
  }
}

void MultifrontalClique::eliminateInPlace(
    double lambda, const LMDampingParams& dampingParams,
    const VectorValues& exactHessianDiagonal) {
  prepareForElimination();
  if (useQR()) {
    applyDampingQR(lambda, dampingParams, exactHessianDiagonal);
  } else {
    applyDampingCholesky(lambda, dampingParams, exactHessianDiagonal);
  }
  factorize();
}

void MultifrontalClique::applyDampingQR(
    double lambda, const LMDampingParams& dampingParams,
    const VectorValues& exactHessianDiagonal) {
  if (lambda <= 0.0) return;
  const DenseIndex baseRows = static_cast<DenseIndex>(factorRows_);
  const DenseIndex dampRows = static_cast<DenseIndex>(frontalDim);
  assert(Ab_.matrix().rows() >= baseRows + dampRows);
  Ab_.matrix().middleRows(baseRows, dampRows).setZero();
  DenseIndex rowOffset = baseRows;
  for (size_t j = 0; j < numFrontals(); ++j) {
    const DenseIndex blockIndex = static_cast<DenseIndex>(j);
    const DenseIndex dim = static_cast<DenseIndex>(blockDims_.at(j));
    auto block = Ab_(blockIndex).middleRows(rowOffset, dim);
    if (dampingParams.diagonalDamping) {
      Vector diag;
      if (dampingParams.exactHessianDiagonal) {
        const Key key = orderedKeys_.at(j);
        diag = exactHessianDiagonal.at(key)
                   .cwiseMax(dampingParams.minDiagonal)
                   .cwiseMin(dampingParams.maxDiagonal);
      } else {
        diag = Ab_(blockIndex)
                   .topRows(baseRows)
                   .array()
                   .square()
                   .colwise()
                   .sum()
                   .transpose();
        diag = diag.cwiseMax(dampingParams.minDiagonal)
                   .cwiseMin(dampingParams.maxDiagonal);
      }
      block.diagonal() = (diag.array() * lambda).sqrt().matrix();
    } else {
      block.diagonal().setConstant(std::sqrt(lambda));
    }
    rowOffset += dim;
  }
}

void MultifrontalClique::applyDampingCholesky(
    double lambda, const LMDampingParams& dampingParams,
    const VectorValues& exactHessianDiagonal) {
  if (lambda <= 0.0) return;
  if (dampingParams.diagonalDamping) {
    if (dampingParams.exactHessianDiagonal) {
      addExactDiagonalDamping(lambda, exactHessianDiagonal,
                              dampingParams.minDiagonal,
                              dampingParams.maxDiagonal);
    } else {
      addDiagonalDamping(lambda, dampingParams.minDiagonal,
                         dampingParams.maxDiagonal);
    }
  } else {
    addIdentityDamping(lambda);
  }
}

void MultifrontalClique::addIdentityDamping(double lambda) {
  const size_t nf = numFrontals();
  for (size_t j = 0; j < nf; ++j) {
    if (useCompactCholesky()) {
      const DenseIndex row = RSd_.offset(static_cast<DenseIndex>(j));
      const DenseIndex dim = static_cast<DenseIndex>(blockDims_.at(j));
      RSd_(static_cast<DenseIndex>(j))
          .middleRows(row, dim)
          .diagonal()
          .array() += lambda;
    } else {
      info_.addScaledIdentity(j, lambda);
    }
  }
}

void MultifrontalClique::addDiagonalDamping(double lambda, double minDiagonal,
                                            double maxDiagonal) {
  const size_t nf = numFrontals();
  for (size_t j = 0; j < nf; ++j) {
    if (useCompactCholesky()) {
      const DenseIndex index = static_cast<DenseIndex>(j);
      const DenseIndex row = RSd_.offset(index);
      const DenseIndex dim = static_cast<DenseIndex>(blockDims_.at(j));
      auto diagonal = RSd_(index).middleRows(row, dim).diagonal();
      const Vector scaled =
          lambda * diagonal.cwiseMax(minDiagonal).cwiseMin(maxDiagonal);
      diagonal += scaled;
    } else {
      const Vector scaled =
          lambda *
          info_.diagonal(j).cwiseMax(minDiagonal).cwiseMin(maxDiagonal);
      info_.addToDiagonalBlock(j, scaled);
    }
  }
}

void MultifrontalClique::addExactDiagonalDamping(
    double lambda, const VectorValues& hessianDiagonal, double minDiagonal,
    double maxDiagonal) {
  const size_t nf = numFrontals();
  for (size_t j = 0; j < nf; ++j) {
    const Key key = orderedKeys_.at(j);
    const Vector diag =
        hessianDiagonal.at(key).cwiseMax(minDiagonal).cwiseMin(maxDiagonal);
    if (useCompactCholesky()) {
      const DenseIndex index = static_cast<DenseIndex>(j);
      const DenseIndex row = RSd_.offset(index);
      const DenseIndex dim = static_cast<DenseIndex>(blockDims_.at(j));
      RSd_(index).middleRows(row, dim).diagonal() += lambda * diag;
    } else {
      info_.addToDiagonalBlock(j, lambda * diag);
    }
  }
}

void MultifrontalClique::updateParentDirectFactors(
    SymmetricBlockMatrix& parentInfo) const {
  assert(activeLoadGraph_);
  const DenseIndex nf = static_cast<DenseIndex>(numFrontals());
  const DenseIndex rhsBlock = Ab_.nBlocks() - 1;
  const DenseIndex parentRhs = parentIndices_.back();

  for (const FactorLoadPlan& plan : loadPlans_) {
    if (plan.rows == 0) continue;
    const auto* factor = (*activeLoadGraph_)[plan.factorIndex].get();
    if (!factor) continue;
    if (plan.kind == FactorLoadKind::Batch && plan.canDirectUpdate &&
        !plan.parentMappedSlots.empty()) {
      const auto* batchFactor =
          dynamic_cast<const BatchJacobianFactorBase*>(factor);
      assert(batchFactor);
      batchFactor->updateHessianWithMappedSlots(plan.parentMappedSlots,
                                                &parentInfo);
      continue;
    }

    const DenseIndex row = static_cast<DenseIndex>(plan.rowOffset);
    const DenseIndex rows = static_cast<DenseIndex>(plan.rows);
    const auto b = Ab_(rhsBlock).middleRows(row, rows);
    parentInfo.updateDiagonalBlock(parentRhs, b.transpose() * b);
    for (size_t p = 0; p < plan.blockIndices.size(); ++p) {
      const DenseIndex I = plan.blockIndices[p];
      if (I < nf) continue;
      const DenseIndex parentI = parentIndices_.at(I - nf);
      const auto Ai = Ab_(I).middleRows(row, rows);
      parentInfo.updateDiagonalBlock(parentI, Ai.transpose() * Ai);
      parentInfo.updateOffDiagonalBlock(parentI, parentRhs, Ai.transpose() * b);
      for (size_t q = p + 1; q < plan.blockIndices.size(); ++q) {
        const DenseIndex J = plan.blockIndices[q];
        if (J < nf) continue;
        const DenseIndex parentJ = parentIndices_.at(J - nf);
        const auto Aj = Ab_(J).middleRows(row, rows);
        parentInfo.updateOffDiagonalBlock(parentI, parentJ,
                                          Ai.transpose() * Aj);
      }
    }
  }
}

void MultifrontalClique::updateParentInfo(
    SymmetricBlockMatrix& parentInfo) const {
  assert(fullyEliminated());
  assert(RSd_.rowStart() == 0);
  if (useQR()) {
    // Accumulate separator (and RHS) normal equations from the QR residual.
    assert(RSdReady_ && RSd_.firstBlock() == 0);
    const DenseIndex nfBlocks = static_cast<DenseIndex>(numFrontals());
    const DenseIndex rowStart = RSd_.rowStart();
    const DenseIndex rowEnd = RSd_.rowEnd();
    RSd_.rowStart() = static_cast<DenseIndex>(frontalDim);
    RSd_.rowEnd() = static_cast<DenseIndex>(RSd_.matrix().rows());
    RSd_.firstBlock() = nfBlocks;
    parentInfo.updateFromOuterProductBlocks(RSd_, parentIndices_);
    RSd_.firstBlock() = 0;
    RSd_.rowStart() = rowStart;
    RSd_.rowEnd() = rowEnd;
  } else if (useCompactCholesky()) {
    assert(RSdReady_ && RSd_.firstBlock() == 0);
    updateParentDirectFactors(parentInfo);
    RSd_.firstBlock() = static_cast<DenseIndex>(numFrontals());
    parentInfo.updateFromOuterProductBlocks(RSd_, parentIndices_, -1.0);
    RSd_.firstBlock() = 0;
  } else {
    // Accumulate the S^T S part from this clique's info matrix into the parent.
    assert(info_.nBlocks() > 0 && info_.blockStart() == 0);
    info_.blockStart() = numFrontals();
    parentInfo.updateFromMappedBlocks(info_, parentIndices_);
    info_.blockStart() = 0;
  }
}

void MultifrontalClique::gatherUpdatesSequential() {
  for (const auto& child : children) {
    assert(child);
    if (child->fullyEliminated()) child->updateParentInfo(info_);
  }
}

void MultifrontalClique::gatherUpdatesParallel(size_t numThreads) {
#ifdef GTSAM_USE_TBB
  (void)numThreads;  // TBB controls the effective worker count.
  tbb::enumerable_thread_specific<SymmetricBlockMatrix> locals([this]() {
    return makeZeroLocalInfo(blockDims_);
  });  // Per-thread accumulators.
  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, children.size()),
      [&](const tbb::blocked_range<size_t>& range) {
        auto& local = locals.local();  // Thread-local info matrix.
        for (size_t i = range.begin(); i < range.end(); ++i) {
          const auto& child = children[i];
          assert(child);
          if (child->fullyEliminated()) {
            child->updateParentInfo(
                local);  // No locking: each thread writes its own info matrix.
          }
        }
      });
  locals.combine_each([this](const SymmetricBlockMatrix& local) {
    info_.addUpperTriangular(
        local);  // Merge per-thread partial info matrices into this clique.
  });
#else
  std::vector<SymmetricBlockMatrix> locals;
  locals.reserve(numThreads);  // Fixed-size per-thread accumulators.
  for (size_t i = 0; i < numThreads; ++i) {
    locals.push_back(makeZeroLocalInfo(blockDims_));
  }
  std::vector<std::thread> threads;
  threads.reserve(numThreads);
  const size_t chunk =
      (children.size() + numThreads - 1) / numThreads;  // Static partitioning.
  for (size_t t = 0; t < numThreads; ++t) {
    const size_t start = t * chunk;
    const size_t end = std::min(start + chunk, children.size());
    if (start >= end) break;
    threads.emplace_back([this, start, end, &locals, t]() {
      auto& local = locals[t];
      for (size_t i = start; i < end; ++i) {
        const auto& child = children[i];
        assert(child);
        if (child->fullyEliminated()) {
          child->updateParentInfo(
              local);  // No locking: each thread writes its own info matrix.
        }
      }
    });
  }
  for (auto& thread : threads) {
    thread.join();  // Ensure all locals are complete before merge.
  }
  for (const auto& local : locals) {
    info_.addUpperTriangular(
        local);  // Merge per-thread partial info matrices into this clique.
  }
#endif
}

std::shared_ptr<GaussianConditional> MultifrontalClique::conditional() const {
  assert(RSdReady_);
  // RSd_ is cached at elimination time.
  return std::make_shared<GaussianConditional>(orderedKeys_, numFrontals(),
                                               RSd_);
}

std::shared_ptr<HessianFactor> MultifrontalClique::remainingFactor() const {
  assert(infoReady_);
  assert(!fullyEliminated());
  if (numFrontals() > 0) assert(RSdReady_);

  SymmetricBlockMatrix activeInfo = info_;
  activeInfo.blockStart() = static_cast<DenseIndex>(numFrontals());
  SymmetricBlockMatrix compactInfo =
      SymmetricBlockMatrix::LikeActiveViewOf(activeInfo);
  const Matrix denseActiveInfo = activeInfo.selfadjointView();
  compactInfo.setFullMatrix(denseActiveInfo);

  KeyVector retainedKeys(orderedKeys_.begin() + numFrontals(),
                         orderedKeys_.end());
  return std::make_shared<HessianFactor>(retainedKeys, compactInfo);
}

// Solve with block back-substitution on the Cholesky-stored info matrix.
void MultifrontalClique::updateSolution() {
  if (numFrontals() == 0) return;
  assert(RSdReady_);
  assert(RSd_.rowStart() == 0);
  // Use cached [R S d] for fast back-substitution.
  const size_t nf = numFrontals();
  const size_t n = RSd_.nBlocks() - 1;  // # frontals + # separators

  // The in-place factorization yields an upper-triangular system [R S d]:
  //   R * x_f + S * x_s = d,
  // with x_f the frontals and x_s the separators.
  const auto R = RSd_.range(0, nf).triangularView<Eigen::Upper>();
  const auto S = RSd_.range(nf, n);
  const auto d = RSd_.range(n, n + 1);

  // We first solve rhs = d - S * x_s
  rhsScratch_.noalias() = d;
  const Vector* x_s = nullptr;
  if (!separatorPtrs_.empty()) {
    x_s = &buildSeparatorVector(separatorPtrs_, &separatorScratch_);
    rhsScratch_.noalias() -= S * (*x_s);
  }

  // Then solve for x_f, our solution, via R * x_f = rhs
  // We solve the contiguous frontal system in one triangular solve.
  R.solveInPlace(rhsScratch_);
  const Vector& x_f = rhsScratch_;

  // Write solved frontal blocks back into the global solution.
  size_t offset = 0;
  for (Vector* values : frontalPtrs_) {
    const size_t dim = values->size();
    values->noalias() = x_f.segment(offset, dim);
    offset += dim;
  }

  lastOldError_ = 0.0;
  lastNewError_ = 0.0;
  if (frontalDim > 0) {
    lastOldError_ = 0.5 * d.squaredNorm();
    Vector residual = R * x_f;
    if (x_s != nullptr) {
      residual.noalias() += S * (*x_s);
    }
    residual.noalias() -= d;
    lastNewError_ = 0.5 * residual.squaredNorm();
  }
}

double MultifrontalClique::constantTermError() const {
  if (!RSdReady_) {
    return 0.0;
  }
  double constantError = 0.0;
  if (useQR()) {
    const DenseIndex extraRows =
        RSd_.matrix().rows() - static_cast<DenseIndex>(frontalDim);
    if (extraRows > 0) {
      const DenseIndex lastCol =
          static_cast<DenseIndex>(RSd_.matrix().cols() - 1);
      constantError =
          0.5 * RSd_.matrix().bottomRows(extraRows).col(lastCol).squaredNorm();
    }
  } else {
    const DenseIndex rhsIndex =
        static_cast<DenseIndex>(info_.nBlocks() - info_.blockStart() - 1);
    if (rhsIndex >= 0) {
      constantError = 0.5 * info_.diagonalBlock(rhsIndex)(0, 0);
    }
  }
  return constantError;
}

void MultifrontalClique::print(const std::string& s,
                               const KeyFormatter& keyFormatter) const {
  if (!s.empty()) std::cout << s;
  const KeyVector& orderedKeys = orderedKeys_;
  std::cout << "Clique(frontals=[";
  printKeyRange(std::cout, orderedKeys, 0,
                std::min(numFrontals(), orderedKeys.size()), keyFormatter);
  std::cout << "], separators=[";
  printKeyRange(std::cout, orderedKeys,
                std::min(numFrontals(), orderedKeys.size()), orderedKeys.size(),
                keyFormatter);
  std::cout << "], factors=" << factorIndices_.size()
            << ", children=" << children.size()
            << ", infoBlocks=" << info_.nBlocks()
            << ", AbRows=" << Ab_.matrix().rows() << ")\n";

  auto assembleInfo = [](const SymmetricBlockMatrix& info) {
    const size_t nBlocks = info.nBlocks();
    std::vector<size_t> offsets(nBlocks + 1, 0);
    for (size_t i = 0; i < nBlocks; ++i) {
      offsets[i + 1] = offsets[i] + info.getDim(i);
    }
    Matrix full = Matrix::Zero(offsets.back(), offsets.back());
    for (size_t i = 0; i < nBlocks; ++i) {
      for (size_t j = 0; j < nBlocks; ++j) {
        Matrix block = info.block(i, j);
        full.block(offsets[i], offsets[j], block.rows(), block.cols()) = block;
      }
    }
    return full;
  };

  std::cout << "  Ab:\n" << Ab_.matrix() << "\n";
  std::cout << "  info:\n" << assembleInfo(info_) << "\n";
}

std::ostream& operator<<(std::ostream& os, const MultifrontalClique& clique) {
  const KeyVector& orderedKeys = clique.orderedKeys_;
  const KeyFormatter formatter = DefaultKeyFormatter;
  os << "Clique(frontals=";
  printKeyRange(os, orderedKeys, 0,
                std::min(clique.numFrontals(), orderedKeys.size()), formatter);
  os << ", separators=";
  printKeyRange(os, orderedKeys,
                std::min(clique.numFrontals(), orderedKeys.size()),
                orderedKeys.size(), formatter);
  os << ", factors=" << clique.factorIndices_.size();
  os << ", children=" << clique.children.size();
  os << ", infoBlocks=" << clique.info().nBlocks();
  os << ", AbRows=" << clique.Ab().matrix().rows() << ")";
  return os;
}

}  // namespace gtsam
