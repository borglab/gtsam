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
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/MultifrontalClique.h>
#include <gtsam/linear/MultifrontalSolver.h>
#include <gtsam/linear/internal/BatchJacobianFactorElimination.h>
#include <gtsam/linear/internal/CompactLeafSchurKernel.h>

#include <Eigen/Cholesky>

#ifdef GTSAM_USE_TBB
#include <tbb/blocked_range.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

#include <functional>
#endif

#include <algorithm>
#include <cassert>
#include <cmath>
#include <exception>
#include <iostream>
#include <thread>
#include <utility>

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
  // Partial elimination can leave a smaller active view, while cached child
  // mappings still address the clique's complete symbolic block layout.
  SymmetricBlockMatrix local(blockDims, true);
  local.setZero();
  return local;
}

#ifndef GTSAM_USE_TBB
std::pair<size_t, size_t> staticRange(size_t itemCount, size_t worker,
                                      size_t workerCount) {
  const size_t itemsPerWorker = itemCount / workerCount;
  const size_t extraItems = itemCount % workerCount;
  const size_t begin = worker * itemsPerWorker + std::min(worker, extraItems);
  const size_t end = begin + itemsPerWorker + (worker < extraItems ? 1 : 0);
  return {begin, end};
}

template <class Work>
void runThreadBatch(size_t workerCount, Work&& work) {
  assert(workerCount > 0);
  std::vector<std::exception_ptr> failures(workerCount);
  auto runWorker = [&](size_t worker) {
    try {
      work(worker, workerCount);
    } catch (...) {
      failures[worker] = std::current_exception();
    }
  };

  // This clique is already running on a traversal worker. Count it as worker
  // zero rather than creating workerCount additional operating-system threads.
  std::vector<std::thread> threads;
  threads.reserve(workerCount - 1);
  try {
    for (size_t worker = 1; worker < workerCount; ++worker) {
      threads.emplace_back(runWorker, worker);
    }
  } catch (...) {
    for (std::thread& thread : threads) thread.join();
    throw;
  }
  runWorker(0);
  for (std::thread& thread : threads) thread.join();
  for (const std::exception_ptr& failure : failures) {
    if (failure) std::rethrow_exception(failure);
  }
}
#endif

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
  const size_t numSeparatorBlocks = blockDims_.size() - numFrontals();
  separatorIndices_.reserve(numSeparatorBlocks + 1);
  separatorScalarOffsets_.reserve(numSeparatorBlocks + 1);
  DenseIndex separatorScalarOffset = 0;
  for (size_t i = 0; i <= numSeparatorBlocks; ++i) {
    separatorIndices_.push_back(static_cast<DenseIndex>(i));
    separatorScalarOffsets_.push_back(separatorScalarOffset);
    if (i < numSeparatorBlocks) {
      separatorScalarOffset +=
          static_cast<DenseIndex>(blockDims_[numFrontals() + i]);
    }
  }
  factorRows_ = vbmRows;
}

void MultifrontalClique::finalize(std::vector<ChildInfo> children,
                                  const MultifrontalParameters& params) {
  this->children.clear();
  this->children.reserve(children.size());
  for (const auto& child : children) {
    this->children.push_back(child.clique);
  }

  // Compute parent indices and scalar offsets for all children. The RHS block
  // follows the variable blocks and has dimension one.
  std::vector<DenseIndex> blockScalarOffsets(blockDims_.size() + 1);
  DenseIndex scalarOffset = 0;
  for (size_t i = 0; i < blockDims_.size(); ++i) {
    blockScalarOffsets[i] = scalarOffset;
    scalarOffset += static_cast<DenseIndex>(blockDims_[i]);
  }
  blockScalarOffsets.back() = scalarOffset;
  for (const auto& child : children) {
    if (!child.clique) continue;
    std::vector<DenseIndex> indices;
    std::vector<DenseIndex> scalarOffsets;
    indices.reserve(child.separatorKeys.size() + 1);
    scalarOffsets.reserve(child.separatorKeys.size() + 1);
    for (Key key : child.separatorKeys) {
      const DenseIndex index = blockIndex(key);
      indices.push_back(index);
      scalarOffsets.push_back(blockScalarOffsets[index]);
    }
    // The RHS block is always the last block in Ab/info.
    indices.push_back(static_cast<DenseIndex>(orderedKeys_.size()));
    scalarOffsets.push_back(blockScalarOffsets.back());
    child.clique->setParentIndices(indices, scalarOffsets);
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
  const bool fusedStarCandidate = !useQR && eliminatesWholeClique && isLeaf &&
                                  !parent.expired() && numFrontals() == 1;
  starFallbackMode_ =
      useCompactCholesky ? SolveMode::CompactCholeskyLeaf : SolveMode::Cholesky;
  solveMode_ = useQR ? SolveMode::QrLeaf
                     : (fusedStarCandidate ? SolveMode::FusedStarCandidate
                                           : starFallbackMode_);
  deferredSingleFactorAutoQr_ =
      params.qrMode == MultifrontalParameters::QRMode::Allow && useQR &&
      eliminatesWholeClique && isLeaf && !parent.expired() &&
      numFrontals() == 1 && factorIndices_.size() == 1;
  RSdReady_ = false;

  if (!deferredSingleFactorAutoQr_) allocateSolveStorage();

  if (params.leafMode == MultifrontalParameters::LeafMode::SameSeparator &&
      params.leafAggregationProblemSize > 0) {
    // Equal parent mappings identify siblings that share one local accumulator.
    std::map<std::vector<DenseIndex>, std::vector<size_t>> groups;
    for (size_t i = 0; i < this->children.size(); ++i) {
      const auto& child = this->children[i];
      if (child && child->children.empty() && child->fullyEliminated() &&
          !child->useQR()) {
        groups[child->parentIndices_].push_back(i);
      }
    }
    childInSameSeparatorGroup_.assign(this->children.size(), 0);
    for (const auto& [parentIndices, childIndices] : groups) {
      if (childIndices.size() < 2) continue;
      const auto& child = this->children[childIndices.front()];
      size_t maximumProblemSize = 1;
      for (size_t childIndex : childIndices) {
        maximumProblemSize = std::max(
            maximumProblemSize,
            static_cast<size_t>(this->children[childIndex]->problemSize()));
      }
      const size_t maximumChildrenPerBatch = std::max<size_t>(
          1, params.leafAggregationProblemSize / maximumProblemSize);
      const size_t numBatches =
          (childIndices.size() + maximumChildrenPerBatch - 1) /
          maximumChildrenPerBatch;
      // Split evenly so a final singleton is not stranded by a greedy cap.
      const auto firstSeparatorDim =
          child->blockDims_.begin() + child->numFrontals();
      std::vector<size_t> separatorDims(firstSeparatorDim,
                                        child->blockDims_.end());
      for (size_t batch = 0; batch < numBatches; ++batch) {
        const size_t begin = childIndices.size() * batch / numBatches;
        const size_t end = childIndices.size() * (batch + 1) / numBatches;
        if (end - begin < 2) continue;
        sameSeparatorChildGroups_.emplace_back(childIndices.begin() + begin,
                                               childIndices.begin() + end);
        sameSeparatorParentIndices_.push_back(parentIndices);
        std::vector<DenseIndex> scalarOffsets;
        scalarOffsets.reserve(parentIndices.size());
        for (const DenseIndex index : parentIndices) {
          scalarOffsets.push_back(blockScalarOffsets[index]);
        }
        sameSeparatorParentScalarOffsets_.push_back(std::move(scalarOffsets));
        sameSeparatorInfos_.push_back(makeZeroLocalInfo(separatorDims));
        for (size_t childIndex : sameSeparatorChildGroups_.back()) {
          childInSameSeparatorGroup_[childIndex] = 1;
        }
      }
    }
  }
}

void MultifrontalClique::allocateSolveStorage() {
  assert(RSd_.nBlocks() == 0 && info_.nBlocks() == 0);
  if (useQR()) {
    const DenseIndex totalRows = static_cast<DenseIndex>(factorRows_) +
                                 static_cast<DenseIndex>(frontalDim);
    RSd_ = VerticalBlockMatrix(blockDims_, totalRows, true);
  } else {
    RSd_ = VerticalBlockMatrix(blockDims_, static_cast<DenseIndex>(frontalDim),
                               true);
    if (solveMode_ == SolveMode::Cholesky) {
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

MultifrontalClique::FactorLoadPlan
MultifrontalClique::FactorLoadPlan::forNullFactor(size_t factorIndex) {
  FactorLoadPlan plan;
  plan.factorIndex = factorIndex;
  return plan;
}

MultifrontalClique::FactorLoadPlan
MultifrontalClique::FactorLoadPlan::forJacobian(
    size_t factorIndex, const JacobianFactor& factor,
    const MultifrontalClique& clique) {
  FactorLoadPlan plan;
  plan.kind = FactorLoadKind::Jacobian;
  plan.factorIndex = factorIndex;
  plan.model = factor.get_model();
  plan.rows = factor.rows();
  plan.mapKeys(factor.keys(), clique);
  return plan;
}

MultifrontalClique::FactorLoadPlan MultifrontalClique::FactorLoadPlan::forBatch(
    size_t factorIndex, const BatchJacobianFactorBase& factor,
    const MultifrontalClique& clique) {
  FactorLoadPlan plan;
  plan.kind = FactorLoadKind::Batch;
  plan.factorIndex = factorIndex;
  plan.model = factor.get_model();
  plan.rows = factor.rows();
  plan.mapKeys(factor.keys(), clique);
  const bool hasFixedKey =
      std::any_of(plan.blockIndices.begin(), plan.blockIndices.end(),
                  [](DenseIndex index) { return index < 0; });
  plan.canDirectUpdate =
      !hasFixedKey && !(plan.model && plan.model->isConstrained());
  plan.buildLocalBatchMapping(factor, clique);
  return plan;
}

bool MultifrontalClique::FactorLoadPlan::needsMaterializedRows(
    const MultifrontalClique& clique) const {
  if (kind == FactorLoadKind::Empty) return false;
  if (kind == FactorLoadKind::Jacobian) return true;
  return clique.useQR() || !canDirectUpdate || !kUseDirectBatchHessianUpdate;
}

void MultifrontalClique::FactorLoadPlan::assignMaterializedRows(
    size_t* nextRow) {
  assert(nextRow);
  abRowOffset = *nextRow;
  *nextRow += rows;
}

void MultifrontalClique::FactorLoadPlan::mapKeys(
    const KeyVector& keys, const MultifrontalClique& clique) {
  blockIndices.reserve(keys.size());
  for (Key key : keys) {
    if (clique.fixedKeys_ && clique.fixedKeys_->count(key)) {
      blockIndices.push_back(-1);
    } else {
      blockIndices.push_back(clique.blockIndex(key));
    }
  }
}

void MultifrontalClique::FactorLoadPlan::buildLocalBatchMapping(
    const BatchJacobianFactorBase& factor, const MultifrontalClique& clique) {
  std::vector<DenseIndex> localSlots = blockIndices;
  localSlots.push_back(static_cast<DenseIndex>(clique.orderedKeys_.size()));
  if (!canDirectUpdate) return;
  localMapping = internal::BatchJacobianFactorElimination::buildMapping(
      factor, localSlots);
}

bool MultifrontalClique::FactorLoadPlan::supportsFusedStarLeaf() const {
  if (kind == FactorLoadKind::Empty) return true;
  if (kind == FactorLoadKind::Batch) return canDirectUpdate;
  // Fallback rows may touch only the single frontal or fixed variables.
  return std::all_of(blockIndices.begin(), blockIndices.end(),
                     [](DenseIndex index) { return index <= 0; });
}

void MultifrontalClique::FactorLoadPlan::buildSolveMappings(
    const BatchJacobianFactorBase& factor, const MultifrontalClique& clique) {
  if (!canDirectUpdate) return;

  if (!clique.useQR() && !clique.useCompactCholesky()) {
    internal::BatchJacobianFactorElimination::cacheScalarOffsets(clique.info_,
                                                                 &localMapping);
  }
  if (!clique.useCompactCholesky()) return;

  assert(clique.parentIndices_.size() ==
         clique.orderedKeys_.size() - clique.numFrontals() + 1);
  const DenseIndex numFrontals = static_cast<DenseIndex>(clique.numFrontals());

  parentMapping = buildRetainedMapping(
      factor, numFrontals, clique.parentIndices_, clique.parentScalarOffsets_);
  separatorMapping =
      buildRetainedMapping(factor, numFrontals, clique.separatorIndices_,
                           clique.separatorScalarOffsets_);
}

internal::BatchHessianMapping
MultifrontalClique::FactorLoadPlan::buildRetainedMapping(
    const BatchJacobianFactorBase& factor, DenseIndex numFrontals,
    const std::vector<DenseIndex>& targetIndices,
    const std::vector<DenseIndex>& targetScalarOffsets) const {
  assert(targetIndices.size() == targetScalarOffsets.size());
  std::vector<DenseIndex> factorTargetSlots;
  std::vector<DenseIndex> factorTargetScalarOffsets;
  factorTargetSlots.reserve(blockIndices.size() + 1);
  factorTargetScalarOffsets.reserve(blockIndices.size() + 1);
  // Preserve factor slot order while excluding eliminated frontal blocks.
  for (const DenseIndex localIndex : blockIndices) {
    if (localIndex < numFrontals) {
      factorTargetSlots.push_back(-1);
      factorTargetScalarOffsets.push_back(-1);
    } else {
      const DenseIndex separatorIndex = localIndex - numFrontals;
      factorTargetSlots.push_back(targetIndices.at(separatorIndex));
      factorTargetScalarOffsets.push_back(
          targetScalarOffsets.at(separatorIndex));
    }
  }
  factorTargetSlots.push_back(targetIndices.back());
  factorTargetScalarOffsets.push_back(targetScalarOffsets.back());
  return internal::BatchJacobianFactorElimination::buildMapping(
      factor, factorTargetSlots, factorTargetScalarOffsets);
}

void MultifrontalClique::FactorLoadPlan::assertInvariants(
    const GaussianFactor* factor, const MultifrontalClique& clique) const {
#ifdef NDEBUG
  (void)factor;
  (void)clique;
#else
  if (kind == FactorLoadKind::Empty) {
    assert(!factor);
    assert(rows == 0 && blockIndices.empty());
    return;
  }
  assert(factor);
  assert(blockIndices.size() == factor->keys().size());
  if (kind == FactorLoadKind::Jacobian) {
    assert(!canDirectUpdate && localMapping.empty());
    return;
  }
  assert(localMapping.scalarOffsets.empty() ||
         localMapping.scalarOffsets.size() == localMapping.blockSlots.size());
  assert(parentMapping.scalarOffsets.empty() ||
         parentMapping.scalarOffsets.size() == parentMapping.blockSlots.size());
  assert(separatorMapping.scalarOffsets.empty() ||
         separatorMapping.scalarOffsets.size() ==
             separatorMapping.blockSlots.size());
  if (!clique.useCompactCholesky()) {
    assert(parentMapping.empty() && separatorMapping.empty());
  }
#endif
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
  const size_t rows = internal::BatchJacobianFactorElimination::scatterRows(
      batchFactor, Ab_, rowOffset, plan.blockIndices);
  if (plan.model && !plan.model->isConstrained()) {
    plan.model->WhitenInPlace(Ab_.matrix().middleRows(rowOffset, rows));
  }
  return rows;
}

void MultifrontalClique::resolveLeafSolveMode() {
  // Auto-QR defers allocation until the sole factor's representation is known.
  const bool needsDeferredStorage = deferredSingleFactorAutoQr_;
  if (deferredSingleFactorAutoQr_) {
    assert(solveMode_ == SolveMode::QrLeaf && loadPlans_.size() == 1);
    deferredSingleFactorAutoQr_ = false;
    if (loadPlans_.front().isDirectBatch()) {
      solveMode_ = SolveMode::FusedStarCandidate;
    }
  }

  if (solveMode_ == SolveMode::FusedStarCandidate) {
    // Fused-star updates require every retained contribution to stay direct.
    const bool supported = kUseDirectBatchHessianUpdate &&
                           hasDirectBatchFactors_ &&
                           std::all_of(loadPlans_.begin(), loadPlans_.end(),
                                       [](const FactorLoadPlan& plan) {
                                         return plan.supportsFusedStarLeaf();
                                       });
    solveMode_ =
        supported ? SolveMode::FusedStarCholeskyLeaf : starFallbackMode_;
  }

  if (needsDeferredStorage) {
    allocateSolveStorage();
  } else if (solveMode_ == SolveMode::Cholesky && info_.nBlocks() == 0) {
    info_ = SymmetricBlockMatrix(blockDims_, true);
  }
}

void MultifrontalClique::buildFusedStarMappings() {
  if (solveMode_ != SolveMode::FusedStarCholeskyLeaf) return;
  std::vector<size_t> retainedDimensions(blockDims_.begin() + numFrontals(),
                                         blockDims_.end());
  retainedDimensions.push_back(1);
  parentSchurScalarOffsets_ =
      internal::CompactLeafSchurKernel::expandScalarOffsets(
          retainedDimensions, parentScalarOffsets_);
  separatorSchurScalarOffsets_ =
      internal::CompactLeafSchurKernel::expandScalarOffsets(
          retainedDimensions, separatorScalarOffsets_);
}

void MultifrontalClique::buildLoadPlans(const GaussianFactorGraph& graph) {
  if (loadPlansBuilt_) return;
  loadPlans_.clear();
  loadPlans_.reserve(factorIndices_.size());
  allBatchFactors_ = true;
  hasDirectBatchFactors_ = false;
  materializedRows_ = 0;
  for (size_t factorIndex : factorIndices_) {
    assert(factorIndex < graph.size());
    const GaussianFactor::shared_ptr& factor = graph[factorIndex];
    FactorLoadPlan plan;
    if (!factor) {
      plan = FactorLoadPlan::forNullFactor(factorIndex);
    } else if (const auto* jacobianFactor =
                   dynamic_cast<const JacobianFactor*>(factor.get())) {
      plan = FactorLoadPlan::forJacobian(factorIndex, *jacobianFactor, *this);
    } else if (const auto* batchFactor =
                   dynamic_cast<const BatchJacobianFactorBase*>(factor.get())) {
      plan = FactorLoadPlan::forBatch(factorIndex, *batchFactor, *this);
    } else {
      throw MultifrontalSolverNotSupported(
          "only JacobianFactor or BatchJacobianFactor inputs are supported");
    }

    allBatchFactors_ &= plan.isDirectBatch();
    hasDirectBatchFactors_ |= plan.isDirectBatch();
    loadPlans_.push_back(std::move(plan));
  }

  // Resolve representation before choosing mappings and packed row storage.
  resolveLeafSolveMode();
  buildFusedStarMappings();
  for (FactorLoadPlan& plan : loadPlans_) {
    const GaussianFactor::shared_ptr& factor = graph[plan.factorIndex];
    if (plan.kind == FactorLoadKind::Batch && plan.canDirectUpdate) {
      const auto* batchFactor =
          dynamic_cast<const BatchJacobianFactorBase*>(factor.get());
      assert(batchFactor);
      plan.buildSolveMappings(*batchFactor, *this);
    }
    if (plan.needsMaterializedRows(*this)) {
      plan.assignMaterializedRows(&materializedRows_);
    }
    plan.assertInvariants(factor.get(), *this);
  }
  assert(loadPlans_.size() == factorIndices_.size());
  assert(!useQR() || materializedRows_ == factorRows_);
  loadPlansBuilt_ = true;
}

void MultifrontalClique::fillAb(const GaussianFactorGraph& graph) {
  // Direct batch updates retain this graph pointer through elimination.
  activeLoadGraph_ = &graph;
  assert(validateFactorKeys(graph, factorIndices_, orderedKeys_, fixedKeys_));
  RSdReady_ = false;
  infoReady_ = false;

  if (!loadPlansBuilt_) {
    buildLoadPlans(graph);
  }

  if (Ab_.nBlocks() == 0) {
    const DenseIndex dampingRows =
        useQR() ? static_cast<DenseIndex>(frontalDim) : 0;
    const DenseIndex totalRows =
        static_cast<DenseIndex>(materializedRows_) + dampingRows;
    Ab_ = VerticalBlockMatrix(blockDims_, totalRows, true);
    // Ab_'s packed structure is fixed after the first compatible load. Clear
    // it once; repeated loads overwrite the same structural nonzeros.
    Ab_.matrix().setZero();
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
        addJacobianFactor(*jacobianFactor, plan.abRowOffset, plan);
      } else {
        const auto* batchFactor =
            dynamic_cast<const BatchJacobianFactorBase*>(factor);
        assert(batchFactor);
        addBatchJacobianFactor(*batchFactor, plan.abRowOffset, plan);
      }
    }
    fillRows_ = materializedRows_;
    return;
  }

  for (const FactorLoadPlan& plan : loadPlans_) {
    const auto* factor = graph[plan.factorIndex].get();
    if (!factor) continue;
    if (plan.kind == FactorLoadKind::Jacobian) {
      auto* jacobianFactor = dynamic_cast<const JacobianFactor*>(factor);
      assert(jacobianFactor);
      addJacobianFactor(*jacobianFactor, plan.abRowOffset, plan);
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
      addBatchJacobianFactor(*batchFactor, plan.abRowOffset, plan);
      continue;
    }
    throw MultifrontalSolverNotSupported(
        "only JacobianFactor or BatchJacobianFactor inputs are supported");
  }

  assert((useQR() && RSd_.matrix().rows() ==
                         static_cast<DenseIndex>(Ab_.matrix().rows())) ||
         (RSd_.matrix().rows() == static_cast<DenseIndex>(frontalDim)));
  assert(useQR() || useCompactCholesky() || info_.nBlocks() > 0);
  fillRows_ = materializedRows_;
}

void MultifrontalClique::eliminateInPlace() {
  prepareForElimination();
  // This path assumes damping has never changed the reusable storage pattern.
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
      assert(!plan.localMapping.empty());
      internal::BatchJacobianFactorElimination::addHessian(
          *batchFactor, plan.localMapping, &info_);
    }
    if (fillRows_ > 0 && !allBatchFactors_) {
      // Mixed mode adds only fallback rows after direct batch contributions.
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
  const size_t numChildren = childInSameSeparatorGroup_.empty()
                                 ? children.size()
                                 : static_cast<size_t>(std::count(
                                       childInSameSeparatorGroup_.begin(),
                                       childInSameSeparatorGroup_.end(), 0));
  // Same-separator groups scatter once and skip the general child gather.
  gatherSameSeparatorUpdates();
  if (numChildren == 0) {
    infoReady_ = true;
    return;
  }
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
  // Assemble frontal rows only; retained-retained terms remain implicit.

  const DenseIndex nf = static_cast<DenseIndex>(numFrontals());
  const DenseIndex rhsBlock = RSd_.nBlocks() - 1;
  for (const FactorLoadPlan& plan : loadPlans_) {
    if (plan.rows == 0) continue;
    if (plan.kind == FactorLoadKind::Batch && plan.canDirectUpdate &&
        kUseDirectBatchHessianUpdate) {
      assert(activeLoadGraph_ && !plan.localMapping.empty());
      const auto* factor = (*activeLoadGraph_)[plan.factorIndex].get();
      const auto* batchFactor =
          dynamic_cast<const BatchJacobianFactorBase*>(factor);
      assert(batchFactor);
      internal::BatchJacobianFactorElimination::addFrontalHessian(
          *batchFactor, plan.localMapping, nf, &RSd_);
      continue;
    }
    for (const DenseIndex I : plan.blockIndices) {
      if (I < 0 || I >= nf) continue;
      const DenseIndex row = RSd_.offset(I);
      const DenseIndex dim = static_cast<DenseIndex>(blockDims_.at(I));
      const auto Ai =
          Ab_(I).middleRows(static_cast<DenseIndex>(plan.abRowOffset),
                            static_cast<DenseIndex>(plan.rows));
      for (const DenseIndex J : plan.blockIndices) {
        if (J < 0) continue;
        const auto Aj =
            Ab_(J).middleRows(static_cast<DenseIndex>(plan.abRowOffset),
                              static_cast<DenseIndex>(plan.rows));
        RSd_(J).middleRows(row, dim).noalias() += Ai.transpose() * Aj;
      }
      const auto b =
          Ab_(rhsBlock).middleRows(static_cast<DenseIndex>(plan.abRowOffset),
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
  } else if (solveMode_ == SolveMode::FusedStarCholeskyLeaf) {
    factorizeFusedStarCholesky();
  } else if (useCompactCholesky()) {
    factorizeCompactCholesky();
  } else {
    info_.choleskyPartial(numFrontals());
    // Split exposes [R S d] while info_ retains the separator Schur complement.
    info_.split(numFrontals(), &RSd_);
    info_.blockStart() = 0;
  }
  RSdReady_ = true;
}

void MultifrontalClique::factorizeFusedStarCholesky() {
  if (!internal::CompactLeafSchurKernel::factorFrontalRows(
          &RSd_, static_cast<DenseIndex>(frontalDim))) {
    throw CholeskyFailed();
  }
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
  // Match the scale-normalized pivot guard used by standard Cholesky.
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
  const DenseIndex baseRows = static_cast<DenseIndex>(materializedRows_);
  assert(materializedRows_ == factorRows_);
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

void MultifrontalClique::updateDirectFactors(
    SymmetricBlockMatrix& targetInfo,
    const std::vector<DenseIndex>& targetIndices,
    bool useParentMappedSlots) const {
  assert(activeLoadGraph_);
  const DenseIndex nf = static_cast<DenseIndex>(numFrontals());
  const DenseIndex targetRhs = targetIndices.back();

  for (const FactorLoadPlan& plan : loadPlans_) {
    if (plan.rows == 0) continue;
    const auto* factor = (*activeLoadGraph_)[plan.factorIndex].get();
    if (!factor) continue;
    if (plan.kind == FactorLoadKind::Batch && plan.canDirectUpdate &&
        !plan.parentMapping.empty()) {
      const auto* batchFactor =
          dynamic_cast<const BatchJacobianFactorBase*>(factor);
      assert(batchFactor);
      const auto& mapping =
          useParentMappedSlots ? plan.parentMapping : plan.separatorMapping;
      internal::BatchJacobianFactorElimination::addHessian(
          *batchFactor, mapping, &targetInfo);
      continue;
    }

    // Only fallback factors reach this branch, so Ab_ has packed row storage.
    const DenseIndex rhsBlock = Ab_.nBlocks() - 1;
    const DenseIndex row = static_cast<DenseIndex>(plan.abRowOffset);
    const DenseIndex rows = static_cast<DenseIndex>(plan.rows);
    const auto b = Ab_(rhsBlock).middleRows(row, rows);
    targetInfo.updateDiagonalBlock(targetRhs, b.transpose() * b);
    for (size_t p = 0; p < plan.blockIndices.size(); ++p) {
      const DenseIndex I = plan.blockIndices[p];
      if (I < nf) continue;
      const DenseIndex targetI = targetIndices.at(I - nf);
      const auto Ai = Ab_(I).middleRows(row, rows);
      targetInfo.updateDiagonalBlock(targetI, Ai.transpose() * Ai);
      targetInfo.updateOffDiagonalBlock(targetI, targetRhs, Ai.transpose() * b);
      for (size_t q = p + 1; q < plan.blockIndices.size(); ++q) {
        const DenseIndex J = plan.blockIndices[q];
        if (J < nf) continue;
        const DenseIndex targetJ = targetIndices.at(J - nf);
        const auto Aj = Ab_(J).middleRows(row, rows);
        targetInfo.updateOffDiagonalBlock(targetI, targetJ,
                                          Ai.transpose() * Aj);
      }
    }
  }
}

void MultifrontalClique::updateFusedStarInfo(
    SymmetricBlockMatrix& targetInfo,
    const std::vector<DenseIndex>& targetIndices,
    const std::vector<DenseIndex>& targetScalarOffsets,
    bool useParentMappedSlots) const {
  assert(RSdReady_ && RSd_.firstBlock() == 0);
  updateDirectFactors(targetInfo, targetIndices, useParentMappedSlots);
  // Fused point batches have very few frontal rows. Scalar dot products avoid
  // the dynamic block-product overhead that materially slows this hot path.
  internal::CompactLeafSchurKernel::subtractMappedOuterProduct(
      RSd_, static_cast<DenseIndex>(frontalDim), targetScalarOffsets,
      &targetInfo);
}

void MultifrontalClique::updateCholeskyInfo(
    SymmetricBlockMatrix& targetInfo,
    const std::vector<DenseIndex>& targetIndices,
    const std::vector<DenseIndex>& targetBlockOffsets,
    bool useParentMappedSlots) const {
  assert(fullyEliminated() && !useQR());
  const DenseIndex frontalBlocks = static_cast<DenseIndex>(numFrontals());
  if (solveMode_ == SolveMode::FusedStarCholeskyLeaf) {
    const auto& targetScalarOffsets = useParentMappedSlots
                                          ? parentSchurScalarOffsets_
                                          : separatorSchurScalarOffsets_;
    updateFusedStarInfo(targetInfo, targetIndices, targetScalarOffsets,
                        useParentMappedSlots);
  } else if (useCompactCholesky()) {
    assert(RSdReady_ && RSd_.firstBlock() == 0);
    updateDirectFactors(targetInfo, targetIndices, useParentMappedSlots);
    // Compact updates subtract [S d]'[S d] from original retained terms.
    targetInfo.updateFromOuterProductBlocks(
        RSd_, 0, static_cast<DenseIndex>(frontalDim), frontalBlocks,
        targetIndices, targetBlockOffsets, -1.0);
  } else {
    assert(info_.nBlocks() > 0 && info_.blockStart() == 0);
    targetInfo.updateFromMappedBlocks(info_, frontalBlocks, targetIndices,
                                      targetBlockOffsets);
  }
}

void MultifrontalClique::updateParentInfo(
    SymmetricBlockMatrix& parentInfo) const {
  assert(fullyEliminated());
  assert(RSd_.rowStart() == 0);
  if (useQR()) {
    // Accumulate separator (and RHS) normal equations from the QR residual.
    assert(RSdReady_ && RSd_.firstBlock() == 0);
    parentInfo.updateFromOuterProductBlocks(
        RSd_, static_cast<DenseIndex>(frontalDim),
        static_cast<DenseIndex>(RSd_.matrix().rows()),
        static_cast<DenseIndex>(numFrontals()), parentIndices_,
        parentScalarOffsets_);
  } else {
    updateCholeskyInfo(parentInfo, parentIndices_, parentScalarOffsets_, true);
  }
}

void MultifrontalClique::updateParentMaterializedColumn(
    SymmetricBlockMatrix& parentInfo, DenseIndex sourceSeparatorBlock) const {
  assert(fullyEliminated() && !useQR() && !useCompactCholesky());
  assert(info_.nBlocks() > 0 && info_.blockStart() == 0);
  parentInfo.updateFromMappedBlockColumn(
      info_, static_cast<DenseIndex>(numFrontals()), sourceSeparatorBlock,
      parentIndices_, parentScalarOffsets_, true);
}

void MultifrontalClique::updateParentQrColumnScratch(
    DenseIndex sourceSeparatorBlock, Matrix* scratch) const {
  assert(scratch && fullyEliminated() && useQR() && RSdReady_);
  assert(RSd_.firstBlock() == 0);
  const DenseIndex frontalBlocks = static_cast<DenseIndex>(numFrontals());
  const DenseIndex retainedBlocks =
      static_cast<DenseIndex>(parentIndices_.size()) - 1;
  assert(sourceSeparatorBlock >= 0 && sourceSeparatorBlock < retainedBlocks);

  const DenseIndex residualRow = static_cast<DenseIndex>(frontalDim);
  const DenseIndex sourceColumn = frontalBlocks + sourceSeparatorBlock;
  const auto owned = RSd_.blockRows(
      sourceColumn, residualRow, static_cast<DenseIndex>(RSd_.matrix().rows()));
  const DenseIndex sourceDimension = owned.cols();

  const DenseIndex targetColumn = parentIndices_[sourceSeparatorBlock];
  const DenseIndex targetOffset = parentScalarOffsets_[sourceSeparatorBlock];
  assert(scratch->cols() == sourceDimension);
  assert(scratch->rows() == targetOffset + sourceDimension + 1);
  scratch->block(targetOffset, 0, sourceDimension, sourceDimension).noalias() +=
      owned.transpose() * owned;

  const auto rhs =
      RSd_.blockRows(frontalBlocks + retainedBlocks, residualRow,
                     static_cast<DenseIndex>(RSd_.matrix().rows()));
  scratch->bottomRows(1).noalias() += rhs.transpose() * owned;

  // Store each cross-block in its physical upper-triangular owner column.
  for (DenseIndex other = 0; other < retainedBlocks; ++other) {
    if (parentIndices_[other] >= targetColumn) continue;
    const DenseIndex otherColumn = frontalBlocks + other;
    const auto otherBlock =
        RSd_.blockRows(otherColumn, residualRow,
                       static_cast<DenseIndex>(RSd_.matrix().rows()));
    const DenseIndex otherDimension = otherBlock.cols();
    scratch
        ->block(parentScalarOffsets_[other], 0, otherDimension, sourceDimension)
        .noalias() += otherBlock.transpose() * owned;
  }
}

double MultifrontalClique::parentRhsDiagonal() const {
  assert(fullyEliminated() && !useCompactCholesky());
  if (!useQR()) {
    assert(info_.nBlocks() > 0);
    return info_.diagonalBlock(info_.nBlocks() - 1).coeff(0, 0);
  }
  assert(RSdReady_);
  return RSd_
      .blockRows(RSd_.nBlocks() - 1, static_cast<DenseIndex>(frontalDim),
                 static_cast<DenseIndex>(RSd_.matrix().rows()))
      .squaredNorm();
}

void MultifrontalClique::updateSeparatorInfo(
    SymmetricBlockMatrix& separatorInfo) const {
  assert(fullyEliminated() && !useQR());
  assert(RSd_.rowStart() == 0);
  updateCholeskyInfo(separatorInfo, separatorIndices_, separatorScalarOffsets_,
                     false);
}

void MultifrontalClique::gatherUpdatesSequential() {
  for (size_t i = 0; i < children.size(); ++i) {
    if (!childInSameSeparatorGroup_.empty() && childInSameSeparatorGroup_[i]) {
      continue;
    }
    const auto& child = children[i];
    assert(child);
    if (child->fullyEliminated()) child->updateParentInfo(info_);
  }
}

struct MultifrontalClique::ParentGatherPlan {
  enum class ColumnRepresentation : uint8_t { Materialized, Qr };

  struct ColumnOwnedChild {
    size_t childIndex;
    ColumnRepresentation representation;
  };

  struct ColumnUpdate {
    size_t childIndex;
    DenseIndex sourceSeparatorBlock;
  };

  struct QrColumnChunk {
    DenseIndex column;
    size_t begin;
    size_t end;
    Matrix scratch;
  };

  std::vector<ColumnOwnedChild> columnOwnedChildren;
  std::vector<size_t> computeOnceChildIndices;
  std::vector<std::vector<ColumnUpdate>> materializedByColumn;
  std::vector<std::vector<ColumnUpdate>> qrByColumn;
  std::vector<QrColumnChunk> qrChunks;
  std::vector<std::vector<size_t>> qrChunkIndicesByColumn;
  bool hasMaterialized = false;
  bool hasQr = false;

  /// Classify children using their final numerical representation.
  explicit ParentGatherPlan(const MultifrontalClique& parent);

  /// Evaluate compact/fused children once into one reduction destination.
  void gatherComputeOnceRange(MultifrontalClique* parent, size_t begin,
                              size_t end,
                              SymmetricBlockMatrix* destination) const;

  /// Add ordinary updates for the owned physical parent columns.
  void gatherMaterializedRange(MultifrontalClique* parent, size_t begin,
                               size_t end) const;

  /// Calculate bounded QR scratch without writing the parent matrix.
  void prepareQrRange(MultifrontalClique* parent, size_t begin, size_t end);

  /// Combine prepared QR scratch into the owned physical parent columns.
  void combineQrRange(MultifrontalClique* parent, size_t begin, size_t end);

  /// Sum RHS diagonal contributions from column-owned children.
  double rhsRange(const MultifrontalClique& parent, size_t begin,
                  size_t end) const;

  /// Execute the shared gather plan with the configured parallel backend.
  void gather(MultifrontalClique* parent, size_t numThreads);
};

MultifrontalClique::ParentGatherPlan::ParentGatherPlan(
    const MultifrontalClique& parent) {
  // Skip same-separator children because their groups are already complete.
  for (size_t childIndex = 0; childIndex < parent.children.size();
       ++childIndex) {
    if (!parent.childInSameSeparatorGroup_.empty() &&
        parent.childInSameSeparatorGroup_[childIndex]) {
      continue;
    }
    const auto& child = parent.children[childIndex];
    if (!child || !child->fullyEliminated()) continue;
    assert(!child->deferredSingleFactorAutoQr_);
    assert(child->solveMode_ != SolveMode::FusedStarCandidate);

    if (child->useCompactCholesky()) {
      computeOnceChildIndices.push_back(childIndex);
      continue;
    }

    const bool qr = child->useQR();
    columnOwnedChildren.push_back(
        {childIndex,
         qr ? ColumnRepresentation::Qr : ColumnRepresentation::Materialized});
    auto& updatesByColumn = qr ? qrByColumn : materializedByColumn;
    if (updatesByColumn.empty()) {
      updatesByColumn.resize(parent.blockDims_.size());
    }
    hasQr |= qr;
    hasMaterialized |= !qr;
    for (size_t sourceBlock = 0; sourceBlock + 1 < child->parentIndices_.size();
         ++sourceBlock) {
      const DenseIndex column = child->parentIndices_[sourceBlock];
      assert(column < static_cast<DenseIndex>(parent.blockDims_.size()));
      updatesByColumn[column].push_back(
          {childIndex, static_cast<DenseIndex>(sourceBlock)});
    }
  }

  if (!hasQr) return;

  constexpr size_t kQrChildrenPerChunk = 128;
  // Chunk busy QR columns so one owner does not serialize all child products.
  qrChunkIndicesByColumn.resize(parent.blockDims_.size());
  for (size_t column = 0; column < qrByColumn.size(); ++column) {
    const auto& columnUpdates = qrByColumn[column];
    for (size_t begin = 0; begin < columnUpdates.size();
         begin += kQrChildrenPerChunk) {
      const size_t end =
          std::min(begin + kQrChildrenPerChunk, columnUpdates.size());
      const size_t scratchRows =
          static_cast<size_t>(parent.info_.blockScalarOffset(column)) +
          parent.blockDims_[column] + 1;
      const size_t chunkIndex = qrChunks.size();
      qrChunkIndicesByColumn[column].push_back(chunkIndex);
      qrChunks.push_back(
          {static_cast<DenseIndex>(column), begin, end,
           Matrix::Zero(static_cast<DenseIndex>(scratchRows),
                        static_cast<DenseIndex>(parent.blockDims_[column]))});
    }
  }
}

void MultifrontalClique::ParentGatherPlan::gatherComputeOnceRange(
    MultifrontalClique* parent, size_t begin, size_t end,
    SymmetricBlockMatrix* destination) const {
  assert(parent && destination);
  for (size_t index = begin; index < end; ++index) {
    const auto& child = parent->children[computeOnceChildIndices[index]];
    assert(child && child->fullyEliminated());
    child->updateParentInfo(*destination);
  }
}

void MultifrontalClique::ParentGatherPlan::gatherMaterializedRange(
    MultifrontalClique* parent, size_t begin, size_t end) const {
  assert(parent);
  // Every task owns a physical upper-triangular parent column. Mapped blocks
  // may change logical order, but no other task can write that destination.
  for (size_t column = begin; column < end; ++column) {
    for (const ColumnUpdate& update : materializedByColumn[column]) {
      const auto& child = parent->children[update.childIndex];
      assert(child && child->fullyEliminated());
      child->updateParentMaterializedColumn(parent->info_,
                                            update.sourceSeparatorBlock);
    }
  }
}

void MultifrontalClique::ParentGatherPlan::prepareQrRange(
    MultifrontalClique* parent, size_t begin, size_t end) {
  assert(parent);
  // Multiple QR children can contribute to the same column. Bounded chunks
  // expose parallel work while keeping every scratch matrix narrow.
  for (size_t chunkIndex = begin; chunkIndex < end; ++chunkIndex) {
    QrColumnChunk& chunk = qrChunks[chunkIndex];
    chunk.scratch.setZero();
    const auto& columnUpdates = qrByColumn[static_cast<size_t>(chunk.column)];
    for (size_t index = chunk.begin; index < chunk.end; ++index) {
      const ColumnUpdate& update = columnUpdates[index];
      parent->children[update.childIndex]->updateParentQrColumnScratch(
          update.sourceSeparatorBlock, &chunk.scratch);
    }
  }
}

void MultifrontalClique::ParentGatherPlan::combineQrRange(
    MultifrontalClique* parent, size_t begin, size_t end) {
  assert(parent);
  for (size_t column = begin; column < end; ++column) {
    const auto& chunkIndices = qrChunkIndicesByColumn[column];
    if (chunkIndices.empty()) continue;
    // The first chunk becomes the reusable per-column accumulator.
    Matrix& accumulated = qrChunks[chunkIndices.front()].scratch;
    for (size_t index = 1; index < chunkIndices.size(); ++index) {
      accumulated += qrChunks[chunkIndices[index]].scratch;
    }

    const DenseIndex targetOffset = parent->info_.blockScalarOffset(column);
    const DenseIndex targetDimension =
        static_cast<DenseIndex>(parent->blockDims_[column]);
    for (size_t rowBlock = 0; rowBlock < column; ++rowBlock) {
      const DenseIndex rowOffset = parent->info_.blockScalarOffset(rowBlock);
      const DenseIndex rowDimension =
          static_cast<DenseIndex>(parent->blockDims_[rowBlock]);
      parent->info_.updateOffDiagonalBlockAt(
          rowOffset, targetOffset,
          accumulated.block(rowOffset, 0, rowDimension, targetDimension));
    }
    parent->info_.updateDiagonalBlockAt(
        targetOffset,
        accumulated.block(targetOffset, 0, targetDimension, targetDimension));
    parent->info_.updateOffDiagonalBlock(static_cast<DenseIndex>(column),
                                         parent->info_.nBlocks() - 1,
                                         accumulated.bottomRows(1).transpose());
  }
}

double MultifrontalClique::ParentGatherPlan::rhsRange(
    const MultifrontalClique& parent, size_t begin, size_t end) const {
  double subtotal = 0.0;
  for (size_t index = begin; index < end; ++index) {
    const ColumnOwnedChild& update = columnOwnedChildren[index];
    assert(update.representation == (parent.children[update.childIndex]->useQR()
                                         ? ColumnRepresentation::Qr
                                         : ColumnRepresentation::Materialized));
    subtotal += parent.children[update.childIndex]->parentRhsDiagonal();
  }
  return subtotal;
}

void MultifrontalClique::ParentGatherPlan::gather(MultifrontalClique* parent,
                                                  size_t numThreads) {
  assert(parent);

  // Compact and fused Schur children stay on a compute-once reduction. Making
  // them column-owned would repeat their expensive update for every column.
  if (!computeOnceChildIndices.empty()) {
#ifdef GTSAM_USE_TBB
    tbb::enumerable_thread_specific<SymmetricBlockMatrix> locals(
        [parent]() { return makeZeroLocalInfo(parent->blockDims_); });
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, computeOnceChildIndices.size()),
        [this, parent, &locals](const tbb::blocked_range<size_t>& range) {
          gatherComputeOnceRange(parent, range.begin(), range.end(),
                                 &locals.local());
        });
    locals.combine_each([parent](const SymmetricBlockMatrix& local) {
      parent->info_.addUpperTriangular(local);
    });
#else
    const size_t workerCount =
        std::min(numThreads, computeOnceChildIndices.size());
    std::vector<SymmetricBlockMatrix> locals;
    locals.reserve(workerCount);
    for (size_t worker = 0; worker < workerCount; ++worker) {
      locals.push_back(makeZeroLocalInfo(parent->blockDims_));
    }
    runThreadBatch(
        workerCount, [this, parent, &locals](size_t worker, size_t workers) {
          const auto [begin, end] =
              staticRange(computeOnceChildIndices.size(), worker, workers);
          gatherComputeOnceRange(parent, begin, end, &locals[worker]);
        });
    for (const SymmetricBlockMatrix& local : locals) {
      parent->info_.addUpperTriangular(local);
    }
#endif
  }

  // A compact-only parent follows exactly the original compute-once path.
  if (columnOwnedChildren.empty()) return;

#ifdef GTSAM_USE_TBB
  (void)numThreads;  // TBB is capped by the enclosing traversal arena.
  // TBB supplies a persistent arena, task-local storage, and a native scalar
  // reduction, so its scheduling adapter can express each stage directly.
  if (hasMaterialized) {
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, materializedByColumn.size(), 1),
        [this, parent](const tbb::blocked_range<size_t>& range) {
          gatherMaterializedRange(parent, range.begin(), range.end());
        });
  }

  if (hasQr) {
    tbb::parallel_for(tbb::blocked_range<size_t>(0, qrChunks.size(), 1),
                      [this, parent](const tbb::blocked_range<size_t>& range) {
                        prepareQrRange(parent, range.begin(), range.end());
                      });

    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, qrChunkIndicesByColumn.size(), 1),
        [this, parent](const tbb::blocked_range<size_t>& range) {
          combineQrRange(parent, range.begin(), range.end());
        });
  }

  const double rhs = tbb::parallel_reduce(
      tbb::blocked_range<size_t>(0, columnOwnedChildren.size()), 0.0,
      [this, parent](const tbb::blocked_range<size_t>& range, double subtotal) {
        return subtotal + rhsRange(*parent, range.begin(), range.end());
      },
      std::plus<double>());
#else
  // The traversal scheduler cannot safely block for nested tasks: it has no
  // cooperative task-group wait, while its inline submission path would run
  // serially. Use a bounded short-lived team for this rare large-parent path.
  // Independent materialized writes, QR scratch preparation, and scalar RHS
  // partials share one launch; QR combination waits for that team to finish.
  const size_t parallelItems = std::max(
      {hasMaterialized ? materializedByColumn.size() : size_t{0},
       hasQr ? qrChunks.size() : size_t{0}, columnOwnedChildren.size()});
  const size_t workerCount = std::min(numThreads, parallelItems);
  std::vector<double> rhsPartials(workerCount, 0.0);
  runThreadBatch(workerCount, [this, parent, &rhsPartials](size_t worker,
                                                           size_t workers) {
    if (hasMaterialized) {
      const auto [begin, end] =
          staticRange(materializedByColumn.size(), worker, workers);
      gatherMaterializedRange(parent, begin, end);
    }
    if (hasQr) {
      const auto [begin, end] = staticRange(qrChunks.size(), worker, workers);
      prepareQrRange(parent, begin, end);
    }
    const auto [rhsBegin, rhsEnd] =
        staticRange(columnOwnedChildren.size(), worker, workers);
    rhsPartials[worker] = rhsRange(*parent, rhsBegin, rhsEnd);
  });

  if (hasQr) {
    const size_t combineWorkers =
        std::min(numThreads, qrChunkIndicesByColumn.size());
    runThreadBatch(
        combineWorkers, [this, parent](size_t worker, size_t workers) {
          const auto [begin, end] =
              staticRange(qrChunkIndicesByColumn.size(), worker, workers);
          combineQrRange(parent, begin, end);
        });
  }

  double rhs = 0.0;
  for (double partial : rhsPartials) rhs += partial;
#endif

  Eigen::Matrix<double, 1, 1> rhsUpdate;
  rhsUpdate(0, 0) = rhs;
  parent->info_.updateDiagonalBlock(parent->info_.nBlocks() - 1, rhsUpdate);
}

void MultifrontalClique::gatherUpdatesParallel(size_t numThreads) {
  // Deferred leaf modes are resolved by the first elimination. Build the plan
  // only after that resolution, then reuse its buckets and scratch on reloads.
  if (!parentGatherPlan_) {
    parentGatherPlan_ = std::make_shared<ParentGatherPlan>(*this);
  }
  parentGatherPlan_->gather(this, numThreads);
}

void MultifrontalClique::gatherSameSeparatorUpdates() {
  if (sameSeparatorChildGroups_.empty()) return;
#ifdef GTSAM_USE_TBB
  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, sameSeparatorChildGroups_.size()),
      [this](const tbb::blocked_range<size_t>& range) {
        for (size_t group = range.begin(); group < range.end(); ++group) {
          auto& groupInfo = sameSeparatorInfos_[group];
          groupInfo.setZero();
          for (size_t childIndex : sameSeparatorChildGroups_[group]) {
            children[childIndex]->updateSeparatorInfo(groupInfo);
          }
        }
      });
#else
  for (size_t group = 0; group < sameSeparatorChildGroups_.size(); ++group) {
    auto& groupInfo = sameSeparatorInfos_[group];
    groupInfo.setZero();
    for (size_t childIndex : sameSeparatorChildGroups_[group]) {
      children[childIndex]->updateSeparatorInfo(groupInfo);
    }
  }
#endif
  // Scatter only after disjoint group-local assembly finishes.
  for (size_t group = 0; group < sameSeparatorInfos_.size(); ++group) {
    info_.updateFromMappedBlocks(sameSeparatorInfos_[group],
                                 sameSeparatorParentIndices_[group],
                                 sameSeparatorParentScalarOffsets_[group]);
  }
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

  const DenseIndex firstRetainedBlock = static_cast<DenseIndex>(numFrontals());
  SymmetricBlockMatrix compactInfo(blockDims_.begin() + firstRetainedBlock,
                                   blockDims_.end(), true);
  // The returned factor can outlive this clique, so one copy is unavoidable.
  // Copy only the active upper triangle directly into its final compact owner.
  compactInfo.setFullMatrix(
      info_.selfadjointView(firstRetainedBlock, info_.nBlocks())
          .nestedExpression());

  KeyVector retainedKeys(orderedKeys_.begin() + numFrontals(),
                         orderedKeys_.end());
  return std::make_shared<HessianFactor>(retainedKeys, std::move(compactInfo));
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
  // Only residual QR rows or the active RHS Schur term contribute here.
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
