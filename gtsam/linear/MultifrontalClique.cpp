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
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/MultifrontalClique.h>

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

MultifrontalClique::MultifrontalClique(
    std::vector<size_t> factorIndices,
    const std::weak_ptr<MultifrontalClique>& parent, const KeyVector& frontals,
    const KeySet& separatorKeys, const KeyDimMap& dims, size_t vbmRows,
    VectorValues* solution, const std::unordered_set<Key>* fixedKeys) {
  factorIndices_ = std::move(factorIndices);
  this->parent = parent;
  fixedKeys_ = fixedKeys;

  if (frontals.empty()) {
    throw std::runtime_error(
        "MultifrontalSolver: cluster has no frontal keys.");
  }

  // Cache keys in block order for fast linear lookup in small cliques.
  orderedKeys_.clear();
  orderedKeys_.reserve(frontals.size() + separatorKeys.size());
  orderedKeys_.insert(orderedKeys_.end(), frontals.begin(), frontals.end());
  orderedKeys_.insert(orderedKeys_.end(), separatorKeys.begin(),
                      separatorKeys.end());

  // Cache total frontal/separator dimensions for scheduling and sizing.
  frontalDim = internal::sumDims(dims, frontals);
  separatorDim = internal::sumDims(dims, separatorKeys);

  rhsScratch_.resize(frontalDim);
  separatorScratch_.resize(separatorDim);

  // Cache pointers into the solution for fast back-substitution.
  cacheSolutionPointers(solution, frontals, separatorKeys);

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
  const bool hasRows =
      frontalDim > 0 && factorRows_ >= static_cast<size_t>(frontalDim);
  bool useQR = false;
  if (params.qrMode == MultifrontalParameters::QRMode::Allow) {
    useQR = isLeaf && hasRows &&
            (frontalDim + separatorDim > params.qrAspectRatio * frontalDim);
  } else if (params.qrMode == MultifrontalParameters::QRMode::Force) {
    useQR = isLeaf && hasRows;
  }
  solveMode_ = useQR ? SolveMode::QrLeaf : SolveMode::Cholesky;
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
    info_ = SymmetricBlockMatrix(blockDims_, true);
  }
}

DenseIndex MultifrontalClique::blockIndex(Key key) const {
  const auto it = std::find(orderedKeys_.begin(), orderedKeys_.end(), key);
  assert(it != orderedKeys_.end());
  return static_cast<DenseIndex>(std::distance(orderedKeys_.begin(), it));
}

void MultifrontalClique::cacheSolutionPointers(VectorValues* solution,
                                               const KeyVector& frontals,
                                               const KeySet& separatorKeys) {
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

size_t MultifrontalClique::addJacobianFactor(
    const JacobianFactor& jacobianFactor, size_t rowOffset) {
  // We only overwrite the fixed sparsity pattern, so Ab must be zeroed once in
  // finalize and then kept consistent across loads.
  const size_t rows = jacobianFactor.rows();
  const size_t rhsBlockIdx = Ab_.nBlocks() - 1;
  for (auto it = jacobianFactor.begin(); it != jacobianFactor.end(); ++it) {
    Key k = *it;
    if (fixedKeys_ && fixedKeys_->count(k)) continue;
    const size_t blockIdx = blockIndex(k);
    Ab_(blockIdx).middleRows(rowOffset, rows) = jacobianFactor.getA(it);
  }
  Ab_(rhsBlockIdx).middleRows(rowOffset, rows) = jacobianFactor.getb();

  if (auto model = jacobianFactor.get_model()) {
    if (!model->isConstrained()) {
      // Only whiten non-constrained rows; constrained factors are handled as
      // hard constraints elsewhere.
      model->WhitenInPlace(Ab_.matrix().middleRows(rowOffset, rows));
    }
  }
  return rows;
}

void MultifrontalClique::fillAb(const GaussianFactorGraph& graph) {
  assert(validateFactorKeys(graph, factorIndices_, orderedKeys_, fixedKeys_));

  size_t rowOffset = 0;
  for (size_t index : factorIndices_) {
    assert(index < graph.size());
    const GaussianFactor::shared_ptr& gf = graph[index];
    if (!gf) continue;
    assert(gf->isJacobian() &&
           "MultifrontalClique::fillAb: inconsistent graph passed.");
    auto jacobianFactor = std::static_pointer_cast<JacobianFactor>(gf);
    rowOffset += addJacobianFactor(*jacobianFactor, rowOffset);
  }

  RSdReady_ = false;
  assert((useQR() && RSd_.matrix().rows() ==
                         static_cast<DenseIndex>(Ab_.matrix().rows())) ||
         (RSd_.matrix().rows() == static_cast<DenseIndex>(frontalDim)));
  assert(useQR() || info_.nBlocks() > 0);
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
  assert(info_.nBlocks() > 0);
  info_.setZero();
  if (Ab_.matrix().rows() > 0) {
    info_.selfadjointView().rankUpdate(Ab_.matrix().transpose());
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
  } else {
    info_.choleskyPartial(numFrontals());
    info_.split(numFrontals(), &RSd_);
    info_.blockStart() = 0;
  }
  RSdReady_ = true;
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
    info_.addScaledIdentity(j, lambda);
  }
}

void MultifrontalClique::addDiagonalDamping(double lambda, double minDiagonal,
                                            double maxDiagonal) {
  const size_t nf = numFrontals();
  for (size_t j = 0; j < nf; ++j) {
    const Vector scaled =
        lambda * info_.diagonal(j).cwiseMax(minDiagonal).cwiseMin(maxDiagonal);
    info_.addToDiagonalBlock(j, scaled);
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
    info_.addToDiagonalBlock(j, lambda * diag);
  }
}

void MultifrontalClique::updateParentInfo(
    SymmetricBlockMatrix& parentInfo) const {
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
    child->updateParentInfo(info_);
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
          child->updateParentInfo(
              local);  // No locking: each thread writes its own info matrix.
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
        child->updateParentInfo(
            local);  // No locking: each thread writes its own info matrix.
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

// Solve with block back-substitution on the Cholesky-stored info matrix.
void MultifrontalClique::updateSolution() {
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
