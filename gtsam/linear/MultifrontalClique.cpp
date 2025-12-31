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

#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/MultifrontalClique.h>

#include <algorithm>
#include <cassert>
#include <iostream>
#include <stdexcept>

namespace gtsam {

namespace {

// Build a stacked separator vector x_sep in the provided scratch buffer.
Vector& buildSeparatorVector(const std::vector<const Vector*>& separatorPtrs,
                             Vector* scratch) {
  size_t separatorDim = 0;
  for (const Vector* values : separatorPtrs) {
    separatorDim += values->size();
  }
  if (static_cast<size_t>(scratch->size()) != separatorDim) {
    scratch->resize(separatorDim);
  }
  size_t offset = 0;
  for (const Vector* values : separatorPtrs) {
    scratch->segment(offset, values->size()) = *values;
    offset += values->size();
  }
  return *scratch;
}

}  // namespace

MultifrontalClique::MultifrontalClique(
    const SymbolicJunctionTree::sharedNode& cluster,
    const std::weak_ptr<MultifrontalClique>& parent) {
  if (!cluster) {
    throw std::runtime_error("MultifrontalSolver: null cluster.");
  }
  cluster_ = cluster;
  this->parent = parent;
  const auto& frontals = cluster_->orderedFrontalKeys;
  if (frontals.empty()) {
    throw std::runtime_error(
        "MultifrontalSolver: cluster has no frontal keys.");
  }
}

void MultifrontalClique::addChild(const shared_ptr& child) {
  children.push_back(child);
}

const KeyVector& MultifrontalClique::frontals() const {
  return cluster_->orderedFrontalKeys;
}

size_t MultifrontalClique::factorCount() const {
  return cluster_->factors.size();
}

void MultifrontalClique::finalize(const std::map<Key, size_t>& dims,
                                  VectorValues* solution) {
  calculateSeparatorKeys();

  // Cache the mapping from key to Ab block index for fast fills.
  blockIndex_.clear();
  size_t blockIdx = 0;
  for (Key key : frontals()) {
    blockIndex_[key] = blockIdx;
    ++blockIdx;
  }
  for (Key key : separatorKeys_) {
    blockIndex_[key] = blockIdx;
    ++blockIdx;
  }

  size_t dim = 0;
  for (Key key : frontals()) {
    auto it = dims.find(key);
    if (it != dims.end()) dim += it->second;
  }
  frontalDim = dim;

  dim = 0;
  for (Key key : separatorKeys_) {
    auto it = dims.find(key);
    if (it != dims.end()) dim += it->second;
  }
  separatorDim = dim;

  // Cache pointers into the solution for fast back-substitution.
  cacheSolutionPointers(solution);

  // Compute parent indices for all children.
  for (const auto& child : children) {
    if (!child) continue;
    child->setParentIndices(child->parentIndicesFor(*this));
  }
}

void MultifrontalClique::calculateSeparatorKeys() {
  // Separator keys are computed from local factor keys and child separators.
  KeySet allKeys;
  for (const auto& factor : cluster_->factors) {
    assert(factor);
    allKeys.insert(factor->begin(), factor->end());
  }
  for (const auto& child : children) {
    if (!child) continue;
    allKeys.insert(child->separatorKeys_.begin(), child->separatorKeys_.end());
  }
  for (Key k : frontals()) {
    allKeys.erase(k);
  }
  separatorKeys_.assign(allKeys.begin(), allKeys.end());
}

void MultifrontalClique::cacheSolutionPointers(VectorValues* solution) {
  frontalPtrs_.clear();
  separatorPtrs_.clear();
  frontalPtrs_.reserve(frontals().size());
  separatorPtrs_.reserve(separatorKeys_.size());
  for (Key key : frontals()) {
    frontalPtrs_.push_back(&solution->at(key));
  }
  for (Key key : separatorKeys_) {
    separatorPtrs_.push_back(&solution->at(key));
  }
}

const KeyVector& MultifrontalClique::separatorKeys() const {
  return separatorKeys_;
}

std::vector<size_t> MultifrontalClique::blockDims(
    const std::map<Key, size_t>& dims) const {
  std::vector<size_t> blockDims;
  for (Key k : frontals()) blockDims.push_back(dims.at(k));
  for (Key k : separatorKeys_) blockDims.push_back(dims.at(k));
  return blockDims;
}

size_t MultifrontalClique::countRows(const GaussianFactorGraph& graph) const {
  size_t vbmRows = 0;
  for (const auto& factor : cluster_->factors) {
    assert(factor);
    auto indexed =
        std::static_pointer_cast<internal::IndexedSymbolicFactor>(factor);
    if (auto jacobianFactor =
            std::dynamic_pointer_cast<JacobianFactor>(graph[indexed->index_])) {
      vbmRows += jacobianFactor->rows();
    }
  }
  return vbmRows;
}

std::vector<size_t> MultifrontalClique::parentIndicesFor(
    const MultifrontalClique& parent) const {
  std::vector<size_t> indices;
  for (Key k : separatorKeys_) {
    auto fIt = std::find(parent.frontals().begin(), parent.frontals().end(), k);
    if (fIt != parent.frontals().end()) {
      indices.push_back(std::distance(parent.frontals().begin(), fIt));
      continue;
    }
    auto sIt = std::find(parent.separatorKeys_.begin(),
                         parent.separatorKeys_.end(), k);
    if (sIt != parent.separatorKeys_.end()) {
      indices.push_back(parent.frontals().size() +
                        std::distance(parent.separatorKeys_.begin(), sIt));
      continue;
    }
    throw std::runtime_error(
        "MultifrontalSolver: separator key not found in parent clique");
  }
  return indices;
}

void MultifrontalClique::initializeMatrices(
    const std::vector<size_t>& blockDims, size_t verticalBlockMatrixRows) {
  sbm_ = SymmetricBlockMatrix(blockDims, true);
  Ab_ = VerticalBlockMatrix(blockDims, verticalBlockMatrixRows, true);
  Ab_.matrix().setZero();
}

void MultifrontalClique::fillAb(const GaussianFactorGraph& graph) {
  sbm_.setZero();

  // We only overwrite the fixed sparsity pattern, so Ab must be zeroed once in
  // initializeMatrices and then kept consistent across loads.
  size_t rowOffset = 0;
  const size_t rhsBlockIdx = Ab_.nBlocks() - 1;
  for (const auto& factor : cluster_->factors) {
    assert(factor);
    auto indexed =
        std::static_pointer_cast<internal::IndexedSymbolicFactor>(factor);
    auto jacobianFactor =
        std::dynamic_pointer_cast<JacobianFactor>(graph[indexed->index_]);
    if (!jacobianFactor) continue;

    const size_t rows = jacobianFactor->rows();
    for (auto it = jacobianFactor->begin(); it != jacobianFactor->end(); ++it) {
      Key k = *it;
      auto blockIt = blockIndex_.find(k);
      if (blockIt == blockIndex_.end()) continue;
      const size_t blockIdx = blockIt->second;
      Ab_(blockIdx).middleRows(rowOffset, jacobianFactor->rows()) =
          jacobianFactor->getA(it);
    }
    Ab_(rhsBlockIdx).middleRows(rowOffset, rows) = jacobianFactor->getb();

    if (auto model = jacobianFactor->get_model()) {
      model->WhitenInPlace(Ab_.matrix().middleRows(rowOffset, rows));
    }

    rowOffset += rows;
  }
}

void MultifrontalClique::eliminateInPlace() {
  // Update SBM with the local factors, Ab^T * Ab
  sbm_.selfadjointView().rankUpdate(Ab_.matrix().transpose());

  for (const auto& child : children) {
    if (!child) continue;
    child->updateParent(*this);
  }

  // Form normal equations and factor the frontal block (Schur complement step).
  sbm_.choleskyPartial(frontals().size());
}

void MultifrontalClique::updateParent(MultifrontalClique& parent) const {
  // Expose only the separator+RHS view when contributing to the parent.
  sbm_.blockStart() = frontals().size();
  const size_t numBlocks = parentIndices_.size();
  assert(sbm_.nBlocks() == numBlocks + 1);
  const size_t rhsBlock = parent.sbm_.nBlocks() - 1;

  for (size_t i = 0; i <= numBlocks; ++i) {
    const size_t p_i = (i < numBlocks) ? parentIndices_[i] : rhsBlock;
    parent.sbm_.updateDiagonalBlock(p_i, sbm_.diagonalBlock(i));
    for (size_t j = i + 1; j <= numBlocks; ++j) {
      const size_t p_j = (j < numBlocks) ? parentIndices_[j] : rhsBlock;
      parent.sbm_.updateOffDiagonalBlock(p_i, p_j,
                                         sbm_.aboveDiagonalBlock(i, j));
    }
  }
  sbm_.blockStart() = 0;
}

void MultifrontalClique::updateSolution() const {
  // Solve with block back-substitution on the Cholesky-stored SBM, avoiding
  // materializing an explicit R matrix or split representation.
  const size_t nFrontals = frontalPtrs_.size();
  const size_t nSeparators = separatorPtrs_.size();

  const size_t rhsBlock = nFrontals + nSeparators;

  size_t frontalDim = 0;
  for (const Vector* values : frontalPtrs_) {
    frontalDim += values->size();
  }
  if (static_cast<size_t>(rhsScratch_.size()) != frontalDim) {
    rhsScratch_.resize(frontalDim);
  }
  rhsScratch_.noalias() =
      sbm_.aboveDiagonalRange(0, nFrontals, rhsBlock, rhsBlock + 1);

  // Eliminate separator contributions: b -= S * x_sep.
  if (nSeparators > 0) {
    const Vector& xSep =
        buildSeparatorVector(separatorPtrs_, &separatorScratch_);
    rhsScratch_.noalias() -= sbm_.aboveDiagonalRange(0, nFrontals, nFrontals,
                                                     nFrontals + nSeparators) *
                             xSep;
  }

  // Solve the contiguous frontal system in one triangular solve.
  sbm_.triangularView(0, nFrontals).solveInPlace(rhsScratch_);

  // Write solved frontal blocks back into the global solution.
  size_t offset = 0;
  for (Vector* values : frontalPtrs_) {
    const size_t dim = values->size();
    values->noalias() = rhsScratch_.segment(offset, dim);
    offset += dim;
  }
}

void MultifrontalClique::print(const std::string& s,
                               const KeyFormatter& keyFormatter) const {
  if (!s.empty()) std::cout << s;
  std::cout << "Clique(frontals=[";
  for (size_t i = 0; i < frontals().size(); ++i) {
    std::cout << keyFormatter(frontals()[i]);
    if (i + 1 < frontals().size()) std::cout << ", ";
  }
  std::cout << "], separators=[";
  for (size_t i = 0; i < separatorKeys_.size(); ++i) {
    std::cout << keyFormatter(separatorKeys_[i]);
    if (i + 1 < separatorKeys_.size()) std::cout << ", ";
  }
  std::cout << "], factors=" << cluster_->factors.size()
            << ", children=" << children.size()
            << ", sbmBlocks=" << sbm_.nBlocks()
            << ", AbRows=" << Ab_.matrix().rows() << ")\n";

  auto assembleSbm = [](const SymmetricBlockMatrix& sbm) {
    const size_t nBlocks = sbm.nBlocks();
    std::vector<size_t> offsets(nBlocks + 1, 0);
    for (size_t i = 0; i < nBlocks; ++i) {
      offsets[i + 1] = offsets[i] + sbm.getDim(i);
    }
    Matrix full = Matrix::Zero(offsets.back(), offsets.back());
    for (size_t i = 0; i < nBlocks; ++i) {
      for (size_t j = 0; j < nBlocks; ++j) {
        Matrix block = sbm.block(i, j);
        full.block(offsets[i], offsets[j], block.rows(), block.cols()) = block;
      }
    }
    return full;
  };

  std::cout << "  Ab:\n" << Ab_.matrix() << "\n";
  std::cout << "  SBM:\n" << assembleSbm(sbm_) << "\n";
}

std::ostream& operator<<(std::ostream& os, const MultifrontalClique& clique) {
  const KeyFormatter formatter = DefaultKeyFormatter;
  auto printKeys = [&](const KeyVector& keys) {
    os << "[";
    for (size_t i = 0; i < keys.size(); ++i) {
      os << formatter(keys[i]);
      if (i + 1 < keys.size()) os << ", ";
    }
    os << "]";
  };

  os << "Clique(frontals=";
  printKeys(clique.frontals());
  os << ", separators=";
  printKeys(clique.separatorKeys());
  os << ", factors=" << clique.factorCount();
  os << ", children=" << clique.children.size();
  os << ", sbmBlocks=" << clique.sbm().nBlocks();
  os << ", AbRows=" << clique.Ab().matrix().rows() << ")";
  return os;
}

}  // namespace gtsam
