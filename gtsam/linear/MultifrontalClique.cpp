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

#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/MultifrontalClique.h>

#include <algorithm>
#include <cassert>
#include <iostream>
#include <stdexcept>

namespace gtsam {

namespace {

constexpr double kConstraintSigmaTol = 1e-12;
constexpr double kConstraintFeasibleTol = 1e-9;

KeyVector orderedKeysFromBlockIndex(const std::map<Key, size_t>& blockIndex) {
  const size_t totalKeys = blockIndex.size();
  KeyVector orderedKeys(totalKeys);
  for (const auto& entry : blockIndex) {
    if (entry.second < totalKeys) {
      orderedKeys[entry.second] = entry.first;
    }
  }
  return orderedKeys;
}

void printKeyRange(std::ostream& os, const KeyVector& keys, size_t start,
                   size_t end, const KeyFormatter& formatter) {
  os << "[";
  for (size_t i = start; i < end; ++i) {
    os << formatter(keys[i]);
    if (i + 1 < end) os << ", ";
  }
  os << "]";
}

// Mark frontal variables that are fixed by the given constrained factor.
void markFixedFrontals(const JacobianFactor& factor,
                       const std::map<Key, size_t>& blockIndex,
                       std::unordered_set<size_t>* fixedFrontals) {
  if (factor.size() != 1) {
    throw std::runtime_error(
        "MultifrontalSolver: only unary constrained factors are supported.");
  }
  const auto model = factor.get_model();
  const Vector sigmas = model->sigmas();
  if (!(sigmas.array().abs() <= kConstraintSigmaTol).all()) {
    throw std::runtime_error(
        "MultifrontalSolver: only fully constrained factors are supported.");
  }
  if (factor.getb().array().abs().maxCoeff() > kConstraintFeasibleTol) {
    throw std::runtime_error(
        "MultifrontalSolver: constrained factor is not feasible.");
  }
  const Key key = *factor.begin();
  const size_t blockIndexValue = blockIndex.at(key);
  fixedFrontals->insert(blockIndexValue);
}

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
    std::vector<size_t> factorIndices,
    const std::weak_ptr<MultifrontalClique>& parent) {
  factorIndices_ = std::move(factorIndices);
  this->parent = parent;
}

size_t MultifrontalClique::factorCount() const { return factorIndices_.size(); }

std::shared_ptr<GaussianConditional> MultifrontalClique::conditional() const {
  const KeyVector keys = orderedKeysFromBlockIndex(blockIndex_);
  SymmetricBlockMatrix& sbm = sbm_;
  const DenseIndex oldBlockStart = sbm.blockStart();
  VerticalBlockMatrix Ab = sbm.split(numFrontals_);
  sbm.blockStart() = oldBlockStart;
  return std::make_shared<GaussianConditional>(keys, numFrontals_,
                                               std::move(Ab));
}

void MultifrontalClique::finalize(const KeyVector& frontals,
                                  const KeyVector& separatorKeys,
                                  const std::map<Key, size_t>& dims,
                                  const GaussianFactorGraph& graph,
                                  VectorValues* solution,
                                  std::vector<ChildInfo> children) {
  if (frontals.empty()) {
    throw std::runtime_error(
        "MultifrontalSolver: cluster has no frontal keys.");
  }
  numFrontals_ = frontals.size();
  this->children.clear();
  this->children.reserve(children.size());
  for (const auto& child : children) {
    this->children.push_back(child.clique);
  }

  // Cache the mapping from key to Ab block index for fast fills.
  blockIndex_.clear();
  size_t blockIdx = 0;
  for (Key key : frontals) {
    blockIndex_[key] = blockIdx;
    ++blockIdx;
  }
  for (Key key : separatorKeys) {
    blockIndex_[key] = blockIdx;
    ++blockIdx;
  }

  size_t dim = 0;
  for (Key key : frontals) {
    auto it = dims.find(key);
    if (it != dims.end()) dim += it->second;
  }
  frontalDim = dim;

  dim = 0;
  for (Key key : separatorKeys) {
    auto it = dims.find(key);
    if (it != dims.end()) dim += it->second;
  }
  separatorDim = dim;

  // Cache pointers into the solution for fast back-substitution.
  cacheSolutionPointers(solution, frontals, separatorKeys);

  // Compute parent indices for all children.
  for (const auto& child : children) {
    if (!child.clique) continue;
    std::vector<size_t> indices;
    indices.reserve(child.separatorKeys.size() + 1);
    for (Key key : child.separatorKeys) {
      auto it = blockIndex_.find(key);
      if (it == blockIndex_.end()) {
        throw std::runtime_error(
            "MultifrontalSolver: separator key not found in parent clique");
      }
      indices.push_back(it->second);
    }
    indices.push_back(blockIndex_.size());
    child.clique->setParentIndices(indices);
  }

  // Pre-allocate matrices and cache constraints once per structure.
  std::vector<size_t> blockDims =
      this->blockDims(dims, frontals, separatorKeys);
  size_t vbmRows = countRows(graph);
  initializeMatrices(blockDims, vbmRows);
  cacheConstraintInfo(graph);
}

void MultifrontalClique::cacheConstraintInfo(const GaussianFactorGraph& graph) {
  fixedFrontals_.clear();

  for (size_t index : factorIndices_) {
    assert(index < graph.size());
    const GaussianFactor::shared_ptr& gf = graph[index];
    if (!gf) continue;
    if (auto jacobianFactor = std::dynamic_pointer_cast<JacobianFactor>(gf)) {
      if (auto model = jacobianFactor->get_model()) {
        if (model->isConstrained()) {
          markFixedFrontals(*jacobianFactor, blockIndex_, &fixedFrontals_);
        }
      }
    }
  }
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
    const std::map<Key, size_t>& dims, const KeyVector& frontals,
    const KeyVector& separatorKeys) const {
  std::vector<size_t> blockDims;
  for (Key k : frontals) blockDims.push_back(dims.at(k));
  for (Key k : separatorKeys) blockDims.push_back(dims.at(k));
  return blockDims;
}

size_t MultifrontalClique::countRows(const GaussianFactorGraph& graph) const {
  size_t vbmRows = 0;
  for (size_t index : factorIndices_) {
    assert(index < graph.size());
    if (auto jacobianFactor =
            std::dynamic_pointer_cast<JacobianFactor>(graph[index])) {
      vbmRows += jacobianFactor->rows();
    }
  }
  return vbmRows;
}

void MultifrontalClique::initializeMatrices(
    const std::vector<size_t>& blockDims, size_t verticalBlockMatrixRows) {
  sbm_ = SymmetricBlockMatrix(blockDims, true);
  Ab_ = VerticalBlockMatrix(blockDims, verticalBlockMatrixRows, true);
  Ab_.matrix().setZero();
}

void MultifrontalClique::addJacobianFactor(const JacobianFactor& jacobianFactor,
                                           size_t rowOffset) {
  // We only overwrite the fixed sparsity pattern, so Ab must be zeroed once in
  // initializeMatrices and then kept consistent across loads.
  const size_t rows = jacobianFactor.rows();
  const size_t rhsBlockIdx = Ab_.nBlocks() - 1;
  for (auto it = jacobianFactor.begin(); it != jacobianFactor.end(); ++it) {
    Key k = *it;
    const size_t blockIdx = blockIndex_.at(k);
    Ab_(blockIdx).middleRows(rowOffset, rows) = jacobianFactor.getA(it);
  }
  Ab_(rhsBlockIdx).middleRows(rowOffset, rows) = jacobianFactor.getb();

  if (auto model = jacobianFactor.get_model()) {
    if (!model->isConstrained()) {
      model->WhitenInPlace(Ab_.matrix().middleRows(rowOffset, rows));
    }
  }
}

void MultifrontalClique::addHessianFactor(const HessianFactor& hessianFactor) {
  const SymmetricBlockMatrix& info = hessianFactor.info();
  const size_t factorBlocks = hessianFactor.size();

  std::vector<DenseIndex> blockIndices(factorBlocks + 1);
  size_t slot = 0;
  for (auto it = hessianFactor.begin(); it != hessianFactor.end();
       ++it, ++slot) {
    blockIndices[slot] = static_cast<DenseIndex>(blockIndex_.at(*it));
  }
  blockIndices[factorBlocks] = static_cast<DenseIndex>(sbm_.nBlocks() - 1);
  sbm_.updateFromMappedBlocks(info, blockIndices);
}

void MultifrontalClique::fillAb(const GaussianFactorGraph& graph) {
  sbm_.setZero();

  size_t rowOffset = 0;
  for (size_t index : factorIndices_) {
    assert(index < graph.size());
    const GaussianFactor::shared_ptr& gf = graph[index];
    if (!gf) continue;
    if (auto jacobianFactor = std::dynamic_pointer_cast<JacobianFactor>(gf)) {
      addJacobianFactor(*jacobianFactor, rowOffset);
      rowOffset += jacobianFactor->rows();
    } else if (auto hessianFactor =
                   std::dynamic_pointer_cast<HessianFactor>(gf)) {
      addHessianFactor(*hessianFactor);
    }
  }
}

void MultifrontalClique::eliminateInPlace() {
  // Update SBM with the local factors, Ab^T * Ab
  sbm_.selfadjointView().rankUpdate(Ab_.matrix().transpose());

  for (const auto& child : children) {
    if (!child) continue;
    child->updateParent(*this);
  }

  if (!fixedFrontals_.empty()) {
    const DenseIndex nBlocks = sbm_.nBlocks();
    for (size_t fixedBlock : fixedFrontals_) {
      const DenseIndex i = static_cast<DenseIndex>(fixedBlock);
      const DenseIndex dimI = sbm_.getDim(i);
      sbm_.setDiagonalBlock(i, Matrix::Identity(dimI, dimI));
      for (DenseIndex j = 0; j < nBlocks; ++j) {
        if (j == i) continue;
        const DenseIndex dimJ = sbm_.getDim(j);
        sbm_.setOffDiagonalBlock(i, j, Matrix::Zero(dimI, dimJ));
      }
    }
  }

  // Form normal equations and factor the frontal block (Schur complement step).
  sbm_.choleskyPartial(numFrontals_);
}

void MultifrontalClique::updateParent(MultifrontalClique& parent) const {
  // Expose only the separator+RHS view when contributing to the parent.
  sbm_.blockStart() = numFrontals_;
  assert(sbm_.nBlocks() == parentIndices_.size());
  parent.sbm_.updateFromMappedBlocks(sbm_, parentIndices_);
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
  const KeyVector orderedKeys = orderedKeysFromBlockIndex(blockIndex_);
  std::cout << "Clique(frontals=[";
  printKeyRange(std::cout, orderedKeys, 0,
                std::min(numFrontals_, orderedKeys.size()), keyFormatter);
  std::cout << "], separators=[";
  printKeyRange(std::cout, orderedKeys,
                std::min(numFrontals_, orderedKeys.size()), orderedKeys.size(),
                keyFormatter);
  std::cout << "], factors=" << factorIndices_.size()
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
  const KeyVector orderedKeys = orderedKeysFromBlockIndex(clique.blockIndex_);
  const KeyFormatter formatter = DefaultKeyFormatter;
  os << "Clique(frontals=";
  printKeyRange(os, orderedKeys, 0,
                std::min(clique.numFrontals_, orderedKeys.size()), formatter);
  os << ", separators=";
  printKeyRange(os, orderedKeys,
                std::min(clique.numFrontals_, orderedKeys.size()),
                orderedKeys.size(), formatter);
  os << ", factors=" << clique.factorCount();
  os << ", children=" << clique.children.size();
  os << ", sbmBlocks=" << clique.sbm().nBlocks();
  os << ", AbRows=" << clique.Ab().matrix().rows() << ")";
  return os;
}

}  // namespace gtsam
