/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianBayesTreeQueries.h
 * @brief   Internal helpers for Gaussian Bayes-tree covariance queries
 * @author  Codex
 */

#pragma once

#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/JointMarginal.h>

#include <algorithm>
#include <numeric>

namespace gtsam {
namespace internal {

/* ************************************************************************* */
inline KeyVector uniqueSortedKeys(const KeyVector& keys) {
  KeyVector result = keys;
  std::sort(result.begin(), result.end());
  result.erase(std::unique(result.begin(), result.end()), result.end());
  return result;
}

/* ************************************************************************* */
inline std::vector<size_t> blockOffsets(const std::vector<size_t>& dims) {
  std::vector<size_t> offsets(dims.size() + 1, 0);
  for (size_t i = 0; i < dims.size(); ++i) {
    offsets[i + 1] = offsets[i] + dims[i];
  }
  return offsets;
}

/* ************************************************************************* */
inline Matrix informationToCovariance(const Matrix& information) {
  if (!information.allFinite()) {
    return Matrix::Zero(information.rows(), information.cols());
  }

  Eigen::LLT<Matrix> llt(information.selfadjointView<Eigen::Upper>());
  Matrix covariance = Matrix::Identity(information.rows(), information.cols());
  llt.solveInPlace(covariance);
  return covariance;
}

/* ************************************************************************* */
inline std::vector<size_t> dimsFromBayesNet(const GaussianBayesNet& bayesNet,
                                            const KeyVector& orderedKeys) {
  FastMap<Key, size_t> dimsByKey;
  for (const auto& conditional : bayesNet) {
    for (auto key = conditional->beginFrontals();
         key != conditional->endFrontals(); ++key) {
      dimsByKey[*key] = static_cast<size_t>(conditional->getDim(key));
    }
  }

  std::vector<size_t> dims;
  dims.reserve(orderedKeys.size());
  for (Key key : orderedKeys) {
    dims.push_back(dimsByKey.at(key));
  }
  return dims;
}

/* ************************************************************************* */
inline Scatter scatterFromKeysAndDims(const KeyVector& orderedKeys,
                                      const std::vector<size_t>& dims) {
  Scatter scatter;
  for (size_t index = 0; index < orderedKeys.size(); ++index) {
    scatter.add(orderedKeys[index], dims.at(index));
  }
  return scatter;
}

/* ************************************************************************* */
inline Matrix covarianceColumns(const GaussianBayesNet& bayesNet,
                                const KeyVector& orderedKeys,
                                const std::vector<size_t>& dims,
                                const std::vector<size_t>& selectedBlocks) {
  const auto [R, rhs] = bayesNet.matrix(Ordering(orderedKeys));
  (void)rhs;

  const std::vector<size_t> offsets = blockOffsets(dims);
  const size_t totalDim = offsets.back();

  size_t selectedDim = 0;
  for (size_t blockIndex : selectedBlocks) {
    selectedDim += dims.at(blockIndex);
  }

  Matrix selectors = Matrix::Zero(totalDim, selectedDim);
  size_t selectedOffset = 0;
  for (size_t blockIndex : selectedBlocks) {
    const size_t begin = offsets[blockIndex];
    const size_t dim = dims[blockIndex];
    selectors.block(begin, selectedOffset, dim, dim).setIdentity();
    selectedOffset += dim;
  }

  R.transpose().triangularView<Eigen::Lower>().solveInPlace(selectors);
  R.triangularView<Eigen::Upper>().solveInPlace(selectors);
  return selectors;
}

/* ************************************************************************* */
inline JointMarginal jointMarginalFromMatrix(const Matrix& matrix,
                                             const KeyVector& orderedKeys,
                                             const std::vector<size_t>& dims) {
  return JointMarginal(matrix, scatterFromKeysAndDims(orderedKeys, dims));
}

/* ************************************************************************* */
inline JointMarginal emptyJointMarginal() {
  return JointMarginal(Matrix(), Scatter());
}

/* ************************************************************************* */
template <class BAYESTREE, class SINGLE_BUILDER, class MULTI_BUILDER>
JointMarginal buildJointMarginal(
    const BAYESTREE& bayesTree, const KeyVector& queryKeys,
    const typename BAYESTREE::FactorGraphType::Eliminate& eliminate,
    const SINGLE_BUILDER& singleBuilder, const MULTI_BUILDER& multiBuilder) {
  const KeyVector orderedKeys = uniqueSortedKeys(queryKeys);
  if (orderedKeys.empty()) {
    return emptyJointMarginal();
  }

  if (orderedKeys.size() == 1) {
    const Matrix matrix = singleBuilder(orderedKeys.front());
    return jointMarginalFromMatrix(matrix, orderedKeys,
                                   {static_cast<size_t>(matrix.rows())});
  }

  const GaussianBayesNet bayesNet =
      *bayesTree.jointBayesNet(orderedKeys, eliminate);
  return multiBuilder(bayesNet, orderedKeys);
}

/* ************************************************************************* */
template <class BAYESTREE>
Matrix marginalInformation(
    const BAYESTREE& bayesTree, Key key,
    const typename BAYESTREE::FactorGraphType::Eliminate& eliminate) {
  return bayesTree.marginalFactor(key, eliminate)->information();
}

/* ************************************************************************* */
template <class BAYESTREE>
JointMarginal jointMarginalInformation(
    const BAYESTREE& bayesTree, const KeyVector& queryKeys,
    const typename BAYESTREE::FactorGraphType::Eliminate& eliminate) {
  return buildJointMarginal(
      bayesTree, queryKeys, eliminate,
      [&bayesTree, &eliminate](Key key) {
        return marginalInformation(bayesTree, key, eliminate);
      },
      [](const GaussianBayesNet& bayesNet, const KeyVector& orderedKeys) {
        const auto [R, rhs] = bayesNet.matrix(Ordering(orderedKeys));
        (void)rhs;
        return jointMarginalFromMatrix(R.transpose() * R, orderedKeys,
                                       dimsFromBayesNet(bayesNet, orderedKeys));
      });
}

/* ************************************************************************* */
template <class BAYESTREE>
JointMarginal jointMarginalCovariance(
    const BAYESTREE& bayesTree, const KeyVector& queryKeys,
    const typename BAYESTREE::FactorGraphType::Eliminate& eliminate) {
  return buildJointMarginal(
      bayesTree, queryKeys, eliminate,
      [&bayesTree, &eliminate](Key key) {
        return informationToCovariance(
            marginalInformation(bayesTree, key, eliminate));
      },
      [](const GaussianBayesNet& bayesNet, const KeyVector& orderedKeys) {
        const std::vector<size_t> dims =
            dimsFromBayesNet(bayesNet, orderedKeys);
        std::vector<size_t> allBlocks(orderedKeys.size());
        std::iota(allBlocks.begin(), allBlocks.end(), 0);
        return jointMarginalFromMatrix(
            covarianceColumns(bayesNet, orderedKeys, dims, allBlocks),
            orderedKeys, dims);
      });
}

}  // namespace internal
}  // namespace gtsam
