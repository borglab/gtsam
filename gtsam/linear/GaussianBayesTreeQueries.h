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
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/JointMarginal.h>

#include <Eigen/Cholesky>

namespace gtsam {
namespace internal {

/* ************************************************************************* */
/** Return unique keys in their first-occurrence order. */
inline KeyVector uniqueKeys(const KeyVector& keys) {
  KeyVector unique;
  unique.reserve(keys.size());
  KeySet seen;
  for (Key key : keys) {
    if (seen.insert(key).second) {
      unique.push_back(key);
    }
  }
  return unique;
}

/* ************************************************************************* */
/** Return the scalar offsets corresponding to a sequence of block dimensions.
 */
inline std::vector<size_t> blockOffsets(const std::vector<size_t>& dims) {
  std::vector<size_t> offsets(dims.size() + 1, 0);
  for (size_t i = 0; i < dims.size(); ++i) {
    offsets[i + 1] = offsets[i] + dims[i];
  }
  return offsets;
}

/* ************************************************************************* */
/**
 * Convert information to covariance using Cholesky solves, returning zeros
 * when the information contains non-finite entries.
 */
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
/** Return variable dimensions from a Bayes net in the requested key order. */
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
/** Construct a scatter with entries in the supplied key and dimension order. */
inline Scatter scatterFromKeysAndDims(const KeyVector& orderedKeys,
                                      const std::vector<size_t>& dims) {
  Scatter scatter;
  for (size_t index = 0; index < orderedKeys.size(); ++index) {
    scatter.add(orderedKeys[index], dims.at(index));
  }
  return scatter;
}

/* ************************************************************************* */
/** Compute covariance blocks in the requested key order using triangular solves
 * in the Bayes net's elimination order.
 */
inline Matrix covarianceColumns(const GaussianBayesNet& bayesNet,
                                const KeyVector& orderedKeys,
                                const std::vector<size_t>& dims) {
  const KeyVector factorKeys = bayesNet.ordering();
  const std::vector<size_t> factorDims = dimsFromBayesNet(bayesNet, factorKeys);
  const auto [R, rhs] = bayesNet.matrix(Ordering(factorKeys));
  (void)rhs;

  const std::vector<size_t> factorOffsets = blockOffsets(factorDims);
  const std::vector<size_t> outputOffsets = blockOffsets(dims);
  const size_t totalDim = factorOffsets.back();

  FastMap<Key, size_t> factorIndex;
  for (size_t index = 0; index < factorKeys.size(); ++index) {
    factorIndex[factorKeys[index]] = index;
  }

  // Solve for covariance columns in elimination order, with output columns
  // laid out in the requested order.
  Matrix selectors = Matrix::Zero(totalDim, totalDim);
  for (size_t outputIndex = 0; outputIndex < orderedKeys.size();
       ++outputIndex) {
    const size_t factorIndexForKey = factorIndex.at(orderedKeys[outputIndex]);
    const size_t dim = dims.at(outputIndex);
    selectors
        .block(factorOffsets[factorIndexForKey], outputOffsets[outputIndex],
               dim, dim)
        .setIdentity();
  }

  R.transpose().triangularView<Eigen::Lower>().solveInPlace(selectors);
  R.triangularView<Eigen::Upper>().solveInPlace(selectors);

  // Gather rows in the requested order without changing the factorization
  // ordering used by the triangular solves above.
  Matrix covariance = Matrix::Zero(totalDim, totalDim);
  for (size_t outputRow = 0; outputRow < orderedKeys.size(); ++outputRow) {
    const size_t factorRow = factorIndex.at(orderedKeys[outputRow]);
    const size_t rowDim = dims.at(outputRow);
    for (size_t outputColumn = 0; outputColumn < orderedKeys.size();
         ++outputColumn) {
      const size_t columnDim = dims.at(outputColumn);
      covariance.block(outputOffsets[outputRow], outputOffsets[outputColumn],
                       rowDim, columnDim) =
          selectors.block(factorOffsets[factorRow], outputOffsets[outputColumn],
                          rowDim, columnDim);
    }
  }
  return covariance;
}

/* ************************************************************************* */
/** Construct a joint marginal from a dense matrix and ordered block metadata.
 */
inline JointMarginal jointMarginalFromMatrix(const Matrix& matrix,
                                             const KeyVector& orderedKeys,
                                             const std::vector<size_t>& dims) {
  return JointMarginal(matrix, scatterFromKeysAndDims(orderedKeys, dims));
}

/* ************************************************************************* */
/** Construct an empty joint marginal. */
inline JointMarginal emptyJointMarginal() {
  return JointMarginal(Matrix(), Scatter());
}

/* ************************************************************************* */
/**
 * Build a joint marginal while sharing empty, single-key, and multi-key query
 * handling between information and covariance calculations.
 */
template <class BAYESTREE, class SINGLE_BUILDER, class MULTI_BUILDER>
JointMarginal buildJointMarginal(
    const BAYESTREE& bayesTree, const KeyVector& queryKeys,
    const typename BAYESTREE::FactorGraphType::Eliminate& eliminate,
    const SINGLE_BUILDER& singleBuilder, const MULTI_BUILDER& multiBuilder) {
  const KeyVector orderedKeys = uniqueKeys(queryKeys);
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
/** Return marginal information for one key using the requested elimination. */
template <class BAYESTREE>
Matrix marginalInformation(
    const BAYESTREE& bayesTree, Key key,
    const typename BAYESTREE::FactorGraphType::Eliminate& eliminate) {
  return bayesTree.marginalFactor(key, eliminate)->information();
}

/* ************************************************************************* */
/** Return joint marginal information with blocks in query-key order. */
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
/** Return joint marginal covariance with blocks in query-key order. */
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
        return jointMarginalFromMatrix(
            covarianceColumns(bayesNet, orderedKeys, dims), orderedKeys, dims);
      });
}

}  // namespace internal
}  // namespace gtsam
