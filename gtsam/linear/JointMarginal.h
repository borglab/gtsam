/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    JointMarginal.h
 * @brief   Block access to joint Gaussian covariance or information matrices
 * @author  Codex
 */

#pragma once

#include <gtsam/linear/Scatter.h>
#include <gtsam/inference/Key.h>
#include <gtsam/base/SymmetricBlockMatrix.h>

namespace gtsam {

class GaussianBayesTree;
class ISAM2;
class Marginals;

/**
 * A class to store and access a joint marginal, returned from Gaussian and
 * nonlinear covariance query APIs.
 */
class GTSAM_EXPORT JointMarginal {
 protected:
  SymmetricBlockMatrix blockMatrix_;
  KeyVector keys_;
  FastMap<Key, size_t> indices_;

 public:
  /// Default constructor only for wrappers
  JointMarginal() {}

  /** Access a block, corresponding to a pair of variables, of the joint
   * marginal. */
  Matrix operator()(Key iVariable, Key jVariable) const {
    const auto indexI = indices_.at(iVariable);
    const auto indexJ = indices_.at(jVariable);
    return blockMatrix_.block(indexI, indexJ);
  }

  /** Synonym for operator() */
  Matrix at(Key iVariable, Key jVariable) const { return (*this)(iVariable, jVariable); }

  /** The full, dense covariance/information matrix of the joint marginal. */
  Matrix fullMatrix() const { return blockMatrix_.selfadjointView(); }

  /** Print */
  void print(const std::string& s = "",
             const KeyFormatter& formatter = DefaultKeyFormatter) const;

  /// Construct from a dense matrix and its block layout.
  JointMarginal(const Matrix& fullMatrix, const Scatter& scatter);
};

}  // namespace gtsam
