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
   * marginal. Note: iVariable and jVariable are the actual nonlinear Keys
   * used to construct this JointMarginal (the keys passed to
   * jointMarginalCovariance()/jointMarginalInformation()), NOT positional
   * indices into the block matrix. To retrieve the single variable
   * marginal for `key` (equivalent to Marginals::marginalCovariance(key)),
   * call at(key, key), not at(0, key) or any other cross covariance
   * block, which represents the covariance *between* two different
   * variables and will generally differ. */
  Matrix operator()(Key iVariable, Key jVariable) const {
    const auto indexI = indices_.at(iVariable);
    const auto indexJ = indices_.at(jVariable);
    return blockMatrix_.block(indexI, indexJ);
  }

  /** Synonym for operator() See operator() for indexing semantics. */
  Matrix at(Key iVariable, Key jVariable) const { return (*this)(iVariable, jVariable); }

  /** The full, dense covariance/information matrix of the joint marginal.
   * Blocks follow the order of the keys in the query that created this object.
   */
  Matrix fullMatrix() const { return blockMatrix_.selfadjointView(); }

  /** Print */
  void print(const std::string& s = "",
             const KeyFormatter& formatter = DefaultKeyFormatter) const;

  /// Construct from a dense matrix and its block layout.
  JointMarginal(const Matrix& fullMatrix, const Scatter& scatter);
};

}  // namespace gtsam
