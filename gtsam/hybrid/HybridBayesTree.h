/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    HybridBayesTree.h
 * @brief   Hybrid Bayes Tree, the result of eliminating a HybridJunctionTree
 * @date Mar 11, 2022
 * @author  Fan Jiang
 */

#pragma once

#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/inference/BayesTreeCliqueBase.h>
#include <gtsam/inference/Conditional.h>

#include <string>

namespace gtsam {

// Forward declarations
class HybridConditional;
class VectorValues;

/* ************************************************************************* */
/** A clique in a HybridBayesTree
 * which is a HybridConditional internally.
 */
class GTSAM_EXPORT HybridBayesTreeClique
    : public BayesTreeCliqueBase<HybridBayesTreeClique, HybridFactorGraph> {
 public:
  typedef HybridBayesTreeClique This;
  typedef BayesTreeCliqueBase<HybridBayesTreeClique, HybridFactorGraph> Base;
  typedef boost::shared_ptr<This> shared_ptr;
  typedef boost::weak_ptr<This> weak_ptr;
  HybridBayesTreeClique() {}
  virtual ~HybridBayesTreeClique() {}
  HybridBayesTreeClique(const boost::shared_ptr<HybridConditional>& conditional)
      : Base(conditional) {}
};

/* ************************************************************************* */
/** A Bayes tree representing a Hybrid density */
class GTSAM_EXPORT HybridBayesTree : public BayesTree<HybridBayesTreeClique> {
 private:
  typedef BayesTree<HybridBayesTreeClique> Base;

 public:
  typedef HybridBayesTree This;
  typedef boost::shared_ptr<This> shared_ptr;

  /// @name Standard interface
  /// @{
  /** Default constructor, creates an empty Bayes tree */
  HybridBayesTree() = default;

  /** Check equality */
  bool equals(const This& other, double tol = 1e-9) const;

  /// @}
};

}  // namespace gtsam
