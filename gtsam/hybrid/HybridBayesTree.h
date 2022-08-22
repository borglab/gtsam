/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    HybridBayesTree.h
 * @brief   Hybrid Bayes Tree, the result of eliminating a
 * HybridJunctionTree
 * @date Mar 11, 2022
 * @author  Fan Jiang
 */

#pragma once

#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
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
    : public BayesTreeCliqueBase<HybridBayesTreeClique,
                                 HybridGaussianFactorGraph> {
 public:
  typedef HybridBayesTreeClique This;
  typedef BayesTreeCliqueBase<HybridBayesTreeClique, HybridGaussianFactorGraph>
      Base;
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

/**
 * @brief Class for Hybrid Bayes tree orphan subtrees.
 *
 * This does special stuff for the hybrid case
 *
 * @tparam CLIQUE
 */
template <class CLIQUE>
class BayesTreeOrphanWrapper<
    CLIQUE, typename std::enable_if<
                boost::is_same<CLIQUE, HybridBayesTreeClique>::value> >
    : public CLIQUE::ConditionalType {
 public:
  typedef CLIQUE CliqueType;
  typedef typename CLIQUE::ConditionalType Base;

  boost::shared_ptr<CliqueType> clique;

  /**
   * @brief Construct a new Bayes Tree Orphan Wrapper object.
   *
   * @param clique Bayes tree clique.
   */
  BayesTreeOrphanWrapper(const boost::shared_ptr<CliqueType>& clique)
      : clique(clique) {
    // Store parent keys in our base type factor so that eliminating those
    // parent keys will pull this subtree into the elimination.
    this->keys_.assign(clique->conditional()->beginParents(),
                       clique->conditional()->endParents());
    this->discreteKeys_.assign(clique->conditional()->discreteKeys().begin(),
                               clique->conditional()->discreteKeys().end());
  }

  /// print utility
  void print(
      const std::string& s = "",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override {
    clique->print(s + "stored clique", formatter);
  }
};

}  // namespace gtsam
