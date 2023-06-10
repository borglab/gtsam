/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DiscreteBayesTree.h
 * @brief   Discrete Bayes Tree, the result of eliminating a
 * DiscreteJunctionTree
 * @brief   DiscreteBayesTree
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */

#pragma once

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/inference/Conditional.h>
#include <gtsam/inference/BayesTreeCliqueBase.h>

#include <string>

namespace gtsam {

// Forward declarations
class DiscreteConditional;
class VectorValues;

/* ************************************************************************* */
/** A clique in a DiscreteBayesTree */
class GTSAM_EXPORT DiscreteBayesTreeClique
    : public BayesTreeCliqueBase<DiscreteBayesTreeClique, DiscreteFactorGraph> {
 public:
  typedef DiscreteBayesTreeClique This;
  typedef BayesTreeCliqueBase<DiscreteBayesTreeClique, DiscreteFactorGraph>
      Base;
  typedef boost::shared_ptr<This> shared_ptr;
  typedef boost::weak_ptr<This> weak_ptr;
  DiscreteBayesTreeClique() {}
  virtual ~DiscreteBayesTreeClique() {}
  DiscreteBayesTreeClique(
      const boost::shared_ptr<DiscreteConditional>& conditional)
      : Base(conditional) {}

  /// print index signature only
  void printSignature(
      const std::string& s = "Clique: ",
      const KeyFormatter& formatter = DefaultKeyFormatter) const {
    conditional_->printSignature(s, formatter);
  }

  //** evaluate conditional probability of subtree for given DiscreteValues */
  double evaluate(const DiscreteValues& values) const;

  //** (Preferred) sugar for the above for given DiscreteValues */
  double operator()(const DiscreteValues& values) const {
    return evaluate(values);
  }
};

/* ************************************************************************* */
/**
 * @brief A Bayes tree representing a Discrete density.
 * @ingroup discrete
 */
class GTSAM_EXPORT DiscreteBayesTree
    : public BayesTree<DiscreteBayesTreeClique> {
 private:
  typedef BayesTree<DiscreteBayesTreeClique> Base;

 public:
  typedef DiscreteBayesTree This;
  typedef boost::shared_ptr<This> shared_ptr;

  /// @name Standard interface
  /// @{
  /** Default constructor, creates an empty Bayes tree */
  DiscreteBayesTree() {}

  /** Check equality */
  bool equals(const This& other, double tol = 1e-9) const;

  //** evaluate probability for given DiscreteValues */
  double evaluate(const DiscreteValues& values) const;

  //** (Preferred) sugar for the above for given DiscreteValues */
  double operator()(const DiscreteValues& values) const {
    return evaluate(values);
  }

  /// @}
  /// @name Wrapper support
  /// @{

  /// Render as markdown tables.
  std::string markdown(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                       const DiscreteFactor::Names& names = {}) const;

  /// Render as html tables.
  std::string html(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                   const DiscreteFactor::Names& names = {}) const;

  /// @}
};

}  // namespace gtsam
