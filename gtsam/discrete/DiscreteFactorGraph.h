/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteFactorGraph.h
 * @date Feb 14, 2011
 * @author Duy-Nguyen Ta
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/base/FastSet.h>
#include <boost/make_shared.hpp>

namespace gtsam {

class DiscreteFactorGraph: public FactorGraph<DiscreteFactor> {
public:

  /** A map from keys to values */
  typedef std::vector<Index> Indices;
  typedef Assignment<Index> Values;
  typedef boost::shared_ptr<Values> sharedValues;

  /** Construct empty factor graph */
  GTSAM_EXPORT DiscreteFactorGraph();

  /** Constructor from a factor graph of GaussianFactor or a derived type */
  template<class DERIVEDFACTOR>
  DiscreteFactorGraph(const FactorGraph<DERIVEDFACTOR>& fg) {
    push_back(fg);
  }

  /** construct from a BayesNet */
  GTSAM_EXPORT DiscreteFactorGraph(const BayesNet<DiscreteConditional>& bayesNet);

  template<class SOURCE>
  void add(const DiscreteKey& j, SOURCE table) {
    DiscreteKeys keys;
    keys.push_back(j);
    push_back(boost::make_shared<DecisionTreeFactor>(keys, table));
  }

  template<class SOURCE>
  void add(const DiscreteKey& j1, const DiscreteKey& j2, SOURCE table) {
    DiscreteKeys keys;
    keys.push_back(j1);
    keys.push_back(j2);
    push_back(boost::make_shared<DecisionTreeFactor>(keys, table));
  }

  /** add shared discreteFactor immediately from arguments */
  template<class SOURCE>
  void add(const DiscreteKeys& keys, SOURCE table) {
    push_back(boost::make_shared<DecisionTreeFactor>(keys, table));
  }

  /** Return the set of variables involved in the factors (set union) */
  GTSAM_EXPORT FastSet<Index> keys() const;

  /** return product of all factors as a single factor */
  GTSAM_EXPORT DecisionTreeFactor product() const;

  /** Evaluates the factor graph given values, returns the joint probability of the factor graph given specific instantiation of values*/
  GTSAM_EXPORT double operator()(const DiscreteFactor::Values & values) const;

  /// print
  GTSAM_EXPORT void print(const std::string& s = "DiscreteFactorGraph",
      const IndexFormatter& formatter =DefaultIndexFormatter) const;

  /** Permute the variables in the factors */
  GTSAM_EXPORT void permuteWithInverse(const Permutation& inversePermutation);

  /** Apply a reduction, which is a remapping of variable indices. */
  GTSAM_EXPORT void reduceWithInverse(const internal::Reduction& inverseReduction);
};
// DiscreteFactorGraph

/** Main elimination function for DiscreteFactorGraph */
GTSAM_EXPORT std::pair<boost::shared_ptr<DiscreteConditional>, DecisionTreeFactor::shared_ptr>
EliminateDiscrete(const FactorGraph<DiscreteFactor>& factors,
    size_t nrFrontals = 1);

} // namespace gtsam
