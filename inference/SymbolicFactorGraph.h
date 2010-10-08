/*
 * SymbolicFactorGraph.h
 *
 *  Created on: Oct 29, 2009
 *      Author: Frank Dellaert
 */

#pragma once

#include <string>
#include <list>
#include <gtsam/base/types.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/Factor-inl.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/Conditional.h>

namespace gtsam {

typedef BayesNet<Conditional> SymbolicBayesNet;

/** Symbolic Factor Graph */
class SymbolicFactorGraph: public FactorGraph<Factor> {
public:
  typedef SymbolicBayesNet bayesnet_type;
  typedef VariableIndex<> variableindex_type;

  /** Construct empty factor graph */
  SymbolicFactorGraph() {}

  /** Construct from a BayesNet */
  SymbolicFactorGraph(const BayesNet<Conditional>& bayesNet);

  /** Push back unary factor */
  void push_factor(varid_t key);

  /** Push back binary factor */
  void push_factor(varid_t key1, varid_t key2);

  /** Push back ternary factor */
  void push_factor(varid_t key1, varid_t key2, varid_t key3);

  /** Push back 4-way factor */
  void push_factor(varid_t key1, varid_t key2, varid_t key3, varid_t key4);

  /**
   * Construct from a factor graph of any type
   */
  template<class Factor>
  SymbolicFactorGraph(const FactorGraph<Factor>& fg);

  /**
   * Return the set of variables involved in the factors (computes a set
   * union).
   */
  std::set<varid_t, std::less<varid_t>, boost::fast_pool_allocator<varid_t> > keys() const;

  /**
   * Same as eliminate in the SymbolicFactorGraph case
   */
  //		SymbolicBayesNet eliminateFrontals(const Ordering& ordering);
};

/* Template function implementation */
template<class FactorType>
SymbolicFactorGraph::SymbolicFactorGraph(const FactorGraph<FactorType>& fg) {
  for (size_t i = 0; i < fg.size(); i++) {
    if(fg[i]) {
      Factor::shared_ptr factor(new Factor(*fg[i]));
      push_back(factor);
    } else
      push_back(Factor::shared_ptr());
  }
}

} // namespace gtsam
