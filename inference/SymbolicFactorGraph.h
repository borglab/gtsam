/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

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
  void push_factor(Index key);

  /** Push back binary factor */
  void push_factor(Index key1, Index key2);

  /** Push back ternary factor */
  void push_factor(Index key1, Index key2, Index key3);

  /** Push back 4-way factor */
  void push_factor(Index key1, Index key2, Index key3, Index key4);

  /**
   * Construct from a factor graph of any type
   */
  template<class Factor>
  SymbolicFactorGraph(const FactorGraph<Factor>& fg);

  /**
   * Return the set of variables involved in the factors (computes a set
   * union).
   */
  std::set<Index, std::less<Index>, boost::fast_pool_allocator<Index> > keys() const;

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
