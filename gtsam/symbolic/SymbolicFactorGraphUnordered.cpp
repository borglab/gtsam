/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicFactorGraph.cpp
 * @date Oct 29, 2009
 * @author Frank Dellaert
 */

#include <boost/make_shared.hpp>

#include <gtsam/inference/FactorGraphUnordered-inst.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>
#include <gtsam/symbolic/SymbolicFactorGraphUnordered.h>
#include <gtsam/symbolic/SymbolicEliminationTreeUnordered.h>
#include <gtsam/symbolic/SymbolicJunctionTreeUnordered.h>
#include <gtsam/symbolic/SymbolicBayesTreeUnordered.h>
#include <gtsam/symbolic/SymbolicConditionalUnordered.h>

namespace gtsam {

  using namespace std;

  /* ************************************************************************* */
  bool SymbolicFactorGraphUnordered::equals(const This& fg, double tol) const
  {
    return Base::equals(fg, tol);
  }

  /* ************************************************************************* */
  void SymbolicFactorGraphUnordered::push_factor(Key key) {
    push_back(boost::make_shared<SymbolicFactorUnordered>(key));
  }

  /* ************************************************************************* */
  void SymbolicFactorGraphUnordered::push_factor(Key key1, Key key2) {
    push_back(boost::make_shared<SymbolicFactorUnordered>(key1,key2));
  }

  /* ************************************************************************* */
  void SymbolicFactorGraphUnordered::push_factor(Key key1, Key key2, Key key3) {
    push_back(boost::make_shared<SymbolicFactorUnordered>(key1,key2,key3));
  }

  /* ************************************************************************* */
  void SymbolicFactorGraphUnordered::push_factor(Key key1, Key key2, Key key3, Key key4) {
    push_back(boost::make_shared<SymbolicFactorUnordered>(key1,key2,key3,key4));
  }

//  /* ************************************************************************* */
//  std::pair<SymbolicFactorGraph::sharedConditional, SymbolicFactorGraph>
//    SymbolicFactorGraph::eliminateFrontals(size_t nFrontals) const
//  {
//    return FactorGraph<IndexFactor>::eliminateFrontals(nFrontals, EliminateSymbolic);
//  }
//
//  /* ************************************************************************* */
//  std::pair<SymbolicFactorGraph::sharedConditional, SymbolicFactorGraph>
//    SymbolicFactorGraph::eliminate(const std::vector<Index>& variables) const
//  {
//    return FactorGraph<IndexFactor>::eliminate(variables, EliminateSymbolic);
//  }
//
//  /* ************************************************************************* */
//  std::pair<SymbolicFactorGraph::sharedConditional, SymbolicFactorGraph>
//    SymbolicFactorGraph::eliminateOne(Index variable) const
//  {
//    return FactorGraph<IndexFactor>::eliminateOne(variable, EliminateSymbolic);
//  }
//
//  /* ************************************************************************* */
//  IndexFactor::shared_ptr CombineSymbolic(
//      const FactorGraph<IndexFactor>& factors, const FastMap<Index,
//          vector<Index> >& variableSlots) {
//    IndexFactor::shared_ptr combined(Combine<IndexFactor, Index> (factors, variableSlots));
////    combined->assertInvariants();
//    return combined;
//  }
//
//  /* ************************************************************************* */
//  pair<IndexConditional::shared_ptr, IndexFactor::shared_ptr> //
//  EliminateSymbolic(const FactorGraph<IndexFactor>& factors, size_t nrFrontals) {
//
//    FastSet<Index> keys;
//    BOOST_FOREACH(const IndexFactor::shared_ptr& factor, factors)
//      BOOST_FOREACH(Index var, *factor)
//      keys.insert(var);
//
//    if (keys.size() < nrFrontals) throw invalid_argument(
//      "EliminateSymbolic requested to eliminate more variables than exist in graph.");
//
//    vector<Index> newKeys(keys.begin(), keys.end());
//    return make_pair(boost::make_shared<IndexConditional>(newKeys, nrFrontals),
//      boost::make_shared<IndexFactor>(newKeys.begin() + nrFrontals, newKeys.end()));
//  }

  /* ************************************************************************* */
}
