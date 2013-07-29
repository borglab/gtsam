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

#include <gtsam/inference/SymbolicFactorGraphOrdered.h>
#include <gtsam/inference/BayesNetOrdered.h>
#include <gtsam/inference/EliminationTreeOrdered.h>
#include <gtsam/inference/IndexConditionalOrdered.h>

#include <boost/make_shared.hpp>

namespace gtsam {

  using namespace std;

  /* ************************************************************************* */
  SymbolicFactorGraphOrdered::SymbolicFactorGraphOrdered(const SymbolicBayesNetOrdered& bayesNet) :
      FactorGraphOrdered<IndexFactorOrdered>(bayesNet) {}

  /* ************************************************************************* */
  SymbolicFactorGraphOrdered::SymbolicFactorGraphOrdered(const SymbolicBayesTreeOrdered& bayesTree) :
      FactorGraphOrdered<IndexFactorOrdered>(bayesTree) {}

  /* ************************************************************************* */
  void SymbolicFactorGraphOrdered::push_factor(Index key) {
    push_back(boost::make_shared<IndexFactorOrdered>(key));
  }

  /** Push back binary factor */
  void SymbolicFactorGraphOrdered::push_factor(Index key1, Index key2) {
    push_back(boost::make_shared<IndexFactorOrdered>(key1,key2));
  }

  /** Push back ternary factor */
  void SymbolicFactorGraphOrdered::push_factor(Index key1, Index key2, Index key3) {
    push_back(boost::make_shared<IndexFactorOrdered>(key1,key2,key3));
  }

  /** Push back 4-way factor */
  void SymbolicFactorGraphOrdered::push_factor(Index key1, Index key2, Index key3, Index key4) {
    push_back(boost::make_shared<IndexFactorOrdered>(key1,key2,key3,key4));
  }

  /* ************************************************************************* */
  FastSet<Index>
  SymbolicFactorGraphOrdered::keys() const {
    FastSet<Index> keys;
    BOOST_FOREACH(const sharedFactor& factor, *this) {
      if(factor) keys.insert(factor->begin(), factor->end()); }
    return keys;
  }

  /* ************************************************************************* */
  std::pair<SymbolicFactorGraphOrdered::sharedConditional, SymbolicFactorGraphOrdered>
    SymbolicFactorGraphOrdered::eliminateFrontals(size_t nFrontals) const
  {
    return FactorGraphOrdered<IndexFactorOrdered>::eliminateFrontals(nFrontals, EliminateSymbolic);
  }

  /* ************************************************************************* */
  std::pair<SymbolicFactorGraphOrdered::sharedConditional, SymbolicFactorGraphOrdered>
    SymbolicFactorGraphOrdered::eliminate(const std::vector<Index>& variables) const
  {
    return FactorGraphOrdered<IndexFactorOrdered>::eliminate(variables, EliminateSymbolic);
  }

  /* ************************************************************************* */
  std::pair<SymbolicFactorGraphOrdered::sharedConditional, SymbolicFactorGraphOrdered>
    SymbolicFactorGraphOrdered::eliminateOne(Index variable) const
  {
    return FactorGraphOrdered<IndexFactorOrdered>::eliminateOne(variable, EliminateSymbolic);
  }

  /* ************************************************************************* */
  void SymbolicFactorGraphOrdered::permuteWithInverse(
    const Permutation& inversePermutation) {
      BOOST_FOREACH(const sharedFactor& factor, factors_) {
        if(factor)
          factor->permuteWithInverse(inversePermutation);
      }
  }

  /* ************************************************************************* */
  void SymbolicFactorGraphOrdered::reduceWithInverse(
    const internal::Reduction& inverseReduction) {
      BOOST_FOREACH(const sharedFactor& factor, factors_) {
        if(factor)
          factor->reduceWithInverse(inverseReduction);
      }
  }

  /* ************************************************************************* */
  IndexFactorOrdered::shared_ptr CombineSymbolic(
      const FactorGraphOrdered<IndexFactorOrdered>& factors, const FastMap<Index,
          vector<Index> >& variableSlots) {
    IndexFactorOrdered::shared_ptr combined(Combine<IndexFactorOrdered, Index> (factors, variableSlots));
//    combined->assertInvariants();
    return combined;
  }

  /* ************************************************************************* */
  pair<IndexConditionalOrdered::shared_ptr, IndexFactorOrdered::shared_ptr> //
  EliminateSymbolic(const FactorGraphOrdered<IndexFactorOrdered>& factors, size_t nrFrontals) {

    FastSet<Index> keys;
    BOOST_FOREACH(const IndexFactorOrdered::shared_ptr& factor, factors)
      BOOST_FOREACH(Index var, *factor)
      keys.insert(var);

    if (keys.size() < nrFrontals) throw invalid_argument(
      "EliminateSymbolic requested to eliminate more variables than exist in graph.");

    vector<Index> newKeys(keys.begin(), keys.end());
    return make_pair(boost::make_shared<IndexConditionalOrdered>(newKeys, nrFrontals),
      boost::make_shared<IndexFactorOrdered>(newKeys.begin() + nrFrontals, newKeys.end()));
  }

  /* ************************************************************************* */
}
