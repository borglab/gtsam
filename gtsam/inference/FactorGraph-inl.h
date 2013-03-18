/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   FactorGraph-inl.h
 * This is a template definition file, include it where needed (only!)
 * so that the appropriate code is generated and link errors avoided.
 * @brief  Factor Graph Base Class
 * @author Carlos Nieto
 * @author Frank Dellaert
 * @author Alireza Fathi
 * @author Michael Kaess
 */

#pragma once

#include <gtsam/base/FastSet.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/inference/VariableIndex.h>

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/format.hpp>
#include <boost/iterator/transform_iterator.hpp>

#include <stdio.h>
#include <list>
#include <sstream>
#include <stdexcept>

namespace gtsam {

  /* ************************************************************************* */
  template<class FACTOR>
  template<class CONDITIONAL>
  FactorGraph<FACTOR>::FactorGraph(const BayesNet<CONDITIONAL>& bayesNet) {
    factors_.reserve(bayesNet.size());
    BOOST_FOREACH(const typename CONDITIONAL::shared_ptr& cond, bayesNet) {
      this->push_back(cond->toFactor());
    }
  }

  /* ************************************************************************* */
  template<class FACTOR>
  void FactorGraph<FACTOR>::print(const std::string& s,
      const IndexFormatter& formatter) const {
    std::cout << s << std::endl;
    std::cout << "size: " << size() << std::endl;
    for (size_t i = 0; i < factors_.size(); i++) {
      std::stringstream ss;
      ss << "factor " << i << ": ";
      if (factors_[i] != NULL) factors_[i]->print(ss.str(), formatter);
    }
  }

  /* ************************************************************************* */
  template<class FACTOR>
  bool FactorGraph<FACTOR>::equals(const This& fg, double tol) const {
    /** check whether the two factor graphs have the same number of factors_ */
    if (factors_.size() != fg.size()) return false;

    /** check whether the factors_ are the same */
    for (size_t i = 0; i < factors_.size(); i++) {
      // TODO: Doesn't this force order of factor insertion?
      sharedFactor f1 = factors_[i], f2 = fg.factors_[i];
      if (f1 == NULL && f2 == NULL) continue;
      if (f1 == NULL || f2 == NULL) return false;
      if (!f1->equals(*f2, tol)) return false;
    }
    return true;
  }

  /* ************************************************************************* */
  template<class FACTOR>
  size_t FactorGraph<FACTOR>::nrFactors() const {
    size_t size_ = 0;
    BOOST_FOREACH(const sharedFactor& factor, factors_)
      if (factor) size_++;
    return size_;
  }

  /* ************************************************************************* */
  template<class FACTOR>
  std::set<typename FACTOR::KeyType> FactorGraph<FACTOR>::keys() const {
    std::set<KeyType> allKeys;
    BOOST_FOREACH(const sharedFactor& factor, factors_)
      if (factor) {
        BOOST_FOREACH(Index j, factor->keys())
          allKeys.insert(j);
      }
    return allKeys;
  }

  /* ************************************************************************* */
  template<class FACTOR>
  std::pair<typename FactorGraph<FACTOR>::sharedConditional, FactorGraph<FACTOR> >
    FactorGraph<FACTOR>::eliminateFrontals(size_t nFrontals, const Eliminate& eliminate) const
  {
    // Build variable index
    VariableIndex variableIndex(*this);

    // Find first variable
    Index firstIndex = 0;
    while(firstIndex < variableIndex.size() && variableIndex[firstIndex].empty())
      ++ firstIndex;

    // Check that number of variables is in bounds
    if(firstIndex + nFrontals > variableIndex.size())
      throw std::invalid_argument("Requested to eliminate more frontal variables than exist in the factor graph.");

    // Get set of involved factors
    FastSet<size_t> involvedFactorIs;
    for(Index j = firstIndex; j < firstIndex + nFrontals; ++j) {
      BOOST_FOREACH(size_t i, variableIndex[j]) {
        involvedFactorIs.insert(i);
      }
    }

    // Separate factors into involved and remaining
    FactorGraph<FactorType> involvedFactors;
    FactorGraph<FactorType> remainingFactors;
    FastSet<size_t>::const_iterator involvedFactorIsIt = involvedFactorIs.begin();
    for(size_t i = 0; i < this->size(); ++i) {
      if(involvedFactorIsIt != involvedFactorIs.end() && *involvedFactorIsIt == i) {
        // If the current factor is involved, add it to involved and increment involved iterator
        involvedFactors.push_back((*this)[i]);
        ++ involvedFactorIsIt;
      } else {
        // If not involved, add to remaining
        remainingFactors.push_back((*this)[i]);
      }
    }

    // Do dense elimination on the involved factors
    typename FactorGraph<FactorType>::EliminationResult eliminationResult =
      eliminate(involvedFactors, nFrontals);

    // Add the remaining factor back into the factor graph
    remainingFactors.push_back(eliminationResult.second);

    // Return the eliminated factor and remaining factor graph
    return std::make_pair(eliminationResult.first, remainingFactors);
  }

  /* ************************************************************************* */
  template<class FACTOR>
  std::pair<typename FactorGraph<FACTOR>::sharedConditional, FactorGraph<FACTOR> >
    FactorGraph<FACTOR>::eliminate(const std::vector<KeyType>& variables, const Eliminate& eliminateFcn,
    boost::optional<const VariableIndex&> variableIndex_) const
  {
      const VariableIndex& variableIndex =
        variableIndex_ ? *variableIndex_ : VariableIndex(*this);

      // First find the involved factors
      FactorGraph<FACTOR> involvedFactors;
      Index highestInvolvedVariable = 0; // Largest index of the variables in the involved factors

      // First get the indices of the involved factors, but uniquely in a set
      FastSet<size_t> involvedFactorIndices;
      BOOST_FOREACH(Index variable, variables) {
        involvedFactorIndices.insert(variableIndex[variable].begin(), variableIndex[variable].end()); }

      // Add the factors themselves to involvedFactors and update largest index
      involvedFactors.reserve(involvedFactorIndices.size());
      BOOST_FOREACH(size_t factorIndex, involvedFactorIndices) {
        const sharedFactor factor = this->at(factorIndex);
        involvedFactors.push_back(factor); // Add involved factor
        highestInvolvedVariable = std::max( // Updated largest index
          highestInvolvedVariable,
          *std::max_element(factor->begin(), factor->end()));
      }

      sharedConditional conditional;
      sharedFactor remainingFactor;
      if(involvedFactors.size() > 0) {
        // Now permute the variables to be eliminated to the front of the ordering
        Permutation toFront = Permutation::PullToFront(variables, highestInvolvedVariable+1);
        Permutation toFrontInverse = *toFront.inverse();
        BOOST_FOREACH(const sharedFactor& factor, involvedFactors) {
          factor->permuteWithInverse(toFrontInverse); }

        // Eliminate into conditional and remaining factor
        EliminationResult eliminated = eliminateFcn(involvedFactors, variables.size());
        conditional = eliminated.first;
        remainingFactor = eliminated.second;

        // Undo the permutation
        BOOST_FOREACH(const sharedFactor& factor, involvedFactors) {
          factor->permuteWithInverse(toFront); }
        conditional->permuteWithInverse(toFront);
        remainingFactor->permuteWithInverse(toFront);
      } else {
        // Eliminate 0 variables
        EliminationResult eliminated = eliminateFcn(involvedFactors, 0);
        conditional = eliminated.first;
        remainingFactor = eliminated.second;
      }

      // Build the remaining graph, without the removed factors
      FactorGraph<FACTOR> remainingGraph;
      remainingGraph.reserve(this->size() - involvedFactors.size() + 1);
      FastSet<size_t>::const_iterator involvedFactorIndexIt = involvedFactorIndices.begin();
      for(size_t i = 0; i < this->size(); ++i) {
        if(involvedFactorIndexIt != involvedFactorIndices.end() && *involvedFactorIndexIt == i)
          ++ involvedFactorIndexIt;
        else
          remainingGraph.push_back(this->at(i));
      }

      // Add the remaining factor if it is not empty.
      if(remainingFactor->size() != 0)
        remainingGraph.push_back(remainingFactor);

      return std::make_pair(conditional, remainingGraph);

  }
  /* ************************************************************************* */
  template<class FACTOR>
  void FactorGraph<FACTOR>::replace(size_t index, sharedFactor factor) {
    if (index >= factors_.size()) throw std::invalid_argument(boost::str(
        boost::format("Factor graph does not contain a factor with index %d.")
            % index));
    // Replace the factor
    factors_[index] = factor;
  }

  /* ************************************************************************* */
  template<class FACTORGRAPH>
  FACTORGRAPH combine(const FACTORGRAPH& fg1, const FACTORGRAPH& fg2) {
    // create new linear factor graph equal to the first one
    FACTORGRAPH fg = fg1;

    // add the second factors_ in the graph
    fg.push_back(fg2);

    return fg;
  }

  /* ************************************************************************* */
  template<class DERIVEDFACTOR, class KEY>
  typename DERIVEDFACTOR::shared_ptr Combine(const FactorGraph<DERIVEDFACTOR>& factors,
    const FastMap<KEY, std::vector<KEY> >& variableSlots) {

    typedef const std::pair<const KEY, std::vector<KEY> > KeySlotPair;
    // Local functional for getting keys out of key-value pairs
    struct Local { static KEY FirstOf(const KeySlotPair& pr) { return pr.first; } };

    return typename DERIVEDFACTOR::shared_ptr(new DERIVEDFACTOR(
        boost::make_transform_iterator(variableSlots.begin(), &Local::FirstOf),
        boost::make_transform_iterator(variableSlots.end(), &Local::FirstOf)));
  }

  /* ************************************************************************* */
  // Recursive function to add factors in cliques to vector of factors_io
  template<class FACTOR, class CONDITIONAL, class CLIQUE>
  void _FactorGraph_BayesTree_adder(
      std::vector<typename boost::shared_ptr<FACTOR> >& factors_io,
      const typename BayesTree<CONDITIONAL,CLIQUE>::sharedClique& clique) {

    if(clique) {
      // Add factor from this clique
      factors_io.push_back((*clique)->toFactor());

      // Traverse children
      typedef typename BayesTree<CONDITIONAL,CLIQUE>::sharedClique sharedClique;
      BOOST_FOREACH(const sharedClique& child, clique->children())
        _FactorGraph_BayesTree_adder<FACTOR,CONDITIONAL,CLIQUE>(factors_io, child);
    }
  }

  /* ************************************************************************* */
  template<class FACTOR>
  template<class CONDITIONAL, class CLIQUE>
  FactorGraph<FACTOR>::FactorGraph(const BayesTree<CONDITIONAL,CLIQUE>& bayesTree) {
    factors_.reserve(bayesTree.size());
    _FactorGraph_BayesTree_adder<FACTOR,CONDITIONAL,CLIQUE>(factors_, bayesTree.root());
  }

  /* ************************************************************************* */
} // namespace gtsam
