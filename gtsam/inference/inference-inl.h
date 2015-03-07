/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file inference-inl.h
 * @brief 
 * @author Richard Roberts
 * @date Mar 3, 2012
 */

#pragma once

#include <algorithm>

// Only for Eclipse parser, inference-inl.h (this file) is included at the bottom of inference.h
#include <gtsam/inference/inference.h>

#include <gtsam/base/FastSet.h>

namespace gtsam {

namespace inference {

/* ************************************************************************* */
template<typename CONSTRAINED>
Permutation::shared_ptr PermutationCOLAMD(
		const VariableIndex& variableIndex, const CONSTRAINED& constrainLast) {

  std::vector<int> cmember(variableIndex.size(), 0);

  // If at least some variables are not constrained to be last, constrain the
  // ones that should be constrained.
  if(constrainLast.size() < variableIndex.size()) {
    BOOST_FOREACH(Index var, constrainLast) {
      assert(var < variableIndex.size());
      cmember[var] = 1;
    }
  }

  return PermutationCOLAMD_(variableIndex, cmember);
}

/* ************************************************************************* */
template<typename CONSTRAINED_MAP>
Permutation::shared_ptr PermutationCOLAMDGrouped(
		const VariableIndex& variableIndex, const CONSTRAINED_MAP& constraints) {
  std::vector<int> cmember(variableIndex.size(), 0);

	typedef typename CONSTRAINED_MAP::value_type constraint_pair;
  BOOST_FOREACH(const constraint_pair& p, constraints) {
  	assert(p.first < variableIndex.size());
  	// FIXME: check that no groups are skipped
  	cmember[p.first] = p.second;
  }

  return PermutationCOLAMD_(variableIndex, cmember);
}

/* ************************************************************************* */
inline Permutation::shared_ptr PermutationCOLAMD(const VariableIndex& variableIndex) {
  std::vector<int> cmember(variableIndex.size(), 0);
  return PermutationCOLAMD_(variableIndex, cmember);
}

/* ************************************************************************* */
template<class Graph>
std::pair<typename Graph::sharedConditional, Graph> eliminate(
		const Graph& factorGraph,
		const std::vector<typename Graph::KeyType>& variables,
		const typename Graph::Eliminate& eliminateFcn,
		boost::optional<const VariableIndex&> variableIndex_) {

  const VariableIndex& variableIndex =
					variableIndex_ ? *variableIndex_ : VariableIndex(factorGraph);

  // First find the involved factors
  Graph involvedFactors;
  Index highestInvolvedVariable = 0; // Largest index of the variables in the involved factors

  // First get the indices of the involved factors, but uniquely in a set
  FastSet<size_t> involvedFactorIndices;
  BOOST_FOREACH(Index variable, variables) {
    involvedFactorIndices.insert(variableIndex[variable].begin(), variableIndex[variable].end()); }

  // Add the factors themselves to involvedFactors and update largest index
  involvedFactors.reserve(involvedFactorIndices.size());
  BOOST_FOREACH(size_t factorIndex, involvedFactorIndices) {
    const typename Graph::sharedFactor factor = factorGraph[factorIndex];
    involvedFactors.push_back(factor); // Add involved factor
    highestInvolvedVariable = std::max( // Updated largest index
        highestInvolvedVariable,
        *std::max_element(factor->begin(), factor->end()));
  }

  // Now permute the variables to be eliminated to the front of the ordering
  Permutation toFront = Permutation::PullToFront(variables, highestInvolvedVariable+1);
  Permutation toFrontInverse = *toFront.inverse();
  involvedFactors.permuteWithInverse(toFrontInverse);

  // Eliminate into conditional and remaining factor
  typename Graph::EliminationResult eliminated = eliminateFcn(involvedFactors, variables.size());
  boost::shared_ptr<typename Graph::FactorType::ConditionalType> conditional = eliminated.first;
  typename Graph::sharedFactor remainingFactor = eliminated.second;

  // Undo the permutation
  conditional->permuteWithInverse(toFront);
  remainingFactor->permuteWithInverse(toFront);

  // Build the remaining graph, without the removed factors
  Graph remainingGraph;
  remainingGraph.reserve(factorGraph.size() - involvedFactors.size() + 1);
  FastSet<size_t>::const_iterator involvedFactorIndexIt = involvedFactorIndices.begin();
  for(size_t i = 0; i < factorGraph.size(); ++i) {
    if(involvedFactorIndexIt != involvedFactorIndices.end() && *involvedFactorIndexIt == i)
      ++ involvedFactorIndexIt;
    else
      remainingGraph.push_back(factorGraph[i]);
  }

  // Add the remaining factor if it is not empty.
  if(remainingFactor->size() != 0)
  	remainingGraph.push_back(remainingFactor);

  return std::make_pair(conditional, remainingGraph);

} // eliminate

} // namespace inference
} // namespace gtsam


