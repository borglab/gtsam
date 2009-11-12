/**
 * @file    inference.h
 * @brief   Contains *generic* inference algorithms that convert between templated
 * graphical models, i.e., factor graphs, Bayes nets, and Bayes trees
 * @author  Frank Dellaert
 */

#pragma once

#include "FactorGraph.h"
#include "BayesNet.h"

namespace gtsam {

	class Ordering;

	// ELIMINATE: FACTOR GRAPH -> BAYES NET

	/**
   * Eliminate a single node yielding a Conditional
   * Eliminates the factors from the factor graph through findAndRemoveFactors
   * and adds a new factor on the separator to the factor graph
   */
	template<class Factor, class Conditional>
	boost::shared_ptr<Conditional>
	eliminateOne(FactorGraph<Factor>& factorGraph, const std::string& key);

	/**
	 * eliminate factor graph using the given (not necessarily complete)
	 * ordering, yielding a chordal Bayes net and (partially eliminated) FG
	 */
	template<class Factor, class Conditional>
	BayesNet<Conditional> eliminate(FactorGraph<Factor>& factorGraph, const Ordering& ordering);

	// FACTOR/MARGINALIZE: BAYES NET -> FACTOR GRAPH

	/**
	 * Factor P(X) as P(not keys|keys) P(keys)
	 * @return P(not keys|keys) as an incomplete BayesNet, and P(keys) as a factor graph
	 */
	template<class Factor, class Conditional>
	std::pair< BayesNet<Conditional>, FactorGraph<Factor> >
	factor(const BayesNet<Conditional>& bn, const Ordering& keys);

	/**
	 * integrate out all except ordering, might be inefficient as the ordering
	 * will simply be the current ordering with the keys put in the back
	 */
	template<class Factor, class Conditional>
	FactorGraph<Factor> marginalize(const BayesNet<Conditional>& bn, const Ordering& keys);

} /// namespace gtsam
