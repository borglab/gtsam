/**
 * @file    BayesTree
 * @brief   Bayes Tree is a tree of cliques of a Bayes Chain
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <list>
#include <boost/serialization/map.hpp>
#include <boost/serialization/list.hpp>

#include "Testable.h"
#include "BayesChain.h"

namespace gtsam {

/**
 * Bayes tree
 * Templated on the Conditional class, the type of node in the underlying Bayes chain.
 * This could be a ConditionalProbabilityTable, a ConditionalGaussian, or a SymbolicConditional
 */
template <class Conditional>
class BayesTree : public Testable<BayesTree<Conditional> >
{
public:

	/** Create a Bayes Tree from a SymbolicBayesChain */
	BayesTree(BayesChain<Conditional>& bayesChain);

	/** Destructor */
	virtual ~BayesTree() {}

	/** print */
	void print(const std::string& s = "") const;

	/** check equality */
	bool equals(const BayesTree<Conditional>& other, double tol = 1e-9) const;

}; // BayesTree

} /// namespace gtsam
