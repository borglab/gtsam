/**
 * @file    BayesChain
 * @brief   Bayes Chain, the result of eliminating a factor graph
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/list.hpp>

#include "Testable.h"

namespace gtsam {

	/**
	 * Bayes Chain, the result of eliminating a factor graph
	 * This is the base class for SymbolicBayesChain, DiscreteBayesChain, and GaussianBayesChain
	 * Corresponding to what is used for the "Conditional" template argument:
	 * a ConditionalProbabilityTable, a ConditionalGaussian, or a SymbolicConditional.
	 */
	template<class Conditional>
	class BayesChain: public Testable<BayesChain<Conditional> > {
	protected:

		/** nodes keys stored in topological sort order, i.e. from parents to children */
		std::list<std::string> keys_;

		/** nodes stored on key */
		typedef typename std::map<std::string, boost::shared_ptr<Conditional> > Nodes;
		Nodes nodes_;

		typedef typename Nodes::const_iterator const_iterator;

	public:

		/** print */
		void print(const std::string& s = "") const;

		/** check equality */
		bool equals(const BayesChain& other, double tol = 1e-9) const;

		/** insert: use reverse topological sort (i.e. parents last) */
		void insert(const std::string& key, boost::shared_ptr<Conditional> node);
	};

} /// namespace gtsam
