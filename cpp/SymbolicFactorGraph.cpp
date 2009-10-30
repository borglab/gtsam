/*
 * SymbolicFactorGraph.cpp
 *
 *  Created on: Oct 29, 2009
 *      Author: Frank Dellaert
 */

#include <boost/foreach.hpp>
#include "Ordering.h"
#include "FactorGraph-inl.h"
#include "SymbolicFactorGraph.h"
#include "SymbolicBayesChain.h"

using namespace std;

namespace gtsam {

	// Explicitly instantiate so we don't have to include everywhere
	template class FactorGraph<SymbolicFactor>;

	/* ************************************************************************* */
	SymbolicBayesChain::shared_ptr
	SymbolicFactorGraph::eliminate(const Ordering& ordering)
	{
		SymbolicBayesChain::shared_ptr bayesChain (new SymbolicBayesChain());

		BOOST_FOREACH(string key, ordering) {
			SymbolicConditional::shared_ptr conditional = eliminateOne<SymbolicConditional>(key);
			bayesChain->insert(key,conditional);
		}

		return bayesChain;
	}

	/* ************************************************************************* */

}
