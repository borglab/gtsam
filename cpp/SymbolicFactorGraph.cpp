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
#include "SymbolicBayesNet.h"

using namespace std;

namespace gtsam {

	// Explicitly instantiate so we don't have to include everywhere
	template class FactorGraph<SymbolicFactor>;

	/* ************************************************************************* */
	SymbolicBayesNet::shared_ptr
	SymbolicFactorGraph::eliminate(const Ordering& ordering)
	{
		SymbolicBayesNet::shared_ptr bayesNet (new SymbolicBayesNet());

		BOOST_FOREACH(string key, ordering) {
			SymbolicConditional::shared_ptr conditional = eliminateOne<SymbolicConditional>(key);
			bayesNet->push_back(conditional);
		}

		return bayesNet;
	}

	/* ************************************************************************* */

}
