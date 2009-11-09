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
	SymbolicBayesNet
	SymbolicFactorGraph::eliminate(const Ordering& ordering)
	{
		SymbolicBayesNet bayesNet;

		BOOST_FOREACH(string key, ordering) {
			SymbolicConditional::shared_ptr conditional =
					_eliminateOne<SymbolicFactor,SymbolicConditional>(*this,key);
			bayesNet.push_back(conditional);
		}
		return bayesNet;
	}

	/* ************************************************************************* */
}
