/**
 * @file   SymbolicBayesChain-inl.h
 * @brief  Template definitions for SymbolicBayesChain
 * @author Frank Dellaert
 */

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>

#include "SymbolicBayesChain.h"

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	template<class Factor, class Config>
	SymbolicBayesChain::SymbolicBayesChain(
			const FactorGraph<Factor, Config>& factorGraph, const Ordering& ordering) {
	}

/* ************************************************************************* */

} // namespace gtsam
