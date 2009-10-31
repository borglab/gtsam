/**
 * @file   SymbolicBayesChain.cpp
 * @brief  Chordal Bayes Net, the result of eliminating a factor graph
 * @author Frank Dellaert
 */

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

#include "SymbolicBayesChain.h"
#include "BayesChain-inl.h"

using namespace std;

namespace gtsam {

	// Explicitly instantiate so we don't have to include everywhere
	template class BayesChain<SymbolicConditional>;

/* ************************************************************************* */

} // namespace gtsam
