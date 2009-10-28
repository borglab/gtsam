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

using namespace std;

namespace gtsam {

	typedef pair<string,SymbolicConditional::shared_ptr> pp;

	/* ************************************************************************* */
	SymbolicBayesChain::SymbolicBayesChain(const std::map<std::string,
			SymbolicConditional::shared_ptr>& nodes) {
		BOOST_FOREACH(pp p, nodes) {
		  keys_.push_front(p.first);
			nodes_.insert(p);
		}
	}

/* ************************************************************************* */

} // namespace gtsam
