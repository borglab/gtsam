/**
 * @file   SymbolicBayesChain.cpp
 * @brief  Chordal Bayes Net, the result of eliminating a factor graph
 * @author Frank Dellaert
 */

#include <stdarg.h>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>

#include "SymbolicBayesChain.h"

using namespace std;
using namespace gtsam;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL) 

/* ************************************************************************* */
void SymbolicBayesChain::print(const string& s) const {
	//  BOOST_FOREACH(string key, keys) {
	//    const_iterator it = nodes.find(key);
	//    it->second->print("\nNode[" + key + "]");
	//  }
}

/* ************************************************************************* */
bool SymbolicBayesChain::equals(const SymbolicBayesChain& cbn, double tol) const {
	//  const_iterator it1 = nodes.begin(), it2 = cbn.nodes.begin();
	//
	//  if(nodes.size() != cbn.nodes.size()) return false;
	//  for(; it1 != nodes.end(); it1++, it2++){
	//    const string& j1 = it1->first, j2 = it2->first;
	//    ConditionalGaussian::shared_ptr node1 = it1->second, node2 = it2->second;
	//    if (j1 != j2) return false;
	//    if (!node1->equals(*node2,tol))
	//      return false;
	//  }
	//  return true;
	return false;
}

/* ************************************************************************* */
