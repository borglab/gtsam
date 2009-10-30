/**
 * @file    BayesTree.cpp
 * @brief   Bayes Tree is a tree of cliques of a Bayes Chain
 * @author  Frank Dellaert
 */

#include "BayesTree.h"

using namespace std;

namespace gtsam {

	template<class Conditional>
	BayesTree<Conditional>::BayesTree(BayesChain<Conditional>& bayesChain) {
	}

	template<class Conditional>
	void BayesTree<Conditional>::print(const string& s) const {
	}

	template<class Conditional>
	bool BayesTree<Conditional>::equals(const BayesTree<Conditional>& other,
			double tol) const {
		return false;
	}

} /// namespace gtsam
