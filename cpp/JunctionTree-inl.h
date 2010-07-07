/*
 * JunctionTree-inl.h
 *
 *   Created on: Feb 4, 2010
 *       Author: nikai
 *  Description: the junction tree
 */

#include <boost/tuple/tuple.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>
#include "Pose2.h"
#include "BayesTree-inl.h"
#include "SymbolicFactorGraph.h"

#include "JunctionTree.h"

#define DEBUG(i) \
        if (verboseLevel>i) cout

namespace gtsam {

	using namespace std;

	/* ************************************************************************* */
	/**
	 * Linear JunctionTree
	 */
	template <class Conditional, class FG>
	void JunctionTree<Conditional, FG>::SubFG::printTree(const string& indent) const {
		print(indent);
		BOOST_FOREACH(const shared_ptr& child, children_)
			child->printTree(indent+"  ");
	}

	/* ************************************************************************* */
	template <class Conditional, class FG>
	pair<FG, typename BayesTree<Conditional>::sharedClique>
	JunctionTree<Conditional, FG>::eliminateOneClique(sharedSubFG current, BayesTree<Conditional>& bayesTree) {
		FG fg; // factor graph will be assembled from local factors and marginalized children
		list<sharedClique> children;
		fg.push_back(*current); // add the local factor graph
		BOOST_FOREACH(sharedSubFG& child, current->children_) {
			// receive the factors from the child and its clique point
			FG fgChild; sharedClique cliqueChild;
			boost::tie(fgChild, cliqueChild) = eliminateOneClique(child, bayesTree);
			if (!cliqueChild.get()) throw runtime_error("eliminateOneClique: child clique is invalid!");

			fg.push_back(fgChild);
			children.push_back(cliqueChild);
		}

		// eliminate the combined factors
		// warning: fg is being eliminated in-place and will contain marginal afterwards
//		BayesNet<Conditional> bn = fg.eliminate(current->frontal_);
		BayesNet<Conditional> bn = fg.eliminateFrontals(current->frontal_);

		// create a new clique corresponding the combined factors
		sharedClique new_clique = bayesTree.insert(bn, children);

		return make_pair(fg, new_clique);
	}

	/* ************************************************************************* */
	template <class Conditional, class FG>
	BayesTree<Conditional> JunctionTree<Conditional, FG>::eliminate() {
		BayesTree<Conditional> bayesTree;
		eliminateOneClique(rootFG_, bayesTree);
		return bayesTree;
	}

	/* ************************************************************************* */
	template <class Conditional, class FG>
	void JunctionTree<Conditional, FG>::iterSubGraphsDFS(VisitorSubFG visitor, sharedSubFG current) {
		if (!current.get()) current = rootFG_;
//		iterateBFS<SubFG>(visitor, current);
	}

	/* ************************************************************************* */
	template <class Conditional, class FG>
	void JunctionTree<Conditional, FG>::iterSubGraphsBFS(VisitorSubFG visitor) {
//		iterateDFS<SubFG>(visitor, rootFG_);
	}

	/* ************************************************************************* */
	/**
	 * Linear JunctionTree
	 */
	template <class Conditional, class FG>
	void LinearJunctionTree<Conditional, FG>::btreeBackSubstitue(typename BayesTree<Conditional>::sharedClique current, VectorConfig& config) {
		// solve the bayes net in the current node
		typename BayesNet<Conditional>::const_reverse_iterator it = current->rbegin();
		for (; it!=current->rend(); it++) {
			Vector x = (*it)->solve(config); // Solve for that variable
			config.insert((*it)->key(),x);   // store result in partial solution
		}

		// solve the bayes nets in the child nodes
		BOOST_FOREACH(sharedClique child, current->children_) {
			btreeBackSubstitue(child, config);
		}
	}

	/* ************************************************************************* */
	template <class Conditional, class FG>
	VectorConfig LinearJunctionTree<Conditional, FG>::optimize() {
		// eliminate from leaves to the root
		BayesTree<Conditional> bayesTree = JunctionTree<Conditional, FG>::eliminate();

		VectorConfig result;
		btreeBackSubstitue(bayesTree.root(), result);
		return result;
	}

} //namespace gtsam
