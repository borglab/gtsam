/*
 * JunctionTree-inl.h
 * Created on: Feb 4, 2010
 * @Author: Kai Ni
 * @Author: Frank Dellaert
 * @brief: The junction tree, template bodies
 */

#pragma once

#include <boost/foreach.hpp>

#include "SymbolicFactorGraph.h"
#include "BayesTree-inl.h"
#include "JunctionTree.h"

namespace gtsam {

	using namespace std;

	/* ************************************************************************* */
	template <class FG>
	JunctionTree<FG>::JunctionTree(FG& fg, const Ordering& ordering) {
		// Symbolic factorization: GaussianFactorGraph -> SymbolicFactorGraph
		// -> SymbolicBayesNet -> SymbolicBayesTree
		SymbolicFactorGraph sfg(fg);
		SymbolicBayesNet sbn = sfg.eliminate(ordering);
		BayesTree<SymbolicConditional> sbt(sbn);

		// distribtue factors
		this->root_ = distributeFactors(fg, sbt.root());
	}

	/* ************************************************************************* */
	template<class FG>
	typename JunctionTree<FG>::sharedClique JunctionTree<FG>::distributeFactors(
			FG& fg, const BayesTree<SymbolicConditional>::sharedClique bayesClique) {
		// create a new clique in the junction tree
		sharedClique clique(new Clique());
		clique->frontal_ = bayesClique->ordering();
		clique->separator_.insert(bayesClique->separator_.begin(),
				bayesClique->separator_.end());

		// recursively call the children
		BOOST_FOREACH(const BayesTree<SymbolicConditional>::sharedClique bayesChild, bayesClique->children()) {
			sharedClique child = distributeFactors(fg, bayesChild);
			clique->children_.push_back(child);
			child->parent_ = clique;
		}

		// collect the factors
		typedef vector<typename FG::sharedFactor> Factors;
		BOOST_FOREACH(const Symbol& frontal, clique->frontal_) {
			Factors factors = fg.template findAndRemoveFactors(frontal);
			BOOST_FOREACH(const typename FG::sharedFactor& factor_, factors)
				clique->push_back(factor_);
		}

		return clique;
	}

	/* ************************************************************************* */
	template <class FG> template <class Conditional>
	pair<FG, BayesTree<Conditional> >
	JunctionTree<FG>::eliminateOneClique(sharedClique current) {

//		current->frontal_.print("current clique:");

		typedef typename BayesTree<Conditional>::sharedClique sharedBtreeClique;
		FG fg; // factor graph will be assembled from local factors and marginalized children
		list<BayesTree<Conditional> > children;
		fg.push_back(*current); // add the local factor graph

//		BOOST_FOREACH(const typename FG::sharedFactor& factor_, fg)
//			Ordering(factor_->keys()).print("local factor:");

		BOOST_FOREACH(sharedClique& child, current->children_) {
			// receive the factors from the child and its clique point
			FG fgChild; BayesTree<Conditional> childTree;
			boost::tie(fgChild, childTree) = eliminateOneClique<Conditional>(child);

//			BOOST_FOREACH(const typename FG::sharedFactor& factor_, fgChild)
//				Ordering(factor_->keys()).print("factor from child:");

			fg.push_back(fgChild);
			children.push_back(childTree);
		}

		// eliminate the combined factors
		// warning: fg is being eliminated in-place and will contain marginal afterwards
		BayesNet<Conditional> bn = fg.eliminateFrontals(current->frontal_);

		// create a new clique corresponding the combined factors
		BayesTree<Conditional> bayesTree(bn, children);

		return make_pair(fg, bayesTree);
	}

	/* ************************************************************************* */
	template <class FG> template <class Conditional>
	BayesTree<Conditional> JunctionTree<FG>::eliminate() {
		pair<FG, BayesTree<Conditional> > ret = this->eliminateOneClique<Conditional>(this->root());
		if (ret.first.nrFactors() != 0)
			throw runtime_error("JuntionTree::eliminate: elimination failed because of factors left over!");
		return ret.second;
	}

} //namespace gtsam
