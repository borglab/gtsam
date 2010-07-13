/*
 * ClusterTree-inl.h
 * Created on: July 13, 2010
 * @author Kai Ni
 * @author Frank Dellaert
 * @brief: Collects factorgraph fragments defined on variable clusters, arranged in a tree
 */

#pragma once

#include <boost/foreach.hpp>

#include "SymbolicFactorGraph.h"
#include "BayesTree-inl.h"
#include "ClusterTree.h"

namespace gtsam {

	using namespace std;

	/* ************************************************************************* */
	template <class FG>
	bool ClusterTree<FG>::Clique::equals(const ClusterTree<FG>::Clique& other) const {
		if (!frontal_.equals(other.frontal_))
			return false;

		if (!separator_.equals(other.separator_))
			return false;

		if (children_.size() != other.children_.size())
			return false;

		typename vector<shared_ptr>::const_iterator it1 = children_.begin();
		typename vector<shared_ptr>::const_iterator it2 = other.children_.begin();
		for(; it1!=children_.end(); it1++, it2++)
			if (!(*it1)->equals(**it2)) return false;

		return true;
	}

	/* ************************************************************************* */
	/**
	 * ClusterTree
	 */
	template <class FG>
	void ClusterTree<FG>::Clique::print(const string& indent) const {
		// FG::print(indent);
		cout << indent;
		BOOST_FOREACH(const Symbol& key, frontal_)
		cout << (string)key << " ";
		cout << ":";
		BOOST_FOREACH(const Symbol& key, separator_)
		cout << (string)key << " ";
		cout << endl;
	}

	/* ************************************************************************* */
	template <class FG>
	void ClusterTree<FG>::Clique::printTree(const string& indent) const {
		print(indent);
		BOOST_FOREACH(const shared_ptr& child, children_)
			child->printTree(indent+"  ");
	}

	/* ************************************************************************* */
	template <class FG>
	ClusterTree<FG>::ClusterTree(FG& fg, const Ordering& ordering) {
		// Symbolic factorization: GaussianFactorGraph -> SymbolicFactorGraph -> SymbolicBayesNet -> SymbolicBayesTree
		SymbolicFactorGraph sfg(fg);
		SymbolicBayesNet sbn = sfg.eliminate(ordering);
		BayesTree<SymbolicConditional> sbt(sbn);

		// distribtue factors
		root_ = distributeFactors(fg, sbt.root());
	}

	/* ************************************************************************* */
	template <class FG>
	typename ClusterTree<FG>::sharedClique ClusterTree<FG>::distributeFactors(FG& fg,
			const BayesTree<SymbolicConditional>::sharedClique bayesClique) {
		// create a new clique in the junction tree
		sharedClique clique(new Clique());
		clique->frontal_ = bayesClique->ordering();
		clique->separator_.insert(bayesClique->separator_.begin(), bayesClique->separator_.end());

		// recursively call the children
		BOOST_FOREACH(const BayesTree<SymbolicConditional>::sharedClique bayesChild, bayesClique->children()) {
			sharedClique child = distributeFactors(fg, bayesChild);
			clique->children_.push_back(child);
			child->parent_ = clique;
		}

		// collect the factors
		typedef vector<typename FG::sharedFactor> Factors;
		BOOST_FOREACH(const Symbol& frontal, clique->frontal_) {
			Factors factors = fg.template findAndRemoveFactors<Factors>(frontal);
			BOOST_FOREACH(const typename FG::sharedFactor& factor_, factors) {
				clique->push_back(factor_);
			}
		}

		return clique;
	}

	/* ************************************************************************* */
	template <class FG> template <class Conditional>
	pair<FG, BayesTree<Conditional> >
	ClusterTree<FG>::eliminateOneClique(sharedClique current) {

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
	BayesTree<Conditional> ClusterTree<FG>::eliminate() {
		pair<FG, BayesTree<Conditional> > ret = this->eliminateOneClique<Conditional>(root_);
//		ret.first.print("ret.first");
		if (ret.first.nrFactors() != 0)
			throw runtime_error("JuntionTree::eliminate: elimination failed because of factors left over!");
		return ret.second;
	}

	/* ************************************************************************* */
	template <class FG>
	bool ClusterTree<FG>::equals(const ClusterTree<FG>& other, double tol) const {
		if (!root_ || !other.root_) return false;
		return root_->equals(*other.root_);
	}

} //namespace gtsam
