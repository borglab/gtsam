/*
 * EliminationTree-inl.h
 * Created on: Feb 4, 2010
 * @Author: Kai Ni
 * @Author: Frank Dellaert
 * @brief: The elimination tree, template bodies
 */

#pragma once

#include <stdexcept>
#include <functional>
#include <boost/foreach.hpp>
#include <gtsam/inference/EliminationTree.h>

namespace gtsam {

	using namespace std;

	/* ************************************************************************* */
	template<class FG>
	void EliminationTree<FG>::add(const FG& fg, const Symbol& key,
			const IndexTable<Symbol>& indexTable) {

		// Make a node and put it in the nodes_ array:
		sharedNode node(new Node(fg, key));
		size_t j = indexTable(key);
		nodes_[j] = node;

		// if the separator is empty, this is the root
		if (node->separator_.empty()) {
			this->root_ = node;
		}
		else {
			// find parent by iterating over all separator keys, and taking the lowest
			// one in the ordering. That is the index of the parent clique.
			size_t parentIndex = nrVariables_;
			BOOST_FOREACH(const Symbol& j, node->separator_) {
					size_t index = indexTable(j);
					if (index<parentIndex) parentIndex = index;
				}
			// attach to parent
			sharedNode& parent = nodes_[parentIndex];
			if (!parent) throw
					invalid_argument("EliminationTree::add: parent clique does not exist");
			node->parent_ = parent;
			parent->children_.push_back(node);
		}
	}

	/* ************************************************************************* */
	template<class FG>
	EliminationTree<FG>::EliminationTree(const OrderedGraphs& graphs) :
		nrVariables_(graphs.size()), nodes_(nrVariables_) {

		// Get ordering by (map first graphs)
		Ordering ordering;
		transform(graphs.begin(), graphs.end(), back_inserter(ordering),
				_Select1st<typename OrderedGraphs::value_type> ());

		// Create a temporary map from key to ordering index
		IndexTable<Symbol> indexTable(ordering);

		// Go over the collection in reverse elimination order
		// and add one node for every of the n variables.
		BOOST_REVERSE_FOREACH(const NamedGraph& namedGraph, graphs)
			add(namedGraph.second, namedGraph.first, indexTable);
	}

	/* ************************************************************************* */
	template<class FG>
	EliminationTree<FG>::EliminationTree(FG& fg, const Ordering& ordering) :
		nrVariables_(ordering.size()), nodes_(nrVariables_) {

		// Loop over all variables and get factors that are connected
		OrderedGraphs graphs;
		BOOST_FOREACH(const Symbol& key, ordering) {
			// TODO: a collection of factors is a factor graph and this should be returned
			// below  rather than having to copy. GaussianFactorGraphSet should go...
			vector<typename FG::sharedFactor> found = fg.findAndRemoveFactors(key);
			FG fragment;
			NamedGraph namedGraph(key,fragment);
			BOOST_FOREACH(const typename FG::sharedFactor& factor, found)
				namedGraph.second.push_back(factor);
			graphs.push_back(namedGraph);
		}

		// Create a temporary map from key to ordering index
		IndexTable<Symbol> indexTable(ordering);

		// Go over the collection in reverse elimination order
		// and add one node for every of the n variables.
		BOOST_REVERSE_FOREACH(const NamedGraph& namedGraph, graphs)
			add(namedGraph.second, namedGraph.first, indexTable);
	}

/* ************************************************************************* */
} //namespace gtsam
