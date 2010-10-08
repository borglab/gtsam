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
//	template<class FG>
//	void EliminationTree<FG>::add(const FG& fg, varid_t j) {
//		sharedNode node(new Node(fg, j));
//		add(node);
//	}

  /* ************************************************************************* */
  template<class FG>
  void EliminationTree<FG>::add(const sharedNode& node) {

    assert(node->frontal.size() == 1);
    varid_t j = node->frontal.front();

    // Make a node and put it in the nodes_ array:
    nodes_[j] = node;

    // if the separator is empty, this is the root
    if (node->separator.empty()) {
      this->root_ = node;
    }
    else {
      // find parent by iterating over all separator keys, and taking the lowest
      // one in the ordering. That is the index of the parent clique.
      vector<varid_t>::const_iterator parentIndex = min_element(node->separator.begin(), node->separator.end());
      assert(parentIndex != node->separator.end());
      // attach to parent
      sharedNode& parent = nodes_[*parentIndex];
      if (!parent) throw
          invalid_argument("EliminationTree::add: parent clique does not exist");
      node->parent() = parent;
      parent->addChild(node);
    }
  }

	/* ************************************************************************* */
//	template<class FG>
//	EliminationTree<FG>::EliminationTree(const OrderedGraphs& graphs) :
//		nrVariables_(graphs.size()), nodes_(nrVariables_) {
//
//		// Get ordering by (map first graphs)
//		Ordering ordering;
//		transform(graphs.begin(), graphs.end(), back_inserter(ordering),
//				_Select1st<typename OrderedGraphs::value_type> ());
//
//		// Create a temporary map from key to ordering index
//		IndexTable<Symbol> indexTable(ordering);
//
//		// Go over the collection in reverse elimination order
//		// and add one node for every of the n variables.
//		BOOST_REVERSE_FOREACH(const NamedGraph& namedGraph, graphs)
//			add(namedGraph.second, namedGraph.first, indexTable);
//	}

	/* ************************************************************************* */
	template<class FG>
	EliminationTree<FG>::EliminationTree(FG& fg) {

	  static const bool debug = false;

	  // If the factor graph is empty, return an empty index because inside this
	  // if block we assume at least one factor.
	  if(fg.size() > 0) {

	    vector<deque<size_t> > clusters;

	    // Build clusters
	    {
	      // Find highest-numbered variable
	      varid_t maxVar = 0;
	      BOOST_FOREACH(const typename FG::sharedFactor& factor, fg) {
	        if(factor) {
	          typename FG::factor_type::const_iterator maxj = std::max_element(factor->begin(), factor->end());
	          if(maxj != factor->end() && *maxj > maxVar) maxVar = *maxj;
	        }
	      }
	      // Build index mapping from variable id to factor index - we only use
	      // the first variable because after this variable is eliminated the
	      // factor will no longer exist.
	      clusters.resize(maxVar+1);
	      for(size_t fi=0; fi<fg.size(); ++fi)
	        if(fg[fi] && !fg[fi]->keys().empty()) {
	          typename FG::factor_type::const_iterator firstvar = std::min_element(fg[fi]->begin(), fg[fi]->end());
	          assert(firstvar != fg[fi]->end());
	          clusters[*firstvar].push_back(fi);
	        }
	    }

	    // Create column index that will be modified during elimination - this is
	    // not the most efficient way of doing this, a modified version of
	    // Gilbert01bit would be more efficient.
	    vector<deque<size_t> > columnIndex = clusters;

	    nrVariables_ = columnIndex.size();
	    nodes_.resize(nrVariables_);

	    // Loop over all variables and get factors that are connected
	    OrderedGraphs graphs;
	    Nodes nodesToAdd; nodesToAdd.reserve(columnIndex.size());
	    for(varid_t j=0; j<columnIndex.size(); ++j) {

	      if(debug) cout << "Eliminating " << j << endl;

	      // The factor index of the new joint factor
	      size_t jointFactorI = fg.size();

	      // Get all of the factors associated with the variable.
        // If the factor has not already been removed - I think this is
        // somehow equivalent to the "find root" computation in Gilbert01bit.
	      vector<size_t> involvedFactors; involvedFactors.reserve(columnIndex[j].size());
	      BOOST_FOREACH(const size_t factorI, columnIndex[j]) {
	        if(fg[factorI]) involvedFactors.push_back(factorI);
	      }

	      if(!involvedFactors.empty()) {
	        // Compute a mapping (called variableSlots) *from* each involved
	        // variable that will be in the new joint factor *to* the slot in each
	        // removed factor in which that variable appears.  For each variable,
	        // this is stored as a vector of slot numbers, stored in order of the
	        // removed factors.  The slot number is the max integer value if the
	        // factor does not involve that variable.
	        typedef map<varid_t, vector<varid_t> > VariableSlots;
	        map<varid_t, vector<varid_t> > variableSlots;
	        FG removedFactors; removedFactors.reserve(involvedFactors.size());
	        size_t jointFactorPos = 0;
	        BOOST_FOREACH(const size_t factorI, involvedFactors) {
	          // Remove the factor from the factor graph
	          assert(fg[factorI]);
	          const typename FG::factor_type& removedFactor(*fg[factorI]);
	          assert(removedFactors.size() == jointFactorPos);
	          removedFactors.push_back(fg[factorI]);
	          fg.remove(factorI);

	          varid_t factorVarSlot = 0;
	          BOOST_FOREACH(const varid_t involvedVariable, removedFactor.keys()) {

	            // Set the slot in this factor for this variable.  If the
	            // variable was not already discovered, create an array for it
	            // that we'll fill with the slot indices for each factor that
	            // we're combining.  Initially we put the max integer value in
	            // the array entry for each factor that will indicate the factor
	            // does not involve the variable.
	            static vector<varid_t> empty;
	            VariableSlots::iterator thisVarSlots = variableSlots.insert(make_pair(involvedVariable,empty)).first;
	            if(thisVarSlots->second.empty())
	              thisVarSlots->second.resize(involvedFactors.size(), numeric_limits<varid_t>::max());
	            thisVarSlots->second[jointFactorPos] = factorVarSlot;
	            if(debug) cout << "  var " << involvedVariable << " rowblock " << jointFactorPos << " comes from factor " << factorI << " slot " << factorVarSlot << endl;

	            ++ factorVarSlot;
	          }
	          ++ jointFactorPos;
	        }
	        assert(variableSlots.begin()->first == j);

	        // Now that we know which factors and variables, and where variables
	        // come from and go to, create and eliminate the new joint factor.
	        typename FG::sharedFactor jointFactor = FG::factor_type::Combine(removedFactors, variableSlots);
	        assert(*jointFactor->begin() == j);
	        typename FG::factor_type::Conditional::shared_ptr conditional = jointFactor->eliminateFirst();
	        assert(conditional->key() == j);

	        // Add the eliminated joint factor to the partially-eliminated factor graph
	        fg.push_back(jointFactor);
	        assert(jointFactorI == fg.size()-1);

          // Add the joint factor to the column index for this variable if
          // it's not already added and it's not the variable we're about to
          // eliminate.
	        if(!jointFactor->keys().empty())
	          columnIndex[jointFactor->front()].push_back(jointFactorI);

	        // Create the new node, although it's parent and children will not be
	        // computed yet.
	        // todo: use cluster factors instead of removedFactors here.
	        nodesToAdd.push_back(typename Node::shared_ptr(new Node(removedFactors, conditional->key(),
	            conditional->beginParents(), conditional->endParents())));
	      }
	    }

	    // Go over the collection in reverse elimination order
	    // and add one node for every of the n variables.
	    BOOST_REVERSE_FOREACH(const sharedNode& node, nodesToAdd) {
	      add(node); }

	    if(debug) this->print("Completed elimination tree: ");
	  }
	}

/* ************************************************************************* */
} //namespace gtsam
