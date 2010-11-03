/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * JunctionTree-inl.h
 * Created on: Feb 4, 2010
 * @Author: Kai Ni
 * @Author: Frank Dellaert
 * @brief: The junction tree, template bodies
 */

#pragma once

#include <gtsam/base/timing.h>
#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/BayesTree-inl.h>
#include <gtsam/inference/JunctionTree.h>
#include <gtsam/inference/inference-inl.h>
#include <gtsam/inference/VariableSlots.h>
#include <gtsam/inference/SymbolicSequentialSolver.h>
#include <gtsam/inference/ClusterTree-inl.h>

#include <boost/foreach.hpp>
#include <boost/pool/pool_alloc.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>

namespace gtsam {

	using namespace std;

	/* ************************************************************************* */
	template <class FG>
	JunctionTree<FG>::JunctionTree(const FG& fg) {
	  tic("JT 1  constructor");
		// Symbolic factorization: GaussianFactorGraph -> SymbolicFactorGraph
		// -> SymbolicBayesNet -> SymbolicBayesTree
		tic("JT 1.1  symbolic elimination");
		SymbolicBayesNet::shared_ptr sbn = SymbolicSequentialSolver(fg).eliminate();
//		SymbolicFactorGraph sfg(fg);
//		SymbolicBayesNet::shared_ptr sbn_orig = Inference::Eliminate(sfg);
//		assert(assert_equal(*sbn, *sbn_orig));
    toc("JT 1.1  symbolic elimination");
    tic("JT 1.2  symbolic BayesTree");
		SymbolicBayesTree sbt(*sbn);
		toc("JT 1.2  symbolic BayesTree");

		// distribute factors
    tic("JT 1.3  distributeFactors");
		this->root_ = distributeFactors(fg, sbt.root());
    toc("JT 1.3  distributeFactors");
		toc("JT 1  constructor");
	}

	/* ************************************************************************* */
	template<class FG>
	typename JunctionTree<FG>::sharedClique JunctionTree<FG>::distributeFactors(
			const FG& fg, const typename SymbolicBayesTree::sharedClique& bayesClique) {

	  // Build "target" index.  This is an index for each variable of the factors
	  // that involve this variable as their *lowest-ordered* variable.  For each
	  // factor, it is the lowest-ordered variable of that factor that pulls the
	  // factor into elimination, after which all of the information in the
	  // factor is contained in the eliminated factors that are passed up the
	  // tree as elimination continues.

	  // Two stages - first build an array of the lowest-ordered variable in each
	  // factor and find the last variable to be eliminated.
	  vector<Index> lowestOrdered(fg.size());
	  Index maxVar = 0;
	  for(size_t i=0; i<fg.size(); ++i)
	    if(fg[i]) {
	      typename FG::Factor::const_iterator min = std::min_element(fg[i]->begin(), fg[i]->end());
	      if(min == fg[i]->end())
	        lowestOrdered[i] = numeric_limits<Index>::max();
	      else {
	        lowestOrdered[i] = *min;
	        maxVar = std::max(maxVar, *min);
	      }
	    }

	  // Now add each factor to the list corresponding to its lowest-ordered
	  // variable.
	  vector<list<size_t, boost::fast_pool_allocator<size_t> > > targets(maxVar+1);
	  for(size_t i=0; i<lowestOrdered.size(); ++i)
	    if(lowestOrdered[i] != numeric_limits<Index>::max())
	      targets[lowestOrdered[i]].push_back(i);

	  // Now call the recursive distributeFactors
	  return distributeFactors(fg, targets, bayesClique);
	}

  /* ************************************************************************* */
	template<class FG>
	typename JunctionTree<FG>::sharedClique JunctionTree<FG>::distributeFactors(const FG& fg,
	    const std::vector<std::list<size_t,boost::fast_pool_allocator<size_t> > >& targets,
      const SymbolicBayesTree::sharedClique& bayesClique) {

	  if(bayesClique) {
	    // create a new clique in the junction tree
	    list<Index> frontals = bayesClique->ordering();
	    sharedClique clique(new Clique(frontals.begin(), frontals.end(), bayesClique->separator_.begin(), bayesClique->separator_.end()));

	    // count the factors for this cluster to pre-allocate space
	    {
	      size_t nFactors = 0;
	      BOOST_FOREACH(const Index frontal, clique->frontal) {
	        // There may be less variables in "targets" than there really are if
	        // some of the highest-numbered variables do not pull in any factors.
	        if(frontal < targets.size())
	          nFactors += targets[frontal].size(); }
	      clique->reserve(nFactors);
	    }
	    // add the factors to this cluster
	    BOOST_FOREACH(const Index frontal, clique->frontal) {
	      if(frontal < targets.size()) {
	        BOOST_FOREACH(const size_t factorI, targets[frontal]) {
	          clique->push_back(fg[factorI]); } } }

	    // recursively call the children
	    BOOST_FOREACH(const typename SymbolicBayesTree::sharedClique bayesChild, bayesClique->children()) {
	      sharedClique child = distributeFactors(fg, targets, bayesChild);
	      clique->addChild(child);
	      child->parent() = clique;
	    }
	    return clique;
	  } else
	    return sharedClique();
	}

	/* ************************************************************************* */
	template <class FG>
	pair<typename JunctionTree<FG>::BayesTree::sharedClique, typename FG::sharedFactor>
	JunctionTree<FG>::eliminateOneClique(const boost::shared_ptr<const Clique>& current) const {

		FG fg; // factor graph will be assembled from local factors and marginalized children
		fg.reserve(current->size() + current->children().size());
		fg.push_back(*current); // add the local factors

    // receive the factors from the child and its clique point
    list<typename BayesTree::sharedClique> children;
		BOOST_FOREACH(const boost::shared_ptr<const Clique>& child, current->children()) {
		  pair<typename BayesTree::sharedClique, typename FG::sharedFactor> tree_factor(
		      eliminateOneClique(child));
      children.push_back(tree_factor.first);
			fg.push_back(tree_factor.second);
		}

		// eliminate the combined factors
		// warning: fg is being eliminated in-place and will contain marginal afterwards
		tic("JT 2.1 VariableSlots");
		VariableSlots variableSlots(fg);
    toc("JT 2.1 VariableSlots");
#ifndef NDEBUG
    // Debug check that the keys found in the factors match the frontal and
    // separator keys of the clique.
    list<Index> allKeys;
    allKeys.insert(allKeys.end(), current->frontal.begin(), current->frontal.end());
    allKeys.insert(allKeys.end(), current->separator.begin(), current->separator.end());
    vector<Index> varslotsKeys(variableSlots.size());
    std::transform(variableSlots.begin(), variableSlots.end(), varslotsKeys.begin(),
        boost::lambda::bind(&VariableSlots::iterator::value_type::first, boost::lambda::_1));
    assert(std::equal(allKeys.begin(), allKeys.end(), varslotsKeys.begin()));
#endif

    // Now that we know which factors and variables, and where variables
    // come from and go to, create and eliminate the new joint factor.
    tic("JT 2.2 Combine");
    typename FG::sharedFactor jointFactor = FG::Factor::Combine(fg, variableSlots);
    toc("JT 2.2 Combine");
    tic("JT 2.3 Eliminate");
    typename BayesNet<typename FG::Factor::Conditional>::shared_ptr fragment = jointFactor->eliminate(current->frontal.size());
    toc("JT 2.3 Eliminate");
    assert(std::equal(jointFactor->begin(), jointFactor->end(), current->separator.begin()));

    tic("JT 2.4 Update tree");
		// create a new clique corresponding the combined factors
		typename BayesTree::sharedClique new_clique(new typename BayesTree::Clique(*fragment));
		new_clique->children_ = children;

		BOOST_FOREACH(typename BayesTree::sharedClique& childRoot, children)
			childRoot->parent_ = new_clique;

    toc("JT 2.4 Update tree");
		return make_pair(new_clique, jointFactor);
	}

	/* ************************************************************************* */
	template <class FG>
	typename JunctionTree<FG>::BayesTree::sharedClique JunctionTree<FG>::eliminate() const {
	  if(this->root()) {
	    tic("JT 2 eliminate");
	    pair<typename BayesTree::sharedClique, typename FG::sharedFactor> ret = this->eliminateOneClique(this->root());
	    if (ret.second->size() != 0)
	      throw runtime_error("JuntionTree::eliminate: elimination failed because of factors left over!");
	    toc("JT 2 eliminate");
	    return ret.first;
	  } else
	    return typename BayesTree::sharedClique();
	}

} //namespace gtsam
