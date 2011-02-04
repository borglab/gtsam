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
#include <gtsam/inference/VariableSlots.h>
#include <gtsam/inference/EliminationTree-inl.h>
#include <gtsam/inference/ClusterTree-inl.h>

#include <boost/foreach.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>

namespace gtsam {

  using namespace std;

  /* ************************************************************************* */
  template <class FG>
  void JunctionTree<FG>::construct(const FG& fg, const VariableIndex& variableIndex) {
    tic(1, "JT symbolic ET");
    const typename EliminationTree<IndexFactor>::shared_ptr symETree(EliminationTree<IndexFactor>::Create(fg, variableIndex));
    toc(1, "JT symbolic ET");
    tic(2, "JT symbolic eliminate");
    SymbolicBayesNet::shared_ptr sbn = symETree->eliminate();
    toc(2, "JT symbolic eliminate");
    tic(3, "symbolic BayesTree");
    SymbolicBayesTree sbt(*sbn);
    toc(3, "symbolic BayesTree");

    // distribute factors
    tic(4, "distributeFactors");
    this->root_ = distributeFactors(fg, sbt.root());
    toc(4, "distributeFactors");
  }

  /* ************************************************************************* */
  template <class FG>
  JunctionTree<FG>::JunctionTree(const FG& fg) {
    construct(fg, VariableIndex(fg));
  }

  /* ************************************************************************* */
  template <class FG>
  JunctionTree<FG>::JunctionTree(const FG& fg, const VariableIndex& variableIndex) {
    construct(fg, variableIndex);
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
    vector<FastList<size_t> > targets(maxVar+1);
    for(size_t i=0; i<lowestOrdered.size(); ++i)
      if(lowestOrdered[i] != numeric_limits<Index>::max())
        targets[lowestOrdered[i]].push_back(i);

    // Now call the recursive distributeFactors
    return distributeFactors(fg, targets, bayesClique);
  }

  /* ************************************************************************* */
  template<class FG>
  typename JunctionTree<FG>::sharedClique JunctionTree<FG>::distributeFactors(const FG& fg,
      const std::vector<FastList<size_t> >& targets,
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
  JunctionTree<FG>::eliminateOneClique(const boost::shared_ptr<const Clique>& current, bool cache) const {

    FG fg; // factor graph will be assembled from local factors and marginalized children
    fg.reserve(current->size() + current->children().size());
    fg.push_back(*current); // add the local factors

    // receive the factors from the child and its clique point
    list<typename BayesTree::sharedClique> children;
    BOOST_FOREACH(const boost::shared_ptr<const Clique>& child, current->children()) {
      pair<typename BayesTree::sharedClique, typename FG::sharedFactor> tree_factor(
          eliminateOneClique(child, cache));
      children.push_back(tree_factor.first);
      fg.push_back(tree_factor.second);
    }

    // eliminate the combined factors
    // warning: fg is being eliminated in-place and will contain marginal afterwards

    // Now that we know which factors and variables, and where variables
    // come from and go to, create and eliminate the new joint factor.
    tic(2, "CombineAndEliminate");
    pair<typename BayesNet<typename FG::Factor::Conditional>::shared_ptr, typename FG::sharedFactor> eliminated(
        FG::Factor::CombineAndEliminate(fg, current->frontal.size()));
    toc(2, "CombineAndEliminate");

    assert(std::equal(eliminated.second->begin(), eliminated.second->end(), current->separator.begin()));

    tic(3, "Update tree");
    // create a new clique corresponding the combined factors
    typename BayesTree::sharedClique new_clique(new typename BayesTree::Clique(*eliminated.first));
    new_clique->children_ = children;

    BOOST_FOREACH(typename BayesTree::sharedClique& childRoot, children) {
      childRoot->parent_ = new_clique;
    }
    if(cache)
      new_clique->cachedFactor() = eliminated.second;
    toc(3, "Update tree");

    return make_pair(new_clique, eliminated.second);
  }

  /* ************************************************************************* */
  template <class FG>
  typename JunctionTree<FG>::BayesTree::sharedClique JunctionTree<FG>::eliminate(bool cache) const {
    if(this->root()) {
      pair<typename BayesTree::sharedClique, typename FG::sharedFactor> ret = this->eliminateOneClique(this->root(), cache);
      if (ret.second->size() != 0)
        throw runtime_error("JuntionTree::eliminate: elimination failed because of factors left over!");
      return ret.first;
    } else
      return typename BayesTree::sharedClique();
  }

} //namespace gtsam
