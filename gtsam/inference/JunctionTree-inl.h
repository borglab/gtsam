/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file JunctionTree-inl.h
 * @date Feb 4, 2010
 * @author Kai Ni
 * @author Frank Dellaert
 * @brief The junction tree, template bodies
 */

#pragma once

#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/VariableSlots.h>
#include <gtsam/inference/EliminationTree.h>
#include <gtsam/base/timing.h>

#include <boost/foreach.hpp>

namespace gtsam {

  /* ************************************************************************* */
  template <class FG, class BTCLIQUE>
  void JunctionTree<FG,BTCLIQUE>::construct(const FG& fg, const VariableIndex& variableIndex) {
    gttic(JT_symbolic_ET);
    const typename EliminationTree<IndexFactor>::shared_ptr symETree =
        EliminationTree<IndexFactor>::Create(fg, variableIndex);
    assert(symETree.get());
    gttoc(JT_symbolic_ET);
    gttic(JT_symbolic_eliminate);
    SymbolicBayesNet::shared_ptr sbn = symETree->eliminate(&EliminateSymbolic);
    assert(sbn.get());
    gttoc(JT_symbolic_eliminate);
    gttic(symbolic_BayesTree);
    SymbolicBayesTree sbt(*sbn);
    gttoc(symbolic_BayesTree);

    // distribute factors
    gttic(distributeFactors);
    this->root_ = distributeFactors(fg, sbt.root());
    gttoc(distributeFactors);
  }

  /* ************************************************************************* */
  template <class FG, class BTCLIQUE>
  JunctionTree<FG,BTCLIQUE>::JunctionTree(const FG& fg) {
    gttic(VariableIndex);
    VariableIndex varIndex(fg);
    gttoc(VariableIndex);
    construct(fg, varIndex);
  }

  /* ************************************************************************* */
  template <class FG, class BTCLIQUE>
  JunctionTree<FG,BTCLIQUE>::JunctionTree(const FG& fg, const VariableIndex& variableIndex) {
    construct(fg, variableIndex);
  }

  /* ************************************************************************* */
  template<class FG, class BTCLIQUE>
  typename JunctionTree<FG,BTCLIQUE>::sharedClique JunctionTree<FG,BTCLIQUE>::distributeFactors(
      const FG& fg, const SymbolicBayesTree::sharedClique& bayesClique) {

    // Build "target" index.  This is an index for each variable of the factors
    // that involve this variable as their *lowest-ordered* variable.  For each
    // factor, it is the lowest-ordered variable of that factor that pulls the
    // factor into elimination, after which all of the information in the
    // factor is contained in the eliminated factors that are passed up the
    // tree as elimination continues.

    // Two stages - first build an array of the lowest-ordered variable in each
    // factor and find the last variable to be eliminated.
    std::vector<Index> lowestOrdered(fg.size(), std::numeric_limits<Index>::max());
    Index maxVar = 0;
    for(size_t i=0; i<fg.size(); ++i)
      if(fg[i]) {
        typename FG::FactorType::const_iterator min = std::min_element(fg[i]->begin(), fg[i]->end());
        if(min != fg[i]->end()) {
          lowestOrdered[i] = *min;
          maxVar = std::max(maxVar, *min);
        }
      }

    // Now add each factor to the list corresponding to its lowest-ordered
    // variable.
    std::vector<FastList<size_t> > targets(maxVar+1);
    for(size_t i=0; i<lowestOrdered.size(); ++i)
      if(lowestOrdered[i] != std::numeric_limits<Index>::max())
        targets[lowestOrdered[i]].push_back(i);

    // Now call the recursive distributeFactors
    return distributeFactors(fg, targets, bayesClique);
  }

  /* ************************************************************************* */
  template<class FG, class BTCLIQUE>
  typename JunctionTree<FG,BTCLIQUE>::sharedClique JunctionTree<FG,BTCLIQUE>::distributeFactors(const FG& fg,
      const std::vector<FastList<size_t> >& targets,
      const SymbolicBayesTree::sharedClique& bayesClique) {

    if(bayesClique) {
      // create a new clique in the junction tree
      sharedClique clique(new Clique((*bayesClique)->beginFrontals(), (*bayesClique)->endFrontals(),
          (*bayesClique)->beginParents(), (*bayesClique)->endParents()));

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
  template<class FG, class BTCLIQUE>
  std::pair<typename JunctionTree<FG,BTCLIQUE>::BTClique::shared_ptr,
      typename FG::sharedFactor> JunctionTree<FG,BTCLIQUE>::eliminateOneClique(
      typename FG::Eliminate function,
      const boost::shared_ptr<const Clique>& current) const {

    FG fg; // factor graph will be assembled from local factors and marginalized children
    fg.reserve(current->size() + current->children().size());
    fg.push_back(*current); // add the local factors

    // receive the factors from the child and its clique point
    std::list<typename BTClique::shared_ptr> children;
    BOOST_FOREACH(const boost::shared_ptr<const Clique>& child, current->children()) {
      std::pair<typename BTClique::shared_ptr, typename FG::sharedFactor> tree_factor(
          eliminateOneClique(function, child));
      children.push_back(tree_factor.first);
      fg.push_back(tree_factor.second);
    }

    // eliminate the combined factors
    // warning: fg is being eliminated in-place and will contain marginal afterwards

    // Now that we know which factors and variables, and where variables
    // come from and go to, create and eliminate the new joint factor.
    gttic(CombineAndEliminate);
    typename FG::EliminationResult eliminated(function(fg,
        current->frontal.size()));
    gttoc(CombineAndEliminate);

#ifdef GTSAM_EXTRA_CONSISTENCY_CHECKS
    assert(std::equal(eliminated.second->begin(), eliminated.second->end(), current->separator.begin()));
#endif

    gttic(Update_tree);
    // create a new clique corresponding the combined factors
    typename BTClique::shared_ptr new_clique(BTClique::Create(eliminated));
    new_clique->children_ = children;

    BOOST_FOREACH(typename BTClique::shared_ptr& childRoot, children) {
      childRoot->parent_ = new_clique;
    }
    gttoc(Update_tree);

    return std::make_pair(new_clique, eliminated.second);
  }

  /* ************************************************************************* */
  template<class FG, class BTCLIQUE>
  typename BTCLIQUE::shared_ptr JunctionTree<FG,BTCLIQUE>::eliminate(
      typename FG::Eliminate function) const {
    if (this->root()) {
      gttic(JT_eliminate);
      std::pair<typename BTClique::shared_ptr, typename FG::sharedFactor> ret =
          this->eliminateOneClique(function, this->root());
      if (ret.second->size() != 0) throw std::runtime_error(
          "JuntionTree::eliminate: elimination failed because of factors left over!");
      gttoc(JT_eliminate);
      return ret.first;
    } else
      return typename BTClique::shared_ptr();
  }

} //namespace gtsam
