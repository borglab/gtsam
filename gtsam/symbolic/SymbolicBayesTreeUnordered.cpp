/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicBayesTree.h
 * @date Oct 29, 2009
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <boost/foreach.hpp>

#include <gtsam/symbolic/SymbolicBayesTreeUnordered.h>

namespace gtsam {

  /* ************************************************************************* */
  void SymbolicBayesTreeUnordered::insert(const sharedConditional& conditional)
  {
    static const bool debug = false;

    // get indices and parents
    const typename CONDITIONAL::Parents& parents = conditional->parents();

    if(debug) conditional->print("Adding conditional ");

    // if no parents, start a new root clique
    if (parents.empty()) {
      if(debug) std::cout << "No parents so making root" << std::endl;
      bayesTree.root_ = bayesTree.addClique(conditional);
      return;
    }

    // otherwise, find the parent clique by using the index data structure
    // to find the lowest-ordered parent
    Index parentRepresentative = bayesTree.findParentClique(parents);
    if(debug) std::cout << "First-eliminated parent is " << parentRepresentative << ", have " << bayesTree.nodes_.size() << " nodes." << std::endl;
    sharedClique parent_clique = bayesTree[parentRepresentative];
    if(debug) parent_clique->print("Parent clique is ");

    // if the parents and parent clique have the same size, add to parent clique
    if ((*parent_clique)->size() == size_t(parents.size())) {
      if(debug) std::cout << "Adding to parent clique" << std::endl;
#ifndef NDEBUG
      // Debug check that the parent indices of the new conditional match the indices
      // currently in the clique.
      //      list<Index>::const_iterator parent = parents.begin();
      //      typename Clique::const_iterator cond = parent_clique->begin();
      //      while(parent != parents.end()) {
      //        assert(cond != parent_clique->end());
      //        assert(*parent == (*cond)->key());
      //        ++ parent;
      //        ++ cond;
      //      }
#endif
      addToCliqueFront(bayesTree, conditional, parent_clique);
    } else {
      if(debug) std::cout << "Starting new clique" << std::endl;
      // otherwise, start a new clique and add it to the tree
      bayesTree.addClique(conditional,parent_clique);
    }
  }

  /* ************************************************************************* */
  void SymbolicBayesTreeUnordered::addToCliqueFront(const sharedConditional& conditional, const sharedClique& clique) {
    static const bool debug = false;
#ifndef NDEBUG
    // Debug check to make sure the conditional variable is ordered lower than
    // its parents and that all of its parents are present either in this
    // clique or its separator.
    BOOST_FOREACH(Key parent, conditional->parents()) {
      assert(parent > conditional->lastFrontalKey());
      const Clique& cliquer(*clique);
      assert(find(cliquer->begin(), cliquer->end(), parent) != cliquer->end());
    }
#endif
    if(debug) conditional->print("Adding conditional ");
    if(debug) clique->print("To clique ");
    Index j = conditional->lastFrontalKey();
    this->nodes_.resize(std::max(j+1, this->nodes_.size()));
    this->nodes_[j] = clique;
    FastVector<Index> newIndices(clique->conditional()->size() + 1);
    newIndices[0] = j;
    std::copy(clique->conditional()->begin(), clique->conditional()->end(), newIndices.begin()+1);
    clique->conditional_ = ConditionalType::FromKeys(newIndices, (*clique)->nrFrontals() + 1);
    if(debug) clique->print("Expanded clique is ");
    clique->assertInvariants();
  }

}
