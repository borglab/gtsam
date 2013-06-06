/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BayesTree-inl.h
 * @brief   Bayes Tree is a tree of cliques of a Bayes Chain
 * @author  Frank Dellaert
 * @author  Michael Kaess
 * @author  Viorela Ila
 * @author  Richard Roberts
 */

#pragma once

#include <gtsam/inference/BayesTreeUnordered.h>
#include <gtsam/base/treeTraversal-inst.h>

#include <boost/foreach.hpp>
#include <fstream>

namespace gtsam {

  /* ************************************************************************* */
  template<class CLIQUE>
  typename BayesTreeUnordered<CLIQUE>::CliqueData
  BayesTreeUnordered<CLIQUE>::getCliqueData() const {
    CliqueData data;
    BOOST_FOREACH(const sharedClique& root, roots_)
      getCliqueData(data, root);
    return data;
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  void BayesTreeUnordered<CLIQUE>::getCliqueData(CliqueData& data, sharedClique clique) const {
    data.conditionalSizes.push_back(clique->conditional()->nrFrontals());
    data.separatorSizes.push_back(clique->conditional()->nrParents());
    BOOST_FOREACH(sharedClique c, clique->children) {
      getCliqueData(data, c);
    }
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  size_t BayesTreeUnordered<CLIQUE>::numCachedSeparatorMarginals() const {
    size_t count = 0;
    BOOST_FOREACH(const sharedClique& root, roots_)
      count += root->numCachedSeparatorMarginals();
    return count;
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  void BayesTreeUnordered<CLIQUE>::saveGraph(const std::string &s, const KeyFormatter& keyFormatter) const {
    if (roots_.empty()) throw std::invalid_argument("the root of Bayes tree has not been initialized!");
    std::ofstream of(s.c_str());
    of<< "digraph G{\n";
    BOOST_FOREACH(const sharedClique& root, roots_)
      saveGraph(of, root, keyFormatter);
    of<<"}";
    of.close();
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  void BayesTreeUnordered<CLIQUE>::saveGraph(std::ostream &s, sharedClique clique, const KeyFormatter& indexFormatter, int parentnum) const {
    static int num = 0;
    bool first = true;
    std::stringstream out;
    out << num;
    std::string parent = out.str();
    parent += "[label=\"";

    BOOST_FOREACH(Key index, clique->conditional_->frontals()) {
      if(!first) parent += ","; first = false;
      parent += indexFormatter(index);
    }

    if(clique->parent()){
      parent += " : ";
      s << parentnum << "->" << num << "\n";
    }

    first = true;
    BOOST_FOREACH(Key sep, clique->conditional_->parents()) {
      if(!first) parent += ","; first = false;
      parent += indexFormatter(sep);
    }
    parent += "\"];\n";
    s << parent;
    parentnum = num;

    BOOST_FOREACH(sharedClique c, clique->children) {
      num++;
      saveGraph(s, c, indexFormatter, parentnum);
    }
  }


  /* ************************************************************************* */
  template<class CLIQUE>
  void BayesTreeUnordered<CLIQUE>::CliqueStats::print(const std::string& s) const {
    std::cout << s
              << "avg Conditional Size: " << avgConditionalSize << std::endl
              << "max Conditional Size: " << maxConditionalSize << std::endl
              << "avg Separator Size: "   << avgSeparatorSize   << std::endl
              << "max Separator Size: "   << maxSeparatorSize   << std::endl;
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  typename BayesTreeUnordered<CLIQUE>::CliqueStats
  BayesTreeUnordered<CLIQUE>::CliqueData::getStats() const
  {
    CliqueStats stats;

    double sum = 0.0;
    size_t max = 0;
    BOOST_FOREACH(size_t s, conditionalSizes) {
      sum += (double)s;
      if(s > max) max = s;
    }
    stats.avgConditionalSize = sum / (double)conditionalSizes.size();
    stats.maxConditionalSize = max;

    sum = 0.0;
    max = 1;
    BOOST_FOREACH(size_t s, separatorSizes) {
      sum += (double)s;
      if(s > max) max = s;
    }
    stats.avgSeparatorSize = sum / (double)separatorSizes.size();
    stats.maxSeparatorSize = max;

    return stats;
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  void BayesTreeUnordered<CLIQUE>::Cliques::print(const std::string& s, const KeyFormatter& keyFormatter) const {
    std::cout << s << ":\n";
    BOOST_FOREACH(sharedClique clique, *this) {
      clique->printTree("", keyFormatter); }
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  bool BayesTreeUnordered<CLIQUE>::Cliques::equals(const Cliques& other, double tol) const {
    return other == *this;
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  size_t BayesTreeUnordered<CLIQUE>::size() const {
    size_t size = 0;
    BOOST_FOREACH(const sharedClique& clique, roots_)
      size += clique->treeSize();
    return size;
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  void BayesTreeUnordered<CLIQUE>::addClique(const sharedClique& clique, const sharedClique& parent_clique) {
    BOOST_FOREACH(Index j, clique->conditional()->frontals())
      nodes_[j] = clique;
    if (parent_clique != NULL) {
      clique->parent_ = parent_clique;
      parent_clique->children.push_back(clique);
    } else {
      roots_.push_back(clique);
    }
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  typename BayesTreeUnordered<CLIQUE>::sharedClique BayesTreeUnordered<CLIQUE>::addClique(
      const sharedConditional& conditional, std::list<sharedClique>& child_cliques)
  {
    sharedClique new_clique(new Clique(conditional));
    BOOST_FOREACH(Index j, conditional->frontals())
      nodes_[j] = new_clique;
    new_clique->children.assign(child_cliques.begin(), child_cliques.end());
    BOOST_FOREACH(sharedClique& child, child_cliques)
      child->parent_ = new_clique;
    return new_clique;
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  void BayesTreeUnordered<CLIQUE>::removeClique(sharedClique clique)
  {
    if (clique->isRoot())
      roots_.erase(std::find(roots_.begin(), roots_.end(), clique));
    else { // detach clique from parent
      sharedClique parent = clique->parent_.lock();
      typename std::vector<sharedClique>::iterator child = std::find(parent->children.begin(), parent->children.end(), clique);
      assert(child != parent->children.end());
      parent->children.erase(child);
    }

    // orphan my children
    BOOST_FOREACH(sharedClique child, clique->children)
      child->parent_ = typename Clique::weak_ptr();

    BOOST_FOREACH(Key j, clique->conditional()->frontals()) {
      nodes_.erase(j);
    }
  }

  /* ************************************************************************* */
  //template<class CLIQUE>
  //void BayesTreeUnordered<CLIQUE>::recursiveTreeBuild(const boost::shared_ptr<BayesTreeClique<IndexConditional> >& symbolic,
  //    const std::vector<boost::shared_ptr<CONDITIONAL> >& conditionals,
  //    const typename BayesTreeUnordered<CLIQUE>::sharedClique& parent) {

  //  // Helper function to build a non-symbolic tree (e.g. Gaussian) using a
  //  // symbolic tree, used in the BT(BN) constructor.

  //  // Build the current clique
  //  FastList<typename CONDITIONAL::shared_ptr> cliqueConditionals;
  //  BOOST_FOREACH(Index j, symbolic->conditional()->frontals()) {
  //    cliqueConditionals.push_back(conditionals[j]); }
  //  typename BayesTreeUnordered<CLIQUE>::sharedClique thisClique(new CLIQUE(CONDITIONAL::Combine(cliqueConditionals.begin(), cliqueConditionals.end())));

  //  // Add the new clique with the current parent
  //  this->addClique(thisClique, parent);

  //  // Build the children, whose parent is the new clique
  //  BOOST_FOREACH(const BayesTreeUnordered<IndexConditional>::sharedClique& child, symbolic->children()) {
  //    this->recursiveTreeBuild(child, conditionals, thisClique); }
  //}

  /* ************************************************************************* */
  //template<class CLIQUE>
  //BayesTreeUnordered<CLIQUE>::BayesTreeUnordered(const BayesNet<CONDITIONAL>& bayesNet) {
  //  // First generate symbolic BT to determine clique structure
  //  BayesTreeUnordered<IndexConditional> sbt(bayesNet);

  //  // Build index of variables to conditionals
  //  std::vector<boost::shared_ptr<CONDITIONAL> > conditionals(sbt.root()->conditional()->frontals().back() + 1);
  //  BOOST_FOREACH(const boost::shared_ptr<CONDITIONAL>& c, bayesNet) {
  //    if(c->nrFrontals() != 1)
  //      throw std::invalid_argument("BayesTreeUnordered constructor from BayesNet only supports single frontal variable conditionals");
  //    if(c->firstFrontalKey() >= conditionals.size())
  //      throw std::invalid_argument("An inconsistent BayesNet was passed into the BayesTreeUnordered constructor!");
  //    if(conditionals[c->firstFrontalKey()])
  //      throw std::invalid_argument("An inconsistent BayesNet with duplicate frontal variables was passed into the BayesTreeUnordered constructor!");

  //    conditionals[c->firstFrontalKey()] = c;
  //  }

  //  // Build the new tree
  //  this->recursiveTreeBuild(sbt.root(), conditionals, sharedClique());
  //}

  /* ************************************************************************* */
  //template<>
  //inline BayesTreeUnordered<IndexConditional>::BayesTreeUnordered(const BayesNet<IndexConditional>& bayesNet) {
  //  BayesNet<IndexConditional>::const_reverse_iterator rit;
  //  for ( rit=bayesNet.rbegin(); rit != bayesNet.rend(); ++rit )
  //    insert(*this, *rit);
  //}

  /* ************************************************************************* */
  //template<class CLIQUE>
  //BayesTreeUnordered<CLIQUE>::BayesTreeUnordered(const BayesNet<CONDITIONAL>& bayesNet, std::list<BayesTreeUnordered<CLIQUE> > subtrees) {
  //  if (bayesNet.size() == 0)
  //    throw std::invalid_argument("BayesTreeUnordered::insert: empty bayes net!");

  //  // get the roots of child subtrees and merge their nodes_
  //  std::list<sharedClique> childRoots;
  //  typedef BayesTreeUnordered<CLIQUE> Tree;
  //  BOOST_FOREACH(const Tree& subtree, subtrees) {
  //    nodes_.assign(subtree.nodes_.begin(), subtree.nodes_.end());
  //    childRoots.push_back(subtree.root());
  //  }

  //  // create a new clique and add all the conditionals to the clique
  //  sharedClique new_clique;
  //  typename BayesNet<CONDITIONAL>::sharedConditional conditional;
  //  BOOST_REVERSE_FOREACH(conditional, bayesNet) {
  //    if (!new_clique.get())
  //      new_clique = addClique(conditional,childRoots);
  //    else
  //      addToCliqueFront(*this, conditional, new_clique);
  //  }

  //  root_ = new_clique;
  //}

  /* ************************************************************************* */
  template<class CLIQUE>
  BayesTreeUnordered<CLIQUE>::BayesTreeUnordered(const This& other) {
    *this = other;
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  BayesTreeUnordered<CLIQUE>& BayesTreeUnordered<CLIQUE>::operator=(const This& other) {
    this->clear();
    std::vector<sharedClique> clonedRoots = treeTraversal::CloneForest(other);
    BOOST_FOREACH(const sharedClique& root, clonedRoots)
      insertRoot(root);
    return *this;
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  void BayesTreeUnordered<CLIQUE>::print(const std::string& s, const KeyFormatter& keyFormatter) const {
    std::cout << s << ": cliques: " << size() << ", variables: " << nodes_.size() << std::endl;
    treeTraversal::PrintForest(*this, s, keyFormatter);
  }

  /* ************************************************************************* */
  // binary predicate to test equality of a pair for use in equals
  template<class CLIQUE>
  bool check_sharedCliques(
      const std::pair<Key, typename BayesTreeUnordered<CLIQUE>::sharedClique>& v1,
      const std::pair<Key, typename BayesTreeUnordered<CLIQUE>::sharedClique>& v2
  ) {
    return v1.first == v2.first &&
      ((!v1.second && !v2.second) || (v1.second && v2.second && v1.second->equals(*v2.second)));
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  bool BayesTreeUnordered<CLIQUE>::equals(const BayesTreeUnordered<CLIQUE>& other, double tol) const {
    return size()==other.size() &&
      std::equal(nodes_.begin(), nodes_.end(), other.nodes_.begin(), &check_sharedCliques<CLIQUE>);
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  template<class CONTAINER>
  Key BayesTreeUnordered<CLIQUE>::findParentClique(const CONTAINER& parents) const {
    typename CONTAINER::const_iterator lowestOrderedParent = min_element(parents.begin(), parents.end());
    assert(lowestOrderedParent != parents.end());
    return *lowestOrderedParent;
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  void BayesTreeUnordered<CLIQUE>::fillNodesIndex(const sharedClique& subtree) {
    // Add each frontal variable of this root node
    BOOST_FOREACH(const Key& j, subtree->conditional()->frontals()) { nodes_[j] = subtree; }
    // Fill index for each child
    typedef typename BayesTreeUnordered<CLIQUE>::sharedClique sharedClique;
    BOOST_FOREACH(const sharedClique& child, subtree->children) {
      fillNodesIndex(child); }
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  void BayesTreeUnordered<CLIQUE>::insertRoot(const sharedClique& subtree) {
    roots_.push_back(subtree); // Add to roots
    fillNodesIndex(subtree); // Populate nodes index
  }

  /* ************************************************************************* */
  // First finds clique marginal then marginalizes that
  /* ************************************************************************* */
  template<class CLIQUE>
  typename BayesTreeUnordered<CLIQUE>::sharedFactor BayesTreeUnordered<CLIQUE>::marginalFactor(
      Key j, Eliminate function) const
  {
    return boost::make_shared<FactorType>();
//    gttic(BayesTree_marginalFactor);
//
//    // get clique containing Index j
//    sharedClique clique = this->clique(j);
//
//    // calculate or retrieve its marginal P(C) = P(F,S)
//#ifdef OLD_SHORTCUT_MARGINALS
//    FactorGraph<FactorType> cliqueMarginal = clique->marginal(root_,function);
//#else
//    FactorGraph<FactorType> cliqueMarginal = clique->marginal2(root_,function);
//#endif
//
//    // Reduce the variable indices to start at zero
//    gttic(Reduce);
//    const Permutation reduction = internal::createReducingPermutation(cliqueMarginal.keys());
//    internal::Reduction inverseReduction = internal::Reduction::CreateAsInverse(reduction);
//    BOOST_FOREACH(const boost::shared_ptr<FactorType>& factor, cliqueMarginal) {
//      if(factor) factor->reduceWithInverse(inverseReduction); }
//    gttoc(Reduce);
//
//    // now, marginalize out everything that is not variable j
//    GenericSequentialSolver<FactorType> solver(cliqueMarginal);
//    boost::shared_ptr<FactorType> result = solver.marginalFactor(inverseReduction[j], function);
//
//    // Undo the reduction
//    gttic(Undo_Reduce);
//    result->permuteWithInverse(reduction);
//    BOOST_FOREACH(const boost::shared_ptr<FactorType>& factor, cliqueMarginal) {
//      if(factor) factor->permuteWithInverse(reduction); }
//    gttoc(Undo_Reduce);
//    return result;
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  typename BayesTreeUnordered<CLIQUE>::sharedBayesNet BayesTreeUnordered<CLIQUE>::marginalBayesNet(
      Key j, Eliminate function) const
  {
    return boost::make_shared<BayesNetType>();
    //gttic(BayesTree_marginalBayesNet);

    //// calculate marginal as a factor graph
    //FactorGraph<FactorType> fg;
    //fg.push_back(this->marginalFactor(j,function));

    //// Reduce the variable indices to start at zero
    //gttic(Reduce);
    //const Permutation reduction = internal::createReducingPermutation(fg.keys());
    //internal::Reduction inverseReduction = internal::Reduction::CreateAsInverse(reduction);
    //BOOST_FOREACH(const boost::shared_ptr<FactorType>& factor, fg) {
    //  if(factor) factor->reduceWithInverse(inverseReduction); }
    //gttoc(Reduce);

    //// eliminate factor graph marginal to a Bayes net
    //boost::shared_ptr<BayesNet<CONDITIONAL> > bn = GenericSequentialSolver<FactorType>(fg).eliminate(function);

    //// Undo the reduction
    //gttic(Undo_Reduce);
    //bn->permuteWithInverse(reduction);
    //BOOST_FOREACH(const boost::shared_ptr<FactorType>& factor, fg) {
    //  if(factor) factor->permuteWithInverse(reduction); }
    //gttoc(Undo_Reduce);
    //return bn;
 }

  /* ************************************************************************* */
  // Find two cliques, their joint, then marginalizes
  /* ************************************************************************* */
  template<class CLIQUE>
  typename BayesTreeUnordered<CLIQUE>::sharedFactorGraph
  BayesTreeUnordered<CLIQUE>::joint(Key j1, Key j2, Eliminate function) const {
    return boost::make_shared<FactorGraphType>();
    //gttic(BayesTree_joint);

    //// get clique C1 and C2
    //sharedClique C1 = (*this)[j1], C2 = (*this)[j2];

    //gttic(Lowest_common_ancestor);
    //// Find lowest common ancestor clique
    //sharedClique B; {
    //  // Build two paths to the root
    //  FastList<sharedClique> path1, path2; {
    //    sharedClique p = C1;
    //    while(p) {
    //      path1.push_front(p);
    //      p = p->parent();
    //    }
    //  } {
    //    sharedClique p = C2;
    //    while(p) {
    //      path2.push_front(p);
    //      p = p->parent();
    //    }
    //  }
    //  // Find the path intersection
    //  B = this->root();
    //  typename FastList<sharedClique>::const_iterator p1 = path1.begin(), p2 = path2.begin();
    //  while(p1 != path1.end() && p2 != path2.end() && *p1 == *p2) {
    //    B = *p1;
    //    ++p1;
    //    ++p2;
    //  }
    //}
    //gttoc(Lowest_common_ancestor);

    //// Compute marginal on lowest common ancestor clique
    //gttic(LCA_marginal);
    //FactorGraph<FactorType> p_B = B->marginal2(this->root(), function);
    //gttoc(LCA_marginal);

    //// Compute shortcuts of the requested cliques given the lowest common ancestor
    //gttic(Clique_shortcuts);
    //BayesNet<CONDITIONAL> p_C1_Bred = C1->shortcut(B, function);
    //BayesNet<CONDITIONAL> p_C2_Bred = C2->shortcut(B, function);
    //gttoc(Clique_shortcuts);

    //// Factor the shortcuts to be conditioned on the full root
    //// Get the set of variables to eliminate, which is C1\B.
    //gttic(Full_root_factoring);
    //sharedConditional p_C1_B; {
    //  std::vector<Index> C1_minus_B; {
    //    FastSet<Index> C1_minus_B_set(C1->conditional()->beginParents(), C1->conditional()->endParents());
    //    BOOST_FOREACH(const Index j, *B->conditional()) {
    //      C1_minus_B_set.erase(j); }
    //    C1_minus_B.assign(C1_minus_B_set.begin(), C1_minus_B_set.end());
    //  }
    //  // Factor into C1\B | B.
    //  FactorGraph<FactorType> temp_remaining;
    //  boost::tie(p_C1_B, temp_remaining) = FactorGraph<FactorType>(p_C1_Bred).eliminate(C1_minus_B, function);
    //}
    //sharedConditional p_C2_B; {
    //  std::vector<Index> C2_minus_B; {
    //    FastSet<Index> C2_minus_B_set(C2->conditional()->beginParents(), C2->conditional()->endParents());
    //    BOOST_FOREACH(const Index j, *B->conditional()) {
    //      C2_minus_B_set.erase(j); }
    //    C2_minus_B.assign(C2_minus_B_set.begin(), C2_minus_B_set.end());
    //  }
    //  // Factor into C2\B | B.
    //  FactorGraph<FactorType> temp_remaining;
    //  boost::tie(p_C2_B, temp_remaining) = FactorGraph<FactorType>(p_C2_Bred).eliminate(C2_minus_B, function);
    //}
    //gttoc(Full_root_factoring);

    //gttic(Variable_joint);
    //// Build joint on all involved variables
    //FactorGraph<FactorType> p_BC1C2;
    //p_BC1C2.push_back(p_B);
    //p_BC1C2.push_back(p_C1_B->toFactor());
    //p_BC1C2.push_back(p_C2_B->toFactor());
    //if(C1 != B)
    //  p_BC1C2.push_back(C1->conditional()->toFactor());
    //if(C2 != B)
    //  p_BC1C2.push_back(C2->conditional()->toFactor());

    //// Reduce the variable indices to start at zero
    //gttic(Reduce);
    //const Permutation reduction = internal::createReducingPermutation(p_BC1C2.keys());
    //internal::Reduction inverseReduction = internal::Reduction::CreateAsInverse(reduction);
    //BOOST_FOREACH(const boost::shared_ptr<FactorType>& factor, p_BC1C2) {
    //  if(factor) factor->reduceWithInverse(inverseReduction); }
    //std::vector<Index> js; js.push_back(inverseReduction[j1]); js.push_back(inverseReduction[j2]);
    //gttoc(Reduce);

    //// now, marginalize out everything that is not variable j
    //GenericSequentialSolver<FactorType> solver(p_BC1C2);
    //boost::shared_ptr<FactorGraph<FactorType> > result = solver.jointFactorGraph(js, function);

    //// Undo the reduction
    //gttic(Undo_Reduce);
    //BOOST_FOREACH(const boost::shared_ptr<FactorType>& factor, *result) {
    //  if(factor) factor->permuteWithInverse(reduction); }
    //BOOST_FOREACH(const boost::shared_ptr<FactorType>& factor, p_BC1C2) {
    //  if(factor) factor->permuteWithInverse(reduction); }
    //gttoc(Undo_Reduce);
    //return result;
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  typename BayesTreeUnordered<CLIQUE>::sharedBayesNet BayesTreeUnordered<CLIQUE>::jointBayesNet(
      Key j1, Key j2, Eliminate function) const
  {
    return boost::make_shared<BayesNetType>();
    //// eliminate factor graph marginal to a Bayes net
    //return GenericSequentialSolver<FactorType> (
    //    *this->joint(j1, j2, function)).eliminate(function);
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  void BayesTreeUnordered<CLIQUE>::clear() {
    // Remove all nodes and clear the root pointer
    nodes_.clear();
    roots_.clear();
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  void BayesTreeUnordered<CLIQUE>::deleteCachedShortcuts() {
    BOOST_FOREACH(const sharedClique& root, roots_) {
      root->deleteCachedShortcuts();
    }
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  void BayesTreeUnordered<CLIQUE>::removePath(sharedClique clique, BayesNetType& bn, Cliques& orphans)
  {
    // base case is NULL, if so we do nothing and return empties above
    if (clique!=NULL) {

      // remove the clique from orphans in case it has been added earlier
      orphans.remove(clique);

      // remove me
      this->removeClique(clique);

      // remove path above me
      this->removePath(typename Clique::shared_ptr(clique->parent_.lock()), bn, orphans);

      // add children to list of orphans (splice also removed them from clique->children_)
      orphans.insert(orphans.begin(), clique->children.begin(), clique->children.end());
      clique->children.clear();

      bn.push_back(clique->conditional());

    }
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  template<class CONTAINER>
  void BayesTreeUnordered<CLIQUE>::removeTop(const CONTAINER& keys, BayesNetType& bn, Cliques& orphans)
  {
    // process each key of the new factor
    BOOST_FOREACH(const Key& j, keys) {

      // get the clique
      if(j < nodes_.size()) {
        const sharedClique& clique(nodes_[j]);
        if(clique) {
          // remove path from clique to root
          this->removePath(clique, bn, orphans);
        }
      }
    }

    // Delete cachedShortcuts for each orphan subtree
    //TODO: Consider Improving
    BOOST_FOREACH(sharedClique& orphan, orphans)
      orphan->deleteCachedShortcuts();
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  typename BayesTreeUnordered<CLIQUE>::Cliques BayesTreeUnordered<CLIQUE>::removeSubtree(
    const sharedClique& subtree)
  {
    // Result clique list
    Cliques cliques;
    cliques.push_back(subtree);

    // Remove the first clique from its parents
    if(!subtree->isRoot())
      subtree->parent()->children.erase(std::find(
      subtree->parent()->children.begin(), subtree->parent()->children.end(), subtree));
    else
      roots_.erase(std::find(roots_.begin(), roots_.end(), subtree));

    // Add all subtree cliques and erase the children and parent of each
    for(typename Cliques::iterator clique = cliques.begin(); clique != cliques.end(); ++clique)
    {
      // Add children
      BOOST_FOREACH(const sharedClique& child, (*clique)->children) {
        cliques.push_back(child); }

      // Delete cached shortcuts
      (*clique)->deleteCachedShortcutsNonRecursive();

      // Remove this node from the nodes index
      BOOST_FOREACH(Key j, (*clique)->conditional()->frontals()) {
        nodes_.erase(j); }

      // Erase the parent and children pointers
      (*clique)->parent_.reset();
      (*clique)->children.clear();
    }

    return cliques;
  }

}
/// namespace gtsam
