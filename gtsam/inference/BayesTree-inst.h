/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BayesTree-inst.h
 * @brief   Bayes Tree is a tree of cliques of a Bayes Chain
 * @author  Frank Dellaert
 * @author  Michael Kaess
 * @author  Viorela Ila
 * @author  Richard Roberts
 */

#pragma once

#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/BayesTree.h>
#include <gtsam/base/treeTraversal-inst.h>
#include <gtsam/base/timing.h>

#include <fstream>

namespace gtsam {

  /* ************************************************************************* */
  template<class CLIQUE>
  BayesTreeCliqueData BayesTree<CLIQUE>::getCliqueData() const {
    BayesTreeCliqueData stats;
    for (const sharedClique& root : roots_) getCliqueData(root, &stats);
    return stats;
  }

  /* ************************************************************************* */
  template <class CLIQUE>
  void BayesTree<CLIQUE>::getCliqueData(sharedClique clique,
                                        BayesTreeCliqueData* stats) const {
    const auto conditional = clique->conditional();
    stats->conditionalSizes.push_back(conditional->nrFrontals());
    stats->separatorSizes.push_back(conditional->nrParents());
    for (sharedClique c : clique->children) {
      getCliqueData(c, stats);
    }
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  size_t BayesTree<CLIQUE>::numCachedSeparatorMarginals() const {
    size_t count = 0;
    for(const sharedClique& root: roots_)
      count += root->numCachedSeparatorMarginals();
    return count;
  }

  /* ************************************************************************* */
  template <class CLIQUE>
  void BayesTree<CLIQUE>::dot(std::ostream& os,
                              const KeyFormatter& keyFormatter) const {
    if (roots_.empty())
      throw std::invalid_argument(
          "the root of Bayes tree has not been initialized!");
    os << "digraph G{\n";
    for (const sharedClique& root : roots_) dot(os, root, keyFormatter);
    os << "}";
    std::flush(os);
  }

  /* ************************************************************************* */
  template <class CLIQUE>
  std::string BayesTree<CLIQUE>::dot(const KeyFormatter& keyFormatter) const {
    std::stringstream ss;
    dot(ss, keyFormatter);
    return ss.str();
  }

  /* ************************************************************************* */
  template <class CLIQUE>
  void BayesTree<CLIQUE>::saveGraph(const std::string& filename,
                                    const KeyFormatter& keyFormatter) const {
    std::ofstream of(filename.c_str());
    dot(of, keyFormatter);
    of.close();
  }

  /* ************************************************************************* */
  template <class CLIQUE>
  void BayesTree<CLIQUE>::dot(std::ostream& s, sharedClique clique,
                              const KeyFormatter& keyFormatter,
                              int parentnum) const {
    static int num = 0;
    bool first = true;
    std::stringstream out;
    out << num;
    std::string parent = out.str();
    parent += "[label=\"";

    for (Key key : clique->conditional_->frontals()) {
      if (!first) parent += ", ";
      first = false;
      parent += keyFormatter(key);
    }

    if (clique->parent()) {
      parent += " : ";
      s << parentnum << "->" << num << "\n";
    }

    first = true;
    for (Key parentKey : clique->conditional_->parents()) {
      if (!first) parent += ", ";
      first = false;
      parent += keyFormatter(parentKey);
    }
    parent += "\"];\n";
    s << parent;
    parentnum = num;

    for (sharedClique c : clique->children) {
      num++;
      dot(s, c, keyFormatter, parentnum);
    }
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  size_t BayesTree<CLIQUE>::size() const {
    size_t size = 0;
    for(const sharedClique& clique: roots_)
      size += clique->treeSize();
    return size;
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  void BayesTree<CLIQUE>::addClique(const sharedClique& clique, const sharedClique& parent_clique) {
    for(Key j: clique->conditional()->frontals())
      nodes_[j] = clique;
    if (parent_clique != nullptr) {
      clique->parent_ = parent_clique;
      parent_clique->children.push_back(clique);
    } else {
      roots_.push_back(clique);
    }
  }

  /* ************************************************************************* */
  namespace {
  template <class FACTOR, class CLIQUE>
  struct _pushCliqueFunctor {
    _pushCliqueFunctor(FactorGraph<FACTOR>* graph_) : graph(graph_) {}
    FactorGraph<FACTOR>* graph;
    int operator()(const std::shared_ptr<CLIQUE>& clique, int dummy) {
      graph->push_back(clique->conditional_);
      return 0;
    }
  };
  }  // namespace

  /* ************************************************************************* */
  template <class CLIQUE>
  void BayesTree<CLIQUE>::addFactorsToGraph(
      FactorGraph<FactorType>* graph) const {
    // Traverse the BayesTree and add all conditionals to this graph
    int data = 0;  // Unused
    _pushCliqueFunctor<FactorType, CLIQUE> functor(graph);
    treeTraversal::DepthFirstForest(*this, data, functor);
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  BayesTree<CLIQUE>::BayesTree(const This& other) {
    *this = other;
  }

  /* ************************************************************************* */
  namespace {
    template<typename NODE>
    std::shared_ptr<NODE>
      BayesTreeCloneForestVisitorPre(const std::shared_ptr<NODE>& node, const std::shared_ptr<NODE>& parentPointer)
    {
      // Clone the current node and add it to its cloned parent
      std::shared_ptr<NODE> clone = std::make_shared<NODE>(*node);
      clone->children.clear();
      clone->parent_ = parentPointer;
      parentPointer->children.push_back(clone);
      return clone;
    }
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  BayesTree<CLIQUE>& BayesTree<CLIQUE>::operator=(const This& other) {
    this->clear();
    std::shared_ptr<Clique> rootContainer = std::make_shared<Clique>();
    treeTraversal::DepthFirstForest(other, rootContainer, BayesTreeCloneForestVisitorPre<Clique>);
    for(const sharedClique& root: rootContainer->children) {
      root->parent_ = typename Clique::weak_ptr(); // Reset the parent since it's set to the dummy clique
      insertRoot(root);
    }
    return *this;
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  void BayesTree<CLIQUE>::print(const std::string& s, const KeyFormatter& keyFormatter) const {
    std::cout << s << ": cliques: " << size() << ", variables: " << nodes_.size() << std::endl;
    treeTraversal::PrintForest(*this, s, keyFormatter);
  }

  /* ************************************************************************* */
  // binary predicate to test equality of a pair for use in equals
  template<class CLIQUE>
  bool check_sharedCliques(
      const std::pair<Key, typename BayesTree<CLIQUE>::sharedClique>& v1,
      const std::pair<Key, typename BayesTree<CLIQUE>::sharedClique>& v2
  ) {
    return v1.first == v2.first &&
      ((!v1.second && !v2.second) || (v1.second && v2.second && v1.second->equals(*v2.second)));
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  bool BayesTree<CLIQUE>::equals(const BayesTree<CLIQUE>& other, double tol) const {
    return size()==other.size() &&
      std::equal(nodes_.begin(), nodes_.end(), other.nodes_.begin(), &check_sharedCliques<CLIQUE>);
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  template<class CONTAINER>
  Key BayesTree<CLIQUE>::findParentClique(const CONTAINER& parents) const {
    typename CONTAINER::const_iterator lowestOrderedParent = min_element(parents.begin(), parents.end());
    assert(lowestOrderedParent != parents.end());
    return *lowestOrderedParent;
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  void BayesTree<CLIQUE>::fillNodesIndex(const sharedClique& subtree) {
    // Add each frontal variable of this root node
    for(const Key& j: subtree->conditional()->frontals()) {
      bool inserted = nodes_.insert({j, subtree}).second;
      assert(inserted); (void)inserted;
    }
    // Fill index for each child
    typedef typename BayesTree<CLIQUE>::sharedClique sharedClique;
    for(const sharedClique& child: subtree->children) {
      fillNodesIndex(child); }
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  void BayesTree<CLIQUE>::insertRoot(const sharedClique& subtree) {
    roots_.push_back(subtree); // Add to roots
    fillNodesIndex(subtree); // Populate nodes index
  }

  /* ************************************************************************* */
  // First finds clique marginal then marginalizes that
  /* ************************************************************************* */
  template<class CLIQUE>
  typename BayesTree<CLIQUE>::sharedConditional
    BayesTree<CLIQUE>::marginalFactor(Key j, const Eliminate& function) const
  {
    gttic(BayesTree_marginalFactor);

    // get clique containing Key j
    sharedClique clique = this->clique(j);

    // calculate or retrieve its marginal P(C) = P(F,S)
    FactorGraphType cliqueMarginal = clique->marginal2(function);

    // Now, marginalize out everything that is not variable j
    BayesNetType marginalBN =
        *cliqueMarginal.marginalMultifrontalBayesNet(Ordering{j}, function);

    // The Bayes net should contain only one conditional for variable j, so return it
    return marginalBN.front();
  }

  /* ************************************************************************* */
  // Find two cliques, their joint, then marginalizes
  /* ************************************************************************* */
  template<class CLIQUE>
  typename BayesTree<CLIQUE>::sharedFactorGraph
    BayesTree<CLIQUE>::joint(Key j1, Key j2, const Eliminate& function) const
  {
    gttic(BayesTree_joint);
    return std::make_shared<FactorGraphType>(*jointBayesNet(j1, j2, function));
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  typename BayesTree<CLIQUE>::sharedBayesNet
    BayesTree<CLIQUE>::jointBayesNet(Key j1, Key j2, const Eliminate& function) const
  {
    gttic(BayesTree_jointBayesNet);
    // get clique C1 and C2
    sharedClique C1 = (*this)[j1], C2 = (*this)[j2];

    gttic(Lowest_common_ancestor);
    // Find lowest common ancestor clique
    sharedClique B; {
      // Build two paths to the root
      FastList<sharedClique> path1, path2; {
        sharedClique p = C1;
        while(p) {
          path1.push_front(p);
          p = p->parent();
        }
      } {
        sharedClique p = C2;
        while(p) {
          path2.push_front(p);
          p = p->parent();
        }
      }
      // Find the path intersection
      typename FastList<sharedClique>::const_iterator p1 = path1.begin(), p2 = path2.begin();
      if(*p1 == *p2)
        B = *p1;
      while(p1 != path1.end() && p2 != path2.end() && *p1 == *p2) {
        B = *p1;
        ++p1;
        ++p2;
      }
    }
    gttoc(Lowest_common_ancestor);

    // Build joint on all involved variables
    FactorGraphType p_BC1C2;

    if(B)
    {
      // Compute marginal on lowest common ancestor clique
      gttic(LCA_marginal);
      FactorGraphType p_B = B->marginal2(function);
      gttoc(LCA_marginal);

      // Compute shortcuts of the requested cliques given the lowest common ancestor
      gttic(Clique_shortcuts);
      BayesNetType p_C1_Bred = C1->shortcut(B, function);
      BayesNetType p_C2_Bred = C2->shortcut(B, function);
      gttoc(Clique_shortcuts);

      // Factor the shortcuts to be conditioned on the full root
      // Get the set of variables to eliminate, which is C1\B.
      gttic(Full_root_factoring);
      std::shared_ptr<typename EliminationTraitsType::BayesTreeType> p_C1_B; {
        KeyVector C1_minus_B; {
          KeySet C1_minus_B_set(C1->conditional()->beginParents(), C1->conditional()->endParents());
          for(const Key j: *B->conditional()) {
            C1_minus_B_set.erase(j); }
          C1_minus_B.assign(C1_minus_B_set.begin(), C1_minus_B_set.end());
        }
        // Factor into C1\B | B.
        p_C1_B =
            FactorGraphType(p_C1_Bred)
                .eliminatePartialMultifrontal(Ordering(C1_minus_B), function)
                .first;
      }
      std::shared_ptr<typename EliminationTraitsType::BayesTreeType> p_C2_B; {
        KeyVector C2_minus_B; {
          KeySet C2_minus_B_set(C2->conditional()->beginParents(), C2->conditional()->endParents());
          for(const Key j: *B->conditional()) {
            C2_minus_B_set.erase(j); }
          C2_minus_B.assign(C2_minus_B_set.begin(), C2_minus_B_set.end());
        }
        // Factor into C2\B | B.
        p_C2_B =
            FactorGraphType(p_C2_Bred)
                .eliminatePartialMultifrontal(Ordering(C2_minus_B), function)
                .first;
      }
      gttoc(Full_root_factoring);

      gttic(Variable_joint);
      p_BC1C2 += p_B;
      p_BC1C2 += *p_C1_B;
      p_BC1C2 += *p_C2_B;
      if(C1 != B)
        p_BC1C2 += C1->conditional();
      if(C2 != B)
        p_BC1C2 += C2->conditional();
      gttoc(Variable_joint);
    }
    else
    {
      // The nodes have no common ancestor, they're in different trees, so they're joint is just the
      // product of their marginals.
      gttic(Disjoint_marginals);
      p_BC1C2 += C1->marginal2(function);
      p_BC1C2 += C2->marginal2(function);
      gttoc(Disjoint_marginals);
    }

    // now, marginalize out everything that is not variable j1 or j2
    return p_BC1C2.marginalMultifrontalBayesNet(Ordering{j1, j2}, function);
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  void BayesTree<CLIQUE>::clear() {
    // Remove all nodes and clear the root pointer
    nodes_.clear();
    roots_.clear();
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  void BayesTree<CLIQUE>::deleteCachedShortcuts() {
    for(const sharedClique& root: roots_) {
      root->deleteCachedShortcuts();
    }
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  void BayesTree<CLIQUE>::removeClique(sharedClique clique)
  {
    if (clique->isRoot()) {
      typename Roots::iterator root = std::find(roots_.begin(), roots_.end(), clique);
      if(root != roots_.end())
        roots_.erase(root);
    } else { // detach clique from parent
      sharedClique parent = clique->parent_.lock();
      typename Roots::iterator child = std::find(parent->children.begin(), parent->children.end(), clique);
      assert(child != parent->children.end());
      parent->children.erase(child);
    }

    // orphan my children
    for(sharedClique child: clique->children)
      child->parent_ = typename Clique::weak_ptr();

    for(Key j: clique->conditional()->frontals()) {
      nodes_.unsafe_erase(j);
    }
  }

  /* ************************************************************************* */
  template <class CLIQUE>
  void BayesTree<CLIQUE>::removePath(sharedClique clique, BayesNetType* bn,
                                     Cliques* orphans) {
    // base case is nullptr, if so we do nothing and return empties above
    if (clique) {
      // remove the clique from orphans in case it has been added earlier
      orphans->remove(clique);

      // remove me
      this->removeClique(clique);

      // remove path above me
      this->removePath(typename Clique::shared_ptr(clique->parent_.lock()), bn,
                       orphans);

      // add children to list of orphans (splice also removed them from
      // clique->children_)
      orphans->insert(orphans->begin(), clique->children.begin(),
                      clique->children.end());
      clique->children.clear();

      bn->push_back(clique->conditional_);
    }
  }

  /* *************************************************************************
   */
  template <class CLIQUE>
  void BayesTree<CLIQUE>::removeTop(const KeyVector& keys, BayesNetType* bn,
                                    Cliques* orphans) {
    gttic(removetop);
    // process each key of the new factor
    for (const Key& j : keys) {
      // get the clique
      // TODO(frank): Nodes will be searched again in removeClique
      typename Nodes::const_iterator node = nodes_.find(j);
      if (node != nodes_.end()) {
        // remove path from clique to root
        this->removePath(node->second, bn, orphans);
      }
    }

    // Delete cachedShortcuts for each orphan subtree
    // TODO(frank): Consider Improving
    for (sharedClique& orphan : *orphans) orphan->deleteCachedShortcuts();
  }

  /* ************************************************************************* */
  template<class CLIQUE>
  typename BayesTree<CLIQUE>::Cliques BayesTree<CLIQUE>::removeSubtree(
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
      for(const sharedClique& child: (*clique)->children) {
        cliques.push_back(child); }

      // Delete cached shortcuts
      (*clique)->deleteCachedShortcutsNonRecursive();

      // Remove this node from the nodes index
      for(Key j: (*clique)->conditional()->frontals()) {
        nodes_.unsafe_erase(j); }

      // Erase the parent and children pointers
      (*clique)->parent_.reset();
      (*clique)->children.clear();
    }

    return cliques;
  }

}
/// namespace gtsam
