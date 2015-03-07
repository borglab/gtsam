/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ConcurrentFilteringAndSmoothing.cpp
 * @brief   Base classes for the 'filter' and 'smoother' portion of the Concurrent
 *          Filtering and Smoothing architecture, as well as an external synchronization
 *          function. These classes act as an interface only.
 * @author  Stephen Williams
 */

// \callgraph

#include <gtsam_unstable/nonlinear/ConcurrentFilteringAndSmoothing.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

namespace gtsam {

/* ************************************************************************* */
void synchronize(ConcurrentFilter& filter, ConcurrentSmoother& smoother) {

  NonlinearFactorGraph smootherFactors, filterSumarization, smootherSummarization;
  Values smootherValues, filterSeparatorValues, smootherSeparatorValues;

  // Call the pre-sync functions of the filter and smoother
  filter.presync();
  smoother.presync();

  // Get the updates from the smoother and apply them to the filter
  smoother.getSummarizedFactors(smootherSummarization, smootherSeparatorValues);
  filter.synchronize(smootherSummarization, smootherSeparatorValues);

  // Get the updates from the filter and apply them to the smoother
  filter.getSmootherFactors(smootherFactors, smootherValues);
  filter.getSummarizedFactors(filterSumarization, filterSeparatorValues);
  smoother.synchronize(smootherFactors, smootherValues, filterSumarization, filterSeparatorValues);

  // Call the post-sync functions of the filter and smoother
  filter.postsync();
  smoother.postsync();
}

namespace internal {

// TODO: Remove this and replace with the standard Elimination Tree once GTSAM 3.0 is released and supports forests
// A custom elimination tree that supports forests and partial elimination
class EliminationForest {
public:
  typedef boost::shared_ptr<EliminationForest> shared_ptr; ///< Shared pointer to this class

private:
  typedef FastList<GaussianFactor::shared_ptr> Factors;
  typedef FastList<shared_ptr> SubTrees;
  typedef std::vector<GaussianConditional::shared_ptr> Conditionals;

  Index key_; ///< index associated with root
  Factors factors_; ///< factors associated with root
  SubTrees subTrees_; ///< sub-trees

  /** default constructor, private, as you should use Create below */
  EliminationForest(Index key = 0) : key_(key) {}

  /**
   * Static internal function to build a vector of parent pointers using the
   * algorithm of Gilbert et al., 2001, BIT.
   */
  static std::vector<Index> ComputeParents(const VariableIndex& structure);

  /** add a factor, for Create use only */
  void add(const GaussianFactor::shared_ptr& factor) { factors_.push_back(factor); }

  /** add a subtree, for Create use only */
  void add(const shared_ptr& child) { subTrees_.push_back(child); }

public:

  /** return the key associated with this tree node */
  Index key() const { return key_; }

  /** return the const reference of children */
  const SubTrees& children() const { return subTrees_; }

  /** return the const reference to the factors */
  const Factors& factors() const { return factors_; }

  /** Create an elimination tree from a factor graph */
  static std::vector<shared_ptr> Create(const GaussianFactorGraph& factorGraph, const VariableIndex& structure);

  /** Recursive routine that eliminates the factors arranged in an elimination tree */
  GaussianFactor::shared_ptr eliminateRecursive(GaussianFactorGraph::Eliminate function);

  /** Recursive function that helps find the top of each tree */
  static void removeChildrenIndices(std::set<Index>& indices, const EliminationForest::shared_ptr& tree);
};

/* ************************************************************************* */
std::vector<Index> EliminationForest::ComputeParents(const VariableIndex& structure) {
  // Number of factors and variables
  const size_t m = structure.nFactors();
  const size_t n = structure.size();

  static const Index none = std::numeric_limits<Index>::max();

  // Allocate result parent vector and vector of last factor columns
  std::vector<Index> parents(n, none);
  std::vector<Index> prevCol(m, none);

  // for column j \in 1 to n do
  for (Index j = 0; j < n; j++) {
    // for row i \in Struct[A*j] do
    BOOST_FOREACH(const size_t i, structure[j]) {
      if (prevCol[i] != none) {
        Index k = prevCol[i];
        // find root r of the current tree that contains k
        Index r = k;
        while (parents[r] != none)
          r = parents[r];
        if (r != j) parents[r] = j;
      }
      prevCol[i] = j;
    }
  }

  return parents;
}

/* ************************************************************************* */
std::vector<EliminationForest::shared_ptr> EliminationForest::Create(const GaussianFactorGraph& factorGraph, const VariableIndex& structure) {
  // Compute the tree structure
  std::vector<Index> parents(ComputeParents(structure));

  // Number of variables
  const size_t n = structure.size();

  static const Index none = std::numeric_limits<Index>::max();

  // Create tree structure
  std::vector<shared_ptr> trees(n);
  for (Index k = 1; k <= n; k++) {
    Index j = n - k;  // Start at the last variable and loop down to 0
    trees[j].reset(new EliminationForest(j));  // Create a new node on this variable
    if (parents[j] != none)  // If this node has a parent, add it to the parent's children
      trees[parents[j]]->add(trees[j]);
  }

  // Hang factors in right places
  BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, factorGraph) {
    if(factor && factor->size() > 0) {
      Index j = *std::min_element(factor->begin(), factor->end());
      if(j < structure.size())
        trees[j]->add(factor);
    }
  }

  return trees;
}

/* ************************************************************************* */
GaussianFactor::shared_ptr EliminationForest::eliminateRecursive(GaussianFactorGraph::Eliminate function) {

  // Create the list of factors to be eliminated, initially empty, and reserve space
  GaussianFactorGraph factors;
  factors.reserve(this->factors_.size() + this->subTrees_.size());

  // Add all factors associated with the current node
  factors.push_back(this->factors_.begin(), this->factors_.end());

  // for all subtrees, eliminate into Bayes net and a separator factor, added to [factors]
  BOOST_FOREACH(const shared_ptr& child, subTrees_)
    factors.push_back(child->eliminateRecursive(function));

  // Combine all factors (from this node and from subtrees) into a joint factor
  GaussianFactorGraph::EliminationResult eliminated(function(factors, 1));

  return eliminated.second;
}

/* ************************************************************************* */
void EliminationForest::removeChildrenIndices(std::set<Index>& indices, const EliminationForest::shared_ptr& tree) {
  BOOST_FOREACH(const EliminationForest::shared_ptr& child, tree->children()) {
    indices.erase(child->key());
    removeChildrenIndices(indices, child);
  }
}

/* ************************************************************************* */
NonlinearFactorGraph calculateMarginalFactors(const NonlinearFactorGraph& graph, const Values& theta,
    const FastSet<Key>& remainingKeys, const GaussianFactorGraph::Eliminate& eliminateFunction) {

  // Calculate the set of RootKeys = AllKeys \Intersect RemainingKeys
  FastSet<Key> rootKeys;
  FastSet<Key> allKeys(graph.keys());
  std::set_intersection(allKeys.begin(), allKeys.end(), remainingKeys.begin(), remainingKeys.end(), std::inserter(rootKeys, rootKeys.end()));

  // Calculate the set of MarginalizeKeys = AllKeys - RemainingKeys
  FastSet<Key> marginalizeKeys;
  std::set_difference(allKeys.begin(), allKeys.end(), remainingKeys.begin(), remainingKeys.end(), std::inserter(marginalizeKeys, marginalizeKeys.end()));

  if(marginalizeKeys.size() == 0) {
    // There are no keys to marginalize. Simply return the input factors
    return graph;
  } else {
    // Create a subset of theta that only contains the required keys
    Values values;
    BOOST_FOREACH(Key key, allKeys) {
      values.insert(key, theta.at(key));
    }

    // Calculate the ordering: [Others Root]
    std::map<Key, int> constraints;
    BOOST_FOREACH(Key key, marginalizeKeys) {
      constraints[key] = 0;
    }
    BOOST_FOREACH(Key key, rootKeys) {
      constraints[key] = 1;
    }
    Ordering ordering = *graph.orderingCOLAMDConstrained(values, constraints);

    // Create the linear factor graph
    GaussianFactorGraph linearFactorGraph = *graph.linearize(values, ordering);

    // Construct a variable index
    VariableIndex variableIndex(linearFactorGraph, ordering.size());

    // Construct an elimination tree to perform sparse elimination
    std::vector<EliminationForest::shared_ptr> forest( EliminationForest::Create(linearFactorGraph, variableIndex) );

    // This is a forest. Only the top-most node/index of each tree needs to be eliminated; all of the children will be eliminated automatically
    // Find the subset of nodes/keys that must be eliminated
    std::set<Index> indicesToEliminate;
    BOOST_FOREACH(Key key, marginalizeKeys) {
      indicesToEliminate.insert(ordering.at(key));
    }
    BOOST_FOREACH(Key key, marginalizeKeys) {
      EliminationForest::removeChildrenIndices(indicesToEliminate, forest.at(ordering.at(key)));
    }

    // Eliminate each top-most key, returning a Gaussian Factor on some of the remaining variables
    // Convert the marginal factors into Linear Container Factors
    NonlinearFactorGraph marginalFactors;
    BOOST_FOREACH(Index index, indicesToEliminate) {
      GaussianFactor::shared_ptr gaussianFactor = forest.at(index)->eliminateRecursive(eliminateFunction);
      if(gaussianFactor->size() > 0) {
        LinearContainerFactor::shared_ptr marginalFactor(new LinearContainerFactor(gaussianFactor, ordering, values));
        marginalFactors.push_back(marginalFactor);
      }
    }

    // Also add any remaining factors that were unaffected by marginalizing out the selected variables.
    // These are part of the marginal on the remaining variables as well.
    BOOST_FOREACH(Key key, rootKeys) {
      BOOST_FOREACH(const GaussianFactor::shared_ptr& gaussianFactor, forest.at(ordering.at(key))->factors()) {
        LinearContainerFactor::shared_ptr marginalFactor(new LinearContainerFactor(gaussianFactor, ordering, values));
        marginalFactors.push_back(marginalFactor);
      }
    }

    return marginalFactors;
  }
}

/* ************************************************************************* */
}/// namespace internal

/* ************************************************************************* */
}/// namespace gtsam
