/**
 * @file    EliminationTree.h
 * @brief   
 * @author  Frank Dellaert
 * @created Oct 13, 2010
 */
#pragma once

#include <list>
#include <string>
#include <utility>
#include <boost/pool/pool_alloc.hpp>

#include <gtsam/inference/VariableIndex.h>
#include <gtsam/inference/BayesNet.h>

class EliminationTreeTester; // for unit tests, see testEliminationTree

namespace gtsam {

/**
 * An elimination tree is a data structure used intermediately during
 * elimination, and it can be used to save work between multiple eliminations.
 */
template<class FACTOR>
class EliminationTree: public Testable<EliminationTree<FACTOR> > {

public:

  typedef typename FACTOR::shared_ptr sharedFactor;
  typedef boost::shared_ptr<EliminationTree<FACTOR> > shared_ptr;
  typedef gtsam::BayesNet<typename FACTOR::Conditional> BayesNet;

private:

  typedef std::list<sharedFactor, boost::fast_pool_allocator<sharedFactor> > Factors;
  typedef std::list<shared_ptr, boost::fast_pool_allocator<shared_ptr> > SubTrees;

  Index key_; /** index associated with root */
  Factors factors_; /** factors associated with root */
  SubTrees subTrees_; /** sub-trees */

  typedef std::pair<BayesNet, sharedFactor> EliminationResult;

  /** default constructor, private, as you should use Create below */
  EliminationTree(Index key = 0) : key_(key) {}

  /** add a factor, for Create use only */
  void add(const sharedFactor& factor) { factors_.push_back(factor); }

  /** add a subtree, for Create use only */
  void add(const shared_ptr& child) { subTrees_.push_back(child); }

  /**
   * Static internal function to build a vector of parent pointers using the
   * algorithm of Gilbert et al., 2001, BIT.
   */
  static std::vector<Index> ComputeParents(const VariableIndex<>& structure);

  /**
   * Recursive routine that eliminates the factors arranged in an elimination tree
   */
  EliminationResult eliminate_() const;

  // Allow access to constructor and add methods for testing purposes
  friend class ::EliminationTreeTester;

public:

  /** Named constructor to build the elimination tree of a factor graph */
  template<class FACTORGRAPH>
  static shared_ptr Create(const FACTORGRAPH& factorGraph);

  /** Print the tree to cout */
  void print(const std::string& name = "EliminationTree: ") const;

  /** Test whether the tree is equal to another */
  bool equals(const EliminationTree& other, double tol = 1e-9) const;

  /** Eliminate the factors to a Bayes Net */
  typename BayesNet::shared_ptr eliminate() const;
};

}
