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

class EliminationTreeTester; // for unit tests, see testEliminationTree

namespace gtsam {

/**
 * An elimination tree is a tree of factors
 */
template<class FACTORGRAPH>
class EliminationTree: public Testable<EliminationTree<FACTORGRAPH> > {

public:

  typedef boost::shared_ptr<typename FACTORGRAPH::factor_type> sharedFactor;
  typedef boost::shared_ptr<EliminationTree<FACTORGRAPH> > shared_ptr;
  typedef FACTORGRAPH FactorGraph;

private:

  typedef std::list<sharedFactor, boost::fast_pool_allocator<sharedFactor> > Factors;
  typedef std::list<shared_ptr, boost::fast_pool_allocator<shared_ptr> > SubTrees;

  Index key_; /** index associated with root */
  Factors factors_; /** factors associated with root */
  SubTrees subTrees_; /** sub-trees */

  typedef std::pair<typename FACTORGRAPH::bayesnet_type, typename FACTORGRAPH::sharedFactor> EliminationResult;

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
  static shared_ptr Create(const FACTORGRAPH& factorGraph);

  /** Print the tree to cout */
  void print(const std::string& name = "EliminationTree: ") const;

  /** Test whether the tree is equal to another */
  bool equals(const EliminationTree& other, double tol = 1e-9) const;

  /** Eliminate the factors to a Bayes Net */
  typename FACTORGRAPH::bayesnet_type::shared_ptr eliminate() const;
};

}
