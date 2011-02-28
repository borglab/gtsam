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

#include <gtsam/base/FastList.h>
#include <gtsam/base/FastSet.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/FactorGraph.h>

class EliminationTreeTester; // for unit tests, see testEliminationTree

namespace gtsam {

/**
 * An elimination tree is a data structure used intermediately during
 * elimination, and it can be used to save work between multiple eliminations.
 *
 * When a variable is eliminated, a new factor is created, which will involve
 * other variables. Of those, the first one that will be eliminated next, will
 * need that factor. In the elimination tree, that first variable is exactly
 * the parent of each variable. This yields a tree in general, and not a chain
 * because of the implicit sparse structure of the resulting Bayes net.
 *
 * This structures is examined even more closely in a JunctionTree, which
 * additionally identifies cliques in the chordal Bayes net.
 */
template<class FACTOR>
class EliminationTree: public Testable<EliminationTree<FACTOR> > {

public:

  typedef typename FACTOR::shared_ptr sharedFactor;
  typedef boost::shared_ptr<EliminationTree<FACTOR> > shared_ptr;
  typedef gtsam::BayesNet<typename FACTOR::Conditional> BayesNet;

private:

  typedef FastList<sharedFactor> Factors;
  typedef FastList<shared_ptr> SubTrees;
  typedef std::vector<typename FACTOR::Conditional::shared_ptr> Conditionals;

  Index key_; /** index associated with root */
  Factors factors_; /** factors associated with root */
  SubTrees subTrees_; /** sub-trees */

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
  static std::vector<Index> ComputeParents(const VariableIndex& structure);

  /**
   * Recursive routine that eliminates the factors arranged in an elimination tree
   */
  sharedFactor eliminate_(Conditionals& conditionals) const;

  /**
   * Special optimized eliminate for symbolic factors.  Will not compile if
   * called in a non-IndexFactor EliminationTree.
   */
  FastSet<Index> eliminateSymbolic_(Conditionals& conditionals) const;

  // Allow access to constructor and add methods for testing purposes
  friend class ::EliminationTreeTester;

public:

  /**
   * Named constructor to build the elimination tree of a factor graph using
   * pre-computed column structure.
   */
  template<class DERIVEDFACTOR>
  static shared_ptr Create(const FactorGraph<DERIVEDFACTOR>& factorGraph, const VariableIndex& structure);

  /** Named constructor to build the elimination tree of a factor graph */
  template<class DERIVEDFACTOR>
  static shared_ptr Create(const FactorGraph<DERIVEDFACTOR>& factorGraph);

  /** Print the tree to cout */
  void print(const std::string& name = "EliminationTree: ") const;

  /** Test whether the tree is equal to another */
  bool equals(const EliminationTree& other, double tol = 1e-9) const;

  /** Eliminate the factors to a Bayes Net */
  typename BayesNet::shared_ptr eliminate() const;
};

}
