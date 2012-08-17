/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    EliminationTree.h
 * @author  Frank Dellaert
 * @date    Oct 13, 2010
 */
#pragma once

#include <utility>

#include <gtsam/base/FastSet.h>
#include <gtsam/base/Testable.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/inference/FactorGraph.h>

class EliminationTreeTester; // for unit tests, see testEliminationTree
namespace gtsam { template<class CONDITIONAL> class BayesNet; }

namespace gtsam {

/**
 * An elimination tree is a data structure used intermediately during
 * elimination.  In future versions it will be used to save work between
 * multiple eliminations.
 *
 * When a variable is eliminated, a new factor is created by combining that
 * variable's neighboring factors.  The new combined factor involves the combined
 * factors' involved variables.  When the lowest-ordered one of those variables
 * is eliminated, it consumes that combined factor.  In the elimination tree,
 * that lowest-ordered variable is the parent of the variable that was eliminated to
 * produce the combined factor.  This yields a tree in general, and not a chain
 * because of the implicit sparse structure of the resulting Bayes net.
 *
 * This structure is examined even more closely in a JunctionTree, which
 * additionally identifies cliques in the chordal Bayes net.
 * \nosubgrouping
 */
template<class FACTOR>
class EliminationTree {

public:

	typedef EliminationTree<FACTOR> This; ///< This class
  typedef boost::shared_ptr<This> shared_ptr; ///< Shared pointer to this class
	typedef typename boost::shared_ptr<FACTOR> sharedFactor;  ///< Shared pointer to a factor
  typedef gtsam::BayesNet<typename FACTOR::ConditionalType> BayesNet; ///< The BayesNet corresponding to FACTOR
  typedef FACTOR Factor;
  typedef typename FACTOR::KeyType KeyType;

  /** Typedef for an eliminate subroutine */
  typedef typename FactorGraph<FACTOR>::Eliminate Eliminate;

private:

  /** concept check */
  GTSAM_CONCEPT_TESTABLE_TYPE(FACTOR)

  typedef FastList<sharedFactor> Factors;
  typedef FastList<shared_ptr> SubTrees;
  typedef std::vector<typename FACTOR::ConditionalType::shared_ptr> Conditionals;

  Index key_; ///< index associated with root // FIXME: doesn't this require that "Index" is the type of keys in the generic factor?
  Factors factors_; ///< factors associated with root
  SubTrees subTrees_; ///< sub-trees

public:

	/// @name Standard Constructors
	/// @{

  /**
   * Named constructor to build the elimination tree of a factor graph using
   * pre-computed column structure.
   * @param factorGraph The factor graph for which to build the elimination tree
   * @param structure The set of factors involving each variable.  If this is not
   * precomputed, you can call the Create(const FactorGraph<DERIVEDFACTOR>&)
   * named constructor instead.
   * @return The elimination tree
   */
  template<class DERIVEDFACTOR>
  static shared_ptr Create(const FactorGraph<DERIVEDFACTOR>& factorGraph, const VariableIndex& structure);

  /** Named constructor to build the elimination tree of a factor graph.  Note
   * that this has to compute the column structure as a VariableIndex, so if you
   * already have this precomputed, use the Create(const FactorGraph<DERIVEDFACTOR>&, const VariableIndex&)
   * named constructor instead.
   * @param factorGraph The factor graph for which to build the elimination tree
   */
  template<class DERIVEDFACTOR>
  static shared_ptr Create(const FactorGraph<DERIVEDFACTOR>& factorGraph);

	/// @}
	/// @name Standard Interface
	/// @{

  /** Eliminate the factors to a Bayes Net
   * @param function The function to use to eliminate, see the namespace functions
   * in GaussianFactorGraph.h
   * @return The BayesNet resulting from elimination
   */
  typename BayesNet::shared_ptr eliminate(Eliminate function) const;

	/// @}
	/// @name Testable
	/// @{

  /** Print the tree to cout */
  void print(const std::string& name = "EliminationTree: ",
  		const IndexFormatter& formatter = DefaultIndexFormatter) const;

  /** Test whether the tree is equal to another */
  bool equals(const EliminationTree& other, double tol = 1e-9) const;

	/// @}

private:
	
  /** default constructor, private, as you should use Create below */
  EliminationTree(Index key = 0) : key_(key) {}

  /**
   * Static internal function to build a vector of parent pointers using the
   * algorithm of Gilbert et al., 2001, BIT.
   */
  static std::vector<Index> ComputeParents(const VariableIndex& structure);

  /** add a factor, for Create use only */
  void add(const sharedFactor& factor) { factors_.push_back(factor); }

  /** add a subtree, for Create use only */
  void add(const shared_ptr& child) { subTrees_.push_back(child); }

  /**
   * Recursive routine that eliminates the factors arranged in an elimination tree
   * @param Conditionals is a vector of shared pointers that will be modified in place
   */
  sharedFactor eliminate_(Eliminate function, Conditionals& conditionals) const;

  /// Allow access to constructor and add methods for testing purposes
  friend class ::EliminationTreeTester;

};


/**
 * An exception thrown when attempting to eliminate a disconnected factor
 * graph, which is not currently possible in gtsam.  If you need to work with
 * disconnected graphs, a workaround is to create zero-information factors to
 * bridge the disconnects.  To do this, create any factor type (e.g.
 * BetweenFactor or RangeFactor) with the noise model
 * <tt>sharedPrecision(dim, 0.0)</tt>, where \c dim is the appropriate
 * dimensionality for that factor.
 */
struct DisconnectedGraphException : public std::exception {
  DisconnectedGraphException() {}
  virtual ~DisconnectedGraphException() throw() {}

  /// Returns the string "Attempting to eliminate a disconnected graph - this is not currently possible in gtsam."
  virtual const char* what() const throw() {
    return
			"Attempting to eliminate a disconnected graph - this is not currently possible in\n"
			"GTSAM.  You may add \"empty\" BetweenFactor's to join disconnected graphs, these\n"
			"will affect the symbolic structure and solving complexity of the graph but not\n"
			"the solution.  To do this, create BetweenFactor's with zero-precision noise\n"
			"models, i.e. noiseModel::Isotropic::Precision(n, 0.0);\n"; }
};

}

#include <gtsam/inference/EliminationTree-inl.h>
