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
* @author  Richard Roberts
* @date    Oct 13, 2010
*/
#pragma once

#include <utility>
#include <memory>

#include <gtsam/base/Testable.h>
#include <gtsam/base/FastVector.h>

class EliminationTreeTester; // for unit tests, see testEliminationTree

namespace gtsam {

  class VariableIndex;
  class Ordering;

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
  template<class BAYESNET, class GRAPH>
  class EliminationTree
  {
  protected:
    typedef EliminationTree<BAYESNET, GRAPH> This; ///< This class
    typedef std::shared_ptr<This> shared_ptr; ///< Shared pointer to this class

  public:
    typedef GRAPH FactorGraphType; ///< The factor graph type
    typedef typename GRAPH::FactorType FactorType; ///< The type of factors
    typedef typename std::shared_ptr<FactorType> sharedFactor;  ///< Shared pointer to a factor
    typedef BAYESNET BayesNetType; ///< The BayesNet corresponding to FACTOR
    typedef typename BayesNetType::ConditionalType ConditionalType; ///< The type of conditionals
    typedef typename std::shared_ptr<ConditionalType> sharedConditional; ///< Shared pointer to a conditional
    typedef typename GRAPH::Eliminate Eliminate;

    struct Node {
      typedef FastVector<sharedFactor> Factors;
      typedef FastVector<std::shared_ptr<Node> > Children;

      Key key; ///< key associated with root
      Factors factors; ///< factors associated with root
      Children children; ///< sub-trees

      sharedFactor eliminate(const std::shared_ptr<BayesNetType>& output,
        const Eliminate& function, const FastVector<sharedFactor>& childrenFactors) const;

      void print(const std::string& str, const KeyFormatter& keyFormatter) const;
    };

    typedef std::shared_ptr<Node> sharedNode; ///< Shared pointer to Node

  protected:
    /** concept check */
    GTSAM_CONCEPT_TESTABLE_TYPE(FactorType)

    FastVector<sharedNode> roots_;
    FastVector<sharedFactor> remainingFactors_;

    /// @name Standard Constructors
    /// @{

    /**
    * Build the elimination tree of a factor graph using pre-computed column structure.
    * @param factorGraph The factor graph for which to build the elimination tree
    * @param structure The set of factors involving each variable.  If this is not
    * precomputed, you can call the Create(const FactorGraph<DERIVEDFACTOR>&)
    * named constructor instead.
    * @return The elimination tree
    */
    EliminationTree(const FactorGraphType& factorGraph,
      const VariableIndex& structure, const Ordering& order);

    /** Build the elimination tree of a factor graph.  Note that this has to compute the column
    * structure as a VariableIndex, so if you already have this precomputed, use the other
    * constructor instead.
    * @param factorGraph The factor graph for which to build the elimination tree
    */
    EliminationTree(const FactorGraphType& factorGraph, const Ordering& order);

    /** Copy constructor - makes a deep copy of the tree structure, but only pointers to factors are
     *  copied, factors are not cloned. */
    EliminationTree(const This& other) { *this = other; }

    /** Assignment operator - makes a deep copy of the tree structure, but only pointers to factors
     *  are copied, factors are not cloned. */
    This& operator=(const This& other);

    /// @}

  public:
    ~EliminationTree(); 
    /// @name Standard Interface
    /// @{

    /** Eliminate the factors to a Bayes net and remaining factor graph
    * @param function The function to use to eliminate, see the namespace functions
    * in GaussianFactorGraph.h
    * @return The Bayes net and factor graph resulting from elimination
    */
    std::pair<std::shared_ptr<BayesNetType>, std::shared_ptr<FactorGraphType> >
      eliminate(Eliminate function) const;

    /// @}
    /// @name Testable
    /// @{

    /** Print the tree to cout */
    void print(const std::string& name = "EliminationTree: ",
      const KeyFormatter& formatter = DefaultKeyFormatter) const;

  protected:
    /** Test whether the tree is equal to another */
    bool equals(const This& other, double tol = 1e-9) const;

    /// @}

  public:
    /// @name Advanced Interface
    /// @{

    /** Return the set of roots (one for a tree, multiple for a forest) */
    const FastVector<sharedNode>& roots() const { return roots_; }

    /** Return the remaining factors that are not pulled into elimination */
    const FastVector<sharedFactor>& remainingFactors() const { return remainingFactors_; }

    /** Swap the data of this tree with another one, this operation is very fast. */
    void swap(This& other);

  protected:
    /// Protected default constructor
    EliminationTree() {}

  private:
    /// Allow access to constructor and add methods for testing purposes
    friend class ::EliminationTreeTester;
  };

}
