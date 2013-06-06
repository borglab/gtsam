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
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <gtsam/base/FastList.h>
#include <gtsam/base/Testable.h>
#include <gtsam/inference/Key.h>

class EliminationTreeUnorderedTester; // for unit tests, see testEliminationTree

namespace gtsam {

  class VariableIndexUnordered;

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
  class EliminationTreeUnordered {

  public:

    typedef GRAPH FactorGraphType; ///< The factor graph type
    typedef typename GRAPH::FactorType FactorType; ///< The type of factors
    typedef EliminationTreeUnordered<BAYESNET, GRAPH> This; ///< This class
    typedef boost::shared_ptr<This> shared_ptr; ///< Shared pointer to this class
    typedef typename boost::shared_ptr<FactorType> sharedFactor;  ///< Shared pointer to a factor
    typedef BAYESNET BayesNetType; ///< The BayesNet corresponding to FACTOR
    typedef typename BayesNetType::ConditionalType ConditionalType; ///< The type of conditionals
    typedef typename boost::shared_ptr<ConditionalType> sharedConditional; ///< Shared pointer to a conditional
    typedef boost::function<std::pair<sharedConditional,sharedFactor>(std::vector<sharedFactor>, std::vector<Key>)>
      Eliminate; ///< Typedef for an eliminate subroutine

    struct Node {
      typedef FastList<sharedFactor> Factors;
      typedef FastList<boost::shared_ptr<Node> > Children;

      Key key; ///< key associated with root
      Factors factors; ///< factors associated with root
      Children children; ///< sub-trees

      sharedFactor eliminate(const boost::shared_ptr<BayesNetType>& output,
        const Eliminate& function, const std::vector<sharedFactor>& childrenFactors) const;
    };

    typedef boost::shared_ptr<Node> sharedNode; ///< Shared pointer to Node

  private:

    /** concept check */
    GTSAM_CONCEPT_TESTABLE_TYPE(FactorType);

    FastList<sharedNode> roots_;
    std::vector<sharedFactor> remainingFactors_;

  public:

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
    EliminationTreeUnordered(const FactorGraphType& factorGraph,
      const VariableIndexUnordered& structure, const std::vector<Key>& order);

    /** Build the elimination tree of a factor graph.  Note that this has to compute the column
    * structure as a VariableIndex, so if you already have this precomputed, use the other
    * constructor instead.
    * @param factorGraph The factor graph for which to build the elimination tree
    */
    EliminationTreeUnordered(const FactorGraphType& factorGraph, const std::vector<Key>& order);

    /** TODO: Copy constructor - makes a deep copy of the tree structure, but only pointers to factors are
     *  copied, factors are not cloned. */
    EliminationTreeUnordered(const This& other) { *this = other; }

    /** TODO: Assignment operator - makes a deep copy of the tree structure, but only pointers to factors are
     *  copied, factors are not cloned. */
    This& operator=(const This& other);

    /// @}
    /// @name Standard Interface
    /// @{

    /** Eliminate the factors to a Bayes net and remaining factor graph
    * @param function The function to use to eliminate, see the namespace functions
    * in GaussianFactorGraph.h
    * @return The Bayes net and factor graph resulting from elimination
    */
    std::pair<boost::shared_ptr<BayesNetType>, boost::shared_ptr<FactorGraphType> >
      eliminate(Eliminate function) const;

    /// @}
    /// @name Testable
    /// @{

    /** Print the tree to cout */
    void print(const std::string& name = "EliminationTree: ",
      const KeyFormatter& formatter = DefaultKeyFormatter) const;

    /** Test whether the tree is equal to another */
    bool equals(const This& other, double tol = 1e-9) const;

    /// @}
    /// @name Advanced Interface
    /// @{
    
    const FastList<sharedNode>& roots() const { return roots_; }

  protected:
    /// Protected default constructor
    EliminationTreeUnordered() {}

  private:
    /// Allow access to constructor and add methods for testing purposes
    friend class ::EliminationTreeUnorderedTester;
  };

}
