/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file JunctionTree.h
 * @date Feb 4, 2010
 * @author Kai Ni
 * @author Frank Dellaert
 * @author Richard Roberts
 * @brief The junction tree
 */

#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/inference/Key.h>

namespace gtsam {

  /**
   * A ClusterTree, i.e., a set of variable clusters with factors, arranged in a tree, with
   * the additional property that it represents the clique tree associated with a Bayes net.
   *
   * In GTSAM a junction tree is an intermediate data structure in multifrontal
   * variable elimination.  Each node is a cluster of factors, along with a
   * clique of variables that are eliminated all at once. In detail, every node k represents
   * a clique (maximal fully connected subset) of an associated chordal graph, such as a
   * chordal Bayes net resulting from elimination.
   *
   * The difference with the BayesTree is that a JunctionTree stores factors, whereas a
   * BayesTree stores conditionals, that are the product of eliminating the factors in the
   * corresponding JunctionTree cliques.
   *
   * The tree structure and elimination method are exactly analagous to the EliminationTree,
   * except that in the JunctionTree, at each node multiple variables are eliminated at a time.
   *
   * \addtogroup Multifrontal
   * \nosubgrouping
   */
  template<class BAYESTREE, class GRAPH>
  class JunctionTreeUnordered {

  public:

    typedef GRAPH FactorGraphType; ///< The factor graph type
    typedef typename GRAPH::FactorType FactorType; ///< The type of factors
    typedef JunctionTreeUnordered<BAYESTREE, GRAPH> This; ///< This class
    typedef boost::shared_ptr<This> shared_ptr; ///< Shared pointer to this class
    typedef boost::shared_ptr<FactorType> sharedFactor;  ///< Shared pointer to a factor
    typedef BAYESTREE BayesTreeType; ///< The BayesTree type produced by elimination
    typedef typename BayesTreeType::ConditionalType ConditionalType; ///< The type of conditionals
    typedef boost::shared_ptr<ConditionalType> sharedConditional; ///< Shared pointer to a conditional
    typedef typename FactorGraphType::Eliminate Eliminate; ///< Typedef for an eliminate subroutine

    struct Node {
      typedef std::vector<Key> Keys;
      typedef std::vector<sharedFactor> Factors;
      typedef std::vector<boost::shared_ptr<Node> > Children;

      Keys keys; ///< Frontal keys of this node
      Factors factors; ///< Factors associated with this node
      Children children; ///< sub-trees

      /** print this node */
      void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;
    };

    typedef boost::shared_ptr<Node> sharedNode; ///< Shared pointer to Node

  private:

    /** concept check */
    GTSAM_CONCEPT_TESTABLE_TYPE(FactorType);

    std::vector<sharedNode> roots_;
    std::vector<sharedFactor> remainingFactors_;

  protected:

    /// @name Standard Constructors
    /// @{

    /** Build the junction tree from an elimination tree. */
    template<class ETREE>
    static This FromEliminationTree(const ETREE& eliminationTree);
    
    /** Copy constructor - makes a deep copy of the tree structure, but only pointers to factors are
     *  copied, factors are not cloned. */
    JunctionTreeUnordered(const This& other) { *this = other; }

    /** Assignment operator - makes a deep copy of the tree structure, but only pointers to factors
     *  are copied, factors are not cloned. */
    This& operator=(const This& other);

    /// @}

  public:

    /// @name Standard Interface
    /// @{

    /** Eliminate the factors to a Bayes net and remaining factor graph
    * @param function The function to use to eliminate, see the namespace functions
    * in GaussianFactorGraph.h
    * @return The Bayes net and factor graph resulting from elimination
    */
    std::pair<boost::shared_ptr<BayesTreeType>, boost::shared_ptr<FactorGraphType> >
      eliminate(const Eliminate& function) const;

    /** Print the junction tree */
    void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

    /// @}

    /// @name Advanced Interface
    /// @{
    
    /** Return the set of roots (one for a tree, multiple for a forest) */
    const std::vector<sharedNode>& roots() const { return roots_; }

    /** Return the remaining factors that are not pulled into elimination */
    const std::vector<sharedFactor>& remainingFactors() const { return remainingFactors_; }

    /// @}

  private:

    // Private default constructor (used in static construction methods)
    JunctionTreeUnordered() {}

  };

}