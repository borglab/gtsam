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
 * @brief: The junction tree
 */

#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/FactorGraphUnordered.h>
#include <gtsam/inference/EliminationTreeUnordered.h>

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
    typedef typename boost::shared_ptr<FactorType> sharedFactor;  ///< Shared pointer to a factor
    typedef BAYESTREE BayesTreeType; ///< The BayesTree type produced by elimination
    typedef typename BayesTreeType::ConditionalType ConditionalType; ///< The type of conditionals
    typedef typename boost::shared_ptr<ConditionalType> sharedConditional; ///< Shared pointer to a conditional
    typedef boost::function<std::pair<sharedConditional,sharedFactor>(std::vector<sharedFactor>, std::vector<Key>)>
      Eliminate; ///< Typedef for an eliminate subroutine

    struct Node {
      typedef std::vector<Key> Keys;
      typedef std::vector<sharedFactor> Factors;
      typedef std::vector<boost::shared_ptr<Node> > Children;

      Keys keys; ///< Frontal keys of this node
      Factors factors; ///< Factors associated with this node
      Children children; ///< sub-trees

      sharedFactor eliminate(const boost::shared_ptr<BayesTreeType>& output,
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

    /** Build the junction tree from an elimination tree and a symbolic Bayes net. */
    template<class ETREE, class SYMBOLIC_CONDITIONAL>
    JunctionTreeUnordered(
      const ETREE& eliminationTree,
      const FactorGraphUnordered<SYMBOLIC_CONDITIONAL>& symbolicBayesNet);

    /// @}

  };

}