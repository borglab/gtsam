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

#include <gtsam/inference/ClusterTree.h>

namespace gtsam {

  // Forward declarations
  template<class BAYESNET, class GRAPH> class EliminationTree;

  /**
   * A JunctionTree is a cluster tree, a set of variable clusters with factors, arranged in a tree,
   * with the additional property that it represents the clique tree associated with a Bayes Net.
   *
   * In GTSAM a junction tree is an intermediate data structure in multifrontal variable
   * elimination.  Each node is a cluster of factors, along with a clique of variables that are
   * eliminated all at once. In detail, every node k represents a clique (maximal fully connected
   * subset) of an associated chordal graph, such as a chordal Bayes net resulting from elimination.
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
  class JunctionTree : public EliminatableClusterTree<BAYESTREE, GRAPH> {

  public:

    typedef JunctionTree<BAYESTREE, GRAPH> This; ///< This class
    typedef boost::shared_ptr<This> shared_ptr; ///< Shared pointer to this class
    typedef EliminatableClusterTree<BAYESTREE, GRAPH> Base; ///< Our base class

  protected:

    /// @name Standard Constructors
    /// @{

    /** Build the junction tree from an elimination tree. */
    template<class ETREE>
      static This FromEliminationTree(const ETREE& eliminationTree) { return This(eliminationTree); }

    /** Build the junction tree from an elimination tree. */
    template<class ETREE_BAYESNET, class ETREE_GRAPH>
    JunctionTree(const EliminationTree<ETREE_BAYESNET, ETREE_GRAPH>& eliminationTree);

    /// @}

  private:

    // Private default constructor (used in static construction methods)
    JunctionTree() {}

  };

}
