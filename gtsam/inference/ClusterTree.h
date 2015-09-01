/**
 * @file ClusterTree.h
 * @date Oct 8, 2013
 * @author Kai Ni
 * @author Richard Roberts
 * @author Frank Dellaert
 * @brief Collects factorgraph fragments defined on variable clusters, arranged in a tree
 */

#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/base/FastVector.h>
#include <gtsam/inference/Ordering.h>

namespace gtsam {

/**
 * A cluster-tree is associated with a factor graph and is defined as in Koller-Friedman:
 * each node k represents a subset \f$ C_k \sub X \f$, and the tree is family preserving, in that
 * each factor \f$ f_i \f$ is associated with a single cluster and \f$ scope(f_i) \sub C_k \f$.
 * \nosubgrouping
 */
template<class BAYESTREE, class GRAPH>
class ClusterTree {
public:
  typedef GRAPH FactorGraphType; ///< The factor graph type
  typedef typename GRAPH::FactorType FactorType; ///< The type of factors
  typedef ClusterTree<BAYESTREE, GRAPH> This; ///< This class
  typedef boost::shared_ptr<This> shared_ptr; ///< Shared pointer to this class
  typedef boost::shared_ptr<FactorType> sharedFactor; ///< Shared pointer to a factor
  typedef BAYESTREE BayesTreeType; ///< The BayesTree type produced by elimination
  typedef typename BayesTreeType::ConditionalType ConditionalType; ///< The type of conditionals
  typedef boost::shared_ptr<ConditionalType> sharedConditional; ///< Shared pointer to a conditional
  typedef typename FactorGraphType::Eliminate Eliminate; ///< Typedef for an eliminate subroutine

  struct Cluster {
    typedef Ordering Keys;
    typedef FastVector<sharedFactor> Factors;
    typedef FastVector<boost::shared_ptr<Cluster> > Children;

    Cluster() {
    }
    Cluster(Key key, const Factors& factors) :
        factors(factors) {
      orderedFrontalKeys.push_back(key);
    }

    Keys orderedFrontalKeys; ///< Frontal keys of this node
    Factors factors; ///< Factors associated with this node
    Children children; ///< sub-trees
    int problemSize_;

    int problemSize() const {
      return problemSize_;
    }

    /// print this node
    void print(const std::string& s = "", const KeyFormatter& keyFormatter =
        DefaultKeyFormatter) const;

    /// Merge all children for which bit is set into this node
    void mergeChildren(const std::vector<bool>& merge);
  };

  typedef boost::shared_ptr<Cluster> sharedCluster; ///< Shared pointer to Cluster
  typedef Cluster Node; ///< Define Node=Cluster for compatibility with tree traversal functions
  typedef sharedCluster sharedNode; ///< Define Node=Cluster for compatibility with tree traversal functions

  /** concept check */
  GTSAM_CONCEPT_TESTABLE_TYPE(FactorType);

protected:
  FastVector<sharedNode> roots_;
  FastVector<sharedFactor> remainingFactors_;

  /// @name Standard Constructors
  /// @{

  /** Copy constructor - makes a deep copy of the tree structure, but only pointers to factors are
   *  copied, factors are not cloned. */
  ClusterTree(const This& other) {*this = other;}

  /// @}

public:
  /// @name Testable
  /// @{

  /** Print the cluster tree */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /// @}

  /// @name Standard Interface
  /// @{

  /** Eliminate the factors to a Bayes tree and remaining factor graph
   * @param function The function to use to eliminate, see the namespace functions
   * in GaussianFactorGraph.h
   * @return The Bayes tree and factor graph resulting from elimination
   */
  std::pair<boost::shared_ptr<BayesTreeType>, boost::shared_ptr<FactorGraphType> >
  eliminate(const Eliminate& function) const;

  /// @}

  /// @name Advanced Interface
  /// @{

  /** Return the set of roots (one for a tree, multiple for a forest) */
  const FastVector<sharedNode>& roots() const {return roots_;}

  /** Return the remaining factors that are not pulled into elimination */
  const FastVector<sharedFactor>& remainingFactors() const {return remainingFactors_;}

  /// @}

protected:
  /// @name Details

  /// Assignment operator - makes a deep copy of the tree structure, but only pointers to factors
  /// are copied, factors are not cloned.
  This& operator=(const This& other);

  /// Default constructor to be used in derived classes
  ClusterTree() {}

  /// @}

};

}

