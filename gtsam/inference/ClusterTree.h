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

    /** print this node */
    void print(const std::string& s = "", const KeyFormatter& keyFormatter =
        DefaultKeyFormatter) const;

    void mergeChildren(const std::vector<bool>& merge) {
      gttic(merge_children);
      size_t nrChildren = children.size();

      // Count how many keys, factors and children we'll end up with
      size_t nrKeys = this->orderedFrontalKeys.size();
      size_t nrFactors = this->factors.size();
      size_t nrNewChildren = 0;
      // Loop over children
      for (size_t i = 0; i < nrChildren; ++i) {
        if (merge[i]) {
          // Get a reference to the i, adjusting the index to account for children
          // previously merged and removed from the i list.
          sharedNode child = this->children[i];
          nrKeys += child->orderedFrontalKeys.size();
          nrFactors += child->factors.size();
          nrNewChildren += child->children.size();
        } else {
          nrNewChildren += 1; // we keep the child
        }
      }

      // now reserve space, and really merge
      this->orderedFrontalKeys.reserve(nrKeys);
      this->factors.reserve(nrFactors);
      typename Node::Children newChildren;
      newChildren.reserve(nrNewChildren);
      // Loop over newChildren
      for (size_t i = 0; i < nrChildren; ++i) {
        // Check if we should merge the i^th child
        sharedNode child = this->children[i];
        if (merge[i]) {
          // Get a reference to the i, adjusting the index to account for newChildren
          // previously merged and removed from the i list.
          // Merge keys. For efficiency, we add keys in reverse order at end, calling reverse after..
          this->orderedFrontalKeys.insert(this->orderedFrontalKeys.end(),
              child->orderedFrontalKeys.rbegin(),
              child->orderedFrontalKeys.rend());
          // Merge keys, factors, and children.
          this->factors.insert(this->factors.end(), child->factors.begin(),
              child->factors.end());
          newChildren.insert(newChildren.end(), child->children.begin(),
              child->children.end());
          // Increment problem size
          problemSize_ = std::max(problemSize_, child->problemSize_);
          // Increment number of frontal variables
        } else {
          newChildren.push_back(child); // we keep the child
        }
      }
      this->children = newChildren;
      std::reverse(this->orderedFrontalKeys.begin(),
          this->orderedFrontalKeys.end());
    }
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

