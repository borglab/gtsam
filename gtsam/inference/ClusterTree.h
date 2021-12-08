/**
 * @file EliminatableClusterTree.h
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
template <class GRAPH>
class ClusterTree {
 public:
  typedef GRAPH FactorGraphType;               ///< The factor graph type
  typedef ClusterTree<GRAPH> This;             ///< This class
  typedef boost::shared_ptr<This> shared_ptr;  ///< Shared pointer to this class

  typedef typename GRAPH::FactorType FactorType;       ///< The type of factors
  typedef boost::shared_ptr<FactorType> sharedFactor;  ///< Shared pointer to a factor

  /// A Cluster is just a collection of factors
  // TODO(frank): re-factor JunctionTree so we can make members private
  struct Cluster {
    typedef FastVector<boost::shared_ptr<Cluster> > Children;
    Children children;  ///< sub-trees

    typedef Ordering Keys;
    Keys orderedFrontalKeys;  ///< Frontal keys of this node

    FactorGraphType factors;  ///< Factors associated with this node

    int problemSize_;

    Cluster() : problemSize_(0) {}

    virtual ~Cluster() {}

    const Cluster& operator[](size_t i) const {
      return *(children[i]);
    }

    /// Construct from factors associated with a single key
    template <class CONTAINER>
    Cluster(Key key, const CONTAINER& factorsToAdd)
        : problemSize_(0) {
      addFactors(key, factorsToAdd);
    }

    /// Add factors associated with a single key
    template <class CONTAINER>
    void addFactors(Key key, const CONTAINER& factorsToAdd) {
      orderedFrontalKeys.push_back(key);
      factors.push_back(factorsToAdd);
      problemSize_ += factors.size();
    }

    /// Add a child cluster
    void addChild(const boost::shared_ptr<Cluster>& cluster) {
      children.push_back(cluster);
      problemSize_ = std::max(problemSize_, cluster->problemSize_);
    }

    size_t nrChildren() const {
      return children.size();
    }

    size_t nrFactors() const {
      return factors.size();
    }

    size_t nrFrontals() const {
      return orderedFrontalKeys.size();
    }

    int problemSize() const {
      return problemSize_;
    }

    /// print this node
    virtual void print(const std::string& s = "",
                       const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

    /// Return a vector with nrFrontal keys for each child
    std::vector<size_t> nrFrontalsOfChildren() const;

    /// Merge in given cluster
    void merge(const boost::shared_ptr<Cluster>& cluster);

    /// Merge all children for which bit is set into this node
    void mergeChildren(const std::vector<bool>& merge);
  };

  typedef boost::shared_ptr<Cluster> sharedCluster;  ///< Shared pointer to Cluster

  // Define Node=Cluster for compatibility with tree traversal functions
  typedef Cluster Node;
  typedef sharedCluster sharedNode;

  /** concept check */
  GTSAM_CONCEPT_TESTABLE_TYPE(FactorType)

 protected:
  FastVector<sharedNode> roots_;

  /// @name Standard Constructors
  /// @{

  /** Copy constructor - makes a deep copy of the tree structure, but only pointers to factors are
   *  copied, factors are not cloned. */
  ClusterTree(const This& other) {
    *this = other;
  }

  /// @}

 public:

  /// Default constructor
  ClusterTree() {}

  /// @name Testable
  /// @{

  /** Print the cluster tree */
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /// @}

  /// @name Advanced Interface
  /// @{

  void addRoot(const boost::shared_ptr<Cluster>& cluster) {
    roots_.push_back(cluster);
  }

  void addChildrenAsRoots(const boost::shared_ptr<Cluster>& cluster) {
    for (auto child : cluster->children)
      this->addRoot(child);
  }

  size_t nrRoots() const {
    return roots_.size();
  }

  /** Return the set of roots (one for a tree, multiple for a forest) */
  const FastVector<sharedNode>& roots() const {
    return roots_;
  }

  const Cluster& operator[](size_t i) const {
    return *(roots_[i]);
  }

  /// @}

 protected:
  /// @name Details

  /// Assignment operator - makes a deep copy of the tree structure, but only pointers to factors
  /// are copied, factors are not cloned.
  This& operator=(const This& other);

  /// @}
};

/**
 * A cluster-tree that eliminates to a Bayes tree.
 */
template <class BAYESTREE, class GRAPH>
class EliminatableClusterTree : public ClusterTree<GRAPH> {
 public:
  typedef BAYESTREE BayesTreeType;  ///< The BayesTree type produced by elimination
  typedef GRAPH FactorGraphType;    ///< The factor graph type
  typedef EliminatableClusterTree<BAYESTREE, GRAPH> This;  ///< This class
  typedef boost::shared_ptr<This> shared_ptr;              ///< Shared pointer to this class

  typedef typename BAYESTREE::ConditionalType ConditionalType;  ///< The type of conditionals
  typedef boost::shared_ptr<ConditionalType>
      sharedConditional;  ///< Shared pointer to a conditional

  typedef typename GRAPH::Eliminate Eliminate;         ///< Typedef for an eliminate subroutine
  typedef typename GRAPH::FactorType FactorType;       ///< The type of factors
  typedef boost::shared_ptr<FactorType> sharedFactor;  ///< Shared pointer to a factor

 protected:
  FastVector<sharedFactor> remainingFactors_;

  /// @name Standard Constructors
  /// @{

  /** Copy constructor - makes a deep copy of the tree structure, but only pointers to factors are
   *  copied, factors are not cloned. */
  EliminatableClusterTree(const This& other) : ClusterTree<GRAPH>(other) {
    *this = other;
  }

  /// @}

 public:
  /// @name Standard Interface
  /// @{

  /** Eliminate the factors to a Bayes tree and remaining factor graph
   * @param function The function to use to eliminate, see the namespace functions
   * in GaussianFactorGraph.h
   * @return The Bayes tree and factor graph resulting from elimination
   */
  std::pair<boost::shared_ptr<BayesTreeType>, boost::shared_ptr<FactorGraphType> > eliminate(
      const Eliminate& function) const;

  /// @}

  /// @name Advanced Interface
  /// @{

  /** Return the remaining factors that are not pulled into elimination */
  const FastVector<sharedFactor>& remainingFactors() const {
    return remainingFactors_;
  }

  /// @}

 protected:
  /// @name Details

  /// Assignment operator - makes a deep copy of the tree structure, but only pointers to factors
  /// are copied, factors are not cloned.
  This& operator=(const This& other);

  /// Default constructor to be used in derived classes
  EliminatableClusterTree() {}

  /// @}
};
}

#include <gtsam/inference/ClusterTree-inst.h>
