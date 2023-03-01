/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BayesTree.h
 * @brief   Bayes Tree is a tree of cliques of a Bayes Chain
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

#include <memory>

#include <gtsam/inference/Key.h>
#include <gtsam/base/FastList.h>
#include <gtsam/base/ConcurrentMap.h>
#include <gtsam/base/FastVector.h>

#include <string>

namespace gtsam {

  // Forward declarations
  template<class FACTOR> class FactorGraph;
  template<class BAYESTREE, class GRAPH> class EliminatableClusterTree;

  /* ************************************************************************* */
  /** clique statistics */
  struct GTSAM_EXPORT BayesTreeCliqueStats {
    double avgConditionalSize;
    std::size_t maxConditionalSize;
    double avgSeparatorSize;
    std::size_t maxSeparatorSize;
    void print(const std::string& s = "") const ;
  };

  /** store all the sizes  */
  struct GTSAM_EXPORT BayesTreeCliqueData {
    FastVector<std::size_t> conditionalSizes;
    FastVector<std::size_t> separatorSizes;
    BayesTreeCliqueStats getStats() const;
  };

  /* ************************************************************************* */
  /**
   * Bayes tree
   * @tparam CONDITIONAL The type of the conditional densities, i.e. the type of node in the underlying Bayes chain,
   * which could be a ConditionalProbabilityTable, a GaussianConditional, or a SymbolicConditional.
   * @tparam CLIQUE The type of the clique data structure, defaults to BayesTreeClique, normally do not change this
   * as it is only used when developing special versions of BayesTree, e.g. for ISAM2.
   *
   * \ingroup Multifrontal
   * \nosubgrouping
   */
  template<class CLIQUE>
  class BayesTree
  {
  protected:
    typedef BayesTree<CLIQUE> This;
    typedef std::shared_ptr<This> shared_ptr;

  public:
    typedef CLIQUE Clique; ///< The clique type, normally BayesTreeClique
    typedef std::shared_ptr<Clique> sharedClique; ///< Shared pointer to a clique
    typedef Clique Node; ///< Synonym for Clique (TODO: remove)
    typedef sharedClique sharedNode; ///< Synonym for sharedClique (TODO: remove)
    typedef typename CLIQUE::ConditionalType ConditionalType;
    typedef std::shared_ptr<ConditionalType> sharedConditional;
    typedef typename CLIQUE::BayesNetType BayesNetType;
    typedef std::shared_ptr<BayesNetType> sharedBayesNet;
    typedef typename CLIQUE::FactorType FactorType;
    typedef std::shared_ptr<FactorType> sharedFactor;
    typedef typename CLIQUE::FactorGraphType FactorGraphType;
    typedef std::shared_ptr<FactorGraphType> sharedFactorGraph;
    typedef typename FactorGraphType::Eliminate Eliminate;
    typedef typename CLIQUE::EliminationTraitsType EliminationTraitsType;

    /** A convenience class for a list of shared cliques */
    typedef FastList<sharedClique> Cliques;

    /** Map from keys to Clique */
    typedef ConcurrentMap<Key, sharedClique> Nodes;

    /** Root cliques */
    typedef FastVector<sharedClique> Roots;

  protected:

    /** Map from indices to Clique */
    Nodes nodes_;

    /** Root cliques */
    Roots roots_;

    /// @name Standard Constructors
    /// @{

    /** Create an empty Bayes Tree */
    BayesTree() {}

    /** Copy constructor */
    BayesTree(const This& other);

    /// @}

    /** Destructor */
    ~BayesTree(); 

    /** Assignment operator */
    This& operator=(const This& other);

    /// @name Testable
    /// @{

    /** check equality */
    bool equals(const This& other, double tol = 1e-9) const;

  public:
    /** print */
    void print(const std::string& s = "",
        const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;
    /// @}

    /// @name Standard Interface
    /// @{

    /** number of cliques */
    size_t size() const;

    /** Check if there are any cliques in the tree */
    inline bool empty() const {
      return nodes_.empty();
    }

    /** Return nodes. Each node is a clique of variables obtained after elimination. */
    const Nodes& nodes() const { return nodes_; }

    /** Access node by variable */
    sharedClique operator[](Key j) const { return nodes_.at(j); }

    /** return root cliques */
    const Roots& roots() const { return roots_;  }

    /** alternate syntax for matlab: find the clique that contains the variable with Key j */
    const sharedClique& clique(Key j) const {
      typename Nodes::const_iterator c = nodes_.find(j);
      if(c == nodes_.end())
        throw std::out_of_range("Requested the BayesTree clique for a key that is not in the BayesTree");
      else
        return c->second;
    }

    /** Gather data on all cliques */
    BayesTreeCliqueData getCliqueData() const;

    /** Collect number of cliques with cached separator marginals */
    size_t numCachedSeparatorMarginals() const;

    /** Return marginal on any variable.  Note that this actually returns a conditional, for which a
     *  solution may be directly obtained by calling .solve() on the returned object.
     *  Alternatively, it may be directly used as its factor base class.  For example, for Gaussian
     *  systems, this returns a GaussianConditional, which inherits from JacobianFactor and
     *  GaussianFactor. */
    sharedConditional marginalFactor(Key j, const Eliminate& function = EliminationTraitsType::DefaultEliminate) const;

    /**
     * return joint on two variables
     * Limitation: can only calculate joint if cliques are disjoint or one of them is root
     */
    sharedFactorGraph joint(Key j1, Key j2, const Eliminate& function = EliminationTraitsType::DefaultEliminate) const;

    /**
     * return joint on two variables as a BayesNet
     * Limitation: can only calculate joint if cliques are disjoint or one of them is root
     */
    sharedBayesNet jointBayesNet(Key j1, Key j2, const Eliminate& function = EliminationTraitsType::DefaultEliminate) const;

   /// @name Graph Display
   /// @{

   /// Output to graphviz format, stream version.
   void dot(std::ostream& os, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

   /// Output to graphviz format string.
   std::string dot(
       const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

   /// output to file with graphviz format.
   void saveGraph(const std::string& filename,
                  const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;
  
    /// @}
    /// @name Advanced Interface
    /// @{

    /**
     * Find parent clique of a conditional.  It will look at all parents and
     * return the one with the lowest index in the ordering.
     */
    template<class CONTAINER>
    Key findParentClique(const CONTAINER& parents) const;

    /** Remove all nodes */
    void clear();

    /** Clear all shortcut caches - use before timing on marginal calculation to avoid residual cache data */
    void deleteCachedShortcuts();

    /**
     * Remove path from clique to root and return that path as factors
     * plus a list of orphaned subtree roots. Used in removeTop below.
     */
    void removePath(sharedClique clique, BayesNetType* bn, Cliques* orphans);

    /**
     * Given a list of indices, turn "contaminated" part of the tree back into a factor graph.
     * Factors and orphans are added to the in/out arguments.
     */
    void removeTop(const KeyVector& keys, BayesNetType* bn, Cliques* orphans);

    /**
     * Remove the requested subtree. */
    Cliques removeSubtree(const sharedClique& subtree);

    /** Insert a new subtree with known parent clique.  This function does not check that the
     *  specified parent is the correct parent.  This function updates all of the internal data
     *  structures associated with adding a subtree, such as populating the nodes index. */
    void insertRoot(const sharedClique& subtree);

    /** add a clique (top down) */
    void addClique(const sharedClique& clique, const sharedClique& parent_clique = sharedClique());

    /** Add all cliques in this BayesTree to the specified factor graph */
    void addFactorsToGraph(FactorGraph<FactorType>* graph) const;

  protected:

    /** private helper method for saving the Tree to a text file in GraphViz format */
    void dot(std::ostream &s, sharedClique clique, const KeyFormatter& keyFormatter,
             int parentnum = 0) const;

    /** Gather data on a single clique */
    void getCliqueData(sharedClique clique, BayesTreeCliqueData* stats) const;

    /** remove a clique: warning, can result in a forest */
    void removeClique(sharedClique clique);

    /** Fill the nodes index for a subtree */
    void fillNodesIndex(const sharedClique& subtree);

    // Friend JunctionTree because it directly fills roots and nodes index.
    template<class BAYESTREE, class GRAPH> friend class EliminatableClusterTree;

   private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_NVP(nodes_);
      ar & BOOST_SERIALIZATION_NVP(roots_);
    }
#endif

    /// @}

  }; // BayesTree

  /* ************************************************************************* */
  template <class CLIQUE, typename = void>
  class BayesTreeOrphanWrapper : public CLIQUE::ConditionalType {
   public:
    typedef CLIQUE CliqueType;
    typedef typename CLIQUE::ConditionalType Base;

    std::shared_ptr<CliqueType> clique;

    /**
     * @brief Construct a new Bayes Tree Orphan Wrapper object
     *
     * This object stores parent keys in our base type factor so that
     * eliminating those parent keys will pull this subtree into the
     * elimination.
     *
     * @param clique Orphan clique to add for further consideration in
     * elimination.
     */
    BayesTreeOrphanWrapper(const std::shared_ptr<CliqueType>& clique)
        : clique(clique) {
      this->keys_.assign(clique->conditional()->beginParents(),
                         clique->conditional()->endParents());
    }

    void print(
        const std::string& s = "",
        const KeyFormatter& formatter = DefaultKeyFormatter) const override {
      clique->print(s + "stored clique", formatter);
    }
  };

} /// namespace gtsam
