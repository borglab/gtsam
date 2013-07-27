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

#include <vector>
#include <string>

#include <gtsam/base/types.h>
#include <gtsam/base/FastList.h>
#include <gtsam/base/FastMap.h>

namespace gtsam {

  // Forward declarations
  template<class FACTOR> class FactorGraphUnordered;

  /**
   * Bayes tree
   * @tparam CONDITIONAL The type of the conditional densities, i.e. the type of node in the underlying Bayes chain,
   * which could be a ConditionalProbabilityTable, a GaussianConditional, or a SymbolicConditional.
   * @tparam CLIQUE The type of the clique data structure, defaults to BayesTreeClique, normally do not change this
   * as it is only used when developing special versions of BayesTree, e.g. for ISAM2.
   *
   * \addtogroup Multifrontal
   * \nosubgrouping
   */
  template<class CLIQUE>
  class BayesTreeUnordered
  {
  protected:
    typedef BayesTreeUnordered<CLIQUE> This;
    typedef boost::shared_ptr<This> shared_ptr;

  public:
    typedef CLIQUE Clique; ///< The clique type, normally BayesTreeClique
    typedef boost::shared_ptr<Clique> sharedClique; ///< Shared pointer to a clique
    typedef Clique Node; ///< Synonym for Clique (TODO: remove)
    typedef sharedClique sharedNode; ///< Synonym for sharedClique (TODO: remove)
    typedef typename CLIQUE::ConditionalType ConditionalType;
    typedef boost::shared_ptr<ConditionalType> sharedConditional;
    typedef typename CLIQUE::BayesNetType BayesNetType;
    typedef boost::shared_ptr<BayesNetType> sharedBayesNet;
    typedef typename CLIQUE::FactorType FactorType;
    typedef boost::shared_ptr<FactorType> sharedFactor;
    typedef typename CLIQUE::FactorGraphType FactorGraphType;
    typedef boost::shared_ptr<FactorGraphType> sharedFactorGraph;
    typedef typename FactorGraphType::Eliminate Eliminate;
    typedef typename CLIQUE::EliminationTraits EliminationTraits;

    /** A convenience class for a list of shared cliques */
    typedef FastList<sharedClique> Cliques;

    /** clique statistics */
    struct CliqueStats {
      double avgConditionalSize;
      std::size_t maxConditionalSize;
      double avgSeparatorSize;
      std::size_t maxSeparatorSize;
      void print(const std::string& s = "") const ;
    };

    /** store all the sizes  */
    struct CliqueData {
      std::vector<std::size_t> conditionalSizes;
      std::vector<std::size_t> separatorSizes;
      CliqueStats getStats() const;
    };

    /** Map from keys to Clique */
    typedef FastMap<Key, sharedClique> Nodes;

  protected:

    /** Map from indices to Clique */
    Nodes nodes_;

    /** Root cliques */
    std::vector<sharedClique> roots_;

    /// @name Standard Constructors
    /// @{

    /** Create an empty Bayes Tree */
    BayesTreeUnordered() {}

    /** Copy constructor */
    BayesTreeUnordered(const This& other);

    /// @}

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

    /** return nodes */
    const Nodes& nodes() const { return nodes_; }

    /** Access node by variable */
    const sharedNode operator[](Key j) const { return nodes_.at(j); }

    /** return root cliques */
    const std::vector<sharedClique>& roots() const { return roots_;  }

    /** alternate syntax for matlab: find the clique that contains the variable with Index j */
    const sharedClique& clique(Key j) const {
      Nodes::const_iterator c = nodes_.find(j);
      if(c == nodes_.end())
        throw std::out_of_range("Requested the BayesTree clique for a key that is not in the BayesTree");
      else
        return c->second;
    }

    /** Gather data on all cliques */
    CliqueData getCliqueData() const;

    /** Collect number of cliques with cached separator marginals */
    size_t numCachedSeparatorMarginals() const;

    /** Return marginal on any variable.  Note that this actually returns a conditional, for which a
     *  solution may be directly obtained by calling .solve() on the returned object.
     *  Alternatively, it may be directly used as its factor base class.  For example, for Gaussian
     *  systems, this returns a GaussianConditional, which inherits from JacobianFactor and
     *  GaussianFactor. */
    sharedConditional marginalFactor(Key j, const Eliminate& function = EliminationTraits::DefaultEliminate) const;

    /**
     * return joint on two variables
     * Limitation: can only calculate joint if cliques are disjoint or one of them is root
     */
    sharedFactorGraph joint(Index j1, Index j2, const Eliminate& function = EliminationTraits::DefaultEliminate) const;

    /**
     * return joint on two variables as a BayesNet
     * Limitation: can only calculate joint if cliques are disjoint or one of them is root
     */
    sharedBayesNet jointBayesNet(Index j1, Index j2, const Eliminate& function = EliminationTraits::DefaultEliminate) const;

    /**
     * Read only with side effects
     */

    /** saves the Tree to a text file in GraphViz format */
    void saveGraph(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

    /// @}
    /// @name Advanced Interface
    /// @{
    
    /**
     * Find parent clique of a conditional.  It will look at all parents and
     * return the one with the lowest index in the ordering.
     */
    template<class CONTAINER>
    Index findParentClique(const CONTAINER& parents) const;

    /** Remove all nodes */
    void clear();

    /** Clear all shortcut caches - use before timing on marginal calculation to avoid residual cache data */
    void deleteCachedShortcuts();

    /**
     * Remove path from clique to root and return that path as factors
     * plus a list of orphaned subtree roots. Used in removeTop below.
     */
    void removePath(sharedClique clique, BayesNetType& bn, Cliques& orphans);

    /**
     * Given a list of indices, turn "contaminated" part of the tree back into a factor graph.
     * Factors and orphans are added to the in/out arguments.
     */
    void removeTop(const std::vector<Key>& keys, BayesNetType& bn, Cliques& orphans);

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
    void addFactorsToGraph(FactorGraphUnordered<FactorType>& graph) const;

  protected:

    /** private helper method for saving the Tree to a text file in GraphViz format */
    void saveGraph(std::ostream &s, sharedClique clique, const KeyFormatter& keyFormatter,
        int parentnum = 0) const;

    /** Gather data on a single clique */
    void getCliqueData(CliqueData& stats, sharedClique clique) const;

    /** remove a clique: warning, can result in a forest */
    void removeClique(sharedClique clique);

    /** add a clique (bottom up) */
    sharedClique addClique(const sharedConditional& conditional, std::list<sharedClique>& child_cliques);

    /** Fill the nodes index for a subtree */
    void fillNodesIndex(const sharedClique& subtree);

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_NVP(nodes_);
      ar & BOOST_SERIALIZATION_NVP(root_);
    }

    /// @}

  }; // BayesTree

} /// namespace gtsam
