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
#include <stdexcept>
#include <deque>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/lexical_cast.hpp>

#include <gtsam/base/types.h>
#include <gtsam/base/FastVector.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/BayesTreeCliqueBase.h>
#include <gtsam/inference/IndexConditional.h>
#include <gtsam/linear/VectorValues.h>

namespace gtsam {

  // Forward declaration of BayesTreeClique which is defined below BayesTree in this file
  template<class CONDITIONAL> struct BayesTreeClique;

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
	template<class CONDITIONAL, class CLIQUE=BayesTreeClique<CONDITIONAL> >
	class BayesTree {

	public:

	  typedef BayesTree<CONDITIONAL, CLIQUE> This;
	  typedef boost::shared_ptr<BayesTree<CONDITIONAL, CLIQUE> > shared_ptr;
		typedef boost::shared_ptr<CONDITIONAL> sharedConditional;
		typedef boost::shared_ptr<BayesNet<CONDITIONAL> > sharedBayesNet;
		typedef CONDITIONAL ConditionalType;
		typedef typename CONDITIONAL::FactorType FactorType;
	  typedef typename FactorGraph<FactorType>::Eliminate Eliminate;

	  typedef CLIQUE Clique; ///< The clique type, normally BayesTreeClique

		// typedef for shared pointers to cliques
		typedef boost::shared_ptr<Clique> sharedClique;

		// A convenience class for a list of shared cliques
		struct Cliques : public std::list<sharedClique> {
			void print(const std::string& s = "Cliques",
					const IndexFormatter& indexFormatter = DefaultIndexFormatter) const;
			bool equals(const Cliques& other, double tol = 1e-9) const;
		};

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

    /** Map from indices to Clique */
    typedef std::deque<sharedClique> Nodes;

	protected:

    /** Map from indices to Clique */
    Nodes nodes_;

    /** Root clique */
    sharedClique root_;

  public:

		/// @name Standard Constructors
		/// @{

		/** Create an empty Bayes Tree */
		BayesTree() {}

		/** Create a Bayes Tree from a Bayes Net (requires CONDITIONAL is IndexConditional *or* CONDITIONAL::Combine) */
		explicit BayesTree(const BayesNet<CONDITIONAL>& bayesNet);

		/** Copy constructor */
		BayesTree(const This& other);

		/** Assignment operator */
		This& operator=(const This& other);

		/// @}
		/// @name Advanced Constructors
		/// @{

		/**
		 * Create a Bayes Tree from a Bayes Net and some subtrees. The Bayes net corresponds to the
		 * new root clique and the subtrees are connected to the root clique.
		 */
		BayesTree(const BayesNet<CONDITIONAL>& bayesNet, std::list<BayesTree<CONDITIONAL,CLIQUE> > subtrees);

		/** Destructor */
		virtual ~BayesTree() {}

		/// @}
		/// @name Testable
		/// @{

		/** check equality */
		bool equals(const BayesTree<CONDITIONAL,CLIQUE>& other, double tol = 1e-9) const;

		/** print */
		void print(const std::string& s = "",
				const IndexFormatter& indexFormatter = DefaultIndexFormatter ) const;

		/// @}
		/// @name Standard Interface
		/// @{

		/**
		 * Find parent clique of a conditional.  It will look at all parents and
		 * return the one with the lowest index in the ordering.
		 */
		template<class CONTAINER>
		Index findParentClique(const CONTAINER& parents) const;

		/** number of cliques */
		inline size_t size() const {
			if(root_)
				return root_->treeSize();
			else
				return 0;
		}

    /** return nodes */
    const Nodes& nodes() const { return nodes_; }

		/** return root clique */
		const sharedClique& root() const { return root_;	}

		/** find the clique that contains the variable with Index j */
		sharedClique operator[](Index j) const {
			return nodes_.at(j);
		}

		/** Gather data on all cliques */
		CliqueData getCliqueData() const;

		/** return marginal on any variable */
		typename FactorType::shared_ptr marginalFactor(Index j, Eliminate function) const;

		/**
		 * return marginal on any variable, as a Bayes Net
		 * NOTE: this function calls marginal, and then eliminates it into a Bayes Net
		 * This is more expensive than the above function
		 */
		typename BayesNet<CONDITIONAL>::shared_ptr marginalBayesNet(Index j, Eliminate function) const;

		/** return joint on two variables */
		typename FactorGraph<FactorType>::shared_ptr joint(Index j1, Index j2, Eliminate function) const;

		/** return joint on two variables as a BayesNet */
		typename BayesNet<CONDITIONAL>::shared_ptr jointBayesNet(Index j1, Index j2, Eliminate function) const;

		/**
		 * Read only with side effects
		 */

		/** saves the Tree to a text file in GraphViz format */
		void saveGraph(const std::string& s, const IndexFormatter& indexFormatter = DefaultIndexFormatter ) const;

		/// @}
		/// @name Advanced Interface
		/// @{

		/** Access the root clique (non-const version) */
		sharedClique& root() { return root_; }

		/** Access the nodes (non-cost version) */
		Nodes& nodes() { return nodes_; }

		/** Remove all nodes */
		void clear();

		/** Clear all shortcut caches - use before timing on marginal calculation to avoid residual cache data */
		inline void deleteCachedShorcuts() { root_->deleteCachedShorcuts(); }

		/**
		 * Remove path from clique to root and return that path as factors
		 * plus a list of orphaned subtree roots. Used in removeTop below.
		 */
		void removePath(sharedClique clique, BayesNet<CONDITIONAL>& bn, Cliques& orphans);

		/**
		 * Given a list of indices, turn "contaminated" part of the tree back into a factor graph.
		 * Factors and orphans are added to the in/out arguments.
		 */
		template<class CONTAINER>
		void removeTop(const CONTAINER& indices, BayesNet<CONDITIONAL>& bn, Cliques& orphans);

		/**
		 * Hang a new subtree off of the existing tree.  This finds the appropriate
		 * parent clique for the subtree (which may be the root), and updates the
		 * nodes index with the new cliques in the subtree.  None of the frontal
		 * variables in the subtree may appear in the separators of the existing
		 * BayesTree.
		 */
		void insert(const sharedClique& subtree);

		/** Insert a new conditional
		 * This function only applies for Symbolic case with IndexCondtional,
		 * We make it static so that it won't be compiled in GaussianConditional case.
		 * */
		static void insert(BayesTree<CONDITIONAL,CLIQUE>& bayesTree, const sharedConditional& conditional);

		/**
		 * Insert a new clique corresponding to the given Bayes net.
		 * It is the caller's responsibility to decide whether the given Bayes net is a valid clique,
		 * i.e. all the variables (frontal and separator) are connected
		 */
		sharedClique insert(const sharedConditional& clique,
				std::list<sharedClique>& children, bool isRootClique = false);

		/**
		 * Create a clone of this object as a shared pointer
		 * Necessary for inheritance in matlab interface
		 */
		virtual shared_ptr clone() const {
			return shared_ptr(new This(*this));
		}

	protected:

		/** private helper method for saving the Tree to a text file in GraphViz format */
		void saveGraph(std::ostream &s, sharedClique clique, const IndexFormatter& indexFormatter,
				int parentnum = 0) const;

		/** Gather data on a single clique */
		void getCliqueData(CliqueData& stats, sharedClique clique) const;

		/** remove a clique: warning, can result in a forest */
		void removeClique(sharedClique clique);

		/** add a clique (top down) */
		sharedClique addClique(const sharedConditional& conditional, const sharedClique& parent_clique = sharedClique());

    /** add a clique (top down) */
    void addClique(const sharedClique& clique, const sharedClique& parent_clique = sharedClique());

		/** add a clique (bottom up) */
		sharedClique addClique(const sharedConditional& conditional, std::list<sharedClique>& child_cliques);

		/**
		 * Add a conditional to the front of a clique, i.e. a conditional whose
		 * parents are already in the clique or its separators.  This function does
		 * not check for this condition, it just updates the data structures.
		 */
		static void addToCliqueFront(BayesTree<CONDITIONAL,CLIQUE>& bayesTree,
		    const sharedConditional& conditional, const sharedClique& clique);

		/** Fill the nodes index for a subtree */
		void fillNodesIndex(const sharedClique& subtree);

    /** Helper function to build a non-symbolic tree (e.g. Gaussian) using a
     * symbolic tree, used in the BT(BN) constructor.
     */
		void recursiveTreeBuild(const boost::shared_ptr<BayesTreeClique<IndexConditional> >& symbolic,
		      const std::vector<boost::shared_ptr<CONDITIONAL> >& conditionals,
		      const typename BayesTree<CONDITIONAL,CLIQUE>::sharedClique& parent);

  private:

    /** deep copy to another tree */
		void cloneTo(This& newTree) const;

		/** deep copy to another tree */
		void cloneTo(This& newTree, const sharedClique& subtree, const sharedClique& parent) const;

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
    	ar & BOOST_SERIALIZATION_NVP(nodes_);
    	ar & BOOST_SERIALIZATION_NVP(root_);
    }

		/// @}

	}; // BayesTree


  /* ************************************************************************* */
  template<class CONDITIONAL, class CLIQUE>
  void _BayesTree_dim_adder(
      std::vector<size_t>& dims,
      const typename BayesTree<CONDITIONAL,CLIQUE>::sharedClique& clique) {

    if(clique) {
      // Add dims from this clique
      for(typename CONDITIONAL::const_iterator it = (*clique)->beginFrontals(); it != (*clique)->endFrontals(); ++it)
        dims[*it] = (*clique)->dim(it);

      // Traverse children
      typedef typename BayesTree<CONDITIONAL,CLIQUE>::sharedClique sharedClique;
      BOOST_FOREACH(const sharedClique& child, clique->children()) {
        _BayesTree_dim_adder<CONDITIONAL,CLIQUE>(dims, child);
      }
    }
  }

	/* ************************************************************************* */
	template<class CONDITIONAL,class CLIQUE>
	boost::shared_ptr<VectorValues> allocateVectorValues(const BayesTree<CONDITIONAL,CLIQUE>& bt) {
	  std::vector<size_t> dimensions(bt.nodes().size(), 0);
	  _BayesTree_dim_adder<CONDITIONAL,CLIQUE>(dimensions, bt.root());
	  return boost::shared_ptr<VectorValues>(new VectorValues(dimensions));
	}


  /* ************************************************************************* */
  /**
   * A Clique in the tree is an incomplete Bayes net: the variables
   * in the Bayes net are the frontal nodes, and the variables conditioned
   * on are the separator. We also have pointers up and down the tree.
   *
   * Since our Conditional class already handles multiple frontal variables,
   * this Clique contains exactly 1 conditional.
   *
   * This is the default clique type in a BayesTree, but some algorithms, like
   * iSAM2 (see ISAM2Clique), use a different clique type in order to store
   * extra data along with the clique.
   */
  template<class CONDITIONAL>
  struct BayesTreeClique : public BayesTreeCliqueBase<BayesTreeClique<CONDITIONAL>, CONDITIONAL> {
  public:
    typedef CONDITIONAL ConditionalType;
    typedef BayesTreeClique<CONDITIONAL> This;
    typedef BayesTreeCliqueBase<This, CONDITIONAL> Base;
    typedef boost::shared_ptr<This> shared_ptr;
    typedef boost::weak_ptr<This> weak_ptr;
    BayesTreeClique() {}
    BayesTreeClique(const typename ConditionalType::shared_ptr& conditional) : Base(conditional) {}
    BayesTreeClique(const std::pair<typename ConditionalType::shared_ptr, typename ConditionalType::FactorType::shared_ptr>& result) : Base(result) {}

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    }
  };

} /// namespace gtsam

#include <gtsam/inference/BayesTree-inl.h>
#include <gtsam/inference/BayesTreeCliqueBase-inl.h>
