/**
 * @file    FactorGraph.h
 * @brief   Factor Graph Base Class
 * @author  Carlos Nieto
 * @author  Christian Potthast
 * @author  Michael Kaess
 */

// \callgraph

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include "Testable.h"
#include "BayesNet.h"
#include "graph.h"
#include "Key.h"
#include "SymbolMap.h"

namespace gtsam {

	class Ordering;

	/**
	 * A factor graph is a bipartite graph with factor nodes connected to variable nodes.
	 * In this class, however, only factor nodes are kept around.
	 * 
	 * Templated on the type of factors and configuration.
	 */
	template<class Factor> class FactorGraph: public Testable<FactorGraph<Factor> > {
	public:
		typedef typename boost::shared_ptr<Factor> sharedFactor;
		typedef typename std::vector<sharedFactor>::iterator iterator;
		typedef typename std::vector<sharedFactor>::const_iterator const_iterator;

	protected:

    /** Collection of factors */
		std::vector<sharedFactor> factors_;

		/** For each variable a list of factor indices connected to it  */
		typedef SymbolMap<std::list<size_t> > Indices;
		Indices indices_;

		/** Associate factor index with the variables connected to the factor */
		void associateFactor(size_t index, const sharedFactor& factor);

		/**
		 * Return an ordering in first argument, potentially using a set of
		 * keys that need to appear last, and potentially restricting scope
		 */
		void getOrdering(Ordering& ordering, const std::set<Symbol>& lastKeys,
				boost::optional<const std::set<Symbol>&> scope = boost::none) const;

	public:

		/** ------------------ Creating Factor Graphs ---------------------------- */

		/** Default constructor */
		FactorGraph() {}

		/** convert from Bayes net */
		template<class Conditional>
		FactorGraph(const BayesNet<Conditional>& bayesNet);

		/** Add a factor */
		void push_back(sharedFactor factor);

		/** push back many factors */
		void push_back(const FactorGraph<Factor>& factors);

		/** ------------------ Querying Factor Graphs ---------------------------- */

		/** print out graph */
		void print(const std::string& s = "FactorGraph") const;

		/** Check equality */
		bool equals(const FactorGraph& fg, double tol = 1e-9) const;

		/** STL begin and end, so we can use BOOST_FOREACH */
		inline const_iterator begin() const { return factors_.begin();}
		inline const_iterator end()   const { return factors_.end();  }

		/** Get a specific factor by index */
		inline sharedFactor operator[](size_t i) const {return factors_[i];}

		/** return the number of factors and NULLS */
		inline size_t size() const { return factors_.size();}

		/** return the number valid factors */
		size_t nrFactors() const;

		/** return keys in some random order */
		Ordering keys() const;

		/** return the number of the keys */
		inline size_t nrKeys() const {return indices_.size(); };

		/** Check whether a factor with this variable exists */
		bool involves(const Symbol& key) const {
			return !(indices_.find(key)==indices_.end());
		}

    /**
     * Return indices for all factors that involve the given node
     * @param key the key for the given node
     */
    std::list<size_t> factors(const Symbol& key) const;

		/**
		 * Compute colamd ordering, including I/O, constrained ordering,
		 * and shared pointer version.
		 */
		Ordering getOrdering() const;
		boost::shared_ptr<Ordering> getOrdering_() const;
		Ordering getOrdering(const std::set<Symbol>& scope) const;
		Ordering getConstrainedOrdering(const std::set<Symbol>& lastKeys) const;

		/**
		 * find the minimum spanning tree using boost graph library
		 */
		template<class Key, class Factor2> PredecessorMap<Key>
				findMinimumSpanningTree() const;

		/**
		 * Split the graph into two parts: one corresponds to the given spanning tree,
		 * and the other corresponds to the rest of the factors
		 */
		template<class Key, class Factor2> void split(const PredecessorMap<Key>& tree,
				FactorGraph<Factor>& Ab1, FactorGraph<Factor>& Ab2) const;

		/**
		 * find the minimum spanning tree using DSF
		 */
		std::pair<FactorGraph<Factor> , FactorGraph<Factor> >
				splitMinimumSpanningTree() const;

		/**
		 * Check consistency of the index map, useful for debugging
		 */
		void checkGraphConsistency() const;

		/** ----------------- Modifying Factor Graphs ---------------------------- */

		/** STL begin and end, so we can use BOOST_FOREACH */
		inline       iterator begin()       { return factors_.begin();}
		inline       iterator end()         { return factors_.end();  }

		/** delete factor without re-arranging indexes by inserting a NULL pointer */
		inline void remove(size_t i) { factors_[i].reset();}

		/** replace a factor by index */
		void replace(size_t index, sharedFactor factor);

    /**
     * Find all the factors that involve the given node and remove them
     * from the factor graph
     * @param key the key for the given node
     */
    std::vector<sharedFactor> findAndRemoveFactors(const Symbol& key);

		/** remove singleton variables and the related factors */
		std::pair<FactorGraph<Factor>, std::set<Symbol> > removeSingletons();

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(factors_);
			ar & BOOST_SERIALIZATION_NVP(indices_);
		}
	}; // FactorGraph

	/**
   * static function that combines two factor graphs
   * @param const &fg1 Linear factor graph
   * @param const &fg2 Linear factor graph
   * @return a new combined factor graph
   */
	template<class Factor>
	FactorGraph<Factor> combine(const FactorGraph<Factor>& fg1, const FactorGraph<Factor>& fg2);

  /**
   * Extract and combine all the factors that involve a given node
   * Put this here as not all Factors have a combine constructor
   * @param key the key for the given node
   * @return the combined linear factor
   */
	template<class Factor> boost::shared_ptr<Factor>
		removeAndCombineFactors(FactorGraph<Factor>& factorGraph, const Symbol& key);

} // namespace gtsam

