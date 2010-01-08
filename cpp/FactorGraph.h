/**
 * @file    FactorGraph.h
 * @brief   Factor Graph Base Class
 * @author  Carlos Nieto
 * @author  Christian Potthast
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
		typedef std::map<std::string, std::list<int> > Indices;
		Indices indices_;

	public:

		/** Default constructor */
		FactorGraph() {}

		/** convert from Bayes net */
		template<class Conditional>
		FactorGraph(const BayesNet<Conditional>& bayesNet);

		/** print out graph */
		void print(const std::string& s = "FactorGraph") const;

		/** Check equality */
		bool equals(const FactorGraph& fg, double tol = 1e-9) const;

		/** STL begin and end, so we can use BOOST_FOREACH */

		inline       iterator begin()       { return factors_.begin();}
		inline const_iterator begin() const { return factors_.begin();}
		inline       iterator end()         { return factors_.end();  }
		inline const_iterator end()   const { return factors_.end();  }

		/** Get a specific factor by index */
		inline sharedFactor operator[](size_t i) const {return factors_[i];}

		/** delete factor without re-arranging indexes by inserting a NULL pointer */
		inline void remove(size_t i) { factors_[i].reset();}

		/** return the number of factors and NULLS */
		inline size_t size() const { return factors_.size();}

		/** return the number valid factors */
		size_t nrFactors() const;

		/** Add a factor */
		void push_back(sharedFactor factor);

		/** push back many factors */
		void push_back(const FactorGraph<Factor>& factors);

		/** replace a factor by index */
		void replace(int index, sharedFactor factor);

		/** return keys in some random order */
		Ordering keys() const;

		/** Check whether a factor with this variable exists */
		bool involves(const std::string& key) const {
			return !(indices_.find(key)==indices_.end());
		}

		/**
		 * Compute colamd ordering
		 */
		Ordering getOrdering() const;

	  /**
	   * shared pointer versions for MATLAB
	   */
		boost::shared_ptr<Ordering>  getOrdering_() const;

    /**
     * Return indices for all factors that involve the given node
     * @param key the key for the given node
     */
    std::list<int> factors(const std::string& key) const;

    /**
     * find all the factors that involve the given node and remove them
     * from the factor graph
     * @param key the key for the given node
     */
		std::vector<sharedFactor> findAndRemoveFactors(const std::string& key);

		/**
		 * find the minimum spanning tree.
		 */
		std::map<std::string, std::string> findMinimumSpanningTree() const;

		/**
		 * Split the graph into two parts: one corresponds to the given spanning tre,
		 * and the other corresponds to the rest of the factors
		 */
		void split(std::map<std::string, std::string> tree,
				FactorGraph<Factor>& Ab1, FactorGraph<Factor>& Ab2) const;

	private:
		/** Associate factor index with the variables connected to the factor */
		void associateFactor(int index, sharedFactor factor);

		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(factors_);
			ar & BOOST_SERIALIZATION_NVP(indices_);
		}
	}; // FactorGraph

  /**
   * Extract and combine all the factors that involve a given node
   * Put this here as not all Factors have a combine constructor
   * @param key the key for the given node
   * @return the combined linear factor
   */
	template<class Factor> boost::shared_ptr<Factor>
		removeAndCombineFactors(FactorGraph<Factor>& factorGraph, const std::string& key);

	/**
   * static function that combines two factor graphs
   * @param const &fg1 Linear factor graph
   * @param const &fg2 Linear factor graph
   * @return a new combined factor graph
   */
	template<class Factor>
	FactorGraph<Factor> combine(const FactorGraph<Factor>& fg1, const FactorGraph<Factor>& fg2);

} // namespace gtsam

