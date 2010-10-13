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
#include <boost/foreach.hpp>
//#include <boost/serialization/map.hpp>
//#include <boost/serialization/list.hpp>
//#include <boost/serialization/vector.hpp>
//#include <boost/serialization/shared_ptr.hpp>
#include <boost/pool/pool_alloc.hpp>

#include <gtsam/base/types.h>
#include <gtsam/base/Testable.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/graph.h>

namespace gtsam {

	/**
	 * A factor graph is a bipartite graph with factor nodes connected to variable nodes.
	 * In this class, however, only factor nodes are kept around.
	 * 
	 * Templated on the type of factors and values structure.
	 */
	template<class Factor>
	class FactorGraph: public Testable<FactorGraph<Factor> > {
	public:
	  typedef Factor factor_type;
	  typedef boost::shared_ptr<FactorGraph<Factor> > shared_ptr;
		typedef typename boost::shared_ptr<Factor> sharedFactor;
		typedef typename std::vector<sharedFactor>::iterator iterator;
		typedef typename std::vector<sharedFactor>::const_iterator const_iterator;

	protected:

    /** Collection of factors */
		std::vector<sharedFactor> factors_;

//		/**
//		 * Return an ordering in first argument, potentially using a set of
//		 * keys that need to appear last, and potentially restricting scope
//		 */
//		void getOrdering(Ordering& ordering, const std::set<Index>& lastKeys,
//				boost::optional<const std::set<Index>&> scope = boost::none) const;

	public:

		/** ------------------ Creating Factor Graphs ---------------------------- */

		/** Default constructor */
		FactorGraph() {}

		/** convert from Bayes net */
		template<class Conditional>
		FactorGraph(const BayesNet<Conditional>& bayesNet);

		/** convert from a derived type */
		template<class DerivedFactor>
		FactorGraph(const FactorGraph<DerivedFactor>& factorGraph);

		/** Add a factor */
		template<class DerivedFactor>
		void push_back(const boost::shared_ptr<DerivedFactor>& factor);

		/** push back many factors */
		void push_back(const FactorGraph<Factor>& factors);

		/** ------------------ Querying Factor Graphs ---------------------------- */

		/** print out graph */
		void print(const std::string& s = "FactorGraph") const;

		/** Check equality */
		bool equals(const FactorGraph& fg, double tol = 1e-9) const;

		/** const cast to the underlying vector of factors */
		operator const std::vector<sharedFactor>&() const { return factors_; }

		/** STL begin and end, so we can use BOOST_FOREACH */
		inline const_iterator begin() const { return factors_.begin();}
		inline const_iterator end()   const { return factors_.end();  }

		/** Get a specific factor by index */
		inline sharedFactor operator[](size_t i) const { assert(i<factors_.size()); return factors_[i]; }

    /** Get the first factor */
    inline sharedFactor front() const { return factors_.front(); }

		/** Get the last factor */
		inline sharedFactor back() const { return factors_.back(); }

		/** return the number of factors and NULLS */
		inline size_t size() const { return factors_.size();}

		/** return the number valid factors */
		size_t nrFactors() const;

//		/** return keys in some random order */
//		Ordering keys() const;

//		/**
//		 * Compute colamd ordering, including I/O, constrained ordering,
//		 * and shared pointer version.
//		 */
//		Ordering getOrdering() const;
//		boost::shared_ptr<Ordering> getOrdering_() const;
//		Ordering getOrdering(const std::set<Index>& scope) const;
//		Ordering getConstrainedOrdering(const std::set<Index>& lastKeys) const;

		/**
		 * find the minimum spanning tree using boost graph library
		 */
		template<class Key, class Factor2>
		PredecessorMap<Key> findMinimumSpanningTree() const;

		/**
		 * Split the graph into two parts: one corresponds to the given spanning tree,
		 * and the other corresponds to the rest of the factors
		 */
		template<class Key, class Factor2>
		void split(const PredecessorMap<Key>& tree, FactorGraph<Factor>& Ab1, FactorGraph<Factor>& Ab2) const;

//		/**
//		 * find the minimum spanning tree using DSF
//		 */
//		std::pair<FactorGraph<Factor> , FactorGraph<Factor> >
// SL-NEEDED?				splitMinimumSpanningTree() const;

//		/**
//		 * Check consistency of the index map, useful for debugging
//		 */
//		void checkGraphConsistency() const;

		/** ----------------- Modifying Factor Graphs ---------------------------- */

		/** STL begin and end, so we can use BOOST_FOREACH */
		inline       iterator begin()       { return factors_.begin();}
		inline       iterator end()         { return factors_.end();  }

		/**
		 * Reserve space for the specified number of factors if you know in
		 * advance how many there will be (works like std::vector::reserve).
		 */
		void reserve(size_t size) { factors_.reserve(size); }

		/** delete factor without re-arranging indexes by inserting a NULL pointer */
		inline void remove(size_t i) { factors_[i].reset();}

		/** replace a factor by index */
		void replace(size_t index, sharedFactor factor);

//    /**
//     * Find all the factors that involve the given node and remove them
//     * from the factor graph
//     * @param key the key for the given node
//     */
//    std::vector<sharedFactor> findAndRemoveFactors(Index key);
//
//		/** remove singleton variables and the related factors */
//		std::pair<FactorGraph<Factor>, std::set<Index> > removeSingletons();

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(factors_);
		}
	}; // FactorGraph

	/**
   * static function that combines two factor graphs
   * @param const &fg1 Linear factor graph
   * @param const &fg2 Linear factor graph
   * @return a new combined factor graph
   */
	template<class FactorGraph>
	FactorGraph combine(const FactorGraph& fg1, const FactorGraph& fg2);

//  /**
//   * Extract and combine all the factors that involve a given node
//   * Put this here as not all Factors have a combine constructor
//   * @param key the key for the given node
//   * @return the combined linear factor
//   */
//	template<class Factor> boost::shared_ptr<Factor>
//		removeAndCombineFactors(FactorGraph<Factor>& factorGraph, const Index& key);


	/**
	 * These functions are defined here because they are templated on an
	 * additional parameter.  Putting them in the -inl.h file would mean these
	 * would have to be explicitly instantiated for any possible derived factor
	 * type.
	 */

  /* ************************************************************************* */
  template<class Factor>
  template<class DerivedFactor>
  FactorGraph<Factor>::FactorGraph(const FactorGraph<DerivedFactor>& factorGraph) {
    factors_.reserve(factorGraph.size());
    BOOST_FOREACH(const typename DerivedFactor::shared_ptr& factor, factorGraph) {
      if(factor)
        this->push_back(sharedFactor(new Factor(*factor)));
      else
        this->push_back(sharedFactor());
    }
  }

  /* ************************************************************************* */
  template<class Factor>
  template<class Conditional>
  FactorGraph<Factor>::FactorGraph(const BayesNet<Conditional>& bayesNet) {
    factors_.reserve(bayesNet.size());
    BOOST_FOREACH(const typename Conditional::shared_ptr& cond, bayesNet) {
      this->push_back(sharedFactor(new Factor(*cond)));
    }
  }

  /* ************************************************************************* */
  template<class Factor>
  template<class DerivedFactor>
  inline void FactorGraph<Factor>::push_back(const boost::shared_ptr<DerivedFactor>& factor) {
#ifndef NDEBUG
    factors_.push_back(boost::dynamic_pointer_cast<Factor>(factor)); // add the actual factor
#else
    factors_.push_back(boost::static_pointer_cast<Factor>(factor));
#endif
  }


} // namespace gtsam

