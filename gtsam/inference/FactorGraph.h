/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    FactorGraph.h
 * @brief   Factor Graph Base Class
 * @author  Carlos Nieto
 * @author  Christian Potthast
 * @author  Michael Kaess
 */

// \callgraph

#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/inference/BayesNet.h>

#include <boost/serialization/nvp.hpp>
#include <boost/function.hpp>

namespace gtsam {

// Forward declarations
template<class CONDITIONAL, class CLIQUE> class BayesTree;

	/**
	 * A factor graph is a bipartite graph with factor nodes connected to variable nodes.
	 * In this class, however, only factor nodes are kept around.
	 * \nosubgrouping
	 */
	template<class FACTOR>
	class FactorGraph {

	public:

		typedef FACTOR FactorType;  ///< factor type
		typedef typename FACTOR::KeyType KeyType; ///< type of Keys we use to index factors with
		typedef boost::shared_ptr<FACTOR> sharedFactor;  ///< Shared pointer to a factor
		typedef boost::shared_ptr<typename FACTOR::ConditionalType> sharedConditional;  ///< Shared pointer to a conditional

		typedef FactorGraph<FACTOR> This;  ///< Typedef for this class
		typedef boost::shared_ptr<This> shared_ptr;  ///< Shared pointer for this class
		typedef typename std::vector<sharedFactor>::iterator iterator;
		typedef typename std::vector<sharedFactor>::const_iterator const_iterator;

		/** typedef for elimination result */
		typedef std::pair<sharedConditional, sharedFactor> EliminationResult;

		/** typedef for an eliminate subroutine */
		typedef boost::function<EliminationResult(const This&, size_t)> Eliminate;

	protected:

	  /** concept check, makes sure FACTOR defines print and equals */
	  GTSAM_CONCEPT_TESTABLE_TYPE(FACTOR)

    /** Collection of factors */
		std::vector<sharedFactor> factors_;

	public:

		/// @name Standard Constructor
		/// @{

		/** Default constructor */
		FactorGraph() {}

		/// @}
		/// @name Advanced Constructors
		/// @{

	  /**
	   * @brief Constructor from a Bayes net
	   * @param bayesNet the Bayes net to convert, type CONDITIONAL must yield compatible factor
	   * @return a factor graph with all the conditionals, as factors
	   */
		template<class CONDITIONAL>
		FactorGraph(const BayesNet<CONDITIONAL>& bayesNet);

    /** convert from Bayes tree */
		template<class CONDITIONAL, class CLIQUE>
    FactorGraph(const BayesTree<CONDITIONAL, CLIQUE>& bayesTree);

		/** convert from a derived type */
		template<class DERIVEDFACTOR>
		FactorGraph(const FactorGraph<DERIVEDFACTOR>& factors) {
			factors_.assign(factors.begin(), factors.end());
		}

		/// @}
		/// @name Adding Factors
		/// @{

		/**
		 * Reserve space for the specified number of factors if you know in
		 * advance how many there will be (works like std::vector::reserve).
		 */
		void reserve(size_t size) { factors_.reserve(size); }

		/** Add a factor */
		template<class DERIVEDFACTOR>
		void push_back(const boost::shared_ptr<DERIVEDFACTOR>& factor) {
			factors_.push_back(boost::shared_ptr<FACTOR>(factor));
		}

		/** push back many factors */
		void push_back(const This& factors) {
			factors_.insert(end(), factors.begin(), factors.end());
		}

		/** push back many factors with an iterator */
		template<typename ITERATOR>
		void push_back(ITERATOR firstFactor, ITERATOR lastFactor) {
			factors_.insert(end(), firstFactor, lastFactor);
		}

	  /**
	   * @brief Add a vector of derived factors
	   * @param factors to add
	   */
    template<typename DERIVEDFACTOR>
    void push_back(const std::vector<typename boost::shared_ptr<DERIVEDFACTOR> >& factors) {
			factors_.insert(end(), factors.begin(), factors.end());
		}

		/// @}
  	/// @name Testable
  	/// @{

		/** print out graph */
		void print(const std::string& s = "FactorGraph",
				const IndexFormatter& formatter = DefaultIndexFormatter) const;

		/** Check equality */
		bool equals(const This& fg, double tol = 1e-9) const;

		/// @}
		/// @name Standard Interface
		/// @{

		/** return the number of factors and NULLS */
		size_t size() const { return factors_.size();}

		/** Simple check for an empty graph - faster than comparing size() to zero */
		bool empty() const { return factors_.empty(); }

		/** const cast to the underlying vector of factors */
		operator const std::vector<sharedFactor>&() const { return factors_; }

		/** Get a specific factor by index */
		const sharedFactor at(size_t i) const { assert(i<factors_.size()); return factors_[i]; }
		sharedFactor& at(size_t i) { assert(i<factors_.size()); return factors_[i]; }
		const sharedFactor operator[](size_t i) const { return at(i); }
		sharedFactor& operator[](size_t i) { return at(i); }

		/** STL begin, so we can use BOOST_FOREACH */
		const_iterator begin() const { return factors_.begin();}

		/** STL end, so we can use BOOST_FOREACH */
		const_iterator end()   const { return factors_.end();  }

    /** Get the first factor */
    sharedFactor front() const { return factors_.front(); }

		/** Get the last factor */
		sharedFactor back() const { return factors_.back(); }

		/** Eliminate the first \c n frontal variables, returning the resulting
		 * conditional and remaining factor graph - this is very inefficient for
		 * eliminating all variables, to do that use EliminationTree or
		 * JunctionTree.
		 */
		std::pair<sharedConditional, FactorGraph<FactorType> > eliminateFrontals(size_t nFrontals, const Eliminate& eliminate) const;

		/// @}
		/// @name Modifying Factor Graphs (imperative, discouraged)
		/// @{

		/** non-const STL-style begin() */
		iterator begin()       { return factors_.begin();}

		/** non-const STL-style end() */
		iterator end()         { return factors_.end();  }

		/** resize the factor graph. TODO: effects? */
    void resize(size_t size) { factors_.resize(size); }

		/** delete factor without re-arranging indexes by inserting a NULL pointer */
		inline void remove(size_t i) { factors_[i].reset();}

		/** replace a factor by index */
		void replace(size_t index, sharedFactor factor);

		/// @}
		/// @name Advanced Interface
		/// @{

		/** return the number valid factors */
		size_t nrFactors() const;

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(factors_);
		}

		/// @}

	}; // FactorGraph

  /** Create a combined joint factor (new style for EliminationTree). */
	template<class DERIVEDFACTOR, class KEY>
	typename DERIVEDFACTOR::shared_ptr Combine(const FactorGraph<DERIVEDFACTOR>& factors,
			const FastMap<KEY, std::vector<KEY> >& variableSlots);

	/**
   * static function that combines two factor graphs
   * @param fg1 Linear factor graph
   * @param fg2 Linear factor graph
   * @return a new combined factor graph
   */
	template<class FACTORGRAPH>
	FACTORGRAPH combine(const FACTORGRAPH& fg1, const FACTORGRAPH& fg2);

} // namespace gtsam

#include <gtsam/inference/FactorGraph-inl.h>
