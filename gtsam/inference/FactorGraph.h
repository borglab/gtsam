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

#include <boost/foreach.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/function.hpp>

#include <gtsam/base/Testable.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/graph.h>

namespace gtsam {

// Forward declarations
template<class CONDITIONAL, class CLIQUE> class BayesTree;

	/**
	 * A factor graph is a bipartite graph with factor nodes connected to variable nodes.
	 * In this class, however, only factor nodes are kept around.
	 * 
	 * Templated on the type of factors and values structure.
	 */
	template<class FACTOR>
	class FactorGraph {
	public:
	  typedef FACTOR FactorType;
	  typedef boost::shared_ptr<FactorGraph<FACTOR> > shared_ptr;
		typedef typename boost::shared_ptr<FACTOR> sharedFactor;
		typedef typename std::vector<sharedFactor>::iterator iterator;
		typedef typename std::vector<sharedFactor>::const_iterator const_iterator;

	  /** typedef for elimination result */
	  typedef std::pair<
				boost::shared_ptr<typename FACTOR::ConditionalType>,
				typename FACTOR::shared_ptr> EliminationResult;

	  /** typedef for an eliminate subroutine */
	  typedef boost::function<EliminationResult(const FactorGraph<FACTOR>&, size_t)> Eliminate;

	protected:

	  /** concept check */
	  GTSAM_CONCEPT_TESTABLE_TYPE(FACTOR)

    /** Collection of factors */
		std::vector<sharedFactor> factors_;

	public:

		/** ------------------ Creating Factor Graphs ---------------------------- */

		/** Default constructor */
		FactorGraph() {}

		/** convert from Bayes net */
		template<class CONDITIONAL>
		FactorGraph(const BayesNet<CONDITIONAL>& bayesNet);

    /** convert from Bayes tree */
		template<class CONDITIONAL, class CLIQUE>
    FactorGraph(const BayesTree<CONDITIONAL, CLIQUE>& bayesTree);

		/** convert from a derived type */
		template<class DERIVEDFACTOR>
		FactorGraph(const FactorGraph<DERIVEDFACTOR>& factors) { factors_.insert(end(), factors.begin(), factors.end()); }

		/** Add a factor */
		template<class DERIVEDFACTOR>
		void push_back(const boost::shared_ptr<DERIVEDFACTOR>& factor) { factors_.push_back(sharedFactor(factor)); }

		/** push back many factors */
		void push_back(const FactorGraph<FACTOR>& factors) { factors_.insert(end(), factors.begin(), factors.end()); }

		/** push back many factors with an iterator */
		template<typename ITERATOR>
		void push_back(ITERATOR firstFactor, ITERATOR lastFactor) { factors_.insert(end(), firstFactor, lastFactor); }

    /** push back many factors stored in a vector*/
    template<typename DERIVEDFACTOR>
    void push_back(const std::vector<boost::shared_ptr<DERIVEDFACTOR> >& factors);

		/** ------------------ Querying Factor Graphs ---------------------------- */

		/** print out graph */
		void print(const std::string& s = "FactorGraph") const;

		/** Check equality */
		bool equals(const FactorGraph<FACTOR>& fg, double tol = 1e-9) const;

		/** const cast to the underlying vector of factors */
		operator const std::vector<sharedFactor>&() const { return factors_; }

		/** STL begin and end, so we can use BOOST_FOREACH */
		const_iterator begin() const { return factors_.begin();}
		const_iterator end()   const { return factors_.end();  }

		/** Get a specific factor by index */
		const sharedFactor operator[](size_t i) const { assert(i<factors_.size()); return factors_[i]; }
		sharedFactor& operator[](size_t i) { assert(i<factors_.size()); return factors_[i]; }

    /** Get the first factor */
    sharedFactor front() const { return factors_.front(); }

		/** Get the last factor */
		sharedFactor back() const { return factors_.back(); }

		/** return the number of factors and NULLS */
		size_t size() const { return factors_.size();}

		/** return the number valid factors */
		size_t nrFactors() const;

		/** dynamic_cast the factor pointers down or up the class hierarchy */
		template<class RELATED>
		typename RELATED::shared_ptr dynamicCastFactors() const {
		  typename RELATED::shared_ptr ret(new RELATED);
		  ret->reserve(this->size());
		  BOOST_FOREACH(const sharedFactor& factor, *this) {
		    typename RELATED::FactorType::shared_ptr castedFactor(boost::dynamic_pointer_cast<typename RELATED::FactorType>(factor));
		    if(castedFactor)
		      ret->push_back(castedFactor);
		    else
		      throw std::invalid_argument("In FactorGraph<FACTOR>::dynamic_factor_cast(), dynamic_cast failed, meaning an invalid cast was requested.");
		  }
		  return ret;
		}

		/**
		 * dynamic_cast factor pointers if possible, otherwise convert with a
		 * constructor of the target type.
		 */
		template<class TARGET>
		typename TARGET::shared_ptr convertCastFactors() const {
      typename TARGET::shared_ptr ret(new TARGET);
      ret->reserve(this->size());
      BOOST_FOREACH(const sharedFactor& factor, *this) {
        typename TARGET::FactorType::shared_ptr castedFactor(boost::dynamic_pointer_cast<typename TARGET::FactorType>(factor));
        if(castedFactor)
          ret->push_back(castedFactor);
        else
          ret->push_back(typename TARGET::FactorType::shared_ptr(new typename TARGET::FactorType(*factor)));
      }
      return ret;
    }

		/** ----------------- Modifying Factor Graphs ---------------------------- */

		/** STL begin and end, so we can use BOOST_FOREACH */
		iterator begin()       { return factors_.begin();}
		iterator end()         { return factors_.end();  }

		/**
		 * Reserve space for the specified number of factors if you know in
		 * advance how many there will be (works like std::vector::reserve).
		 */

		void reserve(size_t size) { factors_.reserve(size); }

		/**
		 * resize the factor graph
		 */
    void resize(size_t size) { factors_.resize(size); }

		/** delete factor without re-arranging indexes by inserting a NULL pointer */
		inline void remove(size_t i) { factors_[i].reset();}

		/** replace a factor by index */
		void replace(size_t index, sharedFactor factor);

	private:

		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(factors_);
		}
	}; // FactorGraph

  /** Create a combined joint factor (new style for EliminationTree). */
	template<class DERIVED, class KEY>
	typename DERIVED::shared_ptr Combine(const FactorGraph<DERIVED>& factors,
			const FastMap<KEY, std::vector<KEY> >& variableSlots);

	/**
   * static function that combines two factor graphs
   * @param const &fg1 Linear factor graph
   * @param const &fg2 Linear factor graph
   * @return a new combined factor graph
   */
	template<class FACTORGRAPH>
	FACTORGRAPH combine(const FACTORGRAPH& fg1, const FACTORGRAPH& fg2);

	/*
	 * These functions are defined here because they are templated on an
	 * additional parameter.  Putting them in the -inl.h file would mean these
	 * would have to be explicitly instantiated for any possible derived factor
	 * type.
	 */

  /* ************************************************************************* */
  template<class FACTOR>
  template<class CONDITIONAL>
  FactorGraph<FACTOR>::FactorGraph(const BayesNet<CONDITIONAL>& bayesNet) {
    factors_.reserve(bayesNet.size());
    BOOST_FOREACH(const typename CONDITIONAL::shared_ptr& cond, bayesNet) {
      this->push_back(cond->toFactor());
    }
  }

  /* ************************************************************************* */
  template<class FACTOR>
  template<class DERIVEDFACTOR>
  void FactorGraph<FACTOR>::push_back(const std::vector<boost::shared_ptr<DERIVEDFACTOR> >& factors) {
    BOOST_FOREACH(const boost::shared_ptr<DERIVEDFACTOR>& factor, factors)
      this->push_back(factor);
  }

} // namespace gtsam

#include <gtsam/inference/FactorGraph-inl.h>
