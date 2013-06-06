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

#include <boost/serialization/nvp.hpp>
#include <boost/assign/list_inserter.hpp>
#include <boost/make_shared.hpp>

#include <gtsam/base/Testable.h>
#include <gtsam/inference/Key.h>

namespace gtsam {

  /**
   * A factor graph is a bipartite graph with factor nodes connected to variable nodes.
   * In this class, however, only factor nodes are kept around.
   * \nosubgrouping
   */
  template<class FACTOR>
  class FactorGraphUnordered {

  public:

    typedef FactorGraphUnordered<FACTOR> This;
    typedef FACTOR FactorType;  ///< factor type
    typedef boost::shared_ptr<FACTOR> sharedFactor;  ///< Shared pointer to a factor
    typedef boost::shared_ptr<typename FACTOR::ConditionalType> sharedConditional;  ///< Shared pointer to a conditional

    typedef FactorGraphUnordered<FACTOR> This;  ///< Typedef for this class
    typedef boost::shared_ptr<This> shared_ptr;  ///< Shared pointer for this class
    typedef typename std::vector<sharedFactor>::iterator iterator;
    typedef typename std::vector<sharedFactor>::const_iterator const_iterator;

  protected:

    /** concept check, makes sure FACTOR defines print and equals */
    GTSAM_CONCEPT_TESTABLE_TYPE(FACTOR)

    /** Collection of factors */
    std::vector<sharedFactor> factors_;

  public:

    /// @name Standard Constructors
    /// @{

    /** Default constructor */
    FactorGraphUnordered() {}

    /** Constructor from iterator over factors */
    template<typename ITERATOR>
    FactorGraphUnordered(ITERATOR firstFactor, ITERATOR lastFactor) { push_back(firstFactor, lastFactor); }

    /// @}
    /// @name Advanced Constructors
    /// @{
    
    // TODO: are these needed?

    ///**
    // * @brief Constructor from a Bayes net
    // * @param bayesNet the Bayes net to convert, type CONDITIONAL must yield compatible factor
    // * @return a factor graph with all the conditionals, as factors
    // */
    //template<class CONDITIONAL>
    //FactorGraph(const BayesNet<CONDITIONAL>& bayesNet);

    ///** convert from Bayes tree */
    //template<class CONDITIONAL, class CLIQUE>
    //FactorGraph(const BayesTree<CONDITIONAL, CLIQUE>& bayesTree);

    ///** convert from a derived type */
    //template<class DERIVEDFACTOR>
    //FactorGraph(const FactorGraph<DERIVEDFACTOR>& factors) {
    //  factors_.assign(factors.begin(), factors.end());
    //}

    /// @}
    /// @name Adding Factors
    /// @{

    /**
     * Reserve space for the specified number of factors if you know in
     * advance how many there will be (works like std::vector::reserve).
     */
    void reserve(size_t size) { factors_.reserve(size); }

    // TODO: are these needed?

    /** Add a factor directly using a shared_ptr */
    template<class DERIVEDFACTOR>
    void push_back(const boost::shared_ptr<DERIVEDFACTOR>& factor) {
      factors_.push_back(boost::shared_ptr<FACTOR>(factor));
    }

    /** Add a factor, will be copy-constructed into a shared_ptr (use push_back to avoid the copy). */
    template<class DERIVEDFACTOR>
    void add(const DERIVEDFACTOR& factor) {
      factors_.push_back(boost::make_shared<DERIVEDFACTOR>(factor));
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
    
    /** += syntax for push_back, e.g. graph += f1, f2, f3 */
    boost::assign::list_inserter<boost::assign_detail::call_push_back<This>, sharedFactor>
      operator+=(const sharedFactor& factor)
    {
      return boost::assign::make_list_inserter(
        boost::assign_detail::call_push_back<This>(*this))(factor);
    }


    /**
     * @brief Add a vector of derived factors
     * @param factors to add
     */
    //template<typename DERIVEDFACTOR>
    //void push_back(const std::vector<typename boost::shared_ptr<DERIVEDFACTOR> >& factors) {
    //  factors_.insert(end(), factors.begin(), factors.end());
    //}

    /// @}
    /// @name Testable
    /// @{

    /** print out graph */
    void print(const std::string& s = "FactorGraph",
        const KeyFormatter& formatter = DefaultKeyFormatter) const;

    /** Check equality */
    bool equals(const This& fg, double tol = 1e-9) const;

    /// @}
    /// @name Standard Interface
    /// @{

    /** return the number of factors (including any null factors set by remove() ). */
    size_t size() const { return factors_.size(); }

    /** Check if the graph is empty (null factors set by remove() will cause this to return false). */
    bool empty() const { return factors_.empty(); }

    /** Get a specific factor by index (this checks array bounds and may throw an exception, as
     *  opposed to operator[] which does not).
     */
    const sharedFactor at(size_t i) const { return factors_.at(i); }

    /** Get a specific factor by index (this checks array bounds and may throw an exception, as
     *  opposed to operator[] which does not).
     */
    sharedFactor& at(size_t i) { return factors_.at(i); }

    /** Get a specific factor by index (this does not check array bounds, as opposed to at() which
     *  does).
     */
    const sharedFactor operator[](size_t i) const { return at(i); }

    /** Get a specific factor by index (this does not check array bounds, as opposed to at() which
     *  does).
     */
    sharedFactor& operator[](size_t i) { return at(i); }

    /** Iterator to beginning of factors. */
    const_iterator begin() const { return factors_.begin();}

    /** Iterator to end of factors. */
    const_iterator end()   const { return factors_.end();  }

    /** Get the first factor */
    sharedFactor front() const { return factors_.front(); }

    /** Get the last factor */
    sharedFactor back() const { return factors_.back(); }

    /// @}
    /// @name Modifying Factor Graphs (imperative, discouraged)
    /// @{

    /** non-const STL-style begin() */
    iterator begin()       { return factors_.begin();}

    /** non-const STL-style end() */
    iterator end()         { return factors_.end();  }

    /** Directly resize the number of factors in the graph. If the new size is less than the
     * original, factors at the end will be removed.  If the new size is larger than the original,
     * null factors will be appended.
     */
    void resize(size_t size) { factors_.resize(size); }

    /** delete factor without re-arranging indexes by inserting a NULL pointer */
    void remove(size_t i) { factors_[i].reset();}

    /** replace a factor by index */
    void replace(size_t index, sharedFactor factor) { at(index) = factor; }

    /// @}
    /// @name Advanced Interface
    /// @{

    /** return the number of non-null factors */
    size_t nrFactors() const;

    /** Potentially very slow function to return all keys involved */
    FastSet<Key> keys() const;

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_NVP(factors_);
    }

    /// @}

  }; // FactorGraph

} // namespace gtsam
