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
 * @author  Richard Roberts
 */

// \callgraph

#pragma once

#include <boost/serialization/nvp.hpp>
#include <boost/assign/list_inserter.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits.hpp>

#include <gtsam/base/Testable.h>
#include <gtsam/base/FastVector.h>
#include <gtsam/inference/Key.h>

namespace gtsam {

  // Forward declarations
  template<class CLIQUE> class BayesTree;

  /** Helper */
  template<class C>
  class CRefCallPushBack
  {
    C& obj;
  public:
    CRefCallPushBack(C& obj) : obj(obj) {}
    template<typename A>
    void operator()(const A& a) { obj.push_back(a); }
  };

  /** Helper */
  template<class C>
  class RefCallPushBack
  {
    C& obj;
  public:
    RefCallPushBack(C& obj) : obj(obj) {}
    template<typename A>
    void operator()(A& a) { obj.push_back(a); }
  };

  /** Helper */
  template<class C>
  class CRefCallAddCopy
  {
    C& obj;
  public:
    CRefCallAddCopy(C& obj) : obj(obj) {}
    template<typename A>
    void operator()(const A& a) { obj.addCopy(a); }
  };

  /**
   * A factor graph is a bipartite graph with factor nodes connected to variable nodes.
   * In this class, however, only factor nodes are kept around.
   * \nosubgrouping
   */
  template<class FACTOR>
  class FactorGraph {

  public:
    typedef FACTOR FactorType;  ///< factor type
    typedef boost::shared_ptr<FACTOR> sharedFactor;  ///< Shared pointer to a factor
    typedef sharedFactor value_type;
    typedef typename FastVector<sharedFactor>::iterator iterator;
    typedef typename FastVector<sharedFactor>::const_iterator const_iterator;

  private:
    typedef FactorGraph<FACTOR> This;  ///< Typedef for this class
    typedef boost::shared_ptr<This> shared_ptr;  ///< Shared pointer for this class

  protected:
    /** concept check, makes sure FACTOR defines print and equals */
    GTSAM_CONCEPT_TESTABLE_TYPE(FACTOR)

    /** Collection of factors */
    FastVector<sharedFactor> factors_;

    /// @name Standard Constructors
    /// @{

    /** Default constructor */
    FactorGraph() {}

    /** Constructor from iterator over factors (shared_ptr or plain objects) */
    template<typename ITERATOR>
    FactorGraph(ITERATOR firstFactor, ITERATOR lastFactor) { push_back(firstFactor, lastFactor); }

    /** Construct from container of factors (shared_ptr or plain objects) */
    template<class CONTAINER>
    explicit FactorGraph(const CONTAINER& factors) { push_back(factors); }

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

  public:
    /// @name Adding Factors
    /// @{

    /**
     * Reserve space for the specified number of factors if you know in
     * advance how many there will be (works like FastVector::reserve).
     */
    void reserve(size_t size) { factors_.reserve(size); }

    // TODO: are these needed?

    /** Add a factor directly using a shared_ptr */
    template<class DERIVEDFACTOR>
    typename boost::enable_if<boost::is_base_of<FactorType, DERIVEDFACTOR> >::type
    push_back(boost::shared_ptr<DERIVEDFACTOR> factor) {
      factors_.push_back(boost::shared_ptr<FACTOR>(factor)); }

    /** Add a factor directly using a shared_ptr */
    void push_back(const sharedFactor& factor) {
      factors_.push_back(factor); }

    /** push back many factors with an iterator over shared_ptr (factors are not copied) */
    template<typename ITERATOR>
    typename boost::enable_if<boost::is_base_of<FactorType, typename ITERATOR::value_type::element_type> >::type
      push_back(ITERATOR firstFactor, ITERATOR lastFactor) {
        factors_.insert(end(), firstFactor, lastFactor); }

    /** push back many factors as shared_ptr's in a container (factors are not copied) */
    template<typename CONTAINER>
    typename boost::enable_if<boost::is_base_of<FactorType, typename CONTAINER::value_type::element_type> >::type
      push_back(const CONTAINER& container) {
        push_back(container.begin(), container.end());
    }

    /** push back a BayesTree as a collection of factors.  NOTE: This should be hidden in derived
     *  classes in favor of a type-specialized version that calls this templated function. */
    template<class CLIQUE>
    typename boost::enable_if<boost::is_base_of<This, typename CLIQUE::FactorGraphType> >::type
    push_back(const BayesTree<CLIQUE>& bayesTree) {
      bayesTree.addFactorsToGraph(*this);
    }

    /** Add a factor by value, will be copy-constructed (use push_back with a shared_ptr to avoid
    *  the copy). */
    template<class DERIVEDFACTOR>
    typename boost::enable_if<boost::is_base_of<FactorType, DERIVEDFACTOR> >::type
      push_back(const DERIVEDFACTOR& factor) {
        factors_.push_back(boost::make_shared<DERIVEDFACTOR>(factor));
    }

    /** push back many factors with an iterator over plain factors (factors are copied) */
    template<typename ITERATOR>
    typename boost::enable_if<boost::is_base_of<FactorType, typename ITERATOR::value_type> >::type
      push_back(ITERATOR firstFactor, ITERATOR lastFactor) {
        for (ITERATOR f = firstFactor; f != lastFactor; ++f)
          push_back(*f);
    }

    /** push back many factors as non-pointer objects in a container (factors are copied) */
    template<typename CONTAINER>
    typename boost::enable_if<boost::is_base_of<FactorType, typename CONTAINER::value_type> >::type
      push_back(const CONTAINER& container) {
        push_back(container.begin(), container.end());
    }

    /** Add a factor directly using a shared_ptr */
    template<class DERIVEDFACTOR>
    typename boost::enable_if<boost::is_base_of<FactorType, DERIVEDFACTOR>,
      boost::assign::list_inserter<RefCallPushBack<This> > >::type
      operator+=(boost::shared_ptr<DERIVEDFACTOR> factor) {
        return boost::assign::make_list_inserter(RefCallPushBack<This>(*this))(factor);
    }

    /** Add a factor directly using a shared_ptr */
    boost::assign::list_inserter<CRefCallPushBack<This> >
      operator+=(const sharedFactor& factor) {
        return boost::assign::make_list_inserter(CRefCallPushBack<This>(*this))(factor);
    }

    /** Add a factor or container of factors, including STL collections, BayesTrees, etc. */
    template<class FACTOR_OR_CONTAINER>
    boost::assign::list_inserter<CRefCallPushBack<This> >
      operator+=(const FACTOR_OR_CONTAINER& factorOrContainer) {
        return boost::assign::make_list_inserter(CRefCallPushBack<This>(*this))(factorOrContainer);
    }

    /** Add a factor directly using a shared_ptr */
    template<class DERIVEDFACTOR>
    typename boost::enable_if<boost::is_base_of<FactorType, DERIVEDFACTOR> >::type
      add(boost::shared_ptr<DERIVEDFACTOR> factor) {
        push_back(factor);
    }

    /** Add a factor directly using a shared_ptr */
    void add(const sharedFactor& factor) {
      push_back(factor);
    }

    /** Add a factor or container of factors, including STL collections, BayesTrees, etc. */
    template<class FACTOR_OR_CONTAINER>
    void add(const FACTOR_OR_CONTAINER& factorOrContainer) {
      push_back(factorOrContainer);
    }

    /// @}
    /// @name Testable
    /// @{

    /** print out graph */
    void print(const std::string& s = "FactorGraph",
      const KeyFormatter& formatter = DefaultKeyFormatter) const;

    /** Check equality */
    bool equals(const This& fg, double tol = 1e-9) const;
    /// @}

  public:
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

    /** Erase factor and rearrange other factors to take up the empty space */
    iterator erase(iterator item) { return factors_.erase(item); }

    /** Erase factors and rearrange other factors to take up the empty space */
    iterator erase(iterator first, iterator last) { return factors_.erase(first, last); }

    /// @}
    /// @name Advanced Interface
    /// @{

    /** return the number of non-null factors */
    size_t nrFactors() const;

    /** Potentially slow function to return all keys involved, sorted, as a set */
    KeySet keys() const;

    /** Potentially slow function to return all keys involved, sorted, as a vector */
    KeyVector keyVector() const;

    /** MATLAB interface utility: Checks whether a factor index idx exists in the graph and is a live pointer */
    inline bool exists(size_t idx) const { return idx < size() && at(idx); }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_NVP(factors_);
    }

    /// @}

  }; // FactorGraph

} // namespace gtsam

#include <gtsam/inference/FactorGraph-inst.h>