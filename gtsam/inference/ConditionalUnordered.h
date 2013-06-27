/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Conditional.h
 * @brief   Base class for conditional densities
 * @author  Frank Dellaert
 */

// \callgraph
#pragma once

#include <boost/range.hpp>

#include <gtsam/inference/Key.h>

namespace gtsam {

  /**
   * Base class for conditional densities, templated on KEY type.  This class
   * provides storage for the keys involved in a conditional, and iterators and
   * access to the frontal and separator keys.
   *
   * Derived classes *must* redefine the Factor and shared_ptr typedefs to refer
   * to the associated factor type and shared_ptr type of the derived class.  See
   * IndexConditional and GaussianConditional for examples.
   * \nosubgrouping
   */
  template<class FACTOR, class DERIVEDCONDITIONAL>
  class ConditionalUnordered {

  protected:

    /** The first nrFrontal variables are frontal and the rest are parents. */
    size_t nrFrontals_;

    /** Iterator over keys */
    using typename FACTOR::iterator; // 'using' instead of typedef to avoid ambiguous symbol from multiple inheritance

    /** Const iterator over keys */
    using typename FACTOR::const_iterator; // 'using' instead of typedef to avoid ambiguous symbol from multiple inheritance

  public:

    typedef ConditionalUnordered<FACTOR,DERIVEDCONDITIONAL> This;

    /** View of the frontal keys (call frontals()) */
    typedef boost::iterator_range<const_iterator> Frontals;

    /** View of the separator keys (call parents()) */
    typedef boost::iterator_range<const_iterator> Parents;

    /// @name Standard Constructors
    /// @{

    /** Empty Constructor to make serialization possible */
    ConditionalUnordered() : nrFrontals_(0) {}

    /** Constructor */
    ConditionalUnordered(size_t nrFrontals) : nrFrontals_(nrFrontals) {}

    /// @}
    /// @name Testable
    /// @{

    /** print with optional formatter */
    void print(const std::string& s = "Conditional", const KeyFormatter& formatter = DefaultKeyFormatter) const;

    /** check equality */
    bool equals(const This& c, double tol = 1e-9) const {
      return asFactor().equals(c.asFactor()) && nrFrontals_ == c.nrFrontals_; }

    /// @}
    /// @name Standard Interface
    /// @{

    /** return the number of frontals */
    size_t nrFrontals() const { return nrFrontals_; }

    /** return the number of parents */
    size_t nrParents() const { return asFactor().size() - nrFrontals_; }

    /** Convenience function to get the first frontal key */
    Key firstFrontalKey() const {
      if(nrFrontals_ > 0)
        return asFactor().front();
      else
        throw std::invalid_argument("Requested Conditional::firstFrontalKey from a conditional with zero frontal keys");
    }

    /** return a view of the frontal keys */
    Frontals frontals() const { return boost::make_iterator_range(beginFrontals(), endFrontals()); }

    /** return a view of the parent keys */
    Parents parents() const { return boost::make_iterator_range(beginParents(), endParents()); }

    /** Iterator pointing to first frontal key. */
    const_iterator beginFrontals() const { return asFactor().begin(); }
    
    /** Iterator pointing past the last frontal key. */
    const_iterator endFrontals() const { return asFactor().begin() + nrFrontals_; }
    
    /** Iterator pointing to the first parent key. */
    const_iterator beginParents() const { return endFrontals(); }

    /** Iterator pointing past the last parent key. */
    const_iterator endParents() const { return asFactor().end(); }

    /// @}
    /// @name Advanced Interface
    /// @{

    /** Mutable iterators and accessors */
    iterator beginFrontals() {
      return asFactor().begin();
    } ///<TODO: comment
    iterator endFrontals() {
      return asFactor().begin() + nrFrontals_;
    } ///<TODO: comment
    iterator beginParents() {
      return asFactor().begin() + nrFrontals_;
    } ///<TODO: comment
    iterator endParents() {
      return asFactor().end();
    } ///<TODO: comment

  private:
    // Cast to factor type (non-const) (casts down to derived conditional type, then up to factor type)
    FACTOR& asFactor() { return static_cast<FACTOR&>(static_cast<DERIVEDCONDITIONAL&>(*this)); }

    // Cast to derived type (const) (casts down to derived conditional type, then up to factor type)
    const FACTOR& asFactor() const { return static_cast<const FACTOR&>(static_cast<const DERIVEDCONDITIONAL&>(*this)); }

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(nrFrontals_);
    }

    /// @}

  };

} // gtsam
