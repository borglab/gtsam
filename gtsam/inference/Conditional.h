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

#include <gtsam/inference/AbstractConditional.h>
#include <gtsam/inference/Key.h>

namespace gtsam {

/**
 * Base class for conditional densities.  This class iterators and
 * access to the frontal and separator keys.
 *
 * Derived classes *must* redefine the Factor and shared_ptr typedefs to refer
 * to the associated factor type and shared_ptr type of the derived class.  See
 * SymbolicConditional and GaussianConditional for examples.
 * \nosubgrouping
 */
template<class FACTOR, class DERIVEDCONDITIONAL>
class Conditional : public AbstractConditional {

 private:
  /// Typedef to this class
  typedef Conditional<FACTOR, DERIVEDCONDITIONAL> This;

 protected:

  /// @name Standard Constructors
  /// @{

  /** Empty Constructor to make serialization possible */
  Conditional() {}

  /** Constructor */
  Conditional(size_t nrFrontals) : AbstractConditional(nrFrontals) {}

  /// @}

 public:
  /// @name Testable
  /// @{

  /** print with optional formatter */
  void print(const std::string &s = "Conditional",
             const KeyFormatter &formatter = DefaultKeyFormatter) const override;

  /** check equality */
  bool equals(const This &c, double tol = 1e-9) const {
    return AbstractConditional::equals(c, tol);
  }

  /// @}

  /// @name Standard Interface
  /// @{

  /** return the number of parents */
  size_t nrParents() const override { return asFactor().size() - nrFrontals_; }

  /** Convenience function to get the first frontal key */
  Key firstFrontalKey() const {
    if (nrFrontals_ > 0)
      return asFactor().front();
    else
      throw std::invalid_argument(
          "Requested Conditional::firstFrontalKey from a conditional with zero frontal keys");
  }

  /** return a view of the frontal keys */
  Frontals frontals() const override {
    return boost::make_iterator_range(beginFrontals(),
                                      endFrontals());
  }

  /** return a view of the parent keys */
  Parents parents() const override {
    return boost::make_iterator_range(beginParents(),
                                      endParents());
  }

  /** Iterator pointing to first frontal key. */
  typename FACTOR::const_iterator beginFrontals() const { return asFactor().begin(); }

  /** Iterator pointing past the last frontal key. */
  typename FACTOR::const_iterator endFrontals() const {
    return asFactor().begin() + nrFrontals_;
  }

  /** Iterator pointing to the first parent key. */
  typename FACTOR::const_iterator beginParents() const { return endFrontals(); }

  /** Iterator pointing past the last parent key. */
  typename FACTOR::const_iterator endParents() const { return asFactor().end(); }

  /// @}
  /// @name Advanced Interface
  /// @{

  /** Mutable iterator pointing to first frontal key. */
  typename FACTOR::iterator beginFrontals() { return asFactor().begin(); }

  /** Mutable iterator pointing past the last frontal key. */
  typename FACTOR::iterator endFrontals() {
    return asFactor().begin() + nrFrontals_;
  }

  /** Mutable iterator pointing to the first parent key. */
  typename FACTOR::iterator beginParents() {
    return asFactor().begin() + nrFrontals_;
  }

  /** Mutable iterator pointing past the last parent key. */
  typename FACTOR::iterator endParents() { return asFactor().end(); }

 private:
  // Cast to factor type (non-const) (casts down to derived conditional type, then up to factor type)
  FACTOR &asFactor() { return static_cast<FACTOR &>(static_cast<DERIVEDCONDITIONAL &>(*this)); }

  // Cast to derived type (const) (casts down to derived conditional type, then up to factor type)
  const FACTOR &asFactor() const { return static_cast<const FACTOR &>(static_cast<const DERIVEDCONDITIONAL &>(*this)); }

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(nrFrontals_);
  }

  /// @}

};

} // gtsam
