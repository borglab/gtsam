/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SymbolicFactor.h
 * @author  Richard Roberts
 * @date    Oct 17, 2010
 */

#pragma once

#include <gtsam/inference/Factor.h>
#include <gtsam/inference/Key.h>
#include <gtsam/base/Testable.h>

#include <memory>

#include <utility>

namespace gtsam {

  // Forward declarations
  class SymbolicConditional;
  class HybridValues;
  class Ordering;

  /** SymbolicFactor represents a symbolic factor that specifies graph topology but is not
   *  associated with any numerical function.
   *  \nosubgrouping */
  class GTSAM_EXPORT SymbolicFactor : public Factor {

  public:

    typedef SymbolicFactor This;
    typedef Factor Base;
    typedef SymbolicConditional ConditionalType;

    /** Overriding the shared_ptr typedef */
    typedef std::shared_ptr<This> shared_ptr;

    /// @name Standard Constructors
    /// @{

    /** Default constructor for I/O */
    SymbolicFactor() {}

    /** Construct unary factor */
    explicit SymbolicFactor(Key j) :
      Base(KeyVector{j}) {}

    /** Construct binary factor */
    SymbolicFactor(Key j1, Key j2) :
      Base(KeyVector{j1, j2}) {}

    /** Construct ternary factor */
    SymbolicFactor(Key j1, Key j2, Key j3) :
      Base(KeyVector{j1, j2, j3}) {}

    /** Construct 4-way factor */
    SymbolicFactor(Key j1, Key j2, Key j3, Key j4) :
      Base(KeyVector{j1, j2, j3, j4}) {}

    /** Construct 5-way factor */
    SymbolicFactor(Key j1, Key j2, Key j3, Key j4, Key j5) :
      Base(KeyVector{j1, j2, j3, j4, j5}) {}

    /** Construct 6-way factor */
    SymbolicFactor(Key j1, Key j2, Key j3, Key j4, Key j5, Key j6) :
      Base(KeyVector{j1, j2, j3, j4, j5, j6}) {}

    /** Create symbolic version of any factor */
    explicit SymbolicFactor(const Factor& factor) : Base(factor.keys()) {}

    virtual ~SymbolicFactor() {}

    /// Copy this object as its actual derived type.
    SymbolicFactor::shared_ptr clone() const { return std::make_shared<This>(*this); }

    /// @}

    /// @name Testable
    /// @{

    bool equals(const This& other, double tol = 1e-9) const;

    /// print
    void print(
        const std::string& s = "SymbolicFactor",
        const KeyFormatter& formatter = DefaultKeyFormatter) const override {
      Base::print(s, formatter);
    }

    /// print only keys
    void printKeys(
        const std::string& s = "SymbolicFactor",
        const KeyFormatter& formatter = DefaultKeyFormatter) const override {
      Base::printKeys(s, formatter);
    }

    /// @}
    /// @name Advanced Constructors
    /// @{
  
    /** Constructor from a collection of keys */
    template<typename KEYITERATOR>
    static SymbolicFactor FromIterators(KEYITERATOR beginKey, KEYITERATOR endKey) {
      return SymbolicFactor(Base::FromIterators(beginKey, endKey));
    }

    /** Constructor from a collection of keys */
    template<typename KEYITERATOR>
    static SymbolicFactor::shared_ptr FromIteratorsShared(KEYITERATOR beginKey, KEYITERATOR endKey) {
      SymbolicFactor::shared_ptr result = std::make_shared<SymbolicFactor>();
      result->keys_.assign(beginKey, endKey);
      return result;
    }

    /** Constructor from a collection of keys - compatible with boost assign::list_of and
     *  boost assign::cref_list_of */
    template<class CONTAINER>
    static SymbolicFactor FromKeys(const CONTAINER& keys) {
      return SymbolicFactor(Base::FromKeys(keys));
    }

    /** Constructor from a collection of keys - compatible with boost assign::list_of and
     *  boost assign::cref_list_of */
    template<class CONTAINER>
    static SymbolicFactor::shared_ptr FromKeysShared(const CONTAINER& keys) {
      return FromIteratorsShared(keys.begin(), keys.end());
    }

    /// @}

    /// @name Standard Interface
    /// @{

    /// The `error` method throws an exception.
    double error(const HybridValues& c) const override;

    /** Eliminate the variables in \c keys, in the order specified in \c keys, returning a
     *  conditional and marginal. */
    std::pair<std::shared_ptr<SymbolicConditional>, std::shared_ptr<SymbolicFactor> >
      eliminate(const Ordering& keys) const;

    /// @}

  private:
    /** Serialization function */
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    }
#endif
  }; // IndexFactor

  // Forward declarations
  class SymbolicFactorGraph;
  class Ordering;

  /** Dense elimination function for symbolic factors.  This is usually provided as an argument to
   *  one of the factor graph elimination functions (see EliminateableFactorGraph).  The factor
   *  graph elimination functions do sparse variable elimination, and use this function to eliminate
   *  single variables or variable cliques. */
  GTSAM_EXPORT std::pair<std::shared_ptr<SymbolicConditional>, std::shared_ptr<SymbolicFactor> >
    EliminateSymbolic(const SymbolicFactorGraph& factors, const Ordering& keys);

  /// traits
  template<>
  struct traits<SymbolicFactor> : public Testable<SymbolicFactor> {
  };

} //\ namespace gtsam

