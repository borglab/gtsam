/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SymbolicConditional.h
 * @author  Richard Roberts
 * @date    Oct 17, 2010
 */

#pragma once

#include <gtsam/base/types.h>
#include <gtsam/inference/ConditionalUnordered.h>
#include <gtsam/symbolic/SymbolicFactorUnordered.h>

namespace gtsam {

  /**
   * SymbolicConditionalUnordered is a conditional with keys but no probability
   * data, produced by symbolic elimination of SymbolicFactorUnordered.
   *
   * It is also a SymbolicFactorUnordered, and thus derives from it.  It
   * derives also from ConditionalUnordered<This>, which is a generic interface
   * class for conditionals.
   * \nosubgrouping
   */
  class GTSAM_EXPORT SymbolicConditionalUnordered : public SymbolicFactorUnordered, public ConditionalUnordered<SymbolicFactorUnordered,SymbolicConditionalUnordered> {

  public:
    typedef SymbolicConditionalUnordered This; /// Typedef to this class
    typedef SymbolicFactorUnordered BaseFactor; /// Typedef to the factor base class
    typedef ConditionalUnordered<SymbolicFactorUnordered,SymbolicConditionalUnordered> BaseConditional; /// Typedef to the conditional base class
    typedef boost::shared_ptr<This> shared_ptr; /// Boost shared_ptr to this class
    typedef BaseFactor::iterator iterator; /// iterator to keys
    typedef BaseFactor::const_iterator const_iterator; /// const_iterator to keys

    /// @name Standard Constructors
    /// @{

    /** Empty Constructor to make serialization possible */
    SymbolicConditionalUnordered() {}

    /** No parents */
    SymbolicConditionalUnordered(Index j) : BaseFactor(j), BaseConditional(1) {}

    /** Single parent */
    SymbolicConditionalUnordered(Index j, Index parent) : BaseFactor(j, parent), BaseConditional(1) {}

    /** Two parents */
    SymbolicConditionalUnordered(Index j, Index parent1, Index parent2) : BaseFactor(j, parent1, parent2), BaseConditional(1) {}

    /** Three parents */
    SymbolicConditionalUnordered(Index j, Index parent1, Index parent2, Index parent3) : BaseFactor(j, parent1, parent2, parent3), BaseConditional(1) {}

    /** Named constructor from an arbitrary number of keys and frontals */
    template<typename ITERATOR>
    static SymbolicConditionalUnordered FromIterator(ITERATOR firstKey, ITERATOR lastKey, size_t nrFrontals)
    {
      SymbolicConditionalUnordered result;
      (BaseFactor&)result = BaseFactor::FromIterator(firstKey, lastKey);
      result.nrFrontals_ = nrFrontals;
      return result; }

    /** Named constructor from an arbitrary number of keys and frontals */
    template<class CONTAINER>
    static SymbolicConditionalUnordered FromKeys(const CONTAINER& keys, size_t nrFrontals) {
      return FromIterator(keys.begin(), keys.end(), nrFrontals); }

    /// @}

    /// @name Testable

    /** Print with optional formatter */
    void print(const std::string& str = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      BaseConditional::print(str, keyFormatter); }

    /** Check equality */
    bool equals(const This& c, double tol = 1e-9) const { return BaseConditional::equals(c); }

    /// @}

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    }
  };

}
