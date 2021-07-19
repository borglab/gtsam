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

#include <gtsam/symbolic/SymbolicFactor.h>
#include <gtsam/inference/Conditional.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/types.h>

namespace gtsam {

  /**
   * SymbolicConditional is a conditional with keys but no probability
   * data, produced by symbolic elimination of SymbolicFactor.
   *
   * It is also a SymbolicFactor, and thus derives from it.  It
   * derives also from Conditional<This>, which is a generic interface
   * class for conditionals.
   * \nosubgrouping
   */
  class GTSAM_EXPORT SymbolicConditional :
    public SymbolicFactor,
    public Conditional<SymbolicFactor, SymbolicConditional> {

  public:
    typedef SymbolicConditional This; /// Typedef to this class
    typedef SymbolicFactor BaseFactor; /// Typedef to the factor base class
    typedef Conditional<BaseFactor, This> BaseConditional; /// Typedef to the conditional base class
    typedef boost::shared_ptr<This> shared_ptr; /// Boost shared_ptr to this class
    typedef BaseFactor::iterator iterator; /// iterator to keys
    typedef BaseFactor::const_iterator const_iterator; /// const_iterator to keys

    /// @name Standard Constructors
    /// @{

    /** Empty Constructor to make serialization possible */
    SymbolicConditional() {}

    /** No parents */
    SymbolicConditional(Key j) : BaseFactor(j), BaseConditional(1) {}

    /** Single parent */
    SymbolicConditional(Key j, Key parent) : BaseFactor(j, parent), BaseConditional(1) {}

    /** Two parents */
    SymbolicConditional(Key j, Key parent1, Key parent2) : BaseFactor(j, parent1, parent2), BaseConditional(1) {}

    /** Three parents */
    SymbolicConditional(Key j, Key parent1, Key parent2, Key parent3) : BaseFactor(j, parent1, parent2, parent3), BaseConditional(1) {}

    /** Named constructor from an arbitrary number of keys and frontals */
    template<typename ITERATOR>
    static SymbolicConditional FromIterators(ITERATOR firstKey, ITERATOR lastKey, size_t nrFrontals)
    {
      SymbolicConditional result;
      (BaseFactor&)result = BaseFactor::FromIterators(firstKey, lastKey);
      result.nrFrontals_ = nrFrontals;
      return result;
    }

    /** Named constructor from an arbitrary number of keys and frontals */
    template<typename ITERATOR>
    static SymbolicConditional::shared_ptr FromIteratorsShared(ITERATOR firstKey, ITERATOR lastKey, size_t nrFrontals)
    {
      SymbolicConditional::shared_ptr result = boost::make_shared<SymbolicConditional>();
      result->keys_.assign(firstKey, lastKey);
      result->nrFrontals_ = nrFrontals;
      return result;
    }

    /** Named constructor from an arbitrary number of keys and frontals */
    template<class CONTAINER>
    static SymbolicConditional FromKeys(const CONTAINER& keys, size_t nrFrontals) {
      return FromIterators(keys.begin(), keys.end(), nrFrontals);
    }

    /** Named constructor from an arbitrary number of keys and frontals */
    template<class CONTAINER>
    static SymbolicConditional::shared_ptr FromKeysShared(const CONTAINER& keys, size_t nrFrontals) {
      return FromIteratorsShared(keys.begin(), keys.end(), nrFrontals);
    }

    ~SymbolicConditional() override {}

    /// Copy this object as its actual derived type.
    SymbolicFactor::shared_ptr clone() const { return boost::make_shared<This>(*this); }

    /// @}

    /// @name Testable

    /** Print with optional formatter */
    void print(
        const std::string& str = "",
        const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

    /** Check equality */
    bool equals(const This& c, double tol = 1e-9) const;

    /// @}

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(BaseFactor);
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(BaseConditional);
    }
  };

/// traits
template<>
struct traits<SymbolicConditional> : public Testable<SymbolicConditional> {
};

} //\ namespace gtsam
