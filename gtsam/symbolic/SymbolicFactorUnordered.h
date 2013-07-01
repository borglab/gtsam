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

#include <utility>
#include <boost/shared_ptr.hpp>
#include <boost/assign/list_of.hpp>

#include <gtsam/inference/FactorUnordered.h>
#include <gtsam/inference/Key.h>

namespace gtsam {

  // Forward declarations
  class SymbolicConditionalUnordered;

  /**
   * SymbolicFactorUnordered serves two purposes.  It is the base class for all linear
   * factors (GaussianFactor, JacobianFactor, HessianFactor), and also functions
   * as a symbolic factor, used to do symbolic elimination by JunctionTree.
   *
   * It derives from Factor with a key type of Key, an unsigned integer.
   * \nosubgrouping
   */
  class GTSAM_EXPORT SymbolicFactorUnordered: public FactorUnordered {

  public:

    typedef SymbolicFactorUnordered This;
    typedef FactorUnordered Base;
    typedef SymbolicConditionalUnordered ConditionalType;

    /** Overriding the shared_ptr typedef */
    typedef boost::shared_ptr<This> shared_ptr;

    /// @name Standard Interface
    /// @{

    /** Default constructor for I/O */
    SymbolicFactorUnordered() {}

    /** Construct unary factor */
    SymbolicFactorUnordered(Key j) :
      Base(boost::assign::cref_list_of<1>(j)) {}

    /** Construct binary factor */
    SymbolicFactorUnordered(Key j1, Key j2) :
      Base(boost::assign::cref_list_of<2>(j1)(j2)) {}

    /** Construct ternary factor */
    SymbolicFactorUnordered(Key j1, Key j2, Key j3) :
      Base(boost::assign::cref_list_of<3>(j1)(j2)(j3)) {}

    /** Construct 4-way factor */
    SymbolicFactorUnordered(Key j1, Key j2, Key j3, Key j4) :
      Base(boost::assign::cref_list_of<4>(j1)(j2)(j3)(j4)) {}

    /** Construct 5-way factor */
    SymbolicFactorUnordered(Key j1, Key j2, Key j3, Key j4, Key j5) :
      Base(boost::assign::cref_list_of<5>(j1)(j2)(j3)(j4)(j5)) {}

    /** Construct 6-way factor */
    SymbolicFactorUnordered(Key j1, Key j2, Key j3, Key j4, Key j5, Key j6) :
      Base(boost::assign::cref_list_of<6>(j1)(j2)(j3)(j4)(j5)(j6)) {}

    /// @}
    
    /// @name Advanced Constructors
    /// @{
  private:
    explicit SymbolicFactorUnordered(const Base& base) :
      Base(base) {}

  public:
    /** Constructor from a collection of keys */
    template<typename KEYITERATOR>
    static SymbolicFactorUnordered FromIterator(KEYITERATOR beginKey, KEYITERATOR endKey) {
      return SymbolicFactorUnordered(Base::FromIterators(beginKey, endKey)); }

    /** Constructor from a collection of keys */
    template<class CONTAINER>
    static SymbolicFactorUnordered FromKeys(const CONTAINER& keys) {
      return SymbolicFactorUnordered(Base::FromKeys(keys)); }

    /// @}

    /// @name Standard Interface
    /// @{
    
    /** Whether the factor is empty (involves zero variables). */
    bool empty() const { return keys_.empty(); }

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    }
  }; // IndexFactor

  // Forward declarations
  class SymbolicFactorGraphUnordered;
  class OrderingUnordered;

  /** Dense elimination function for symbolic factors.  This is usually provided as an argument to
   *  one of the factor graph elimination functions (see EliminateableFactorGraph).  The factor
   *  graph elimination functions do sparse variable elimination, and use this function to eliminate
   *  single variables or variable cliques. */
  GTSAM_EXPORT std::pair<boost::shared_ptr<SymbolicConditionalUnordered>, boost::shared_ptr<SymbolicFactorUnordered> >
    EliminateSymbolicUnordered(const SymbolicFactorGraphUnordered& factors, const OrderingUnordered& keys);

}
