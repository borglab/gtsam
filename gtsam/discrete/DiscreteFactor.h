/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file DiscreteFactor.h
 *  @date Feb 14, 2011
 *  @author Duy-Nguyen Ta
 *  @author Frank Dellaert
 */

#pragma once

#include <gtsam/discrete/Assignment.h>
#include <gtsam/inference/Factor.h>

#include <boost/assign/list_of.hpp>

namespace gtsam {

  class DecisionTreeFactor;
  class DiscreteConditional;

  /**
   * Base class for discrete probabilistic factors
   * The most general one is the derived DecisionTreeFactor
   */
  class GTSAM_EXPORT DiscreteFactor : public Factor {

  public:

    // typedefs needed to play nice with gtsam
    typedef DiscreteFactor This;
    typedef Factor Base;
    typedef DiscreteConditional ConditionalType;
    typedef boost::shared_ptr<DiscreteFactor> shared_ptr;

    /** A map from keys to values */
    typedef Assignment<Index> Values;
    typedef boost::shared_ptr<Values> sharedValues;

  protected:

    /// Construct n-way factor
    DiscreteFactor(const std::vector<Index>& js) :
        Base(js) {
    }

    /// Construct unary factor
    DiscreteFactor(Index j) :
        Base(boost::assign::cref_list_of<1>(j)) {
    }

    /// Construct binary factor
    DiscreteFactor(Index j1, Index j2) :
        Base(boost::assign::cref_list_of<2>(j1)(j2)) {
    }

    /// construct from container
    template<class KeyIterator>
    DiscreteFactor(KeyIterator beginKey, KeyIterator endKey) :
        Base(beginKey, endKey) {
    }

  public:

    /// @name Standard Constructors
    /// @{

    /// Default constructor for I/O
    DiscreteFactor();

    /// Virtual destructor
    virtual ~DiscreteFactor() {}

    /// @}
    /// @name Testable
    /// @{

    // print
    virtual void print(const std::string& s = "DiscreteFactor\n",
        const KeyFormatter& formatter = DefaultKeyFormatter) const {
      Base::print(s,formatter);
    }

    /// @}
    /// @name Standard Interface
    /// @{

    /// Find value for given assignment of values to variables
    virtual double operator()(const Values&) const = 0;

    /// Multiply in a DecisionTreeFactor and return the result as DecisionTreeFactor
    virtual DecisionTreeFactor operator*(const DecisionTreeFactor&) const = 0;

    virtual DecisionTreeFactor toDecisionTreeFactor() const = 0;

    /// @}
  };
// DiscreteFactor

}// namespace gtsam
