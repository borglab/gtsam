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
#include <gtsam/base/Testable.h>

namespace gtsam {

class DecisionTreeFactor;
class DiscreteConditional;

/**
 * Base class for discrete probabilistic factors
 * The most general one is the derived DecisionTreeFactor
 */
class GTSAM_EXPORT DiscreteFactor: public Factor {

public:

  // typedefs needed to play nice with gtsam
  typedef DiscreteFactor This; ///< This class
  typedef boost::shared_ptr<DiscreteFactor> shared_ptr; ///< shared_ptr to this class
  typedef Factor Base; ///< Our base class

  /** A map from keys to values
   * TODO: Do we need this? Should we just use gtsam::Values?
   * We just need another special DiscreteValue to represent labels,
   * However, all other Lie's operators are undefined in this class.
   * The good thing is we can have a Hybrid graph of discrete/continuous variables
   * together..
   * Another good thing is we don't need to have the special DiscreteKey which stores
   * cardinality of a Discrete variable. It should be handled naturally in
   * the new class DiscreteValue, as the varible's type (domain)
   */
  typedef Assignment<Key> Values;
  typedef boost::shared_ptr<Values> sharedValues;

public:

  /// @name Standard Constructors
  /// @{

  /** Default constructor creates empty factor */
  DiscreteFactor() {}

  /** Construct from container of keys.  This constructor is used internally from derived factor
   *  constructors, either from a container of keys or from a boost::assign::list_of. */
  template<typename CONTAINER>
  DiscreteFactor(const CONTAINER& keys) : Base(keys) {}

  /// Virtual destructor
  virtual ~DiscreteFactor() {
  }

  /// @}
  /// @name Testable
  /// @{

  /// equals
  virtual bool equals(const DiscreteFactor& lf, double tol = 1e-9) const = 0;

  /// print
  void print(
      const std::string& s = "DiscreteFactor\n",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override {
    Base::print(s, formatter);
  }

  /** Test whether the factor is empty */
  virtual bool empty() const { return size() == 0; }

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

// traits
template<> struct traits<DiscreteFactor> : public Testable<DiscreteFactor> {};
template<> struct traits<DiscreteFactor::Values> : public Testable<DiscreteFactor::Values> {};

}// namespace gtsam
