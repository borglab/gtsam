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

#include <gtsam/discrete/DiscreteValues.h>
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

  using Values = DiscreteValues; ///< backwards compatibility

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
  virtual double operator()(const DiscreteValues&) const = 0;

  /// Multiply in a DecisionTreeFactor and return the result as DecisionTreeFactor
  virtual DecisionTreeFactor operator*(const DecisionTreeFactor&) const = 0;

  virtual DecisionTreeFactor toDecisionTreeFactor() const = 0;

  /// @}
};
// DiscreteFactor

// traits
template<> struct traits<DiscreteFactor> : public Testable<DiscreteFactor> {};

}// namespace gtsam
