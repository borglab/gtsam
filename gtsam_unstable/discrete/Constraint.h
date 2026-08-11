/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file Constraint.h
 *  @date May 15, 2012
 *  @author Frank Dellaert
 */

#pragma once

#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam_unstable/dllexport.h>

#include <map>

namespace gtsam {

class Domain;
using Domains = std::map<Key, Domain>;

/**
 * Base class for constraint factors
 * Derived classes include SingleValue, BinaryAllDiff, and AllDiff.
 */
class GTSAM_UNSTABLE_EXPORT Constraint : public DiscreteFactor {
 public:
  typedef std::shared_ptr<Constraint> shared_ptr;

 protected:
  /// Construct unary constraint factor.
  Constraint(Key j) : DiscreteFactor(KeyVector{j}) {}

  /// Construct binary constraint factor.
  Constraint(Key j1, Key j2) : DiscreteFactor(KeyVector{j1, j2}) {}

  /// Construct n-way constraint factor.
  Constraint(const KeyVector& js) : DiscreteFactor(js) {}

  /// construct from container
  template <class KeyIterator>
  Constraint(KeyIterator beginKey, KeyIterator endKey)
      : DiscreteFactor(beginKey, endKey) {}

 public:
  /// @name Standard Constructors
  /// @{

  /// Default constructor for I/O
  Constraint();

  /// Virtual destructor
  ~Constraint() override {}

  /// @}
  /// @name Standard Interface
  /// @{

  /*
   * Ensure Arc-consistency by checking every possible value of domain j.
   * @param j domain to be checked
   * @param (in/out) domains all domains, but only domains->at(j) will be
   * checked.
   * @return true if domains->at(j) was changed, false otherwise.
   */
  virtual bool ensureArcConsistency(Key j, Domains* domains) const = 0;

  /// Partially apply known values
  virtual shared_ptr partiallyApply(const DiscreteValues&) const = 0;

  /// Partially apply known values, domain version
  virtual shared_ptr partiallyApply(const Domains&) const = 0;

  /// Multiply factors, DiscreteFactor::shared_ptr edition
  DiscreteFactor::shared_ptr multiply(
      const DiscreteFactor::shared_ptr& df) const override {
    return std::make_shared<DecisionTreeFactor>(
        this->operator*(df->toDecisionTreeFactor()));
  }

  /// Multiply by a scalar
  virtual DiscreteFactor::shared_ptr operator*(double s) const override {
    return this->toDecisionTreeFactor() * s;
  }

  /// Multiply by a DecisionTreeFactor and return a DecisionTreeFactor
  DecisionTreeFactor operator*(const DecisionTreeFactor& dtf) const override {
    return this->toDecisionTreeFactor() * dtf;
  }

  /// divide by DiscreteFactor::shared_ptr f (safely)
  DiscreteFactor::shared_ptr operator/(
      const DiscreteFactor::shared_ptr& df) const override {
    return this->toDecisionTreeFactor() / df;
  }

  /// Get the number of non-zero values contained in this factor.
  uint64_t nrValues() const override { return 1; };

  DiscreteFactor::shared_ptr sum(size_t nrFrontals) const override {
    return toDecisionTreeFactor().sum(nrFrontals);
  }

  DiscreteFactor::shared_ptr sum(const Ordering& keys) const override {
    return toDecisionTreeFactor().sum(keys);
  }

  /// Find the max value
  double max() const override { return toDecisionTreeFactor().max(); }

  DiscreteFactor::shared_ptr max(size_t nrFrontals) const override {
    return toDecisionTreeFactor().max(nrFrontals);
  }

  DiscreteFactor::shared_ptr max(const Ordering& keys) const override {
    return toDecisionTreeFactor().max(keys);
  }

  /// Compute error for each assignment and return as a tree
  AlgebraicDecisionTree<Key> errorTree() const override {
    throw std::runtime_error("Constraint::error not implemented");
  }

  /// Compute error for each assignment and return as a tree
  DiscreteFactor::shared_ptr restrict(
      const DiscreteValues& assignment) const override {
    throw std::runtime_error("Constraint::restrict not implemented");
  }
  
  /// @}
  /// @name Wrapper support
  /// @{

  /// Render as markdown table.
  std::string markdown(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                       const Names& names = {}) const override {
    return "`Constraint` on " + std::to_string(size()) + " variables\n";
  }

  /// Render as html table.
  std::string html(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                   const Names& names = {}) const override {
    return "<p>Constraint on " + std::to_string(size()) + " variables</p>";
  }

  /// @}
};
// DiscreteFactor

}  // namespace gtsam
