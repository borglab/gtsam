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

#include <boost/format.hpp>
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
   * @param (in/out) domains all domains, but only domains->at(j) will be checked.
   * @return true if domains->at(j) was changed, false otherwise.
   */
  virtual bool ensureArcConsistency(Key j, Domains* domains) const = 0;

  /// Partially apply known values
  virtual shared_ptr partiallyApply(const DiscreteValues&) const = 0;

  /// Partially apply known values, domain version
  virtual shared_ptr partiallyApply(const Domains&) const = 0;
  /// @}
  /// @name Wrapper support
  /// @{

  /// Render as markdown table.
  std::string markdown(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                       const Names& names = {}) const override {
    return (boost::format("`Constraint` on %1% variables\n") % (size())).str();
  }

  /// Render as html table.
  std::string html(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                   const Names& names = {}) const override {
    return (boost::format("<p>Constraint on %1% variables</p>") % (size())).str();
  }

  /// @}
};
// DiscreteFactor

}  // namespace gtsam
