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
#include <gtsam_unstable/dllexport.h>

#include <boost/assign.hpp>
#include <map>

namespace gtsam {

class Domain;
using Domains = std::map<Key, Domain>;

/**
 * Base class for constraint factors
 * Derived classes include SingleValue, BinaryAllDiff, and AllDiff.
 */
class GTSAM_EXPORT Constraint : public DiscreteFactor {
 public:
  typedef boost::shared_ptr<Constraint> shared_ptr;

 protected:
  /// Construct unary constraint factor.
  Constraint(Key j) : DiscreteFactor(boost::assign::cref_list_of<1>(j)) {}

  /// Construct binary constraint factor.
  Constraint(Key j1, Key j2)
      : DiscreteFactor(boost::assign::cref_list_of<2>(j1)(j2)) {}

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
  virtual shared_ptr partiallyApply(const Values&) const = 0;

  /// Partially apply known values, domain version
  virtual shared_ptr partiallyApply(const Domains&) const = 0;
  /// @}
};
// DiscreteFactor

}  // namespace gtsam
