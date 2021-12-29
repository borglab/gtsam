/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file DiscretePrior.h
 *  @date December 2021
 *  @author Frank Dellaert
 */

#pragma once

#include <gtsam/discrete/DiscreteConditional.h>

#include <string>

namespace gtsam {

/**
 * A prior probability on a set of discrete variables.
 * Derives from DiscreteConditional
 */
class GTSAM_EXPORT DiscretePrior : public DiscreteConditional {
 public:
  using Base = DiscreteConditional;

  /// @name Standard Constructors
  /// @{

  /// Default constructor needed for serialization.
  DiscretePrior() {}

  /// Constructor from factor.
  DiscretePrior(const DecisionTreeFactor& f) : Base(f.size(), f) {}

  /**
   * Construct from a Signature.
   *
   * Example: DiscretePrior P(D % "3/2");
   */
  DiscretePrior(const Signature& s) : Base(s) {}

  /**
   * Construct from key and a Signature::Table specifying the
   * conditional probability table (CPT).
   *
   * Example: DiscretePrior P(D, table);
   */
  DiscretePrior(const DiscreteKey& key, const Signature::Table& table)
      : Base(Signature(key, {}, table)) {}

  /**
   * Construct from key and a string specifying the conditional
   * probability table (CPT).
   *
   * Example: DiscretePrior P(D, "9/1 2/8 3/7 1/9");
   */
  DiscretePrior(const DiscreteKey& key, const std::string& spec)
      : DiscretePrior(Signature(key, {}, spec)) {}

  /// @}
  /// @name Testable
  /// @{

  /// GTSAM-style print
  void print(
      const std::string& s = "Discrete Prior: ",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override;

  /// @}
  /// @name Standard interface
  /// @{

  /// Evaluate given a single value.
  double operator()(size_t value) const;

  /// We also want to keep the Base version, taking DiscreteValues:
  // TODO(dellaert): does not play well with wrapper!
  // using Base::operator();

  /// Return entire probability mass function.
  std::vector<double> pmf() const;

  /**
   * solve a conditional
   * @return MPE value of the child (1 frontal variable).
   */
  size_t solve() const { return Base::solve({}); }

  /**
   * sample
   * @return sample from conditional
   */
  size_t sample() const { return Base::sample(DiscreteValues()); }

  /// @}
};
// DiscretePrior

// traits
template <>
struct traits<DiscretePrior> : public Testable<DiscretePrior> {};

}  // namespace gtsam
