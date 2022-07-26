/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file DiscreteDistribution.h
 *  @date December 2021
 *  @author Frank Dellaert
 */

#pragma once

#include <gtsam/discrete/DiscreteConditional.h>

#include <string>
#include <vector>

namespace gtsam {

/**
 * A prior probability on a set of discrete variables.
 * Derives from DiscreteConditional
 *
 * @ingroup discrete
 */
class GTSAM_EXPORT DiscreteDistribution : public DiscreteConditional {
 public:
  using Base = DiscreteConditional;

  /// @name Standard Constructors
  /// @{

  /// Default constructor needed for serialization.
  DiscreteDistribution() {}

  /// Constructor from factor.
  explicit DiscreteDistribution(const DecisionTreeFactor& f)
      : Base(f.size(), f) {}

  /**
   * Construct from a Signature.
   *
   * Example: DiscreteDistribution P(D % "3/2");
   */
  explicit DiscreteDistribution(const Signature& s) : Base(s) {}

  /**
   * Construct from key and a vector of floats specifying the probability mass
   * function (PMF).
   *
   * Example: DiscreteDistribution P(D, {0.4, 0.6});
   */
  DiscreteDistribution(const DiscreteKey& key, const std::vector<double>& spec)
      : DiscreteDistribution(Signature(key, {}, Signature::Table{spec})) {}

  /**
   * Construct from key and a string specifying the probability mass function
   * (PMF).
   *
   * Example: DiscreteDistribution P(D, "9/1 2/8 3/7 1/9");
   */
  DiscreteDistribution(const DiscreteKey& key, const std::string& spec)
      : DiscreteDistribution(Signature(key, {}, spec)) {}

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

  /// @}
#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V42
  /// @name Deprecated functionality
  /// @{
  size_t GTSAM_DEPRECATED solve() const { return Base::solve({}); }
  /// @}
#endif
};
// DiscreteDistribution

// traits
template <>
struct traits<DiscreteDistribution> : public Testable<DiscreteDistribution> {};

}  // namespace gtsam
