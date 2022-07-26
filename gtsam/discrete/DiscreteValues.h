/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file DiscreteValues.h
 *  @date Dec 13, 2021
 *  @author Frank Dellaert
 */

#pragma once

#include <gtsam/discrete/Assignment.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/inference/Key.h>

#include <map>
#include <string>
#include <vector>

namespace gtsam {

/** A map from keys to values
 * TODO(dellaert): Do we need this? Should we just use gtsam::DiscreteValues?
 * We just need another special DiscreteValue to represent labels,
 * However, all other Lie's operators are undefined in this class.
 * The good thing is we can have a Hybrid graph of discrete/continuous variables
 * together..
 * Another good thing is we don't need to have the special DiscreteKey which
 * stores cardinality of a Discrete variable. It should be handled naturally in
 * the new class DiscreteValue, as the variable's type (domain)
 * @ingroup discrete
 */
class GTSAM_EXPORT DiscreteValues : public Assignment<Key> {
 public:
  using Base = Assignment<Key>;  // base class

  using Assignment::Assignment;  // all constructors

  // Define the implicit default constructor.
  DiscreteValues() = default;

  // Construct from assignment.
  explicit DiscreteValues(const Base& a) : Base(a) {}

  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  static std::vector<DiscreteValues> CartesianProduct(
      const DiscreteKeys& keys) {
    return Base::CartesianProduct<DiscreteValues>(keys);
  }

  /// @name Wrapper support
  /// @{

  /// Translation table from values to strings.
  using Names = std::map<Key, std::vector<std::string>>;

  /// Translate an integer index value for given key to a string.
  static std::string Translate(const Names& names, Key key, size_t index);

  /**
   * @brief Output as a markdown table.
   *
   * @param keyFormatter function that formats keys.
   * @param names translation table for values.
   * @return string markdown output.
   */
  std::string markdown(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                       const Names& names = {}) const;

  /**
   * @brief Output as a html table.
   *
   * @param keyFormatter function that formats keys.
   * @param names translation table for values.
   * @return string html output.
   */
  std::string html(const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                   const Names& names = {}) const;

  /// @}
};

/// Free version of markdown.
std::string markdown(const DiscreteValues& values,
                     const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                     const DiscreteValues::Names& names = {});

/// Free version of html.
std::string html(const DiscreteValues& values,
                 const KeyFormatter& keyFormatter = DefaultKeyFormatter,
                 const DiscreteValues::Names& names = {});

// traits
template <>
struct traits<DiscreteValues> : public Testable<DiscreteValues> {};

}  // namespace gtsam
