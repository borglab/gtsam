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

/**
 * A map from keys to values
 * @ingroup discrete
 */
class GTSAM_EXPORT DiscreteValues : public Assignment<Key> {
 public:
  using Base = Assignment<Key>;  // base class

  /// @name Standard Constructors
  /// @{
  using Assignment::Assignment;  // all constructors

  // Define the implicit default constructor.
  DiscreteValues() = default;

  // Construct from assignment.
  explicit DiscreteValues(const Base& a) : Base(a) {}

  // Construct from initializer list.
  DiscreteValues(std::initializer_list<std::pair<const Key, size_t>> init)
      : Assignment<Key>{init} {}

  /// @}
  /// @name Testable
  /// @{

  /// print required by Testable.
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /// equals required by Testable for unit testing.
  bool equals(const DiscreteValues& x, double tol = 1e-9) const;

  /// @}
  /// @name Standard Interface
  /// @{

  /** Insert all values from \c values.  Throws an invalid_argument exception if
   * any keys to be inserted are already used. */
  DiscreteValues& insert(const DiscreteValues& values);

  /** For all key/value pairs in \c values, replace values with corresponding
   * keys in this object with those in \c values.  Throws std::out_of_range if
   * any keys in \c values are not present in this object. */
  DiscreteValues& update(const DiscreteValues& values);

  /**
   * @brief Return a vector of DiscreteValues, one for each possible
   * combination of values.
   */
  static std::vector<DiscreteValues> CartesianProduct(
      const DiscreteKeys& keys) {
    return Base::CartesianProduct<DiscreteValues>(keys);
  }

  /// @}
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
