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

  /// ostream operator:
  friend std::ostream& operator<<(std::ostream& os, const DiscreteValues& x);

  // insert in base class;
  std::pair<iterator, bool> insert(const value_type& value) {
    return Base::insert(value);
  }

  /**
   * @brief Insert key-assignment pair.
   *
   * @param assignment The key-assignment pair to insert.
   * @return DiscreteValues& Reference to the updated DiscreteValues object.
   * @throws std::invalid_argument if any keys to be inserted are already used.
   */
  DiscreteValues& insert(const std::pair<Key, size_t>& assignment);

  /**
   * @brief Insert all values from another DiscreteValues object.
   *
   * @param values The DiscreteValues object containing values to insert.
   * @return DiscreteValues& Reference to the updated DiscreteValues object.
   * @throws std::invalid_argument if any keys to be inserted are already used.
   */
  DiscreteValues& insert(const DiscreteValues& values);

  /**
   * @brief Update values with corresponding keys from another DiscreteValues
   * object.
   *
   * @param values The DiscreteValues object containing values to update.
   * @return DiscreteValues& Reference to the updated DiscreteValues object.
   * @throws std::out_of_range if any keys in values are not present in this
   * object.
   */
  DiscreteValues& update(const DiscreteValues& values);

  /**
   * @brief Insert each element of `values` if it does not already exist,
   * else assign it.
   * 
   * @param values DiscreteValues to insert or assign to this.
   * @return DiscreteValues& 
   */
  DiscreteValues& insert_or_assign(const DiscreteValues& values);

  /**
   * @brief Check if the DiscreteValues contains the given key.
   *
   * @param key The key to check for.
   * @return True if the key is present, false otherwise.
   */
  bool contains(Key key) const { return this->find(key) != this->end(); }

  /**
   * @brief Filter values by keys.
   *
   * @param keys The keys to filter by.
   * @return DiscreteValues The filtered DiscreteValues object.
   */
  DiscreteValues filter(const DiscreteKeys& keys) const {
    DiscreteValues result;
    for (const auto& [key, _] : keys) {
      if (auto it = this->find(key); it != this->end())
        result[key] = it->second;
    }
    return result;
  }

  /**
   * @brief Return the keys that are not present in the DiscreteValues object.
   *
   * @param keys The keys to check for.
   * @return DiscreteKeys Keys not present in the DiscreteValues object.
   */
  DiscreteKeys missingKeys(const DiscreteKeys& keys) const {
    DiscreteKeys result;
    for (const auto& [key, cardinality] : keys) {
      if (!this->contains(key)) result.emplace_back(key, cardinality);
    }
    return result;
  }

  /**
   * @brief Return a vector of DiscreteValues, one for each possible combination
   * of values.
   *
   * @param keys The keys to generate the Cartesian product for.
   * @return std::vector<DiscreteValues> The vector of DiscreteValues.
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

/// Free version of CartesianProduct.
inline std::vector<DiscreteValues> cartesianProduct(const DiscreteKeys& keys) {
  return DiscreteValues::CartesianProduct(keys);
}

/// Free version of print for wrapper
void GTSAM_EXPORT
PrintDiscreteValues(const DiscreteValues& values, const std::string& s = "",
                    const KeyFormatter& keyFormatter = DefaultKeyFormatter);

/// Free version of markdown.
std::string GTSAM_EXPORT
markdown(const DiscreteValues& values,
         const KeyFormatter& keyFormatter = DefaultKeyFormatter,
         const DiscreteValues::Names& names = {});

/// Free version of html.
std::string GTSAM_EXPORT
html(const DiscreteValues& values,
     const KeyFormatter& keyFormatter = DefaultKeyFormatter,
     const DiscreteValues::Names& names = {});

// traits
template <>
struct traits<DiscreteValues> : public Testable<DiscreteValues> {};

}  // namespace gtsam
