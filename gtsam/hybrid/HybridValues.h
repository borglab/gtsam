/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file HybridValues.h
 *  @date Jul 28, 2022
 *  @author Shangjie Xue
 */

#pragma once

#include <gtsam/discrete/Assignment.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/Values.h>

#include <map>
#include <string>
#include <vector>

namespace gtsam {

/**
 * HybridValues represents a collection of DiscreteValues and VectorValues.
 * It is typically used to store the variables of a HybridGaussianFactorGraph.
 * Optimizing a HybridGaussianBayesNet returns this class.
 */
class GTSAM_EXPORT HybridValues {
 private:
  /// Continuous multi-dimensional vectors for \class GaussianFactor.
  VectorValues continuous_;

  /// Discrete values for \class DiscreteFactor.
  DiscreteValues discrete_;

  /// Continuous, differentiable manifold values for \class NonlinearFactor.
  Values nonlinear_;

 public:
  /// @name Standard Constructors
  /// @{

  /// Default constructor creates an empty HybridValues.
  HybridValues() = default;

  /// Construct from DiscreteValues and VectorValues.
  HybridValues(const VectorValues &cv, const DiscreteValues &dv)
      : continuous_(cv), discrete_(dv){}

  /// Construct from all values types.
  HybridValues(const VectorValues& cv, const DiscreteValues& dv,
               const Values& v)
      : continuous_(cv), discrete_(dv), nonlinear_(v){}

  /// @}
  /// @name Testable
  /// @{

  /// print required by Testable for unit testing
  void print(const std::string& s = "HybridValues",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    std::cout << s << ": \n";
    continuous_.print("  Continuous",
                      keyFormatter);              // print continuous components
    discrete_.print("  Discrete", keyFormatter);  // print discrete components
  }

  /// equals required by Testable for unit testing
  bool equals(const HybridValues& other, double tol = 1e-9) const {
    return continuous_.equals(other.continuous_, tol) &&
           discrete_.equals(other.discrete_, tol);
  }

  /// @}
  /// @name Interface
  /// @{

  /// Return the multi-dimensional vector values.
  const VectorValues& continuous() const { return continuous_; }

  /// Return the discrete values.
  const DiscreteValues& discrete() const { return discrete_; }

  /// Return the nonlinear values.
  const Values& nonlinear() const { return nonlinear_; }

  /// Check whether a variable with key \c j exists in VectorValues.
  bool existsVector(Key j) { return continuous_.exists(j); }

  /// Check whether a variable with key \c j exists in DiscreteValues.
  bool existsDiscrete(Key j) { return (discrete_.find(j) != discrete_.end()); }

  /// Check whether a variable with key \c j exists in values.
  bool existsNonlinear(Key j) {
    return nonlinear_.exists(j);
  }

  /// Check whether a variable with key \c j exists.
  bool exists(Key j) {
    return existsVector(j) || existsDiscrete(j) || existsNonlinear(j);
  }

  /** Insert a vector \c value with key \c j.  Throws an invalid_argument
   * exception if the key \c j is already used.
   * @param value The vector to be inserted.
   * @param j The index with which the value will be associated. */
  void insert(Key j, const Vector& value) { continuous_.insert(j, value); }

  /** Insert a discrete \c value with key \c j.  Replaces the existing value if
   * the key \c j is already used.
   * @param value The vector to be inserted.
   * @param j The index with which the value will be associated. */
  void insert(Key j, size_t value) { discrete_[j] = value; }

  /// insert_or_assign() , similar to Values.h
  void insert_or_assign(Key j, const Vector& value) {
    continuous_.insert_or_assign(j, value);
  }

  /// insert_or_assign() , similar to Values.h
  void insert_or_assign(Key j, size_t value) {
    discrete_[j] = value;
  }

  /** Insert all continuous values from \c values.  Throws an invalid_argument
   * exception if any keys to be inserted are already used. */
  HybridValues& insert(const VectorValues& values) {
    continuous_.insert(values);
    return *this;
  }

  /** Insert all discrete values from \c values.  Throws an invalid_argument
   * exception if any keys to be inserted are already used. */
  HybridValues& insert(const DiscreteValues& values) {
    discrete_.insert(values);
    return *this;
  }

  /** Insert all values from \c values.  Throws an invalid_argument
   * exception if any keys to be inserted are already used. */
  HybridValues& insert(const Values& values) {
    nonlinear_.insert(values);
    return *this;
  }

  /** Insert all values from \c values.  Throws an invalid_argument exception if
   * any keys to be inserted are already used. */
  HybridValues& insert(const HybridValues& values) {
    continuous_.insert(values.continuous());
    discrete_.insert(values.discrete());
    nonlinear_.insert(values.nonlinear());
    return *this;
  }

  /**
   * Read/write access to the vector value with key \c j, throws
   * std::out_of_range if \c j does not exist.
   */
  Vector& at(Key j) { return continuous_.at(j); }

  /**
   * Read/write access to the discrete value with key \c j, throws
   * std::out_of_range if \c j does not exist.
   */
  size_t& atDiscrete(Key j) { return discrete_.at(j); }

  /** For all key/value pairs in \c values, replace continuous values with
   * corresponding keys in this object with those in \c values.  Throws
   * std::out_of_range if any keys in \c values are not present in this object.
   */
  HybridValues& update(const VectorValues& values) {
    continuous_.update(values);
    return *this;
  }

  /** For all key/value pairs in \c values, replace discrete values with
   * corresponding keys in this object with those in \c values.  Throws
   * std::out_of_range if any keys in \c values are not present in this object.
   */
  HybridValues& update(const DiscreteValues& values) {
    discrete_.update(values);
    return *this;
  }

  /** For all key/value pairs in \c values, replace all values with
   * corresponding keys in this object with those in \c values.  Throws
   * std::out_of_range if any keys in \c values are not present in this object.
   */
  HybridValues& update(const HybridValues& values) {
    continuous_.update(values.continuous());
    discrete_.update(values.discrete());
    return *this;
  }

  /// Extract continuous values with given keys.
  VectorValues continuousSubset(const KeyVector& keys) const {
    VectorValues measurements;
    for (const auto& key : keys) {
      measurements.insert(key, continuous_.at(key));
    }
    return measurements;
  }

  /// @}
  /// @name Wrapper support
  /// @{

  /**
   * @brief Output as a html table.
   *
   * @param keyFormatter function that formats keys.
   * @return string html output.
   */
  std::string html(
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    std::stringstream ss;
    ss << this->continuous_.html(keyFormatter);
    ss << this->discrete_.html(keyFormatter);
    return ss.str();
  }

  /// @}
};

// traits
template <>
struct traits<HybridValues> : public Testable<HybridValues> {};

}  // namespace gtsam
