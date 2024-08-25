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
 *  @author Varun Agrawal
 *  @author Shangjie Xue
 */

#pragma once

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
  HybridValues(const VectorValues& cv, const DiscreteValues& dv);

  /// Construct from all values types.
  HybridValues(const VectorValues& cv, const DiscreteValues& dv,
               const Values& v);

  /// @}
  /// @name Testable
  /// @{

  /// print required by Testable for unit testing
  void print(const std::string& s = "HybridValues",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /// equals required by Testable for unit testing
  bool equals(const HybridValues& other, double tol = 1e-9) const;

  /// @}
  /// @name Interface
  /// @{

  /// Return the multi-dimensional vector values.
  const VectorValues& continuous() const;

  /// Return the discrete values.
  const DiscreteValues& discrete() const;

  /// Return the nonlinear values.
  const Values& nonlinear() const;

  /// Check whether a variable with key \c j exists in VectorValues.
  bool existsVector(Key j);

  /// Check whether a variable with key \c j exists in DiscreteValues.
  bool existsDiscrete(Key j);

  /// Check whether a variable with key \c j exists in values.
  bool existsNonlinear(Key j);

  /// Check whether a variable with key \c j exists.
  bool exists(Key j);

  /** Add a delta config to current config and returns a new config */
  HybridValues retract(const VectorValues& delta) const;

  /** Insert a vector \c value with key \c j.  Throws an invalid_argument
   * exception if the key \c j is already used.
   * @param value The vector to be inserted.
   * @param j The index with which the value will be associated. */
  void insert(Key j, const Vector& value);

  /** Insert a discrete \c value with key \c j.  Replaces the existing value if
   * the key \c j is already used.
   * @param value The vector to be inserted.
   * @param j The index with which the value will be associated. */
  void insert(Key j, size_t value);

  /// insert_or_assign() , similar to Values.h
  void insert_or_assign(Key j, const Vector& value);

  /// insert_or_assign() , similar to Values.h
  void insert_or_assign(Key j, size_t value);

  /** Insert all continuous values from \c values.  Throws an invalid_argument
   * exception if any keys to be inserted are already used. */
  HybridValues& insert(const VectorValues& values);

  /** Insert all discrete values from \c values.  Throws an invalid_argument
   * exception if any keys to be inserted are already used. */
  HybridValues& insert(const DiscreteValues& values);

  /** Insert all values from \c values.  Throws an invalid_argument
   * exception if any keys to be inserted are already used. */
  HybridValues& insert(const Values& values);

  /** Insert all values from \c values.  Throws an invalid_argument exception if
   * any keys to be inserted are already used. */
  HybridValues& insert(const HybridValues& values);

  /**
   * Read/write access to the vector value with key \c j, throws
   * std::out_of_range if \c j does not exist.
   */
  Vector& at(Key j);

  /**
   * Read/write access to the discrete value with key \c j, throws
   * std::out_of_range if \c j does not exist.
   */
  size_t& atDiscrete(Key j);

  /** For all key/value pairs in \c values, replace continuous values with
   * corresponding keys in this object with those in \c values.  Throws
   * std::out_of_range if any keys in \c values are not present in this object.
   */
  HybridValues& update(const VectorValues& values);

  /** For all key/value pairs in \c values, replace discrete values with
   * corresponding keys in this object with those in \c values.  Throws
   * std::out_of_range if any keys in \c values are not present in this object.
   */
  HybridValues& update(const DiscreteValues& values);

  /** For all key/value pairs in \c values, replace all values with
   * corresponding keys in this object with those in \c values.  Throws
   * std::out_of_range if any keys in \c values are not present in this object.
   */
  HybridValues& update(const HybridValues& values);

  /// Extract continuous values with given keys.
  VectorValues continuousSubset(const KeyVector& keys) const;

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
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /// @}
};

// traits
template <>
struct traits<HybridValues> : public Testable<HybridValues> {};

}  // namespace gtsam
