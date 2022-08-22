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
 * HybridValues represents a collection of DiscreteValues and VectorValues.  It
 * is typically used to store the variables of a HybridGaussianFactorGraph.
 * Optimizing a HybridGaussianBayesNet returns this class.
 */
class GTSAM_EXPORT HybridValues {
 public:
  // DiscreteValue stored the discrete components of the HybridValues.
  DiscreteValues discrete;

  // VectorValue stored the continuous components of the HybridValues.
  VectorValues continuous;

  // Default constructor creates an empty HybridValues.
  HybridValues() : discrete(), continuous(){};

  // Construct from DiscreteValues and VectorValues.
  HybridValues(const DiscreteValues& dv, const VectorValues& cv)
      : discrete(dv), continuous(cv){};

  // print required by Testable for unit testing
  void print(const std::string& s = "HybridValues",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    std::cout << s << ": \n";
    discrete.print("  Discrete", keyFormatter);  // print discrete components
    continuous.print("  Continuous",
                     keyFormatter);  // print continuous components
  };

  // equals required by Testable for unit testing
  bool equals(const HybridValues& other, double tol = 1e-9) const {
    return discrete.equals(other.discrete, tol) &&
           continuous.equals(other.continuous, tol);
  }

  // Check whether a variable with key \c j exists in DiscreteValue.
  bool existsDiscrete(Key j) { return (discrete.find(j) != discrete.end()); };

  // Check whether a variable with key \c j exists in VectorValue.
  bool existsVector(Key j) { return continuous.exists(j); };

  // Check whether a variable with key \c j exists.
  bool exists(Key j) { return existsDiscrete(j) || existsVector(j); };

  /** Insert a discrete \c value with key \c j.  Replaces the existing value if
   * the key \c j is already used.
   * @param value The vector to be inserted.
   * @param j The index with which the value will be associated. */
  void insert(Key j, int value) { discrete[j] = value; };

  /** Insert a vector \c value with key \c j.  Throws an invalid_argument
   * exception if the key \c j is already used.
   * @param value The vector to be inserted.
   * @param j The index with which the value will be associated. */
  void insert(Key j, const Vector& value) { continuous.insert(j, value); }

  // TODO(Shangjie)- update() and insert_or_assign() , similar to Values.h

  /**
   * Read/write access to the discrete value with key \c j, throws
   * std::out_of_range if \c j does not exist.
   */
  size_t& atDiscrete(Key j) { return discrete.at(j); };

  /**
   * Read/write access to the vector value with key \c j, throws
   * std::out_of_range if \c j does not exist.
   */
  Vector& at(Key j) { return continuous.at(j); };

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
    ss << this->discrete.html(keyFormatter);
    ss << this->continuous.html(keyFormatter);
    return ss.str();
  };

  /// @}
};

// traits
template <>
struct traits<HybridValues> : public Testable<HybridValues> {};

}  // namespace gtsam
