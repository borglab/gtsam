/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DCGaussianMixtureFactor.h
 * @brief   A set of GaussianFactors, indexed by a set of discrete keys.
 * @author  Varun Agrawal
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#pragma once

#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/hybrid/DCFactor.h>
#include <gtsam/inference/Factor.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include <algorithm>
#include <boost/format.hpp>
#include <cmath>
#include <limits>
#include <vector>

namespace gtsam {

/**
 * @brief Implementation of a discrete conditional mixture factor. Implements a
 * joint discrete-continuous factor where the discrete variable serves to
 * "select" a mixture component corresponding to a GaussianFactor type
 * of measurement.
 */
class DCGaussianMixtureFactor : public DCFactor {
 public:
  using Base = DCFactor;
  using shared_ptr = boost::shared_ptr<DCGaussianMixtureFactor>;
  using Factors = DecisionTree<Key, GaussianFactor::shared_ptr>;

 protected:
  /// Decision tree of Gaussian factors indexed by discrete keys.
  Factors factors_;

 public:
  /// @name Constructors
  /// @{
  DCGaussianMixtureFactor() = default;

  /**
   * @brief Construct a new DCGaussianMixtureFactor object.
   *
   * @param continuousKeys - the keys for *continuous* variables
   * @param discreteKeys - the keys for *discrete* variables
   * @param factors A decision tree of Gaussian factors (as shared pointers)
   * where each node has a Key label.
   */
  DCGaussianMixtureFactor(const KeyVector& continuousKeys,
                          const DiscreteKeys& discreteKeys,
                          const Factors factors)
      : Base(continuousKeys, discreteKeys), factors_(factors) {}

  /**
   * @brief Construct a new DCGaussianMixtureFactor object using a vector of
   * GaussianFactor shared pointers.
   *
   * @param keys Vector of keys for continuous factors.
   * @param discreteKeys Vector of discrete keys.
   * @param factors Vector of gaussian factor shared pointers.
   */
  DCGaussianMixtureFactor(
      const KeyVector& keys, const DiscreteKeys& discreteKeys,
      const std::vector<GaussianFactor::shared_ptr>& factors)
      : DCGaussianMixtureFactor(keys, discreteKeys,
                                Factors(discreteKeys, factors)) {}

  /// Copy constructor.
  DCGaussianMixtureFactor(const DCGaussianMixtureFactor& x) = default;

  /// Discrete key selecting mixture component
  const DiscreteKeys& discreteKeys() const { return discreteKeys_; }

  ~DCGaussianMixtureFactor() = default;

  /// @}
  /// @name Standard Interface
  /// @{

  double error(const VectorValues& continuousVals,
               const DiscreteValues& discreteVals) const {
    return 0;
  }

  double error(const gtsam::Values& continuousVals,
               const gtsam::DiscreteValues& discreteVals) const override {
    throw std::runtime_error("DCGaussianMixtureFactor::error not implemented");
  }

  GaussianFactor::shared_ptr linearize(
      const gtsam::Values& continuousVals,
      const DiscreteValues& discreteVals) const override {
    throw std::runtime_error(
        "DCGaussianMixtureFactor::linearize(continuous, discrete) not "
        "implemented");
  }

  /// Return linearized version of this factor.
  virtual DCFactor::shared_ptr linearize(
      const Values& continuousVals) const override {
    throw std::runtime_error("DCGaussianFactorMixture is already linear");
  }

  // TODO(dellaert): implement
  size_t dim() const override {
    throw std::runtime_error("DCGaussianMixtureFactor::dim not implemented");
  };

  const GaussianFactor::shared_ptr& operator()(
      gtsam::DiscreteValues& discreteVals) const {
    return factors_(discreteVals);
  }
  /// @}
  /// @name Testable
  /// @{

  /// Print the keys, taking care of continuous and discrete.
  void printKeys(
      const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  /// print to stdout
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /// Check equality
  virtual bool equals(const DCFactor& f, double tol) const override;

  /// @}
  /// @name Decision Tree methods
  /// @{

  // TODO(frank): this could be cheaper with some linked list idea, but I think
  // it is not worth it.

  /// A Sum of mixture factors contains small GaussianFactorGraphs
  using Sum = DecisionTree<Key, GaussianFactorGraph>;

  /// Add MixtureFactor to a Sum
  Sum addTo(const Sum& sum) const;

  /// Add MixtureFactor to a Sum, sugar.
  friend Sum& operator+=(Sum& sum, const DCGaussianMixtureFactor& factor) {
    sum = factor.addTo(sum);
    return sum;
  }
  /// @}

  const Factors& factors();
 private:
  /// Return Sum decision tree with factors wrapped in Singleton FGs.
  Sum wrappedFactors() const;
};

}  // namespace gtsam
