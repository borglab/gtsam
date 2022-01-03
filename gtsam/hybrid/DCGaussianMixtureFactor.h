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
#include <gtsam/linear/GaussianFactor.h>

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
  using FactorDecisionTree = DecisionTree<Key, GaussianFactor::shared_ptr>;

 private:
  /// Decision tree of Gaussian factors indexed by discrete keys.
  FactorDecisionTree factors_;

 public:
  DCGaussianMixtureFactor() = default;

  /**
   * @brief Construct a new DCGaussianMixtureFactor object.
   *
   * @param keys Vector of keys for continuous factors.
   * @param discreteKeys Vector of discrete assignments.
   * @param factors A decision tree of Gaussian factors (as shared pointers)
   * where each node has a Key label.
   */
  DCGaussianMixtureFactor(const KeyVector& keys,
                          const DiscreteKeys& discreteKeys,
                          const FactorDecisionTree factors)
      : Base(keys, discreteKeys), factors_(factors) {}

  /**
   * @brief Construct a new DCGaussianMixtureFactor object using a vector of
   * GaussianFactor shared pointers.
   *
   * @param keys Vector of keys for continuous factors.
   * @param discreteKeys Vector of discrete assignments.
   * @param factors Vector of gaussian factor shared pointers.
   */
  DCGaussianMixtureFactor(
      const KeyVector& keys, const DiscreteKeys& discreteKeys,
      const std::vector<GaussianFactor::shared_ptr>& factors)
      : DCGaussianMixtureFactor(keys, discreteKeys,
                                FactorDecisionTree(discreteKeys, factors)) {}

  /// Copy constructor.
  DCGaussianMixtureFactor(const DCGaussianMixtureFactor& x) = default;

  /// Discrete key selecting mixture component
  const DiscreteKeys& discreteKeys() const { return discreteKeys_; }

  ~DCGaussianMixtureFactor() = default;

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
    throw std::runtime_error("DCGaussianFactor is already linear");
  }

  // TODO(dellaert): implement
  size_t dim() const override {
    throw std::runtime_error("DCGaussianMixtureFactor::dim not implemented");
  };

  /// Testable
  /// @{

  /// print to stdout
  void print(
      const std::string& s = "DCGaussianMixtureFactor",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ");
    std::cout << "[";
    for (Key key : keys()) {
      std::cout << " " << keyFormatter(key);
    }
    std::cout << "; " << keyFormatter(discreteKeys_.front().first) << " ]";
    std::cout << "{\n";
    auto valueFormatter = [](const GaussianFactor::shared_ptr& v) {
      return (boost::format("%p") % v).str();
    };
    factors_.print("", keyFormatter, valueFormatter);
    std::cout << "}";
    std::cout << "\n";
  }

  /// Check equality
  bool equals(const DCFactor& f, double tol = 1e-9) const override {
    // TODO
    return true;
  }
  /// @}
};

}  // namespace gtsam
