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
#include <gtsam/inference/Factor.h>
#include <gtsam/linear/GaussianFactor.h>

#include <algorithm>
#include <boost/format.hpp>
#include <cmath>
#include <limits>
#include <vector>

#include "DCFactor.h"

namespace gtsam {

/**
 * @brief Implementation of a discrete conditional mixture factor. Implements a
 * joint discrete-continuous factor where the discrete variable serves to
 * "select" a mixture component corresponding to a GaussianFactor type
 * of measurement.
 */
class DCGaussianMixtureFactor : public DCFactor {
 private:
  std::vector<GaussianFactor::shared_ptr> factors_;

 public:
  using Base = DCFactor;
  using shared_ptr = boost::shared_ptr<DCGaussianMixtureFactor>;

  DCGaussianMixtureFactor() = default;

  DCGaussianMixtureFactor(
      const KeyVector& keys, const DiscreteKeys& discreteKeys,
      const std::vector<GaussianFactor::shared_ptr>& factors)
      : Base(keys, discreteKeys), factors_(factors) {}

  DCGaussianMixtureFactor(const DCGaussianMixtureFactor& x) = default;

  /// Discrete key selecting mixture component
  const DiscreteKeys& discreteKeys() const { return discreteKeys_; }

  ~DCGaussianMixtureFactor() = default;

  double error(const VectorValues& continuousVals,
               const DiscreteValues& discreteVals) const {
    return 0;
  }

  // TODO(dellaert): Does not make sense
  double error(const gtsam::Values& continuousVals,
               const gtsam::DiscreteValues& discreteVals) const override {
    return 0;
  }

  // TODO(dellaert): Does not make sense
  GaussianFactor::shared_ptr linearize(
      const gtsam::Values& continuousVals,
      const DiscreteValues& discreteVals) const override {
    return factors_[0];
  }

  /// Return this linear factor as a DCFactor::shared_ptr
  virtual DCFactor::shared_ptr linearize(
      const Values& continuousVals) const override {
    return boost::make_shared<DCGaussianMixtureFactor>(*this);
  }

  // TODO(dellaert): implement
  size_t dim() const override { return 0; };

  /// Testable
  /// @{

  /// print to stdout
  void print(
      const std::string& s = "DCGaussianMixtureFactor",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ");
    std::cout << "[";
    for (Key key : keys()) {
      std::cout << " " << formatter(key);
    }
    std::cout << "; " << formatter(discreteKeys_.front().first) << " ]";
    std::cout << "{\n";
    for (size_t i = 0; i < factors_.size(); i++) {
      auto t = boost::format("component %1%: ") % i;
      factors_[i]->print(t.str());
    }
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
