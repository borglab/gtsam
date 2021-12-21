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
class DCGaussianMixtureFactor : public Factor {
 private:
  DiscreteKeys discreteKeys_;
  std::vector<GaussianFactor::shared_ptr> factors_;

 public:
  using Base = Factor;
  using shared_ptr = boost::shared_ptr<DCGaussianMixtureFactor>;
  
  DCGaussianMixtureFactor() = default;

  DCGaussianMixtureFactor(
      const KeyVector& keys, const DiscreteKeys& dk,
      const std::vector<GaussianFactor::shared_ptr>& factors)
      : discreteKeys_(dk), factors_(factors) {
    // Compiler doesn't like `keys_` in the initializer list.
    keys_ = keys;
  }

  /// Discrete key selecting mixture component
  const DiscreteKeys& discreteKeys() const { return discreteKeys_; }

  ~DCGaussianMixtureFactor() = default;

  double error(const VectorValues& continuousVals,
               const DiscreteValues& discreteVals) const {
    return 0;
  }

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
    // std::cout << "; " << formatter(discreteKeys_.front().first) << " ]";
    // if (false) {
    //   std::cout << "{\n";
    //   for (int i = 0; i < factors_.size(); i++) {
    //     auto t = boost::format("component %1%: ") % i;
    //     factors_[i].print(t.str());
    //   }
    //   std::cout << "}";
    // }
    std::cout << "\n";
  }

  /// Check equality
  bool equals(const DCGaussianMixtureFactor& f, double tol = 1e-9) const {
    // TODO
    return true;
  }
  /// @}
};

}  // namespace gtsam
