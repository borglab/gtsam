/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   DCConditional.h
 * @brief  Discrete-continuous conditional density
 * @author Frank Dellaert
 * @date   December 2021
 */

#pragma once

#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/hybrid/DCFactor.h>
#include <gtsam/hybrid/DCGaussianMixtureFactor.h>
#include <gtsam/inference/Conditional.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Symbol.h>
#include <math.h>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace gtsam {

/**
 * @brief A discrete-continuous conditional.
 *
 * keys_ member variable stores keys for *continuous* variables.
 * discreteKeys_ contains the keys (plus cardinalities) for *discrete*
 * variables.
 */
class DCConditional
    : public Conditional<DCGaussianMixtureFactor, DCConditional> {
 protected:
  // Set of DiscreteKeys for this factor.
  DiscreteKeys discreteKeys_;

 public:
  using Base = Conditional<DCGaussianMixtureFactor, DCConditional>;
  using shared_ptr = boost::shared_ptr<DCConditional>;

  DCConditional() = default;

  void print(const std::string& s = "DCConditional",
             const KeyFormatter& formatter = gtsam::DefaultKeyFormatter) const {
    std::cout << (s.empty() ? "" : s + " ") << std::endl;
  }
};
}  // namespace gtsam
