/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    RampFunction.cpp
 * @brief   Ramp function to compute inequality constraint violations.
 * @author  Yetong Zhang
 * @date    Aug 3, 2024
 */

#include <gtsam/constraint/RampFunction.h>

namespace gtsam {

/* ************************************************************************* */
double RampFunction(const double x, OptionalJacobian<1, 1> H) {
  if (x < 0) {
    if (H) {
      H->setConstant(0);
    }
    return 0;
  } else {
    if (H) {
      H->setConstant(1);
    }
    return x;
  }
}

/* ************************************************************************* */
SmoothRampFunction::UnaryScalarFunc SmoothRampFunction::function() const {
  return [=](const double& x, OptionalJacobian<1, 1> H = {}) -> double { return (*this)(x, H); };
}

/* ************************************************************************* */
double RampFunctionPoly2::operator()(const double& x, OptionalJacobian<1, 1> H) const {
  if (x <= 0) {
    if (H) {
      H->setZero();
    }
    return 0;
  } else if (x < epsilon_) {
    if (H) {
      H->setConstant(2 * a_ * x);
    }
    return a_ * pow(x, 2);
  } else {
    if (H) {
      H->setOnes();
    }
    return x - epsilon_ / 2;
  }
}

/* ************************************************************************* */
double RampFunctionPoly3::operator()(const double& x, OptionalJacobian<1, 1> H) const {
  if (x <= 0) {
    if (H) {
      H->setZero();
    }
    return 0;
  } else if (x < epsilon_) {
    if (H) {
      H->setConstant(3 * a_ * pow(x, 2) + 2 * b_ * x);
    }
    return a_ * pow(x, 3) + b_ * pow(x, 2);
  } else {
    if (H) {
      H->setOnes();
    }
    return x;
  }
}

/* ************************************************************************* */
double SoftPlusFunction::operator()(const double& x, OptionalJacobian<1, 1> H) const {
  if (H) {
    H->setConstant(1 / (1 + std::exp(-k_ * x)));
  }
  return std::log(1 + std::exp(k_ * x)) / k_;
}

}  // namespace gtsam
