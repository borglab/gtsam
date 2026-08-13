/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GncOptimizer.cpp
 * @brief   Definitions of functions declared in header file
 * @author  Varun Agrawal
 */

#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/internal/ChiSquaredInverse.h>

namespace gtsam {

/* ************************************************************************* */
double Chi2inv(const double alpha, const size_t dofs) {
  return internal::chiSquaredQuantile(dofs, alpha);
}

/* ************************************************************************* */
bool isNullType(GncFactorType type) {
  return type == GncFactorType::NullPointer;
}

/* ************************************************************************* */
bool isNonNoiseModelType(GncFactorType type) {
  return type == GncFactorType::NonNoiseModel;
}

/* ************************************************************************* */
bool needsWeightUpdate(GncFactorType type) {
  return type == GncFactorType::Normal;
}

/* ************************************************************************* */
bool hasNoise(GncFactorType type) {
  return type == GncFactorType::Normal || type == GncFactorType::Inlier ||
         type == GncFactorType::Outlier;
}

}  // namespace gtsam
