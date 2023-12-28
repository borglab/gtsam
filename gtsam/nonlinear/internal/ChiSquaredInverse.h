/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ChiSquaredInverse.h
 * @brief   Implementation of the Chi Squared inverse function.
 *
 * Uses the cephes 3rd party library to help with gamma inverse functions.
 *
 * @author  Varun Agrawal
 */

#pragma once

#include <gtsam/3rdparty/cephes/cephes.h>
#include <gtsam/dllexport.h>

namespace gtsam {

namespace internal {

/**
 * @brief Compute the quantile function of the Chi-Squared distribution.
 *
 * The quantile function of the Chi-squared distribution is the quantile of
 * the specific (inverse) incomplete Gamma distribution.
 *
 * We have a dedicated function so we can unit test any issues easily while also
 * allowing it to be updated in the future without any backwards-compatibility
 * issues.
 *
 * @param dofs Degrees of freedom
 * @param alpha Quantile value
 * @return double
 */
double GTSAM_EXPORT chi_squared_quantile(const double dofs,
                                         const double alpha) {
  return 2 * cephes_igami(dofs / 2, alpha);
}

}  // namespace internal

}  // namespace gtsam
