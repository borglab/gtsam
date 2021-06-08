/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Chebyshev.cpp
 * @brief Chebyshev basis decompositions
 * @author Varun Agrawal, Frank Dellaert
 * @date July 4, 2020
 */

#include <gtsam/basis/Chebyshev.h>

namespace gtsam {

Weights Chebyshev2Basis::CalculateWeights(size_t N, double x) {
  Weights Ux(N);
  Ux(0) = 1;
  Ux(1) = 2 * x;
  for (size_t i = 2; i < N; i++) {
    // instead of cos(i*acos(x)), this recurrence is much faster
    Ux(i) = 2 * x * Ux(i - 1) - Ux(i - 2);
  }
  return Ux;
}

Weights Chebyshev1Basis::CalculateWeights(size_t N, double x) {
  Weights Tx(1, N);
  Tx(0) = 1;
  Tx(1) = x;
  for (size_t i = 2; i < N; i++) {
    // instead of cos(i*acos(x)), this recurrence is much faster
    Tx(i) = 2 * x * Tx(i - 1) - Tx(i - 2);
  }
  return Tx;
}

}  // namespace gtsam
