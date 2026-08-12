/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Basis.cpp
 * @brief Compute an interpolating basis
 * @author Varun Agrawal
 * @date June 20, 2023
 */

#include <gtsam/basis/Basis.h>

namespace gtsam {

Matrix kroneckerProductIdentity(size_t M, const Weights& w) {
  Matrix result(M, w.cols() * M);
  result.setZero();

  for (int i = 0; i < w.cols(); i++) {
    result.block(0, i * M, M, M).diagonal().array() = w(i);
  }
  return result;
}

}  // namespace gtsam
