/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file FlatGaussianFactor.h
 * @brief Optional preindexed kernels for Gaussian factors.
 * @author Frank Dellaert (using 5.6 Sol)
 */

#pragma once

#include <gtsam/base/Matrix.h>

#include <cstddef>
#include <vector>

namespace gtsam {

/**
 * Optional preindexed kernels for matrix-free Gaussian factors.
 *
 * A GaussianFactor can additionally implement this interface so iterative
 * solvers can apply it using scalar offsets and block slots compiled from the
 * solver's ordering. This avoids rebuilding VectorValues and keyed maps in hot
 * loops without changing the factor's ordinary GaussianFactor interface.
 */
class GTSAM_EXPORT FlatGaussianFactor {
 public:
  virtual ~FlatGaussianFactor();

  /** Add this factor's Hessian-vector product to a flat output vector. */
  virtual void multiplyHessianAdd(
      double alpha, const std::vector<size_t>& scalarOffsets, const double* x,
      double* y) const = 0;

  /** Add this factor's zero-point gradient to a flat output vector. */
  virtual void gradientAtZeroAdd(const std::vector<size_t>& scalarOffsets,
                                 double* gradient) const = 0;

  /** Add this factor's Hessian diagonal to ordered variable blocks. */
  virtual void hessianBlockDiagonalAdd(
      const std::vector<size_t>& blockSlots,
      std::vector<Matrix>* diagonalBlocks) const = 0;
};

}  // namespace gtsam
