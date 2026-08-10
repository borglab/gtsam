/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file FixedJacobianFactorOps.cpp
 * @brief Explicit instantiations of common fixed-size Hessian kernels.
 * @author Frank Dellaert
 */

#include <gtsam/dllexport.h>
#include <gtsam/linear/internal/FixedJacobianFactorOps.h>

namespace gtsam {
namespace internal {

// This registry is intentionally limited to dimensions induced by common
// binary and ternary factors. Other dimensions instantiate the inline fallback.
#define GTSAM_FIXED_JACOBIAN_BLOCK_INSTANTIATIONS(MACRO) \
  MACRO(1, 1)                                            \
  MACRO(1, 2)                                            \
  MACRO(1, 3)                                            \
  MACRO(1, 6)                                            \
  MACRO(2, 1)                                            \
  MACRO(2, 2)                                            \
  MACRO(2, 3)                                            \
  MACRO(2, 5)                                            \
  MACRO(2, 6)                                            \
  MACRO(2, 9)                                            \
  MACRO(3, 1)                                            \
  MACRO(3, 3)                                            \
  MACRO(3, 6)                                            \
  MACRO(3, 9)                                            \
  MACRO(5, 1)                                            \
  MACRO(5, 6)                                            \
  MACRO(6, 1)                                            \
  MACRO(6, 6)                                            \
  MACRO(9, 1)                                            \
  MACRO(9, 3)                                            \
  MACRO(9, 6)                                            \
  MACRO(9, 9)

#define GTSAM_INSTANTIATE_FIXED_BLOCK(M, N)                                \
  template GTSAM_LOCAL EIGEN_DONT_INLINE void updateJacobianHessian<M, N>( \
      DenseIndex, const FixedJacobianBlock<M, N>&, DenseIndex,             \
      const FixedJacobianBlock<M, 1>&, SymmetricBlockMatrix*);

GTSAM_FIXED_JACOBIAN_BLOCK_INSTANTIATIONS(GTSAM_INSTANTIATE_FIXED_BLOCK)

#undef GTSAM_INSTANTIATE_FIXED_BLOCK
#undef GTSAM_FIXED_JACOBIAN_BLOCK_INSTANTIATIONS

}  // namespace internal
}  // namespace gtsam
