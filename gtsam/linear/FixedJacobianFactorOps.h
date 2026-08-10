/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file FixedJacobianFactorOps.h
 * @brief Internal fixed-size kernels shared by specialized Jacobian factors.
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/linear/VectorValues.h>

namespace gtsam {
namespace internal {

template <int M, int N>
using FixedJacobianBlock = Eigen::Block<const Matrix, M, N>;

/** Add one fixed-size Jacobian product to a fixed-size residual. */
template <int M, int N>
inline void accumulateResidual(const FixedJacobianBlock<M, N>& A,
                               const Vector& value,
                               Eigen::Matrix<double, M, 1>& residual) {
  residual.noalias() += A * value;
}

/** Add one fixed-size Jacobian's squared column norms to the diagonal. */
template <int M, int N>
inline void accumulateHessianDiagonal(Key key,
                                      const FixedJacobianBlock<M, N>& A,
                                      VectorValues& diagonal) {
  const Eigen::Matrix<double, N, 1> contribution =
      A.colwise().squaredNorm().transpose();
  auto result = diagonal.emplace(key, N);
  if (result.second) {
    result.first->second = contribution;
  } else {
    result.first->second += contribution;
  }
}

/** Add one fixed-size diagonal product to an augmented Hessian. */
template <int M, int N>
inline void updateSelfHessian(DenseIndex slot,
                              const FixedJacobianBlock<M, N>& A,
                              SymmetricBlockMatrix* info) {
  if constexpr (N == 1) {
    info->updateDiagonalBlock(slot, A.transpose() * A);
  } else {
    info->diagonalBlock(slot).rankUpdate(A.transpose());
  }
}

/** Add a fixed-size cross product whose dimensions are in canonical order. */
template <int M, int NLow, int NHigh>
inline void updateCrossHessianCanonical(
    DenseIndex slotLow, const FixedJacobianBlock<M, NLow>& ALow,
    DenseIndex slotHigh, const FixedJacobianBlock<M, NHigh>& AHigh,
    bool multiplyHighFirst, SymmetricBlockMatrix* info) {
  static_assert(NLow <= NHigh, "Cross-Hessian dimensions must be canonical");
  // Preserve the caller's multiplication order even when its dimensions were
  // swapped for canonical template dispatch.
  if (multiplyHighFirst) {
    info->updateOffDiagonalBlock(slotHigh, slotLow, AHigh.transpose() * ALow);
  } else {
    info->updateOffDiagonalBlock(slotLow, slotHigh, ALow.transpose() * AHigh);
  }
}

/** Add a fixed-size cross product, canonicalizing dimensions for code reuse. */
template <int M, int N1, int N2>
inline void updateCrossHessian(DenseIndex slot1,
                               const FixedJacobianBlock<M, N1>& A1,
                               DenseIndex slot2,
                               const FixedJacobianBlock<M, N2>& A2,
                               SymmetricBlockMatrix* info) {
  if constexpr (N1 <= N2) {
    updateCrossHessianCanonical<M, N1, N2>(slot1, A1, slot2, A2, false, info);
  } else {
    updateCrossHessianCanonical<M, N2, N1>(slot2, A2, slot1, A1, true, info);
  }
}

/** Add one Jacobian's self and right-hand-side Hessian products. */
template <int M, int N>
inline void updateJacobianHessian(DenseIndex slot,
                                  const FixedJacobianBlock<M, N>& A,
                                  DenseIndex slotB,
                                  const FixedJacobianBlock<M, 1>& b,
                                  SymmetricBlockMatrix* info) {
  updateSelfHessian<M, N>(slot, A, info);
  updateCrossHessian<M, N, 1>(slot, A, slotB, b, info);
}

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

#if defined(gtsam_EXPORTS) || defined(GTSAM_EXPORTS)
#if defined(__GNUC__)
#define GTSAM_FIXED_JACOBIAN_HIDDEN __attribute__((visibility("hidden")))
#else
#define GTSAM_FIXED_JACOBIAN_HIDDEN
#endif
#ifdef GTSAM_FIXED_JACOBIAN_DEFINE_INSTANTIATIONS
#define GTSAM_INSTANTIATE_FIXED_BLOCK(M, N)                                \
  template GTSAM_FIXED_JACOBIAN_HIDDEN EIGEN_DONT_INLINE void              \
  updateJacobianHessian<M, N>(DenseIndex, const FixedJacobianBlock<M, N>&, \
                              DenseIndex, const FixedJacobianBlock<M, 1>&, \
                              SymmetricBlockMatrix*);
#else
#define GTSAM_INSTANTIATE_FIXED_BLOCK(M, N)                                \
  extern template GTSAM_FIXED_JACOBIAN_HIDDEN EIGEN_DONT_INLINE void       \
  updateJacobianHessian<M, N>(DenseIndex, const FixedJacobianBlock<M, N>&, \
                              DenseIndex, const FixedJacobianBlock<M, 1>&, \
                              SymmetricBlockMatrix*);
#endif

GTSAM_FIXED_JACOBIAN_BLOCK_INSTANTIATIONS(GTSAM_INSTANTIATE_FIXED_BLOCK)

#undef GTSAM_INSTANTIATE_FIXED_BLOCK
#undef GTSAM_FIXED_JACOBIAN_HIDDEN
#endif
#undef GTSAM_FIXED_JACOBIAN_BLOCK_INSTANTIATIONS

}  // namespace internal
}  // namespace gtsam
