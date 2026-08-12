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
 * @author Frank Dellaert
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

#if defined(gtsam_EXPORTS) || defined(GTSAM_EXPORTS)
extern template EIGEN_DONT_INLINE void updateJacobianHessian<1, 1>(
    DenseIndex, const FixedJacobianBlock<1, 1>&, DenseIndex,
    const FixedJacobianBlock<1, 1>&, SymmetricBlockMatrix*);
extern template EIGEN_DONT_INLINE void updateJacobianHessian<1, 2>(
    DenseIndex, const FixedJacobianBlock<1, 2>&, DenseIndex,
    const FixedJacobianBlock<1, 1>&, SymmetricBlockMatrix*);
extern template EIGEN_DONT_INLINE void updateJacobianHessian<1, 3>(
    DenseIndex, const FixedJacobianBlock<1, 3>&, DenseIndex,
    const FixedJacobianBlock<1, 1>&, SymmetricBlockMatrix*);
extern template EIGEN_DONT_INLINE void updateJacobianHessian<1, 6>(
    DenseIndex, const FixedJacobianBlock<1, 6>&, DenseIndex,
    const FixedJacobianBlock<1, 1>&, SymmetricBlockMatrix*);
extern template EIGEN_DONT_INLINE void updateJacobianHessian<2, 1>(
    DenseIndex, const FixedJacobianBlock<2, 1>&, DenseIndex,
    const FixedJacobianBlock<2, 1>&, SymmetricBlockMatrix*);
extern template EIGEN_DONT_INLINE void updateJacobianHessian<2, 2>(
    DenseIndex, const FixedJacobianBlock<2, 2>&, DenseIndex,
    const FixedJacobianBlock<2, 1>&, SymmetricBlockMatrix*);
extern template EIGEN_DONT_INLINE void updateJacobianHessian<2, 3>(
    DenseIndex, const FixedJacobianBlock<2, 3>&, DenseIndex,
    const FixedJacobianBlock<2, 1>&, SymmetricBlockMatrix*);
extern template EIGEN_DONT_INLINE void updateJacobianHessian<2, 5>(
    DenseIndex, const FixedJacobianBlock<2, 5>&, DenseIndex,
    const FixedJacobianBlock<2, 1>&, SymmetricBlockMatrix*);
extern template EIGEN_DONT_INLINE void updateJacobianHessian<2, 6>(
    DenseIndex, const FixedJacobianBlock<2, 6>&, DenseIndex,
    const FixedJacobianBlock<2, 1>&, SymmetricBlockMatrix*);
extern template EIGEN_DONT_INLINE void updateJacobianHessian<2, 9>(
    DenseIndex, const FixedJacobianBlock<2, 9>&, DenseIndex,
    const FixedJacobianBlock<2, 1>&, SymmetricBlockMatrix*);
extern template EIGEN_DONT_INLINE void updateJacobianHessian<3, 1>(
    DenseIndex, const FixedJacobianBlock<3, 1>&, DenseIndex,
    const FixedJacobianBlock<3, 1>&, SymmetricBlockMatrix*);
extern template EIGEN_DONT_INLINE void updateJacobianHessian<3, 3>(
    DenseIndex, const FixedJacobianBlock<3, 3>&, DenseIndex,
    const FixedJacobianBlock<3, 1>&, SymmetricBlockMatrix*);
extern template EIGEN_DONT_INLINE void updateJacobianHessian<3, 6>(
    DenseIndex, const FixedJacobianBlock<3, 6>&, DenseIndex,
    const FixedJacobianBlock<3, 1>&, SymmetricBlockMatrix*);
extern template EIGEN_DONT_INLINE void updateJacobianHessian<3, 9>(
    DenseIndex, const FixedJacobianBlock<3, 9>&, DenseIndex,
    const FixedJacobianBlock<3, 1>&, SymmetricBlockMatrix*);
extern template EIGEN_DONT_INLINE void updateJacobianHessian<5, 1>(
    DenseIndex, const FixedJacobianBlock<5, 1>&, DenseIndex,
    const FixedJacobianBlock<5, 1>&, SymmetricBlockMatrix*);
extern template EIGEN_DONT_INLINE void updateJacobianHessian<5, 6>(
    DenseIndex, const FixedJacobianBlock<5, 6>&, DenseIndex,
    const FixedJacobianBlock<5, 1>&, SymmetricBlockMatrix*);
extern template EIGEN_DONT_INLINE void updateJacobianHessian<6, 1>(
    DenseIndex, const FixedJacobianBlock<6, 1>&, DenseIndex,
    const FixedJacobianBlock<6, 1>&, SymmetricBlockMatrix*);
extern template EIGEN_DONT_INLINE void updateJacobianHessian<6, 6>(
    DenseIndex, const FixedJacobianBlock<6, 6>&, DenseIndex,
    const FixedJacobianBlock<6, 1>&, SymmetricBlockMatrix*);
extern template EIGEN_DONT_INLINE void updateJacobianHessian<9, 1>(
    DenseIndex, const FixedJacobianBlock<9, 1>&, DenseIndex,
    const FixedJacobianBlock<9, 1>&, SymmetricBlockMatrix*);
extern template EIGEN_DONT_INLINE void updateJacobianHessian<9, 3>(
    DenseIndex, const FixedJacobianBlock<9, 3>&, DenseIndex,
    const FixedJacobianBlock<9, 1>&, SymmetricBlockMatrix*);
extern template EIGEN_DONT_INLINE void updateJacobianHessian<9, 6>(
    DenseIndex, const FixedJacobianBlock<9, 6>&, DenseIndex,
    const FixedJacobianBlock<9, 1>&, SymmetricBlockMatrix*);
extern template EIGEN_DONT_INLINE void updateJacobianHessian<9, 9>(
    DenseIndex, const FixedJacobianBlock<9, 9>&, DenseIndex,
    const FixedJacobianBlock<9, 1>&, SymmetricBlockMatrix*);
#endif

}  // namespace internal
}  // namespace gtsam
